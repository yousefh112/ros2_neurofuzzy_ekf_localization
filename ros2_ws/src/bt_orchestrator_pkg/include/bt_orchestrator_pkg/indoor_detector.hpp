#pragma once

#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <mutex>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

class IndoorDetector
{
public:
    enum class Environment { OUTDOOR, INDOOR };

    struct Config
    {
        // 2D AABB geofence (world frame, derived from indoor_outdoor.world wall poses)
        bool   use_geofence        = true;

        // Geofence source is /odom, NOT /odometry/global. GPS multipath bias varies
        // run-to-run (up to 0.44 m), causing premature geofence fire. /odom drift
        // is <0.2 m per 100 m — accurate enough for the binary inside/outside decision.
        //
        // Inner faces:  west=-10.781  east=+11.124  south=-4.074  north=+4.467
        // South boundary -4.00 (vs inner face -4.074): 0.074 m inside the wall.
        // /odom drift at building entry is typically <0.25 m → robot is truly
        // inside when geofence fires, with margin against premature outdoor fire.
        double building_x_min_world = -10.68;
        double building_x_max_world = +11.02;
        double building_y_min_world =  -4.00;
        double building_y_max_world =  +4.37;

        // spawn_world_x/y = 0.0: /odom is initialised at world spawn pose by
        // Gazebo's diff_drive plugin, so odom == world frame directly.
        double spawn_world_x       = 0.0;
        double spawn_world_y       = 0.0;

        double hysteresis_m        = 0.20;

        // GPS quality scoring (secondary path / real hardware)
        double cov_warn_m2         =   4.0;
        double cov_bad_m2          =  25.0;
        double cov_critical_m2     = 100.0;

        double jump_minor_m        =  0.5;
        double jump_moderate_m     =  2.0;
        double jump_severe_m       =  5.0;

        bool   use_vel_mismatch    = true;
        double vel_mismatch_warn   =  0.20;
        double vel_mismatch_bad    =  0.50;

        bool   use_pos_div         = true;
        double pos_div_warn_m      =  1.0;
        double pos_div_bad_m       =  3.0;

        double signal_timeout_s    = 2.0;

        int    score_indoor_thr    = 3;
        int    score_critical_thr  = 7;

        int    debounce_in_ticks   = 2;
        int    debounce_out_ticks  = 4;

        double diag_period_s       = 0.5;
    };

    IndoorDetector(rclcpp::Node*                      node,
                   std::shared_ptr<std::atomic<bool>> indoor_flag,
                   const Config&                      cfg)
    : node_(node), flag_(indoor_flag), cfg_(cfg),
      env_(Environment::OUTDOOR),
      consec_bad_(0), consec_good_(0),
      gps_timed_out_(false),
      last_gps_stamp_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      has_odom_(false), odom_x_(0.0), odom_y_(0.0),
      has_gps_enu_(false), gps_enu_x_(0.0), gps_enu_y_(0.0),
      gps_enu_vx_(0.0), gps_enu_vy_(0.0),
      has_prev_gps_(false), prev_gps_x_(0.0), prev_gps_y_(0.0),
      last_jump_m_(0.0), last_div_m_(0.0), last_vel_mis_(0.0),
      last_score_(0), last_cov_(0.0), last_fix_status_(0)
    {
        auto sq = rclcpp::SensorDataQoS();

        sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m){ on_odom(m); });

        sub_fix_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", sq,
            [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m){ on_fix(m); });

        sub_gps_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/gps", rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m){ on_gps_odom(m); });

        diag_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/bt/indoor_detection", rclcpp::QoS(10));

        diag_timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(cfg_.diag_period_s),
            [this](){ evaluate_timeout(); publish_diagnostics(); });

        log_startup();
    }

    Environment getEnvironment() const { return env_.load(); }
    bool        isIndoor()       const { return flag_->load(); }

    void forceIndoor(bool indoor)
    {
        env_.store(indoor ? Environment::INDOOR : Environment::OUTDOOR);
        flag_->store(indoor);
        RCLCPP_WARN(node_->get_logger(),
            "[IndoorDetector] MANUAL OVERRIDE → %s",
            indoor ? "INDOOR" : "OUTDOOR");
    }

private:

    static bool in_box(double px, double py,
                       double x_min, double x_max,
                       double y_min, double y_max)
    {
        return px > x_min && px < x_max &&
               py > y_min && py < y_max;
    }

    // Schmitt-trigger hysteresis: INDOOR when robot enters inner AABB;
    // OUTDOOR only when robot exits outer AABB (inner + hysteresis_m).
    // The band prevents oscillation while the robot passes through the door.
    void check_geofence(double world_x, double world_y)
    {
        Environment cur = env_.load();

        if (cur == Environment::OUTDOOR) {
            bool enters = in_box(world_x, world_y,
                                 cfg_.building_x_min_world,
                                 cfg_.building_x_max_world,
                                 cfg_.building_y_min_world,
                                 cfg_.building_y_max_world);
            if (enters) {
                RCLCPP_WARN(node_->get_logger(),
                    "[IndoorDetector] ■ GEOFENCE ENTRY → INDOOR  "
                    "world=(%.2f, %.2f)", world_x, world_y);
                flip_to(Environment::INDOOR);
            }
        } else {
            double h = cfg_.hysteresis_m;
            bool exits = !in_box(world_x, world_y,
                                 cfg_.building_x_min_world - h,
                                 cfg_.building_x_max_world + h,
                                 cfg_.building_y_min_world - h,
                                 cfg_.building_y_max_world + h);
            if (exits) {
                RCLCPP_INFO(node_->get_logger(),
                    "[IndoorDetector] ● GEOFENCE EXIT → OUTDOOR  "
                    "world=(%.2f, %.2f)", world_x, world_y);
                flip_to(Environment::OUTDOOR);
            }
        }
    }

    int compute_gps_score(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg,
                          double jump_m, double div_m, double vel_mis)
    {
        int s = 0;

        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
            s += 5;

        double cov = msg->position_covariance[0];
        if      (cov >= cfg_.cov_critical_m2) s += 3;
        else if (cov >= cfg_.cov_bad_m2)      s += 2;
        else if (cov >= cfg_.cov_warn_m2)     s += 1;

        if      (jump_m >= cfg_.jump_severe_m)   s += 3;
        else if (jump_m >= cfg_.jump_moderate_m) s += 2;
        else if (jump_m >= cfg_.jump_minor_m)    s += 1;

        if (cfg_.use_pos_div) {
            if      (div_m >= cfg_.pos_div_bad_m)  s += 2;
            else if (div_m >= cfg_.pos_div_warn_m) s += 1;
        }

        if (cfg_.use_vel_mismatch) {
            if      (vel_mis >= cfg_.vel_mismatch_bad)  s += 2;
            else if (vel_mis >= cfg_.vel_mismatch_warn) s += 1;
        }

        return s;
    }

    void update_gps_state(bool bad_tick, bool critical)
    {
        Environment cur = env_.load();

        if (bad_tick) { consec_good_ = 0; ++consec_bad_; }
        else          { consec_bad_  = 0; ++consec_good_; }

        if (cur == Environment::OUTDOOR) {
            if (critical || consec_bad_ >= cfg_.debounce_in_ticks) {
                RCLCPP_WARN(node_->get_logger(),
                    "[IndoorDetector] ■ GPS-score → INDOOR  "
                    "score=%d  critical=%d  bad_ticks=%d",
                    last_score_, critical, consec_bad_);
                flip_to(Environment::INDOOR);
            }
        } else {
            if (consec_good_ >= cfg_.debounce_out_ticks) {
                // Geofence veto: GPS score alone cannot declare OUTDOOR while the
                // robot is still physically inside the building. Without this veto
                // the GPS path and geofence path oscillate at ~1 Hz near the door.
                if (cfg_.use_geofence && has_odom_) {
                    double wx = odom_x_ + cfg_.spawn_world_x;
                    double wy = odom_y_ + cfg_.spawn_world_y;
                    double h  = cfg_.hysteresis_m;
                    bool still_inside = in_box(wx, wy,
                        cfg_.building_x_min_world - h,
                        cfg_.building_x_max_world + h,
                        cfg_.building_y_min_world - h,
                        cfg_.building_y_max_world + h);
                    if (still_inside) {
                        RCLCPP_INFO_THROTTLE(node_->get_logger(),
                            *node_->get_clock(), 5000,
                            "[IndoorDetector] GPS OUTDOOR vetoed by geofence  "
                            "world=(%.2f, %.2f)  good_ticks=%d",
                            wx, wy, consec_good_);
                        consec_good_ = 0;
                        return;
                    }
                }
                RCLCPP_INFO(node_->get_logger(),
                    "[IndoorDetector] ● GPS-score → OUTDOOR  good_ticks=%d",
                    consec_good_);
                flip_to(Environment::OUTDOOR);
            }
        }
    }

    void flip_to(Environment next)
    {
        env_.store(next);
        flag_->store(next == Environment::INDOOR);
        consec_bad_  = 0;
        consec_good_ = 0;
    }

    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        odom_x_   = msg->pose.pose.position.x;
        odom_y_   = msg->pose.pose.position.y;
        odom_vx_  = msg->twist.twist.linear.x;
        has_odom_ = true;

        if (!cfg_.use_geofence) return;

        double wx = odom_x_ + cfg_.spawn_world_x;
        double wy = odom_y_ + cfg_.spawn_world_y;
        check_geofence(wx, wy);
    }

    void on_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        last_gps_stamp_  = node_->now();
        gps_timed_out_   = false;
        last_fix_status_ = msg->status.status;
        last_cov_        = msg->position_covariance[0];

        double div_m = 0.0;
        if (has_odom_ && has_gps_enu_) {
            double dx = gps_enu_x_ - odom_x_;
            double dy = gps_enu_y_ - odom_y_;
            div_m = std::sqrt(dx*dx + dy*dy);
        }
        last_div_m_ = div_m;

        double vel_mis = 0.0;
        if (cfg_.use_vel_mismatch && has_odom_ && has_gps_enu_) {
            double v_gps  = std::sqrt(gps_enu_vx_*gps_enu_vx_ +
                                      gps_enu_vy_*gps_enu_vy_);
            double v_odom = std::abs(odom_vx_);
            vel_mis = std::abs(v_gps - v_odom);
        }
        last_vel_mis_ = vel_mis;

        last_score_ = compute_gps_score(msg, last_jump_m_, div_m, vel_mis);
        last_jump_m_ = 0.0;

        bool bad      = (last_score_ >= cfg_.score_indoor_thr);
        bool critical = (last_score_ >= cfg_.score_critical_thr);
        update_gps_state(bad, critical);
    }

    void on_gps_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        double x  = msg->pose.pose.position.x;
        double y  = msg->pose.pose.position.y;
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;

        if (has_prev_gps_) {
            double dx = x - prev_gps_x_;
            double dy = y - prev_gps_y_;
            last_jump_m_ = std::sqrt(dx*dx + dy*dy);
        }

        prev_gps_x_ = x;  prev_gps_y_ = y;
        has_prev_gps_ = true;

        gps_enu_x_  = x;  gps_enu_y_  = y;
        gps_enu_vx_ = vx; gps_enu_vy_ = vy;
        has_gps_enu_ = true;
    }

    void evaluate_timeout()
    {
        std::lock_guard<std::mutex> lk(mtx_);

        if (last_gps_stamp_.nanoseconds() == 0) return;

        double age = (node_->now() - last_gps_stamp_).seconds();
        if (age > cfg_.signal_timeout_s && !gps_timed_out_) {
            gps_timed_out_ = true;
            bool critical  = (age > cfg_.signal_timeout_s * 2.0);
            RCLCPP_WARN(node_->get_logger(),
                "[IndoorDetector] GPS timeout %.1f s (thr=%.1f s)%s",
                age, cfg_.signal_timeout_s,
                critical ? " – CRITICAL" : "");
            update_gps_state(true, critical);
        }
    }

    void publish_diagnostics()
    {
        double wx, wy, div_m, vel_mis, jump_m, age_s, cov;
        int score, fix, cb, cg;
        Environment env;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            wx      = odom_x_ + cfg_.spawn_world_x;
            wy      = odom_y_ + cfg_.spawn_world_y;
            div_m   = last_div_m_;
            vel_mis = last_vel_mis_;
            jump_m  = last_jump_m_;
            age_s   = last_gps_stamp_.nanoseconds() > 0
                      ? (node_->now() - last_gps_stamp_).seconds() : -1.0;
            score   = last_score_;
            fix     = last_fix_status_;
            cov     = last_cov_;
            cb      = consec_bad_;
            cg      = consec_good_;
        }
        env = env_.load();

        double dist_N = cfg_.building_y_max_world - wy;
        double dist_S = wy - cfg_.building_y_min_world;
        double dist_E = cfg_.building_x_max_world - wx;
        double dist_W = wx - cfg_.building_x_min_world;
        double wall_dist = std::min({dist_N, dist_S, dist_E, dist_W});

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << (env == Environment::INDOOR ? "■ INDOOR" : "● OUTDOOR");
        ss << " | world=(" << wx << "," << wy << ")";
        ss << " | wall_dist=" << wall_dist << "m";
        ss << " | gps_score=" << score;
        ss << " | fix=" << fix;
        ss << " | cov=" << cov << "m²";
        ss << " | jump=" << jump_m << "m";
        ss << " | pos_div=" << div_m << "m";
        ss << " | vel_mis=" << vel_mis << "m/s";
        ss << " | age=" << age_s << "s";
        ss << " | bad=" << cb << "/" << cfg_.debounce_in_ticks;
        ss << " good=" << cg << "/" << cfg_.debounce_out_ticks;

        auto m = std_msgs::msg::String{};
        m.data = ss.str();
        diag_pub_->publish(m);

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[IndoorDetector] %s", ss.str().c_str());
    }

    void log_startup()
    {
        RCLCPP_INFO(node_->get_logger(),
            "\n[IndoorDetector]\n"
            "  Mode     : %s\n"
            "  Building : X∈[%.3f, %.3f]  Y∈[%.3f, %.3f]  (world frame)\n"
            "  Hysteresis: %.2f m\n"
            "  Debounce : in=%d  out=%d  (GPS path; geofence = instant)\n"
            "  Topics   : /odom  /gps/fix  /odometry/gps",
            cfg_.use_geofence ? "GEOFENCE+GPS_score" : "GPS_score only",
            cfg_.building_x_min_world, cfg_.building_x_max_world,
            cfg_.building_y_min_world, cfg_.building_y_max_world,
            cfg_.hysteresis_m,
            cfg_.debounce_in_ticks, cfg_.debounce_out_ticks);
    }

    rclcpp::Node*                       node_;
    std::shared_ptr<std::atomic<bool>>  flag_;
    Config                              cfg_;
    std::atomic<Environment>            env_;
    std::mutex                          mtx_;

    int   consec_bad_, consec_good_;

    rclcpp::Time  last_gps_stamp_;
    bool          gps_timed_out_;

    bool   has_odom_;
    double odom_x_, odom_y_, odom_vx_{0.0};

    bool   has_gps_enu_;
    double gps_enu_x_, gps_enu_y_;
    double gps_enu_vx_, gps_enu_vy_;
    bool   has_prev_gps_;
    double prev_gps_x_, prev_gps_y_;

    double last_jump_m_, last_div_m_, last_vel_mis_;
    int    last_score_, last_fix_status_;
    double last_cov_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  sub_fix_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_gps_odom_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           diag_pub_;
    rclcpp::TimerBase::SharedPtr                                  diag_timer_;
};
