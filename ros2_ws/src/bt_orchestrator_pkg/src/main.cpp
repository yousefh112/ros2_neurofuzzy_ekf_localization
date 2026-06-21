#include <iostream>
#include <memory>
#include <string>
#include <mutex>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "bt_orchestrator_pkg/indoor_detector.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace BT;
using namespace std::chrono_literals;

struct FuzzyMembership {
    double low    = 0.0;
    double medium = 0.0;
    double high   = 0.0;
};

static double triangular_mf(double x, double left, double center, double right)
{
    if (x <= left || x >= right) return 0.0;
    if (x <= center) return (x - left) / (center - left);
    return (right - x) / (right - center);
}

static double shoulder_left_mf(double x, double center, double right)
{
    if (x <= center) return 1.0;
    if (x >= right)  return 0.0;
    return (right - x) / (right - center);
}

static double shoulder_right_mf(double x, double left, double center)
{
    if (x <= left)   return 0.0;
    if (x >= center) return 1.0;
    return (x - left) / (center - left);
}

struct RobotState {
    mutable std::mutex mtx;

    bool   gps_available      = false;
    double last_known_gps_x   = 0.0;
    double last_known_gps_y   = 0.0;
    bool   has_ever_had_gps   = false;
    bool   gps_odom_ready     = false;
    double gps_odom_timestamp = 0.0;

    bool bt_fused_valid                              = false;
    bool map_to_odom_valid                           = false;
    geometry_msgs::msg::TransformStamped map_to_odom_cached;

    double imu_ax      = 0.0;
    double imu_ay      = 0.0;
    double imu_wz      = 0.0;
    bool   imu_ready   = false;
    double imu_ts      = 0.0;

    double odom_x      = 0.0;
    double odom_y      = 0.0;
    double odom_yaw    = 0.0;
    double odom_vx     = 0.0;
    double odom_vy     = 0.0;
    double odom_wz     = 0.0;
    bool   odom_ready  = false;
    double odom_ts     = 0.0;

    double kf1_x       = 0.0;
    double kf1_y       = 0.0;
    bool   kf1_ready   = false;
    bool   kf1_active  = true;
    double kf1_ts      = 0.0;

    double kf2_x       = 0.0;
    double kf2_y       = 0.0;
    bool   kf2_ready   = false;
    bool   kf2_active  = true;
    double kf2_ts      = 0.0;

    double ann_x       = 0.0;
    double ann_y       = 0.0;
    bool   ann_ready   = false;
    double ann_ts      = 0.0;

    static constexpr double ALPHA1_FIXED = 0.5;
    static constexpr double ALPHA2_FIXED = 0.5;
    double cf_x        = 0.0;
    double cf_y        = 0.0;

    double slip_error      = 0.0;
    FuzzyMembership fuzzy_mf;
    double alpha1_adaptive = 0.5;
    double alpha2_adaptive = 0.5;

    double final_x     = 0.0;
    double final_y     = 0.0;
    bool   gps_branch  = true;
    bool   indoor_detected = false;
};

class StateHub : public rclcpp::Node
{
public:
    std::shared_ptr<RobotState> state;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         bt_fused_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fuzzy_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            status_pub;

    explicit StateHub()
    : Node("bt_brain"),
      state(std::make_shared<RobotState>())
    {
        auto qos = rclcpp::SensorDataQoS();

        sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", qos,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                bool fix = (msg->status.status >=
                            sensor_msgs::msg::NavSatStatus::STATUS_FIX);
                state->gps_available    = fix;
                if (fix) state->has_ever_had_gps = true;
            });

        sub_gps_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/gps", rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                state->last_known_gps_x = msg->pose.pose.position.x;
                state->last_known_gps_y = msg->pose.pose.position.y;
                state->gps_odom_ready   = true;
                state->gps_odom_timestamp =
                    rclcpp::Time(msg->header.stamp).seconds();
            });

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", qos,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                state->imu_ax    = msg->linear_acceleration.x;
                state->imu_ay    = msg->linear_acceleration.y;
                state->imu_wz    = msg->angular_velocity.z;
                state->imu_ready = true;
                state->imu_ts    = rclcpp::Time(msg->header.stamp).seconds();
            });

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                {
                    std::lock_guard<std::mutex> lk(state->mtx);
                    state->odom_x   = msg->pose.pose.position.x;
                    state->odom_y   = msg->pose.pose.position.y;
                    state->odom_vx  = msg->twist.twist.linear.x;
                    state->odom_vy  = msg->twist.twist.linear.y;
                    state->odom_wz  = msg->twist.twist.angular.z;
                    const auto& q = msg->pose.pose.orientation;
                    double siny   = 2.0 * (q.w * q.z + q.x * q.y);
                    double cosy   = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                    state->odom_yaw  = std::atan2(siny, cosy);
                    state->odom_ready = true;
                    state->odom_ts    = rclcpp::Time(msg->header.stamp).seconds();
                }

                // ekf_local silently fails to publish odom→base_footprint with
                // use_sim_time=true in Humble. Without this edge the TF chain
                // map→odom→base_footprint is broken and navsat_transform never
                // initialises GPS→ENU, so /odometry/gps is never published.
                if (tf_broadcaster_) {
                    geometry_msgs::msg::TransformStamped odom_tf;
                    odom_tf.header.stamp    = msg->header.stamp;
                    odom_tf.header.frame_id = "odom";
                    odom_tf.child_frame_id  = "base_footprint";
                    odom_tf.transform.translation.x = msg->pose.pose.position.x;
                    odom_tf.transform.translation.y = msg->pose.pose.position.y;
                    odom_tf.transform.translation.z = msg->pose.pose.position.z;
                    odom_tf.transform.rotation      = msg->pose.pose.orientation;
                    tf_broadcaster_->sendTransform(odom_tf);
                }
            });

        sub_kf1_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/global", rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                state->kf1_x    = msg->pose.pose.position.x;
                state->kf1_y    = msg->pose.pose.position.y;
                state->kf1_ready = true;
                state->kf1_ts   = rclcpp::Time(msg->header.stamp).seconds();
            });

        sub_kf2_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/global2", rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                state->kf2_x    = msg->pose.pose.position.x;
                state->kf2_y    = msg->pose.pose.position.y;
                state->kf2_ready = true;
                state->kf2_ts   = rclcpp::Time(msg->header.stamp).seconds();
            });

        sub_ann_ = create_subscription<geometry_msgs::msg::Point>(
            "/ann/trajectory", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                state->ann_x    = msg->x;
                state->ann_y    = msg->y;
                state->ann_ready = true;
                state->ann_ts   = now().seconds();

                visualization_msgs::msg::Marker m;
                m.header.stamp    = now();
                m.header.frame_id = "map";
                m.ns              = "ann_pose";
                m.id              = (ann_marker_seq_++) % 10000;
                m.type            = visualization_msgs::msg::Marker::SPHERE;
                m.action          = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = msg->x;
                m.pose.position.y = msg->y;
                m.pose.position.z = 0.0;
                m.pose.orientation.w = 1.0;
                m.scale.x = m.scale.y = m.scale.z = 0.18;
                m.color.r = 1.0f; m.color.g = 0.0f;
                m.color.b = 1.0f; m.color.a = 0.85f;
                m.lifetime = rclcpp::Duration::from_seconds(0.5);
                ann_marker_pub_->publish(m);
            });

        bt_fused_pub = create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/bt_fused", 10);

        fuzzy_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/bt/fuzzy_weights", 10);

        status_pub = create_publisher<std_msgs::msg::String>(
            "/bt/status", 10);

        ann_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/ann/viz_marker", 10);

        path_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/bt/path_viz", 10);

        // libgazebo_ros_joint_state_publisher.so is not shipped with
        // ros-humble-gazebo-plugins on this installation. Without /joint_states,
        // robot_state_publisher cannot emit wheel TF edges (continuous joints
        // require live joint data). bt_brain owns /odom so publishes here.
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10));

        joint_state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            [this]() {
                sensor_msgs::msg::JointState js;
                js.header.stamp = now();
                js.name = {"wheel_left_joint", "wheel_right_joint"};

                double v_x, dt_s;
                {
                    std::lock_guard<std::mutex> lk(state->mtx);
                    v_x = state->odom_vx;
                }
                dt_s          = 33.0e-3;
                double dtheta = (v_x / WHEEL_RADIUS_M) * dt_s;
                wheel_left_angle_  += dtheta;
                wheel_right_angle_ -= dtheta;

                js.position = {wheel_left_angle_, wheel_right_angle_};
                js.velocity = {v_x / WHEEL_RADIUS_M, -v_x / WHEEL_RADIUS_M};
                js.effort   = {};

                joint_state_pub_->publish(js);
            });

        RCLCPP_INFO(get_logger(),
            "=== BT Brain (StateHub) online. Waiting for sensors... ===");

        // Bootstrap identity map→odom so the 30 Hz broadcaster starts immediately.
        // navsat_transform needs map→base_footprint to initialise GPS→ENU.
        // Without this bootstrap, /odometry/gps never publishes and GPS branch
        // never activates. The identity is correct at t=0 (robot spawns at
        // map origin == odom origin). update_map_to_odom_tf() replaces it once
        // the first real bt_fused estimate is computed.
        {
            geometry_msgs::msg::TransformStamped bootstrap;
            bootstrap.header.frame_id    = "map";
            bootstrap.child_frame_id     = "odom";
            bootstrap.transform.rotation.w = 1.0;
            std::lock_guard<std::mutex> lk(state->mtx);
            state->map_to_odom_cached = bootstrap;
            state->map_to_odom_valid  = true;
        }

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_       = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_, this, false);

        // Re-stamp the cached transform every 33 ms to prevent TF buffer expiry
        // between BT ticks (BT ticks at 10 Hz; TF consumers expect a fresher edge).
        tf_broadcast_timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
            [this]() {
                geometry_msgs::msg::TransformStamped ts;
                bool valid;
                {
                    std::lock_guard<std::mutex> lk(state->mtx);
                    ts    = state->map_to_odom_cached;
                    valid = state->map_to_odom_valid;
                }
                if (valid) {
                    ts.header.stamp = now();
                    tf_broadcaster_->sendTransform(ts);
                }

                {
                    std::lock_guard<std::mutex> lk(state->mtx);
                    if (state->odom_ready) {
                        geometry_msgs::msg::TransformStamped odom_tf;
                        odom_tf.header.stamp    = now();
                        odom_tf.header.frame_id = "odom";
                        odom_tf.child_frame_id  = "base_footprint";
                        odom_tf.transform.translation.x = state->odom_x;
                        odom_tf.transform.translation.y = state->odom_y;
                        odom_tf.transform.translation.z = 0.0;
                        odom_tf.transform.rotation.z =
                            std::sin(state->odom_yaw / 2.0);
                        odom_tf.transform.rotation.w =
                            std::cos(state->odom_yaw / 2.0);
                        tf_broadcaster_->sendTransform(odom_tf);
                    }
                }
            });
    }

    // T_map_to_odom = T_map_to_base_fp × inv(T_odom_to_base_fp)
    void update_map_to_odom_tf(double bt_x, double bt_y, double yaw)
    {
        geometry_msgs::msg::TransformStamped odom_to_base;
        try {
            odom_to_base = tf_buffer_->lookupTransform(
                "odom", "base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "[BT TF] Cannot lookup odom→base_footprint: %s", ex.what());
            return;
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tf2::Transform T_map_base;
        T_map_base.setOrigin(tf2::Vector3(bt_x, bt_y, 0.0));
        T_map_base.setRotation(q);

        tf2::Transform T_odom_base;
        tf2::fromMsg(odom_to_base.transform, T_odom_base);

        tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = now();
        ts.header.frame_id = "map";
        ts.child_frame_id  = "odom";
        ts.transform       = tf2::toMsg(T_map_odom);

        std::lock_guard<std::mutex> lk(state->mtx);
        state->map_to_odom_cached = ts;
        state->map_to_odom_valid  = true;
        state->bt_fused_valid     = true;
    }

    void append_path_point(double x, double y)
    {
        if (path_points_.size() >= PATH_MAX_POINTS) {
            const std::size_t drop = PATH_MAX_POINTS / 10;
            path_points_.erase(path_points_.begin(),
                               path_points_.begin() + static_cast<long>(drop));
        }
        geometry_msgs::msg::Point p;
        p.x = x; p.y = y; p.z = 0.02;
        path_points_.push_back(p);

        visualization_msgs::msg::Marker m;
        m.header.stamp       = now();
        m.header.frame_id    = "map";
        m.ns                 = "bt_path";
        m.id                 = 0;
        m.type               = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action             = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x            = 0.05;
        m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.95f;
        m.lifetime = rclcpp::Duration(0, 0);
        m.points   = path_points_;
        path_marker_pub_->publish(m);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  sub_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_gps_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_kf1_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_kf2_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr    sub_ann_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ann_marker_pub_;
    int ann_marker_seq_{0};

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
    std::vector<geometry_msgs::msg::Point>                        path_points_;
    static constexpr std::size_t PATH_MAX_POINTS = 50000;

    static constexpr double WHEEL_RADIUS_M = 0.033;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    double wheel_left_angle_  = 0.0;
    double wheel_right_angle_ = 0.0;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    rclcpp::TimerBase::SharedPtr                   tf_broadcast_timer_;
};

static void publish_bt_fused(StateHub& hub, double x, double y)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = hub.now();
    msg.header.frame_id = "map";
    msg.child_frame_id  = "base_footprint";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    double yaw;
    {
        std::lock_guard<std::mutex> lk(hub.state->mtx);
        yaw = hub.state->odom_yaw;
        msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
        msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    }
    hub.bt_fused_pub->publish(msg);

    {
        std::lock_guard<std::mutex> lk(hub.state->mtx);
        hub.state->final_x = x;
        hub.state->final_y = y;
    }

    hub.update_map_to_odom_tf(x, y, yaw);
    hub.append_path_point(x, y);
}

static double now_sec(StateHub& hub)
{
    return hub.now().seconds();
}

class CheckSensorsReady : public ConditionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    CheckSensorsReady(const std::string& name, std::shared_ptr<StateHub> hub)
    : ConditionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        bool ok = hub_->state->imu_ready && hub_->state->odom_ready;
        if (!ok) {
            RCLCPP_WARN_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 3000,
                "[BT] Waiting for sensors — IMU:%d  Odom:%d",
                hub_->state->imu_ready, hub_->state->odom_ready);
        }
        return ok ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class CheckGpsSignal : public ConditionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double GPS_TIMEOUT_S = 2.0;
public:
    CheckGpsSignal(const std::string& name, std::shared_ptr<StateHub> hub)
    : ConditionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;

        bool fix_ok   = s.gps_available && s.has_ever_had_gps;
        bool odom_ok  = s.gps_odom_ready;
        bool fresh    = (now_sec(*hub_) - s.gps_odom_timestamp) < GPS_TIMEOUT_S;
        bool outdoor  = !s.indoor_detected;

        bool gps_good = fix_ok && odom_ok && fresh && outdoor;

        if (gps_good) {
            RCLCPP_INFO_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 5000,
                "[BT][GPS] ✓ OUTDOOR – GPS active.");
        } else {
            RCLCPP_WARN_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 3000,
                "[BT][GPS] ✗ GPS DISABLED – "
                "fix=%d odom=%d fresh=%d indoor=%d",
                fix_ok, odom_ok, fresh, !outdoor);
        }
        return gps_good ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class AlignSensorFrames : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    AlignSensorFrames(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        bool aligned = hub_->state->gps_odom_ready;
        if (!aligned) {
            RCLCPP_WARN_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 3000,
                "[BT] Frame alignment pending – /odometry/gps not yet live.");
            return NodeStatus::FAILURE;
        }
        return NodeStatus::SUCCESS;
    }
};

class EnsureKF1Active : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double KF_TIMEOUT_S = 1.0;
public:
    EnsureKF1Active(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        bool alive = s.kf1_ready &&
                     (now_sec(*hub_) - s.kf1_ts) < KF_TIMEOUT_S;
        s.kf1_active = alive;
        if (!alive) {
            RCLCPP_WARN_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 3000,
                "[BT] KF1 not responsive on /odometry/global.");
        }
        return alive ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class EnsureKF2Active : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double KF_TIMEOUT_S = 1.0;
public:
    EnsureKF2Active(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        bool alive = s.kf2_ready &&
                     (now_sec(*hub_) - s.kf2_ts) < KF_TIMEOUT_S;
        s.kf2_active = alive;
        if (!alive) {
            RCLCPP_WARN_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 3000,
                "[BT] KF2 not responsive on /odometry/global2.");
        }
        return alive ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class RunComplementaryFilter_GPS : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    RunComplementaryFilter_GPS(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        if (!s.kf1_active || !s.kf2_active) return NodeStatus::FAILURE;

        s.cf_x = RobotState::ALPHA1_FIXED * s.kf1_x +
                 RobotState::ALPHA2_FIXED * s.kf2_x;
        s.cf_y = RobotState::ALPHA1_FIXED * s.kf1_y +
                 RobotState::ALPHA2_FIXED * s.kf2_y;
        return NodeStatus::SUCCESS;
    }
};

class PublishFusedPosition_GPS : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    PublishFusedPosition_GPS(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        double x, y;
        {
            std::lock_guard<std::mutex> lk(hub_->state->mtx);
            x = hub_->state->cf_x;
            y = hub_->state->cf_y;
            hub_->state->gps_branch = true;
        }

        publish_bt_fused(*hub_, x, y);

        auto msg = std_msgs::msg::String{};
        msg.data = "GPS_BRANCH | x=" + std::to_string(x) +
                   " y=" + std::to_string(y);
        hub_->status_pub->publish(msg);

        return NodeStatus::SUCCESS;
    }
};

class CollectAndTrainNN : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    CollectAndTrainNN(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;

        if (s.ann_ready) {
            RCLCPP_INFO_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 10000,
                "[BT] ✓ ANN publishing: (%.3f, %.3f)", s.ann_x, s.ann_y);
        }
        return NodeStatus::SUCCESS;
    }
};

class HaltKF1_FreezeKF2GPS : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    HaltKF1_FreezeKF2GPS(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;

        if (s.kf1_active) {
            RCLCPP_WARN(hub_->get_logger(),
                "[BT] ⚠  GPS LOST.  Halting KF1.  "
                "Last known GPS ENU: (%.3f, %.3f).",
                s.last_known_gps_x, s.last_known_gps_y);
        }
        s.kf1_active  = false;
        s.gps_branch  = false;

        return NodeStatus::SUCCESS;
    }
};

class CalculateSlipError : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    CalculateSlipError(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        s.slip_error = std::abs(s.imu_wz - s.odom_wz);
        return NodeStatus::SUCCESS;
    }
};

class FuzzifySlipError : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double L_CENTER = 0.00, L_RIGHT  = 0.10;
    static constexpr double M_LEFT   = 0.05, M_CENTER = 0.15, M_RIGHT = 0.25;
    static constexpr double H_LEFT   = 0.20, H_CENTER = 0.30;
public:
    FuzzifySlipError(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        double e = s.slip_error;

        s.fuzzy_mf.low    = shoulder_left_mf(e,  L_CENTER, L_RIGHT);
        s.fuzzy_mf.medium = triangular_mf(e,     M_LEFT, M_CENTER, M_RIGHT);
        s.fuzzy_mf.high   = shoulder_right_mf(e, H_LEFT, H_CENTER);
        return NodeStatus::SUCCESS;
    }
};

class ActivateFuzzyRules : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double C1_LOW = 0.80, C1_MEDIUM = 0.50, C1_HIGH = 0.20;
    static constexpr double C2_LOW = 0.20, C2_MEDIUM = 0.50, C2_HIGH = 0.80;
public:
    ActivateFuzzyRules(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        auto& mf = s.fuzzy_mf;

        double num1 = mf.low * C1_LOW + mf.medium * C1_MEDIUM + mf.high * C1_HIGH;
        double num2 = mf.low * C2_LOW + mf.medium * C2_MEDIUM + mf.high * C2_HIGH;
        double den  = mf.low + mf.medium + mf.high;

        if (den < 1e-9) {
            s.alpha1_adaptive = 0.5;
            s.alpha2_adaptive = 0.5;
        } else {
            s.alpha1_adaptive = num1 / den;
            s.alpha2_adaptive = num2 / den;
        }
        return NodeStatus::SUCCESS;
    }
};

class DefuzzifyWeights : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    DefuzzifyWeights(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        double a1, a2, slip;
        {
            std::lock_guard<std::mutex> lk(hub_->state->mtx);
            auto& s = *hub_->state;
            a1   = std::max(0.0, std::min(1.0, s.alpha1_adaptive));
            a2   = 1.0 - a1;
            s.alpha1_adaptive = a1;
            s.alpha2_adaptive = a2;
            slip = s.slip_error;
        }

        auto msg = std_msgs::msg::Float32MultiArray{};
        msg.data = {static_cast<float>(a1),
                    static_cast<float>(a2),
                    static_cast<float>(slip)};
        hub_->fuzzy_pub->publish(msg);

        RCLCPP_INFO_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 2000,
            "[BT][DEFUZZ]  slip=%.4f rad/s  →  α1(NN)=%.3f  α2(KF2)=%.3f",
            slip, a1, a2);
        return NodeStatus::SUCCESS;
    }
};

class FuseNN_KF2_Adaptive : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double NN_TIMEOUT_S  = 1.0;
    static constexpr double KF2_TIMEOUT_S = 1.0;
public:
    FuseNN_KF2_Adaptive(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    NodeStatus tick() override
    {
        double x_final, y_final;
        double a1, a2;
        std::string mode;

        {
            std::lock_guard<std::mutex> lk(hub_->state->mtx);
            auto& s = *hub_->state;
            double t = now_sec(*hub_);

            bool kf2_live = s.kf2_ready && (t - s.kf2_ts) < KF2_TIMEOUT_S;
            bool ann_live = s.ann_ready && (t - s.ann_ts) < NN_TIMEOUT_S;

            if (ann_live && kf2_live) {
                a1 = s.alpha1_adaptive;
                a2 = s.alpha2_adaptive;
                x_final = a1 * s.ann_x + a2 * s.kf2_x;
                y_final = a1 * s.ann_y + a2 * s.kf2_y;
                mode = "NN+KF2_ADAPTIVE";
            } else if (kf2_live) {
                a1 = 0.0; a2 = 1.0;
                x_final = s.kf2_x;
                y_final = s.kf2_y;
                mode = "KF2_ONLY";
            } else {
                a1 = 0.0; a2 = 0.0;
                x_final = s.last_known_gps_x;
                y_final = s.last_known_gps_y;
                mode = "LAST_GPS_ANCHOR";
                RCLCPP_ERROR_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 5000,
                    "[BT] ⛔ KF2 AND NN both offline! Holding GPS anchor (%.3f, %.3f).",
                    x_final, y_final);
            }
        }

        publish_bt_fused(*hub_, x_final, y_final);

        auto sm = std_msgs::msg::String{};
        sm.data = "NO_GPS_BRANCH | " + mode +
                  " | α1=" + std::to_string(a1) +
                  " α2=" + std::to_string(a2) +
                  " | x=" + std::to_string(x_final) +
                  " y=" + std::to_string(y_final);
        hub_->status_pub->publish(sm);

        RCLCPP_INFO_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 2000,
            "[BT][FUSE_NoGPS] mode=%-20s  α=[%.3f,%.3f]  → final=(%.3f, %.3f)",
            mode.c_str(), a1, a2, x_final, y_final);

        return NodeStatus::SUCCESS;
    }
};

static void register_all_nodes(BehaviorTreeFactory& factory,
                                std::shared_ptr<StateHub> hub)
{
    factory.registerBuilder<CheckSensorsReady>("CheckSensorsReady",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<CheckSensorsReady>(n, hub);
        });

    factory.registerBuilder<CheckGpsSignal>("CheckGpsSignal",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<CheckGpsSignal>(n, hub);
        });

    factory.registerBuilder<AlignSensorFrames>("AlignSensorFrames",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<AlignSensorFrames>(n, hub);
        });

    factory.registerBuilder<EnsureKF1Active>("EnsureKF1Active",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<EnsureKF1Active>(n, hub);
        });

    factory.registerBuilder<EnsureKF2Active>("EnsureKF2Active",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<EnsureKF2Active>(n, hub);
        });

    factory.registerBuilder<RunComplementaryFilter_GPS>("RunComplementaryFilter_GPS",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<RunComplementaryFilter_GPS>(n, hub);
        });

    factory.registerBuilder<PublishFusedPosition_GPS>("PublishFusedPosition_GPS",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<PublishFusedPosition_GPS>(n, hub);
        });

    factory.registerBuilder<CollectAndTrainNN>("CollectAndTrainNN",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<CollectAndTrainNN>(n, hub);
        });

    factory.registerBuilder<HaltKF1_FreezeKF2GPS>("HaltKF1_FreezeKF2GPS",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<HaltKF1_FreezeKF2GPS>(n, hub);
        });

    factory.registerBuilder<CalculateSlipError>("CalculateSlipError",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<CalculateSlipError>(n, hub);
        });

    factory.registerBuilder<FuzzifySlipError>("FuzzifySlipError",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<FuzzifySlipError>(n, hub);
        });

    factory.registerBuilder<ActivateFuzzyRules>("ActivateFuzzyRules",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<ActivateFuzzyRules>(n, hub);
        });

    factory.registerBuilder<DefuzzifyWeights>("DefuzzifyWeights",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<DefuzzifyWeights>(n, hub);
        });

    factory.registerBuilder<FuseNN_KF2_Adaptive>("FuseNN_KF2_Adaptive",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<FuseNN_KF2_Adaptive>(n, hub);
        });
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto hub = std::make_shared<StateHub>();

    auto indoor_flag = std::make_shared<std::atomic<bool>>(false);

    IndoorDetector::Config id_cfg;
    // GPS multipath at the building south wall requires longer debounce:
    // 4 ticks = 0.8 s bad GPS before declaring INDOOR (was 2 = 0.4 s, too sensitive).
    id_cfg.debounce_in_ticks  = 4;
    id_cfg.debounce_out_ticks = 8;
    // GPS noise (σ=0.316 m) produces jumps up to 0.9 m between consecutive 5 Hz
    // messages. Old 0.5 m threshold scored +1 on almost every outdoor message.
    id_cfg.jump_minor_m    = 1.0;
    id_cfg.jump_moderate_m = 2.5;
    id_cfg.jump_severe_m   = 6.0;
    // /odometry/global vs /odom legitimately diverges 1.0–1.5 m near the building.
    id_cfg.pos_div_warn_m = 1.5;
    id_cfg.pos_div_bad_m  = 4.0;

    auto indoor_detector = std::make_shared<IndoorDetector>(
        hub.get(), indoor_flag, id_cfg);

    auto bridge_timer = hub->create_wall_timer(
        std::chrono::milliseconds(100),
        [&hub, &indoor_flag]() {
            std::lock_guard<std::mutex> lk(hub->state->mtx);
            hub->state->indoor_detected = indoor_flag->load();
        });

    std::thread ros_thread([hub]() {
        rclcpp::spin(hub);
    });
    ros_thread.detach();

    BehaviorTreeFactory factory;
    register_all_nodes(factory, hub);

    std::string xml_path =
        ament_index_cpp::get_package_share_directory("bt_orchestrator_pkg") +
        "/bt_xml/localization_tree.xml";

    auto tree = factory.createTreeFromFile(xml_path);

    BT::Groot2Publisher groot2_publisher(tree, 1667);

    RCLCPP_INFO(hub->get_logger(),
        "╔══════════════════════════════════════════════════╗\n"
        "║  BT Brain started — GPS/INS/Odometer Fusion      ║\n"
        "║  Tick: 10 Hz  |  Groot2: port 1667               ║\n"
        "║  Output: /odometry/bt_fused                      ║\n"
        "╚══════════════════════════════════════════════════╝");

    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
        tree.tickOnce();
        rate.sleep();
    }

    RCLCPP_INFO(hub->get_logger(), "BT Brain shutting down.");
    rclcpp::shutdown();
    return 0;
}
