/**
 * ============================================================================
 *  bt_brain_full.cpp  –  Full ROS2 + BehaviorTree Orchestrator
 * ============================================================================
 *
 *  Paper: "Information Fusion of GPS, INS and Odometer Sensors"
 *  Robot: TurtleBot3 Burger (ROS2 Humble / Gazebo)
 *
 *  ARCHITECTURE
 *  ─────────────
 *  • StateHub  : rclcpp::Node that subscribes to all sensor topics and keeps
 *                a mutex-protected RobotState struct.  Runs in a background
 *                thread so ROS2 callbacks never block the BT tick.
 *
 *  • BT Nodes  : Each node receives a shared_ptr<StateHub>.  They read from
 *                or write to StateHub::state under the mutex, then return
 *                SUCCESS / FAILURE in the same tick (synchronous nodes).
 *
 *  • Main loop : rclcpp::spin(hub) in background thread.
 *                BT ticks at 10 Hz in the main thread.
 *                ZMQ publisher lets Groot2 visualize the live tree.
 *
 *  SUBSCRIBED TOPICS
 *  ──────────────────
 *    /gps/fix              sensor_msgs/NavSatFix   GPS availability
 *    /imu/data             sensor_msgs/Imu         acceleration + angular vel
 *    /odom                 nav_msgs/Odometry       wheel odometry
 *    /odometry/gps         nav_msgs/Odometry       GPS→ENU (navsat_transform)
 *    /odometry/global      nav_msgs/Odometry       KF1 output (GPS+IMU EKF)
 *    /odometry/global2     nav_msgs/Odometry       KF2 output (GPS+Odom EKF)
 *    /ann/trajectory       geometry_msgs/Point     ANN position estimate
 *
 *  PUBLISHED TOPICS
 *  ─────────────────
 *    /odometry/bt_fused    nav_msgs/Odometry           BT final position estimate
 *    /bt/fuzzy_weights     std_msgs/Float32MultiArray  [α1, α2, slip_error]
 *    /bt/status            std_msgs/String             active branch label
 *    /ann/viz_marker       visualization_msgs/Marker   ANN point (map frame, RViz)
 *    /bt/path_viz          visualization_msgs/Marker   Yellow LINE_STRIP of full
 *                                                      bt_fused trajectory (map frame)
 *    /joint_states         sensor_msgs/JointState      wheel angles → RSP TF edges
 * ============================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <mutex>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>

// ── ROS2 ─────────────────────────────────────────────────────────────────────
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
#include "sensor_msgs/msg/joint_state.hpp"   // wheel TF fix — published at 30 Hz to feed RSP

// ── BehaviorTree CPP v4 ───────────────────────────────────────────────────────
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

// ── Package path resolution ────────────────────────────────────────────────────
#include "ament_index_cpp/get_package_share_directory.hpp"


#include "bt_orchestrator_pkg/indoor_detector.hpp"

// ── TF2 ──────────────────────────────────────────────────────────────────────
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace BT;
using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
//  FUZZY LOGIC HELPERS
// ─────────────────────────────────────────────────────────────────────────────

struct FuzzyMembership {
    double low    = 0.0;
    double medium = 0.0;
    double high   = 0.0;
};

/**
 * Triangular membership function.
 *   left   = value where μ rises from 0
 *   center = value where μ = 1
 *   right  = value where μ falls back to 0
 */
static double triangular_mf(double x, double left, double center, double right)
{
    if (x <= left || x >= right) return 0.0;
    if (x <= center) return (x - left) / (center - left);
    return (right - x) / (right - center);
}

/**
 * Left-shoulder membership function (Low set): falls from 1 to 0.
 *   μ = 1           for x ≤ center
 *   μ = (right-x)/(right-center)  for center < x < right
 *   μ = 0           for x ≥ right
 */
static double shoulder_left_mf(double x, double center, double right)
{
    if (x <= center) return 1.0;
    if (x >= right)  return 0.0;
    return (right - x) / (right - center);
}

/**
 * Right-shoulder membership function (High set): rises from 0 to 1.
 *   μ = 0           for x ≤ left
 *   μ = (x-left)/(center-left) for left < x < center
 *   μ = 1           for x ≥ center
 */
static double shoulder_right_mf(double x, double left, double center)
{
    if (x <= left)   return 0.0;
    if (x >= center) return 1.0;
    return (x - left) / (center - left);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SHARED ROBOT STATE
// ─────────────────────────────────────────────────────────────────────────────

struct RobotState {
    mutable std::mutex mtx;

    // ── GPS ──────────────────────────────────────────────────────────────────
    bool   gps_available      = false;
    double last_known_gps_x   = 0.0;
    double last_known_gps_y   = 0.0;
    bool   has_ever_had_gps   = false;
    bool   gps_odom_ready     = false;   // navsat_transform is publishing
    double gps_odom_timestamp = 0.0;     // seconds since epoch

    // ── BT-driven TF: map → odom ───────────────────────────────────────────
    bool bt_fused_valid                              = false;
    bool map_to_odom_valid                           = false;
    geometry_msgs::msg::TransformStamped map_to_odom_cached;
    
    // ── IMU ──────────────────────────────────────────────────────────────────
    double imu_ax      = 0.0;
    double imu_ay      = 0.0;
    double imu_wz      = 0.0;   // angular velocity Z [rad/s]
    bool   imu_ready   = false;
    double imu_ts      = 0.0;

    // ── Wheel Odometry ───────────────────────────────────────────────────────
    double odom_x      = 0.0;
    double odom_y      = 0.0;
    double odom_yaw    = 0.0;
    double odom_vx     = 0.0;
    double odom_vy     = 0.0;
    double odom_wz     = 0.0;   // angular velocity Z from odom [rad/s]
    bool   odom_ready  = false;
    double odom_ts     = 0.0;

    // ── KF1: GPS+IMU EKF  (/odometry/global) ─────────────────────────────────
    double kf1_x       = 0.0;
    double kf1_y       = 0.0;
    bool   kf1_ready   = false;
    bool   kf1_active  = true;   // BT-controlled (false = GPS lost, ignore KF1)
    double kf1_ts      = 0.0;

    // ── KF2: GPS+Odom EKF  (/odometry/global2) ───────────────────────────────
    double kf2_x       = 0.0;
    double kf2_y       = 0.0;
    bool   kf2_ready   = false;
    bool   kf2_active  = true;
    double kf2_ts      = 0.0;

    // ── ANN output  (/ann/trajectory) ────────────────────────────────────────
    double ann_x       = 0.0;
    double ann_y       = 0.0;
    bool   ann_ready   = false;   // true once ANN has published at least once
    double ann_ts      = 0.0;

    // ── Complementary Filter (GPS branch, fixed weights) ─────────────────────
    static constexpr double ALPHA1_FIXED = 0.5;  // KF1 weight in GPS mode
    static constexpr double ALPHA2_FIXED = 0.5;  // KF2 weight in GPS mode
    double cf_x        = 0.0;
    double cf_y        = 0.0;

    // ── Fuzzy Logic (No-GPS branch) ───────────────────────────────────────────
    double slip_error      = 0.0;
    FuzzyMembership fuzzy_mf;           // μ_Low, μ_Med, μ_High
    double alpha1_adaptive = 0.5;       // NN weight (from defuzzification)
    double alpha2_adaptive = 0.5;       // KF2 weight (from defuzzification)

    // ── BT Final Output ──────────────────────────────────────────────────────
    double final_x     = 0.0;
    double final_y     = 0.0;
    bool   gps_branch  = true;          // diagnostic: which branch is active
    bool   indoor_detected = false;   // set by IndoorDetector; overrides GPS
};

// ─────────────────────────────────────────────────────────────────────────────
//  STATE HUB  –  ROS2 Node collecting all sensor data
// ─────────────────────────────────────────────────────────────────────────────

class StateHub : public rclcpp::Node
{
public:
    std::shared_ptr<RobotState> state;

    // ── Publishers (written by BT action nodes) ──────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         bt_fused_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fuzzy_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            status_pub;

    explicit StateHub()
    : Node("bt_brain"),
      state(std::make_shared<RobotState>())
    {
        auto qos = rclcpp::SensorDataQoS();   // best-effort, volatile

        // ── Subscribers ──────────────────────────────────────────────────────

        // /gps/fix  → GPS availability only (lat/lon not used directly)
        sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", qos,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                bool fix = (msg->status.status >=
                            sensor_msgs::msg::NavSatStatus::STATUS_FIX);
                state->gps_available    = fix;
                if (fix) state->has_ever_had_gps = true;
            });

        // /odometry/gps  → GPS position in ENU frame (navsat_transform output)
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

        // /imu/data  → linear acceleration + angular velocity
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

        // /odom  → wheel odometry (position + velocity)
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // ── State update (under mutex) ────────────────────────────
                {
                    std::lock_guard<std::mutex> lk(state->mtx);
                    state->odom_x   = msg->pose.pose.position.x;
                    state->odom_y   = msg->pose.pose.position.y;
                    state->odom_vx  = msg->twist.twist.linear.x;
                    state->odom_vy  = msg->twist.twist.linear.y;
                    state->odom_wz  = msg->twist.twist.angular.z;
                    // Extract yaw from quaternion
                    const auto& q = msg->pose.pose.orientation;
                    double siny   = 2.0 * (q.w * q.z + q.x * q.y);
                    double cosy   = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                    state->odom_yaw  = std::atan2(siny, cosy);
                    state->odom_ready = true;
                    state->odom_ts    = rclcpp::Time(msg->header.stamp).seconds();
                }

                // ── Publish odom→base_footprint TF (outside mutex) ───────
                // ROOT-CAUSE FIX: ekf_local silently fails to publish
                // odom→base_footprint with use_sim_time:true in Humble
                // (zero log output from all ekf_node processes confirms this).
                // Without this edge the entire TF chain is broken:
                //   bt_brain bootstrap  →  map→odom  (identity, 30 Hz)
                //   THIS CALLBACK       →  odom→base_footprint  (~50 Hz)
                //   composed            →  map→base_footprint
                //   navsat_transform can now find map→base_footprint and
                //   initialize GPS→ENU → /odometry/gps starts publishing
                //   → global EKFs receive GPS → bt_fused correct
                //   → update_map_to_odom_tf() replaces identity with real TF.
                //
                // tf_broadcaster_ is initialised after sub_odom_ in the ctor
                // but this lambda only fires in the spin thread (post-ctor).
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

        // /odometry/global  → KF1: GPS+IMU EKF global estimate
        sub_kf1_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/global", rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                state->kf1_x    = msg->pose.pose.position.x;
                state->kf1_y    = msg->pose.pose.position.y;
                state->kf1_ready = true;
                state->kf1_ts   = rclcpp::Time(msg->header.stamp).seconds();
            });

        // /odometry/global2  → KF2: GPS+Odom EKF global estimate
        sub_kf2_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/global2", rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                state->kf2_x    = msg->pose.pose.position.x;
                state->kf2_y    = msg->pose.pose.position.y;
                state->kf2_ready = true;
                state->kf2_ts   = rclcpp::Time(msg->header.stamp).seconds();
            });

        // /ann/trajectory  → ANN position estimate (robot_control_brain node)
        sub_ann_ = create_subscription<geometry_msgs::msg::Point>(
            "/ann/trajectory", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state->mtx);
                state->ann_x    = msg->x;
                state->ann_y    = msg->y;
                state->ann_ready = true;
                state->ann_ts   = now().seconds();

                // ── Publish RViz Marker (map frame, magenta sphere) ───────────
                // geometry_msgs/Point has no header/frame_id — inject map frame.
                // Rolling ID mod 10000 produces a short trail in RViz.
                // Lifetime 0.5 s auto-expires stale markers between ticks.
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

        // ── Publishers ───────────────────────────────────────────────────────
        bt_fused_pub = create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/bt_fused", 10);

        fuzzy_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/bt/fuzzy_weights", 10);

        status_pub = create_publisher<std_msgs::msg::String>(
            "/bt/status", 10);

        // ANN trajectory → RViz Marker (magenta sphere in map frame).
        // Consumed by the /ann/viz_marker display in rviz/system.rviz.
        ann_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/ann/viz_marker", 10);

        // Yellow LINE_STRIP accumulating all bt_fused positions (map frame).
        // append_path_point() is called by publish_bt_fused() on every BT tick.
        path_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/bt/path_viz", 10);

        // ── Wheel joint states → robot_state_publisher TF edges ──────────────
        // WHY: libgazebo_ros_joint_state_publisher.so is NOT shipped with
        //      ros-humble-gazebo-plugins on this installation; the xacro plugin
        //      block loads nothing. Without /joint_states, robot_state_publisher
        //      cannot emit wheel_left_link / wheel_right_link TF edges (only
        //      continuous joints require live joint data — fixed joints are
        //      unconditional). bt_brain already owns the /odom subscription and
        //      all relevant sensor data, so it is the natural place to publish
        //      /joint_states at 30 Hz with no additional node or package needed.
        //
        // Wheel angle from odometry:
        //   θ_wheel = odom_arc / r_wheel
        //   arc per tick ≈ odom_vx × dt  (dt = 1/30 s)
        //   r_wheel = 0.033 m  (TurtleBot3 Burger)
        //
        // This gives realistic visual wheel spin in RViz without any hardware
        // encoder topic — purely cosmetic, has no effect on localization.
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10));

        joint_state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),   // 30 Hz
            [this]() {
                sensor_msgs::msg::JointState js;
                js.header.stamp = now();

                // Joint names must match URDF exactly.
                // URDF uses ${namespace}wheel_left_joint; default namespace=""
                // so names are simply "wheel_left_joint" / "wheel_right_joint".
                js.name = {"wheel_left_joint", "wheel_right_joint"};

                // Integrate wheel angle from linear velocity and wheel radius.
                // θ += (v_x / r) × dt  — both wheels same speed for straight travel.
                // Left wheel positive, right wheel negative for forward motion
                // (mirrored mount — same direction of rotation gives opposite sign
                //  when viewed from outside).
                double v_x, dt_s;
                {
                    std::lock_guard<std::mutex> lk(state->mtx);
                    v_x = state->odom_vx;
                }
                dt_s          = 33.0e-3;                  // 30 Hz timer period
                double dtheta = (v_x / WHEEL_RADIUS_M) * dt_s;
                wheel_left_angle_  += dtheta;
                wheel_right_angle_ -= dtheta;             // opposite mount orientation

                js.position = {wheel_left_angle_, wheel_right_angle_};
                js.velocity = {v_x / WHEEL_RADIUS_M, -v_x / WHEEL_RADIUS_M};
                js.effort   = {};

                joint_state_pub_->publish(js);
            });

        RCLCPP_INFO(get_logger(),
            "=== BT Brain (StateHub) online. Waiting for sensors... ===");

        // ── Bootstrap: pre-fill identity map→odom so the 30 Hz timer ────────
        // starts broadcasting the moment bt_brain launches, before the first
        // real bt_fused estimate arrives.
        //
        // WHY: navsat_transform needs map→base_footprint to initialise GPS→ENU.
        // That chain is: map→odom (ours) + odom→base_footprint (ekf_local).
        // Without this bootstrap there is a dead window between ekf_delay (8s)
        // and the first BT tick with real data — navsat_transform errors out,
        // /odometry/gps is never published, the global EKFs have no GPS input,
        // gps_odom_ready stays false, CheckGpsSignal always fails, and
        // update_map_to_odom_tf() never gets called → map→odom never appears
        // → /tf stays completely silent for the whole run.
        //
        // The identity transform is correct at t=0 because the robot spawns
        // at the same position in both the map frame and the odom frame.
        // update_map_to_odom_tf() overwrites this with the GPS-corrected
        // value as soon as the first real bt_fused position is computed.
        //
        // NOTE: stamp is left at 0 here — the 30 Hz wall timer calls
        // ts.header.stamp = now() on every tick, so the first broadcast
        // after the sim clock arrives will have a valid timestamp.
        {
            geometry_msgs::msg::TransformStamped bootstrap;
            bootstrap.header.frame_id    = "map";
            bootstrap.child_frame_id     = "odom";
            bootstrap.transform.rotation.w = 1.0;  // identity quaternion
            std::lock_guard<std::mutex> lk(state->mtx);
            state->map_to_odom_cached = bootstrap;
            state->map_to_odom_valid  = true;    // timer starts broadcasting now
        }

        // ── TF2 setup ─────────────────────────────────────────────────────
        // TransformBroadcaster: pass *this (rclcpp::Node&), not this (raw ptr).
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_       = std::make_shared<tf2_ros::Buffer>(get_clock());
        // TransformListener: pass 'this' (StateHub*) — the template deduces
        // NodeT=StateHub* and calls node->get_node_X_interface() internally.
        // Passing *this (reference) breaks '->' inside the template body.
        // spin_thread=false: StateHub is already being spun externally.
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_, this, false);

        // Broadcast the cached map→odom at 30 Hz.
        // update_map_to_odom_tf() populates the cache every time
        // publish_bt_fused() fires (at the BT tick rate, ~10 Hz).
        // Re-stamping every 33 ms keeps the TF buffer from expiring
        // between BT ticks and gives downstream consumers a smooth 30 Hz edge.
        tf_broadcast_timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
            [this]() {
                // ── Edge 1: map → odom (from bt_fused estimate) ───────────────
                geometry_msgs::msg::TransformStamped ts;
                bool valid;
                {
                    std::lock_guard<std::mutex> lk(state->mtx);
                    ts    = state->map_to_odom_cached;
                    valid = state->map_to_odom_valid;
                }
                if (valid) {
                    ts.header.stamp = now();   // refresh timestamp every tick
                    tf_broadcaster_->sendTransform(ts);
                }

                // ── Edge 2: odom → base_footprint (from raw wheel odometry) ──
                // bt_brain owns this edge as a reliable fallback.
                // ekf_local should publish the same edge (fused with IMU), but
                // if it is silent the TF chain map→odom→base_footprint must still
                // exist so that navsat_transform, update_map_to_odom_tf(), and
                // all TF consumers can function.
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

    // ── TF2 ──────────────────────────────────────────────────────────────────
    /**
     * Compute and cache the map → odom TF edge from the bt_fused position.
     *
     * Math:
     *   T_map_to_odom = T_map_to_base_fp × inv(T_odom_to_base_fp)
     *
     * T_map_to_base_fp  : bt_fused (x, y, yaw in map frame)
     * T_odom_to_base_fp : looked up from TF buffer (published by ekf_local)
     *
     * Called from publish_bt_fused() on every BT tick that outputs a position.
     */
    void update_map_to_odom_tf(double bt_x, double bt_y, double yaw)
    {
        // ── Look up odom → base_footprint ─────────────────────────────────
        geometry_msgs::msg::TransformStamped odom_to_base;
        try {
            odom_to_base = tf_buffer_->lookupTransform(
                "odom", "base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "[BT TF] Cannot lookup odom→base_footprint: %s  "
                "(map→odom TF not updated this tick)", ex.what());
            return;
        }

        // ── Build T_map_to_base_fp from bt_fused ──────────────────────────
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tf2::Transform T_map_base;
        T_map_base.setOrigin(tf2::Vector3(bt_x, bt_y, 0.0));
        T_map_base.setRotation(q);

        // ── Build T_odom_to_base_fp from TF lookup ─────────────────────────
        tf2::Transform T_odom_base;
        tf2::fromMsg(odom_to_base.transform, T_odom_base);

        // ── T_map_to_odom = T_map_base × inv(T_odom_base) ─────────────────
        tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

        // ── Cache for the 30 Hz broadcaster timer ─────────────────────────
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

    // ── append_path_point() ──────────────────────────────────────────────────
    // Appends (x,y) from bt_fused to the trajectory buffer and republishes
    // the full yellow LINE_STRIP on /bt/path_viz.
    // Single Marker id=0 with action=ADD overwrites itself — no flicker.
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

    // ANN Marker publisher — converts headerless geometry_msgs/Point to a
    // map-frame visualization_msgs/Marker for the RViz /ann/viz_marker display.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ann_marker_pub_;
    int ann_marker_seq_{0};   // rolling ID; mod 10000 → short RViz trail

    // Yellow LINE_STRIP trajectory (published on /bt/path_viz).
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
    std::vector<geometry_msgs::msg::Point>                        path_points_;
    static constexpr std::size_t PATH_MAX_POINTS = 50000;

    // Wheel joint state publisher — feeds robot_state_publisher so it can emit
    // wheel_left_link / wheel_right_link TF edges (fixes RViz "No transform" error).
    // Uses sensor_msgs::JointState which is already a bt_brain dependency.
    static constexpr double WHEEL_RADIUS_M = 0.033;   // TurtleBot3 Burger
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    double wheel_left_angle_  = 0.0;   // [rad] accumulated wheel angle (visual only)
    double wheel_right_angle_ = 0.0;

    // ── TF2 members ───────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    rclcpp::TimerBase::SharedPtr                   tf_broadcast_timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  HELPER: publish the BT-fused Odometry message
// ─────────────────────────────────────────────────────────────────────────────

static void publish_bt_fused(StateHub& hub, double x, double y)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = hub.now();
    msg.header.frame_id = "map";
    msg.child_frame_id  = "base_footprint";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    // Orientation: copy from KF2 (most stable dead-reckoning source)
    double yaw;
    {
        std::lock_guard<std::mutex> lk(hub.state->mtx);
        // Use yaw from odom as orientation proxy when KF2 quat isn't stored
        yaw = hub.state->odom_yaw;
        msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
        msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    }
    hub.bt_fused_pub->publish(msg);

    // Update state
    {
        std::lock_guard<std::mutex> lk(hub.state->mtx);
        hub.state->final_x = x;
        hub.state->final_y = y;
    }

    // ── Drive the map→odom TF edge from this bt_fused output ─────────────
    // update_map_to_odom_tf() computes T_map_odom = T_map_base * inv(T_odom_base)
    // and caches it for the 30 Hz broadcast timer inside StateHub.
    hub.update_map_to_odom_tf(x, y, yaw);

    // Append to yellow PATH trail on /bt/path_viz
    hub.append_path_point(x, y);
}

// ─────────────────────────────────────────────────────────────────────────────
//  HELPER: staleness check (seconds since last message)
// ─────────────────────────────────────────────────────────────────────────────

static double now_sec(StateHub& hub)
{
    return hub.now().seconds();
}

// ─────────────────────────────────────────────────────────────────────────────
//  ╔═══════════════════════════════════════════╗
//  ║   BT NODE IMPLEMENTATIONS                 ║
//  ╚═══════════════════════════════════════════╝
// ─────────────────────────────────────────────────────────────────────────────

// ── CONDITION: CheckSensorsReady ─────────────────────────────────────────────
/**
 * Returns SUCCESS when both /imu/data and /odom are live and recent.
 * Blocks the full BT with FAILURE until sensors come up at startup.
 */
class CheckSensorsReady : public ConditionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    CheckSensorsReady(const std::string& name, std::shared_ptr<StateHub> hub)
    : ConditionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#0D3B38"},
            {"fg_color", "#4ECDC4"}
        };
    }

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

// ── CONDITION: CheckGpsSignal ────────────────────────────────────────────────
/**
 * Returns SUCCESS when ALL of the following hold:
 *   (a) /gps/fix reports a valid fix
 *   (b) navsat_transform is publishing /odometry/gps (frames aligned)
 *   (c) the last GPS odometry was received within 2 seconds
 *   (d) IndoorDetector has NOT flagged indoor conditions
 *       (i.e. state->indoor_detected == false)
 * Otherwise FAILURE → triggers Branch B immediately.
 */
class CheckGpsSignal : public ConditionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double GPS_TIMEOUT_S = 2.0;
public:
    CheckGpsSignal(const std::string& name, std::shared_ptr<StateHub> hub)
    : ConditionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#0D3B38"},
            {"fg_color", "#4ECDC4"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;

        bool fix_ok   = s.gps_available && s.has_ever_had_gps;
        bool odom_ok  = s.gps_odom_ready;
        bool fresh    = (now_sec(*hub_) - s.gps_odom_timestamp) < GPS_TIMEOUT_S;
        bool outdoor  = !s.indoor_detected;   // IndoorDetector override

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

// ── ACTION: AlignSensorFrames ─────────────────────────────────────────────────
/**
 * Paper §3.1 – DCM R(θ) alignment.
 * In practice: verify that navsat_transform has converted GPS lat/lon
 * into the ENU map frame and is publishing /odometry/gps.
 * Returns FAILURE if the GPS odometry hasn't appeared yet.
 */
class AlignSensorFrames : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    AlignSensorFrames(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#0D2847"},
            {"fg_color", "#5BA4E5"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        bool aligned = hub_->state->gps_odom_ready;
        if (!aligned) {
            RCLCPP_WARN_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 3000,
                "[BT] Frame alignment pending – /odometry/gps not yet live.");
            return NodeStatus::FAILURE;
        }
        RCLCPP_DEBUG(hub_->get_logger(),
            "[BT] ✓ Frames aligned (ENU via navsat_transform). "
            "GPS ENU pos: (%.3f, %.3f)",
            hub_->state->last_known_gps_x, hub_->state->last_known_gps_y);
        return NodeStatus::SUCCESS;
    }
};

// ── ACTION: EnsureKF1Active ───────────────────────────────────────────────────
/**
 * Verifies /odometry/global (KF1 – GPS+IMU EKF) is live and recent.
 * Sets kf1_active = true in shared state.
 */
class EnsureKF1Active : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double KF_TIMEOUT_S = 1.0;
public:
    EnsureKF1Active(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#0D2847"},
            {"fg_color", "#5BA4E5"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        bool alive = s.kf1_ready &&
                     (now_sec(*hub_) - s.kf1_ts) < KF_TIMEOUT_S;
        s.kf1_active = alive;
        if (!alive) {
            RCLCPP_WARN_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 3000,
                "[BT] KF1 (GPS+IMU EKF) not responsive on /odometry/global.");
            return NodeStatus::FAILURE;
        }
        RCLCPP_DEBUG(hub_->get_logger(),
            "[BT] ✓ KF1 active. P1 = (%.3f, %.3f)", s.kf1_x, s.kf1_y);
        return NodeStatus::SUCCESS;
    }
};

// ── ACTION: EnsureKF2Active ───────────────────────────────────────────────────
/**
 * Verifies /odometry/global2 (KF2 – GPS+Odom EKF) is live and recent.
 * Sets kf2_active = true in shared state.
 */
class EnsureKF2Active : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double KF_TIMEOUT_S = 1.0;
public:
    EnsureKF2Active(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#0D2847"},
            {"fg_color", "#5BA4E5"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        bool alive = s.kf2_ready &&
                     (now_sec(*hub_) - s.kf2_ts) < KF_TIMEOUT_S;
        s.kf2_active = alive;
        if (!alive) {
            RCLCPP_WARN_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 3000,
                "[BT] KF2 (GPS+Odom EKF) not responsive on /odometry/global2.");
            return NodeStatus::FAILURE;
        }
        RCLCPP_DEBUG(hub_->get_logger(),
            "[BT] ✓ KF2 active. P2 = (%.3f, %.3f)", s.kf2_x, s.kf2_y);
        return NodeStatus::SUCCESS;
    }
};

// ── ACTION: RunComplementaryFilter_GPS ───────────────────────────────────────
/**
 * Paper §3.2 – Fixed-Weight Complementary Filter (GPS Mode).
 *
 *   Vec1 = [a_x, a_y, x_gps, y_gps]   → input to KF1
 *   Vec2 = [v_x, v_y, x_gps, y_gps]   → input to KF2
 *   P1   = KF1 position estimate       → [kf1_x, kf1_y]
 *   P2   = KF2 position estimate       → [kf2_x, kf2_y]
 *
 *   x_final = α1_fixed * P1.x  +  α2_fixed * P2.x
 *   y_final = α1_fixed * P1.y  +  α2_fixed * P2.y
 *
 *   α1 = α2 = 0.5  (fixed, matches complementary_filter.py)
 */
class RunComplementaryFilter_GPS : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    RunComplementaryFilter_GPS(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#0D2847"},
            {"fg_color", "#5BA4E5"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        if (!s.kf1_active || !s.kf2_active) return NodeStatus::FAILURE;

        s.cf_x = RobotState::ALPHA1_FIXED * s.kf1_x +
                 RobotState::ALPHA2_FIXED * s.kf2_x;
        s.cf_y = RobotState::ALPHA1_FIXED * s.kf1_y +
                 RobotState::ALPHA2_FIXED * s.kf2_y;

        RCLCPP_DEBUG(hub_->get_logger(),
            "[BT][CF_GPS] KF1=(%.3f,%.3f)  KF2=(%.3f,%.3f)  "
            "Fused=(%.3f,%.3f)  α=[%.1f,%.1f]",
            s.kf1_x, s.kf1_y, s.kf2_x, s.kf2_y,
            s.cf_x, s.cf_y,
            RobotState::ALPHA1_FIXED, RobotState::ALPHA2_FIXED);
        return NodeStatus::SUCCESS;
    }
};

// ── ACTION: PublishFusedPosition_GPS ─────────────────────────────────────────
/**
 * Publishes the GPS-branch fused position to /odometry/bt_fused.
 * Also publishes a status string for diagnostics.
 */
class PublishFusedPosition_GPS : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    PublishFusedPosition_GPS(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#0D2847"},
            {"fg_color", "#5BA4E5"}
        };
    }

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

// ── ACTION: CollectAndTrainNN ─────────────────────────────────────────────────
/**
 * Paper §3.3 – Online NN Training (GPS branch).
 *
 * The NN_Train vector = [IMU, Odometer, KF1, KF2] is assembled here
 * and logged for verification. The actual training is performed autonomously
 * by the robot_control_brain/ANN.py node (it subscribes to all sources
 * and trains in background every 5 s). This BT action:
 *   (1) Logs the training vector for diagnostics
 *   (2) Verifies ANN is publishing (training has occurred)
 *
 * Returns SUCCESS always (non-blocking – training happens asynchronously).
 */
class CollectAndTrainNN : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    CollectAndTrainNN(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#0D2847"},
            {"fg_color", "#5BA4E5"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;

        // NN_Train = [ax, ay, wz_imu,  vx, vy, wz_odom,  kf1_x, kf1_y,  kf2_x, kf2_y]
        RCLCPP_DEBUG(hub_->get_logger(),
            "[BT][NN_TRAIN] IMU=[%.3f,%.3f,%.3f] "
            "Odom=[%.3f,%.3f,%.3f] "
            "KF1=[%.3f,%.3f] KF2=[%.3f,%.3f]  "
            "ANN_ready=%d",
            s.imu_ax, s.imu_ay, s.imu_wz,
            s.odom_vx, s.odom_vy, s.odom_wz,
            s.kf1_x, s.kf1_y,
            s.kf2_x, s.kf2_y,
            s.ann_ready);

        if (s.ann_ready) {
            RCLCPP_INFO_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 10000,
                "[BT] ✓ ANN is trained and publishing. Last output: "
                "(%.3f, %.3f)", s.ann_x, s.ann_y);
        }
        return NodeStatus::SUCCESS;
    }
};

// ── ACTION: HaltKF1_FreezeKF2GPS ─────────────────────────────────────────────
/**
 * Paper §2 – GPS Unavailable: "StopKF1 and update parameters of KF2
 *             using most recent GPS update to obtain robot position estimates."
 *
 * Implementation:
 *   • Sets kf1_active = false  → BT will not use KF1 output
 *   • The underlying ekf_filter_node_gps_imu keeps running but its output
 *     will drift; we simply ignore it.
 *   • KF2 (ekf_filter_node_gps_enc) automatically falls back to odometry
 *     integration from its last GPS-updated state when navsat_transform
 *     stops publishing (because /gps/fix is lost).  This is the "frozen GPS"
 *     behavior described in the paper.
 *   • We log the last known GPS ENU position for reference.
 */
class HaltKF1_FreezeKF2GPS : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    HaltKF1_FreezeKF2GPS(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#3D2010"},
            {"fg_color", "#E8A060"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;

        if (s.kf1_active) {
            // First time entering No-GPS branch – log the handover point
            RCLCPP_WARN(hub_->get_logger(),
                "[BT] ⚠  GPS LOST.  Halting KF1.  "
                "Last known GPS ENU: (%.3f, %.3f).  "
                "KF2 continues dead-reckoning from this anchor.",
                s.last_known_gps_x, s.last_known_gps_y);
        }
        s.kf1_active  = false;
        s.gps_branch  = false;

        return NodeStatus::SUCCESS;
    }
};

// ── ACTION: CalculateSlipError ────────────────────────────────────────────────
/**
 * Paper §3.4 – Slip Detection.
 *
 * Compare angular velocity from IMU (ω_imu) with angular velocity from
 * wheel odometry (ω_odom).  Slippage causes them to diverge.
 *
 *   e = |ω_imu(z)  −  ω_odom(z)|   [rad/s]
 *
 * Stores result in state->slip_error for the Fuzzy Logic nodes.
 */
class CalculateSlipError : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    CalculateSlipError(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#3D2010"},
            {"fg_color", "#E8A060"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;

        double e = std::abs(s.imu_wz - s.odom_wz);
        s.slip_error = e;

        RCLCPP_DEBUG(hub_->get_logger(),
            "[BT][SLIP] ω_IMU=%.4f rad/s  ω_odom=%.4f rad/s  e=%.4f rad/s",
            s.imu_wz, s.odom_wz, e);
        return NodeStatus::SUCCESS;
    }
};

// ── ACTION: FuzzifySlipError ──────────────────────────────────────────────────
/**
 * Paper §3.5 – Fuzzification of input error using Fuzzy Membership Functions.
 *
 * Three triangular/shoulder MFs over slip error e [rad/s]:
 *
 *   Low    (L): shoulder-left,   1 → 0 from e=0   to e=0.10
 *   Medium (M): triangle,        0 → 1 → 0   peak at e=0.15  (base 0.05–0.25)
 *   High   (H): shoulder-right,  0 → 1 from e=0.20 to e=0.30
 *
 * Stores μ_Low, μ_Medium, μ_High in state->fuzzy_mf.
 */
class FuzzifySlipError : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    // MF parameters (tunable for your slip range)
    static constexpr double L_CENTER = 0.00, L_RIGHT  = 0.10;
    static constexpr double M_LEFT   = 0.05, M_CENTER = 0.15, M_RIGHT = 0.25;
    static constexpr double H_LEFT   = 0.20, H_CENTER = 0.30;
public:
    FuzzifySlipError(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#3D2010"},
            {"fg_color", "#E8A060"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        double e = s.slip_error;

        s.fuzzy_mf.low    = shoulder_left_mf(e,  L_CENTER, L_RIGHT);
        s.fuzzy_mf.medium = triangular_mf(e,     M_LEFT, M_CENTER, M_RIGHT);
        s.fuzzy_mf.high   = shoulder_right_mf(e, H_LEFT, H_CENTER);

        RCLCPP_DEBUG(hub_->get_logger(),
            "[BT][FUZZ]  e=%.4f → μ_L=%.3f  μ_M=%.3f  μ_H=%.3f",
            e, s.fuzzy_mf.low, s.fuzzy_mf.medium, s.fuzzy_mf.high);
        return NodeStatus::SUCCESS;
    }
};

// ── ACTION: ActivateFuzzyRules ────────────────────────────────────────────────
/**
 * Paper §3.6 – Fuzzy Rule Activation (Mamdani min-inference).
 *
 * Three rules mapping slip error to weight singletons:
 *
 *   R1: IF e IS Low    THEN α1=0.80 (trust NN more)   α2=0.20
 *   R2: IF e IS Medium THEN α1=0.50                   α2=0.50
 *   R3: IF e IS High   THEN α1=0.20 (distrust NN)     α2=0.80 (trust KF2)
 *
 * The firing strength of each rule equals the MF value (min-of-antecedent).
 * We store raw firing strengths; defuzzification aggregates them next.
 */
class ActivateFuzzyRules : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    // Singleton output centres for α1 (NN weight) per rule
    static constexpr double C1_LOW    = 0.80;
    static constexpr double C1_MEDIUM = 0.50;
    static constexpr double C1_HIGH   = 0.20;
    // Singleton centres for α2 (KF2 weight) per rule  (= 1 - α1)
    static constexpr double C2_LOW    = 0.20;
    static constexpr double C2_MEDIUM = 0.50;
    static constexpr double C2_HIGH   = 0.80;
public:
    ActivateFuzzyRules(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#3D2010"},
            {"fg_color", "#E8A060"}
        };
    }

    NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lk(hub_->state->mtx);
        auto& s = *hub_->state;
        auto& mf = s.fuzzy_mf;

        // Weighted numerator accumulators
        double num1 = mf.low * C1_LOW + mf.medium * C1_MEDIUM + mf.high * C1_HIGH;
        double num2 = mf.low * C2_LOW + mf.medium * C2_MEDIUM + mf.high * C2_HIGH;
        double den  = mf.low + mf.medium + mf.high;

        // Store for defuzzification (den stored implicitly via state)
        if (den < 1e-9) {
            // All membership values zero → uniform weights
            s.alpha1_adaptive = 0.5;
            s.alpha2_adaptive = 0.5;
        } else {
            s.alpha1_adaptive = num1 / den;
            s.alpha2_adaptive = num2 / den;
        }

        RCLCPP_DEBUG(hub_->get_logger(),
            "[BT][RULES] w_L=%.3f  w_M=%.3f  w_H=%.3f",
            mf.low, mf.medium, mf.high);
        return NodeStatus::SUCCESS;
    }
};

// ── ACTION: DefuzzifyWeights ──────────────────────────────────────────────────
/**
 * Paper §3.7 – Defuzzification → Fusion Parameters α1 & α2.
 *
 * The singleton weighted-average is already computed in ActivateFuzzyRules.
 * This node:
 *   (1) Clamps α values to [0,1]
 *   (2) Enforces α1 + α2 = 1.0 (complementary constraint)
 *   (3) Publishes diagnostic [α1, α2, slip_error] on /bt/fuzzy_weights
 */
class DefuzzifyWeights : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
public:
    DefuzzifyWeights(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#3D2010"},
            {"fg_color", "#E8A060"}
        };
    }

    NodeStatus tick() override
    {
        double a1, a2, slip;
        {
            std::lock_guard<std::mutex> lk(hub_->state->mtx);
            auto& s = *hub_->state;
            // Clamp and normalise
            a1   = std::max(0.0, std::min(1.0, s.alpha1_adaptive));
            a2   = 1.0 - a1;   // enforce complementary constraint
            s.alpha1_adaptive = a1;
            s.alpha2_adaptive = a2;
            slip = s.slip_error;
        }

        // Publish diagnostic
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

// ── ACTION: FuseNN_KF2_Adaptive ───────────────────────────────────────────────
/**
 * Paper §3.8 – Adaptive FL Complementary Filter (No-GPS mode).
 *
 *   x_final = α1 * NN_x  +  α2 * KF2_x
 *   y_final = α1 * NN_y  +  α2 * KF2_y
 *
 * Fallback hierarchy (in order of preference):
 *   1. NN trained AND KF2 live → adaptive fusion (main path)
 *   2. NN not ready yet        → KF2 only  (α1=0, α2=1)
 *   3. KF2 not live            → last known GPS ENU position (safety fallback)
 *
 * Publishes final position to /odometry/bt_fused.
 */
class FuseNN_KF2_Adaptive : public SyncActionNode
{
    std::shared_ptr<StateHub> hub_;
    static constexpr double NN_TIMEOUT_S  = 1.0;   // ANN must be recent
    static constexpr double KF2_TIMEOUT_S = 1.0;
public:
    FuseNN_KF2_Adaptive(const std::string& name, std::shared_ptr<StateHub> hub)
    : SyncActionNode(name, {}), hub_(hub) {}

    static BT::KeyValueVector metadata()
    {
        return {
            {"bg_color", "#3D2010"},
            {"fg_color", "#E8A060"}
        };
    }

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
                // ── Path 1: Full adaptive fusion ─────────────────────────────
                a1 = s.alpha1_adaptive;
                a2 = s.alpha2_adaptive;
                x_final = a1 * s.ann_x + a2 * s.kf2_x;
                y_final = a1 * s.ann_y + a2 * s.kf2_y;
                mode = "NN+KF2_ADAPTIVE";
            } else if (kf2_live) {
                // ── Path 2: KF2 only (NN not yet trained) ───────────────────
                a1 = 0.0; a2 = 1.0;
                x_final = s.kf2_x;
                y_final = s.kf2_y;
                mode = "KF2_ONLY (NN not trained yet)";
            } else {
                // ── Path 3: Last known GPS anchor (safety fallback) ──────────
                a1 = 0.0; a2 = 0.0;
                x_final = s.last_known_gps_x;
                y_final = s.last_known_gps_y;
                mode = "LAST_GPS_ANCHOR (KF2 also offline)";
                RCLCPP_ERROR_THROTTLE(hub_->get_logger(), *hub_->get_clock(), 5000,
                    "[BT] ⛔ KF2 AND NN both offline! "
                    "Holding last GPS anchor (%.3f, %.3f).",
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
            "[BT][FUSE_NoGPS] mode=%-25s  α=[%.3f,%.3f]  "
            "→ final=(%.3f, %.3f)",
            mode.c_str(), a1, a2, x_final, y_final);

        return NodeStatus::SUCCESS;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  FACTORY REGISTRATION HELPER
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Register all custom BT nodes with the factory.
 * Each node receives the shared hub pointer via a lambda builder.
 */
static void register_all_nodes(BehaviorTreeFactory& factory,
                                std::shared_ptr<StateHub> hub)
{
    // ── Conditions ────────────────────────────────────────────────────────────
    factory.registerBuilder<CheckSensorsReady>("CheckSensorsReady",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<CheckSensorsReady>(n, hub);
        });

    factory.registerBuilder<CheckGpsSignal>("CheckGpsSignal",
        [hub](const std::string& n, const NodeConfig&) {
            return std::make_unique<CheckGpsSignal>(n, hub);
        });

    // ── Branch A Actions ──────────────────────────────────────────────────────
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

    // ── Branch B Actions ──────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // ── 1. Create the shared ROS2 hub node ───────────────────────────────────
    auto hub = std::make_shared<StateHub>();

    // ── 2. Create IndoorDetector (shares the hub node, runs in its callbacks) ──
    //
    //   indoor_flag : atomic<bool> written by the detector, read by CheckGpsSignal
    //
    //   When the detector decides the robot is INDOOR it sets indoor_flag=true.
    //   CheckGpsSignal reads state->indoor_detected (written by the lambda below)
    //   and immediately returns FAILURE, causing the BT to switch to Branch B.
    //
    //   Tune cfg to match your environment (e.g. higher cov threshold for urban).
    auto indoor_flag = std::make_shared<std::atomic<bool>>(false);

    IndoorDetector::Config id_cfg;

    // ── Fix A: GPS multipath debounce hardening ──────────────────────────────
    // The building south wall creates GPS multipath starting ~2m before entry.
    // At GPS 5 Hz, debounce_in_ticks=2 fires INDOOR after only 0.4 s of
    // degraded signal — far too sensitive for a building approach in simulation.
    // 4 ticks = 0.8 s of sustained bad GPS required before declaring INDOOR.
    // debounce_out_ticks raised proportionally to prevent rapid oscillation
    // when the robot lingers near the wall boundary.
    id_cfg.debounce_in_ticks  = 4;    // was 2  → 0.8 s bad GPS to declare INDOOR
    id_cfg.debounce_out_ticks = 8;    // was 4  → 1.6 s good GPS to declare OUTDOOR

    // ── Fix B: GPS position divergence thresholds ────────────────────────────
    // /odometry/global vs /odom legitimately diverges 1.0–1.5 m near the
    // building due to GPS multipath pulling the EKF while /odom is accurate.
    // The old warn threshold (1.0 m) fired on normal outdoor approach, adding
    // +1 to every GPS tick score and compounding with the jump score to reach
    // score_indoor_thr=3 while the robot was still 10–15 m from the door.
    id_cfg.pos_div_warn_m = 1.5;   // was 1.0 m
    id_cfg.pos_div_bad_m  = 4.0;   // was 3.0 m

    // ── Fix C: GPS jump threshold ────────────────────────────────────────────
    // GPS at σ=0.3162 m produces consecutive-message jumps of 0.4–0.9 m from
    // noise alone (5 Hz × 0.316 m std ≈ 0.5 m per step). The old minor jump
    // threshold (0.5 m) scored +1 on almost every outdoor GPS message, meaning
    // the score was never 0 even in open sky. Raise to 1.0 m so only genuine
    // multipath jumps (>3σ) contribute to the score.
    id_cfg.jump_minor_m    = 1.0;   // was 0.5 m  (was below 3σ GPS noise floor)
    id_cfg.jump_moderate_m = 2.5;   // was 2.0 m
    id_cfg.jump_severe_m   = 6.0;   // was 5.0 m

    auto indoor_detector = std::make_shared<IndoorDetector>(
        hub.get(), indoor_flag, id_cfg);

    // Bridge detector output → shared RobotState so BT nodes can read it
    // A small timer polls the atomic flag and syncs to state (10 Hz is plenty)
    auto bridge_timer = hub->create_wall_timer(
        std::chrono::milliseconds(100),
        [&hub, &indoor_flag]() {
            std::lock_guard<std::mutex> lk(hub->state->mtx);
            hub->state->indoor_detected = indoor_flag->load();
        });

    // ── 3. Spin the ROS2 node in a background thread ─────────────────────────
    std::thread ros_thread([hub]() {
        RCLCPP_INFO(hub->get_logger(), "[ROS2] Spin thread started.");
        rclcpp::spin(hub);
    });
    ros_thread.detach();

    // ── 4. Build the BehaviorTree ─────────────────────────────────────────────
    BehaviorTreeFactory factory;
    register_all_nodes(factory, hub);

    // Resolve the XML path relative to the installed package share directory
    // (works both from install/ and when running directly from workspace root)
    std::string xml_path =
        ament_index_cpp::get_package_share_directory("bt_orchestrator_pkg") +
        "/bt_xml/localization_tree.xml";

    // Fallback: try workspace-relative path (for development)
    // xml_path = "src/bt_orchestrator_pkg/bt_xml/localization_tree.xml";

    auto tree = factory.createTreeFromFile(xml_path);

    // ── 5. Attach Groot2 live visualiser (port 1667) ─────────────────────────
    BT::Groot2Publisher groot2_publisher(tree, 1667);

    RCLCPP_INFO(hub->get_logger(),
        "╔══════════════════════════════════════════════════╗");
    RCLCPP_INFO(hub->get_logger(),
        "║  BT Brain started — GPS/INS/Odometer Fusion      ║");
    RCLCPP_INFO(hub->get_logger(),
        "║  Tick rate : 10 Hz                               ║");
    RCLCPP_INFO(hub->get_logger(),
        "║  Groot2    : Connect on port 1667                ║");
    RCLCPP_INFO(hub->get_logger(),
        "║  Output    : /odometry/bt_fused                  ║");
    RCLCPP_INFO(hub->get_logger(),
        "╚══════════════════════════════════════════════════╝");

    // ── 6. Tick the BT at 10 Hz ───────────────────────────────────────────────
    rclcpp::Rate rate(10);   // 10 Hz

    while (rclcpp::ok()) {
        RCLCPP_DEBUG(hub->get_logger(), "--- BT Tick ---");
        NodeStatus status = tree.tickOnce();

        (void)status;   // tree runs continuously; status is informational only

        rate.sleep();
    }

    RCLCPP_INFO(hub->get_logger(), "BT Brain shutting down.");
    rclcpp::shutdown();
    return 0;
}