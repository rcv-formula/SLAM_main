#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include <functional>
#include <utility>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

class TrackedPoseToOdom : public rclcpp::Node
{
public:
    TrackedPoseToOdom()
        : Node("tracked_pose_to_odom"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        if (!this->get_parameter("tracking_frame", tracking_frame_)) {
            tracking_frame_ = "imu";
        }
        if (!this->get_parameter("published_frame", published_frame_)) {
            published_frame_ = "base_link";
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 20,
            std::bind(&TrackedPoseToOdom::trackedPoseCallback, this, _1));
        filtered_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/filtered_tracked_pose", 20,
            std::bind(&TrackedPoseToOdom::filteredTrackedPoseCallback, this, _1));
        offset_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/offset_tracked_pose", 20,
            std::bind(&TrackedPoseToOdom::offsetTrackedPoseCallback, this, _1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
        filtered_publisher_ =
            this->create_publisher<nav_msgs::msg::Odometry>("/filtered_odom", 20);
        offset_publisher_ =
            this->create_publisher<nav_msgs::msg::Odometry>("/offset_odom", 20);

        if (!this->get_parameter("use_sim_time", use_sim_time_)) {
            RCLCPP_INFO(this->get_logger(), "\033[33muse_sim_time NOT SET. Defaulting to false.\033[0m");
            use_sim_time_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "TrackedPoseToOdom node initialized.");
    }

private:
    void trackedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        publishOdometry(*msg, publisher_, "/odom");
    }

    void filteredTrackedPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        publishOdometry(*msg, filtered_publisher_, "/filtered_odom");
    }

    void offsetTrackedPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        publishOdometry(*msg, offset_publisher_, "/offset_odom");
    }

    void publishOdometry(
        const geometry_msgs::msg::PoseStamped& msg,
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& publisher,
        const std::string& topic_name)
    {
        try
        {
            const auto published_to_tracking = tf_buffer_.lookupTransform(
                tracking_frame_, published_frame_, tf2::TimePointZero);

            tf2::Transform tracking_to_map_tf;
            tf2::fromMsg(msg.pose, tracking_to_map_tf);
            tf2::Transform published_to_tracking_tf;
            tf2::fromMsg(published_to_tracking.transform, published_to_tracking_tf);
            const tf2::Transform published_to_map_tf =
                tracking_to_map_tf * published_to_tracking_tf;

            nav_msgs::msg::Odometry odom_msg;

            builtin_interfaces::msg::Time current_time;
            current_time.sec = this->get_clock()->now().seconds();
            current_time.nanosec = this->get_clock()->now().nanoseconds() % 1000000000;
            odom_msg.header.stamp = use_sim_time_ ? current_time : msg.header.stamp;

            odom_msg.header.frame_id = msg.header.frame_id;
            odom_msg.child_frame_id = published_frame_;
            odom_msg.pose.pose.position.x = published_to_map_tf.getOrigin().x();
            odom_msg.pose.pose.position.y = published_to_map_tf.getOrigin().y();
            odom_msg.pose.pose.position.z = published_to_map_tf.getOrigin().z();
            odom_msg.pose.pose.orientation = tf2::toMsg(published_to_map_tf.getRotation());

            publisher->publish(odom_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Could not publish %s from %s to %s: %s",
                        topic_name.c_str(), tracking_frame_.c_str(),
                        published_frame_.c_str(), ex.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr offset_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr offset_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool use_sim_time_ = false;
    std::string tracking_frame_;
    std::string published_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackedPoseToOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
