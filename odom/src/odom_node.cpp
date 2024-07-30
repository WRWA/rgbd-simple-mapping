#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomNode : public rclcpp::Node
{
public:
    OdomNode() : Node("odom_node")
    {
        // Turtlebot3의 오도메트리 토픽을 구독
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&OdomNode::odom_callback, this, std::placeholders::_1));

        // TF 브로드캐스터 초기화
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 오도메트리 메시지를 TF로 변환
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;

        t.transform.rotation = msg->pose.pose.orientation;

        // TF 브로드캐스트
        tf_broadcaster_->sendTransform(t);

        RCLCPP_INFO(this->get_logger(), "Odometry: x: %f, y: %f", 
                    msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}