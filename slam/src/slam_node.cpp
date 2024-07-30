#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

class SlamNode : public rclcpp::Node
{
public:
    SlamNode() : Node("slam_node"), prev_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>), 
                 global_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/pointcloud", 10, std::bind(&SlamNode::pointcloud_callback, this, std::placeholders::_1));

        reconstructed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("reconstructed_pointcloud", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        accumulated_transform_ = Eigen::Matrix4f::Identity();
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(*msg, *cloud);

            RCLCPP_INFO(this->get_logger(), "Input cloud size: %zu", cloud->size());

            if (cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty point cloud. Skipping.");
                return;
            }

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.01f, 0.01f, 0.01f);
            sor.filter(*cloud);

            RCLCPP_INFO(this->get_logger(), "After downsampling: %zu points", cloud->size());

            if (prev_cloud_->empty())
            {
                *prev_cloud_ = *cloud;
                *global_cloud_ += *cloud;
                RCLCPP_INFO(this->get_logger(), "Initialized previous cloud. Skipping first frame.");
                return;
            }

            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setInputSource(cloud);
            icp.setInputTarget(prev_cloud_);
            icp.setMaxCorrespondenceDistance(0.1);
            icp.setMaximumIterations(50);
            icp.setTransformationEpsilon(1e-8);
            icp.setEuclideanFitnessEpsilon(1);
            pcl::PointCloud<pcl::PointXYZRGB> aligned_cloud;
            icp.align(aligned_cloud);

            RCLCPP_INFO(this->get_logger(), "ICP has converged: %s", icp.hasConverged() ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "ICP fitness score: %f", icp.getFitnessScore());

            if (!icp.hasConverged()) {
                RCLCPP_WARN(this->get_logger(), "ICP did not converge. Skipping this frame.");
                return;
            }

            Eigen::Matrix4f transform = icp.getFinalTransformation();
            accumulated_transform_ = accumulated_transform_ * transform;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*cloud, *transformed_cloud, accumulated_transform_);

            *global_cloud_ += *transformed_cloud;

            pcl::VoxelGrid<pcl::PointXYZRGB> global_sor;
            global_sor.setInputCloud(global_cloud_);
            global_sor.setLeafSize(0.05f, 0.05f, 0.05f);
            global_sor.filter(*global_cloud_);

            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*global_cloud_, output);
            output.header = msg->header;
            output.header.frame_id = "odom";
            
            reconstructed_pub_->publish(output);

            RCLCPP_INFO(this->get_logger(), "Global cloud size: %zu", global_cloud_->size());

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";

            Eigen::Vector3f translation = accumulated_transform_.block<3, 1>(0, 3);
            Eigen::Matrix3f rotation = accumulated_transform_.block<3, 3>(0, 0);
            Eigen::Quaternionf q(rotation);

            t.transform.translation.x = translation.x();
            t.transform.translation.y = translation.y();
            t.transform.translation.z = translation.z();
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t);

            *prev_cloud_ = *cloud;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in pointcloud_callback: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr reconstructed_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_;
    Eigen::Matrix4f accumulated_transform_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamNode>());
    rclcpp::shutdown();
    return 0;
}