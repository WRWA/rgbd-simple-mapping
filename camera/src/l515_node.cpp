#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", 10);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera/pointcloud", 10);

        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
        pipe_.start(cfg);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&CameraNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        rs2::frameset frames = pipe_.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        auto color_image = cv::Mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        sensor_msgs::msg::Image::SharedPtr color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg();
        rgb_pub_->publish(*color_msg);

        auto depth_image = cv::Mat(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image).toImageMsg();
        depth_pub_->publish(*depth_msg);

        rs2::pointcloud pc;
        pc.map_to(color_frame);
        rs2::points points = pc.calculate(depth_frame);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        auto vertices = points.get_vertices();
        auto tex_coords = points.get_texture_coordinates();
        for (size_t i = 0; i < points.size(); i++)
        {
            pcl::PointXYZRGB point;
            point.x = vertices[i].x;
            point.y = vertices[i].y;
            point.z = vertices[i].z;
            
            int x = std::min(std::max(int(tex_coords[i].u * color_frame.get_width()), 0), static_cast<int>(color_frame.get_width()) - 1);
            int y = std::min(std::max(int(tex_coords[i].v * color_frame.get_height()), 0), static_cast<int>(color_frame.get_height()) - 1);
            int idx = y * color_frame.get_width() + x;
            const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color_frame.get_data());
            point.r = color_data[3*idx + 2];
            point.g = color_data[3*idx + 1];
            point.b = color_data[3*idx];
            
            pcl_cloud->push_back(point);
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*pcl_cloud, cloud_msg);
        cloud_msg.header.frame_id = "base_link";
        cloud_msg.header.stamp = this->now();

        pointcloud_pub_->publish(cloud_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rs2::pipeline pipe_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}