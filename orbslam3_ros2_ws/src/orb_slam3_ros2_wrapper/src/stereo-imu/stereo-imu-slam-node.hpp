/**
 * @file stereo-slam-node.hpp
 * @brief Definition of the StereoIMUSlamNode Wrapper class.
 */

#ifndef STEREO_IMU_SLAM_NODE_HPP_
#define STEREO_IMU_SLAM_NODE_HPP_

#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "orb_slam3_ros2_wrapper/slam_node_base.hpp"


// 需要注意的是，它继承了 SlamNodeBase 这个东西，这个东西里面，负责ROS的相关
// ORBSLAM3Interface ， 他管理了ORBSLAM3的接口
namespace ORB_SLAM3_Wrapper
{
    class StereoIMUSlamNode : public SlamNodeBase
    {
    public:
        StereoIMUSlamNode(const std::string &strVocFile,
                       const std::string &strSettingsFile,
                       ORB_SLAM3::System::eSensor sensor);
        ~StereoIMUSlamNode();

    private:
        using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgImu);
        void StereoCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft,
                            const sensor_msgs::msg::Image::SharedPtr msgRight);

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> leftSub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rightSub_;
        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
        rclcpp::CallbackGroup::SharedPtr imuCallbackGroup_;

        std::mutex imuMutex_;
        std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
    };
} // namespace ORB_SLAM3_Wrapper

#endif // STEREO_SLAM_NODE_HPP_

