/**
 * @file stereo-slam-node.cpp
 * @brief Implementation of the StereoIMUSlamNode Wrapper class.
 */

#include "stereo-imu-slam-node.hpp"

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM3_Wrapper
{
    using namespace WrapperTypeConversions;

    StereoIMUSlamNode::StereoIMUSlamNode(const std::string &strVocFile,
                                   const std::string &strSettingsFile,
                                   ORB_SLAM3::System::eSensor sensor)
        : SlamNodeBase("ORB_SLAM3_STEREO_IMU_ROS2", strVocFile, strSettingsFile, sensor)
    {
        // 上面都是初始化了部分的话题发布者
        // 下面的都是设置ROS2的话题订阅， 并设置回调函数为SLAM的双目回调
        // Declare parameters (topic names)
        this->declare_parameter("left_image_topic_name", rclcpp::ParameterValue("left/image_raw"));
        this->declare_parameter("right_image_topic_name", rclcpp::ParameterValue("right/image_raw"));
        this->declare_parameter("imu_topic_name", rclcpp::ParameterValue("imu"));

        // ROS Subscribers
        leftSub_  = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, this->get_parameter("left_image_topic_name").as_string());
        rightSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, this->get_parameter("right_image_topic_name").as_string());
        syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *leftSub_, *rightSub_);
        syncApproximate_->registerCallback(&StereoIMUSlamNode::StereoCallback, this);

        // IMU subscriber in an independent callback group.
        imuCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions imuSubOptions;
        imuSubOptions.callback_group = imuCallbackGroup_;

        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic_name").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(&StereoIMUSlamNode::ImuCallback, this, std::placeholders::_1),
            imuSubOptions);

        RCLCPP_INFO(this->get_logger(), "Stereo IMU node started. Left: %s | Right: %s | IMU: %s",
                    this->get_parameter("left_image_topic_name").as_string().c_str(),
                    this->get_parameter("right_image_topic_name").as_string().c_str(),
                    this->get_parameter("imu_topic_name").as_string().c_str());
    }

    StereoIMUSlamNode::~StereoIMUSlamNode()
    {
        leftSub_.reset();
        rightSub_.reset();
        syncApproximate_.reset();
        imuSub_.reset();
        RCLCPP_INFO(this->get_logger(), "Stereo-IMU node stopped.");
    }

    void StereoIMUSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgImu)
    {
        std::lock_guard<std::mutex> lock(imuMutex_);
        imuBuf_.push(msgImu);
    }

    void StereoIMUSlamNode::StereoCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft,
                                        const sensor_msgs::msg::Image::SharedPtr msgRight)
    {
        cv_bridge::CvImageConstPtr cvLeft;
        cv_bridge::CvImageConstPtr cvRight;

        try
        {
            cvLeft = cv_bridge::toCvShare(msgLeft);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception left!" << std::endl;
            return;
        }

        try
        {
            cvRight = cv_bridge::toCvShare(msgRight);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception right!" << std::endl;
            return;
        }

        // 提取 IMU 数据
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            std::lock_guard<std::mutex> lock(imuMutex_);
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                // 【错误修正 2】这里之前是 msgRGB 和 msgD，改为 msgLeft 和 msgRight
                const double tFrame = std::min(stampToSec(msgLeft->header.stamp), stampToSec(msgRight->header.stamp));
                
                while (!imuBuf_.empty() && stampToSec(imuBuf_.front()->header.stamp) <= tFrame)
                {
                    double t = stampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
        }
        if (vImuMeas.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No IMU measurements between two RGBD frames!");
            return;
        }


        // 注意当前是继承了 SLAMNode-Base 类的
        const double tFrame = std::min(stampToSec(msgLeft->header.stamp), stampToSec(msgRight->header.stamp));

        // 为ORBSLAM3提供传感器的输入
        // 逻辑是调用父类的方法interface()返回的指针，然后再调用slam()返回的指针实际上就是ORBSLAM3暴露的接口，然后调用SLAM中的追踪双目
        auto Tcw = interface()->slam()->TrackStereo(cvLeft->image, cvRight->image, tFrame, vImuMeas);

        if (interface()->processTrackedPose(Tcw))
        {
            this->onTracked(msgLeft->header);
        }
    }
} // namespace ORB_SLAM3_Wrapper

