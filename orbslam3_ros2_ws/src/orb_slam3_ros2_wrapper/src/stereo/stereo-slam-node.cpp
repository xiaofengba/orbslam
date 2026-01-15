/**
 * @file stereo-slam-node.cpp
 * @brief Implementation of the StereoSlamNode Wrapper class.
 */

#include "stereo-slam-node.hpp"

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM3_Wrapper
{
    using namespace WrapperTypeConversions;

    StereoSlamNode::StereoSlamNode(const std::string &strVocFile,
                                   const std::string &strSettingsFile,
                                   ORB_SLAM3::System::eSensor sensor)
        : SlamNodeBase("ORB_SLAM3_STEREO_ROS2", strVocFile, strSettingsFile, sensor)
    {
        // 上面都是初始化了部分的话题发布者
        // 下面的都是设置ROS2的话题订阅， 并设置回调函数为SLAM的双目回调
        std::cout << "==========" << std::endl;
        // Declare parameters (topic names)
        this->declare_parameter("left_image_topic_name", rclcpp::ParameterValue("left/image_raw"));
        this->declare_parameter("right_image_topic_name", rclcpp::ParameterValue("right/image_raw"));

        // ROS Subscribers
        leftSub_  = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, this->get_parameter("left_image_topic_name").as_string());
        rightSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, this->get_parameter("right_image_topic_name").as_string());
        syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *leftSub_, *rightSub_);
        syncApproximate_->registerCallback(&StereoSlamNode::StereoCallback, this);

        RCLCPP_INFO(this->get_logger(), "Stereo node started. Left: %s | Right: %s",
                    this->get_parameter("left_image_topic_name").as_string().c_str(),
                    this->get_parameter("right_image_topic_name").as_string().c_str());
    }

    StereoSlamNode::~StereoSlamNode()
    {
        leftSub_.reset();
        rightSub_.reset();
        syncApproximate_.reset();
        RCLCPP_INFO(this->get_logger(), "Stereo node stopped.");
    }

    void StereoSlamNode::StereoCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft,
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

        // 注意当前是继承了 SLAMNode-Base 类的
        const double tFrame = std::min(stampToSec(msgLeft->header.stamp), stampToSec(msgRight->header.stamp));

        // 逻辑是调用父类的方法interface()返回的指针，然后再调用slam()返回的指针，然后调用SLAM中的追踪双目
        auto Tcw = interface()->slam()->TrackStereo(cvLeft->image, cvRight->image, tFrame);

        if (interface()->processTrackedPose(Tcw))
        {
            this->onTracked(msgLeft->header);
        }
    }
} // namespace ORB_SLAM3_Wrapper

