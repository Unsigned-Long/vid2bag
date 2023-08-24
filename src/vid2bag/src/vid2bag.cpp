#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv4/opencv2/opencv.hpp"
#include "vid2bag.h"

namespace ns_v2b {

    Vid2Bag::Vid2Bag(std::string videoPath, std::string rosbagPath, float scale, bool toGrayImg)
            : _vidPath(std::move(videoPath)), _bagPath(std::move(rosbagPath)), _scale(scale), _toGray(toGrayImg) {}

    Vid2Bag::Ptr
    Vid2Bag::Create(const std::string &videoPath, const std::string &rosbagPath, float scale, bool toGrayImg) {
        return std::make_shared<Vid2Bag>(videoPath, rosbagPath, scale, toGrayImg);
    }

    std::pair<bool, std::string> Vid2Bag::Process() {
        cv::VideoCapture cap(_vidPath);
        if (!cap.isOpened()) {
            return {false, "the video named '" + _vidPath + "' can not open."};
        }
        auto bag = std::make_unique<rosbag::Bag>();
        bag->open(_bagPath, rosbag::BagMode::Write);

        // timestamp handler
        ros::Time::init();
        auto firStamp = ros::Time::now();
        double timeDelay = 1.0 / cap.get(cv::CAP_PROP_FPS);
        int frameCount = 0;

        std::cout << "reading frames from video and write them to the rosbag..." << std::endl;
        cv::Mat frame;
        while (cap.read(frame)) {
            if (_toGray) {
                cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            }
            if (_scale != 1.0f) {
                cv::resize(
                        frame, frame,
                        cv::Size2f(static_cast<float>(frame.cols) * _scale, static_cast<float>(frame.rows) * _scale)
                );
            }

            cv_bridge::CvImage cvImage;
            cvImage.image = frame;
            cvImage.header.stamp = firStamp + ros::Duration(timeDelay * (frameCount++));
            cvImage.encoding = _toGray ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;

            sensor_msgs::Image sensorImage;
            cvImage.toImageMsg(sensorImage);
            bag->write("cam_img", cvImage.header.stamp, sensorImage);
        }

        bag->close();
        return {true, {}};
    }
}