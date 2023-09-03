#include <utility>
#include "../include/vid2img.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv4/opencv2/opencv.hpp"
#include "fmt/format.h"

namespace ns_v2i {

    Vid2Img::Vid2Img(std::string vidPath, std::string imgPath, float scale, bool toGray) : _vidPath(std::move(
            vidPath)), _imgPath(std::move(imgPath)), _scale(scale), _toGray(toGray) {}

    Vid2Img::Ptr
    Vid2Img::Create(const std::string &videoPath, const std::string &imgPath, float scale, bool toGrayImg) {
        return std::make_shared<Vid2Img>(videoPath, imgPath, scale, toGrayImg);
    }

    std::pair<bool, std::string> Vid2Img::Process() {
        cv::VideoCapture cap(_vidPath);
        if (!cap.isOpened()) {
            return {false, "the video named '" + _vidPath + "' can not open."};
        }

        auto totalFrames = cap.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        std::cout << "total frames: " << totalFrames << std::endl;

        // timestamp handler
        ros::Time::init();
        auto firStamp = ros::Time::now();
        double timeDelay = 1.0 / cap.get(cv::CAP_PROP_FPS);
        int frameCount = 0;

        std::ofstream timestamps(_imgPath + "/timestamps.txt");
        timestamps << std::fixed << std::setprecision(6);

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
            std::string filename = fmt::format(
                    "v2i_{:0" + std::to_string((int) log10(totalFrames) + 1) + "}.png", frameCount
            );
            std::string fullPath = _imgPath + "/" + filename;
            std::cout << "output image named: '" << fullPath << "'..." << std::endl;
            cv::imwrite(fullPath, frame);
            auto timestamp = firStamp + ros::Duration(timeDelay * (frameCount++));
            timestamps << filename << ',' << timestamp.toSec() << std::endl;
        }
        timestamps.close();

        return {true, {}};
    }
}