#include "vid2bag.h"
#include "ros/ros.h"
#include "filesystem"
#include "argparse.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vid2bag_prog_node");

    argparse::ArgumentParser prog("vid2bag");
    prog.add_argument("video").help("the path of the video");
    prog.add_argument("--output", "-o").help("the path of the rosbag to output").default_value("");
    prog.add_argument("--scale", "-s").help("the scale rate of image frames, range: (0.0, 1.0]").default_value(
            1.0f).scan<'f', float>();
    prog.add_argument("--gray", "-g").implicit_value(true).default_value(false).help(
            "convert color images to gray ones if they are");
    prog.add_argument("--flip", "-f").help(
            "flip option for the image frames, around x-axis (0), y-axis (1), both axis (-1), no flip (2)")
            .default_value(2).scan<'d', int>();
    try {
        prog.parse_args(argc, argv);
        // get args
        auto vidPath = prog.get<std::string>("video");
        if (!std::filesystem::exists(vidPath)) {
            throw std::runtime_error("no video named '" + vidPath + "' exists");
        } else {
            vidPath = std::filesystem::canonical(vidPath);
        }

        auto bagPath = prog.get<std::string>("--output");
        if (bagPath.empty()) {
            // path not given
            bagPath = std::filesystem::path(vidPath).replace_extension(".bag");
        } else {
            auto rootPath = std::filesystem::path(bagPath).parent_path();
            if (!exists(rootPath)) {
                if (!std::filesystem::create_directories(rootPath)) {
                    // create path failed
                    bagPath = std::filesystem::path(vidPath).replace_extension(".bag");
                } else {
                    // create path succeed
                    bagPath = std::filesystem::canonical(rootPath).string() + "/" +
                              std::filesystem::path(bagPath).filename().string();
                }
            } else {
                // path exists
                bagPath = std::filesystem::canonical(rootPath).string() + "/" +
                          std::filesystem::path(bagPath).filename().string();
            }
        }

        auto scale = prog.get<float>("--scale");
        if (scale <= 0.0 || scale > 1.0) {
            throw std::runtime_error("the input scale is out of range (0.0, 1.0]");
        }

        auto toGrayImg = prog.get<bool>("--gray");

        auto flip = prog.get<int>("--flip");
        if(!(flip == 0 || flip == 1 || flip == -1 || flip == 2)) {
            throw std::runtime_error("the input flip is incorrect, value has to be 0 or 1 or -1");
        }

        // display
        std::cout << "  the input video path: '" << vidPath << "'" << std::endl;
        std::cout << "the output rosbag path: '" << bagPath << "'" << std::endl;
        std::cout << "  the scale for images: '" << scale << "'" << std::endl;
        std::cout << "convert images to gray: '" << std::boolalpha << toGrayImg << "'" << std::endl;
        auto flipStr = (flip != 2) ? ((flip == 0) ? "around x-axis" : ((flip == 1) ? "around y-axis" : "around both axis")) : "no flipping";
        std::cout << "  flip images: " << flipStr << std::endl;

        // process
        auto res = ns_v2b::Vid2Bag::Create(vidPath, bagPath, scale, toGrayImg, flip)->Process();

        if (res.first) {
            std::cout << "Process finished! See the output rosbag named '" << bagPath << "'." << std::endl;
        } else {
            throw std::runtime_error("Process failed! Info: '" + res.second + "'.");
        }
    }
    catch (const std::runtime_error &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << prog;
    }
    ros::shutdown();
    return 0;
}