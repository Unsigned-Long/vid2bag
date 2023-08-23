#include "vid2bag.h"
#include "ros/ros.h"
#include "filesystem"
#include "argparse.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vid2bag_prog_node");

    argparse::ArgumentParser prog("vid2bag_argparse");
    prog.add_argument("video").help("the path of the video");
    prog.add_argument("--output", "-o").help("the path of the rosbag to output").default_value("");
    prog.add_argument("--scale", "-s").help("the scale rate of image frames, range: (0.0, 1.0]").default_value(
            1.0f).scan<'f', float>();

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
            bagPath = std::filesystem::path(vidPath).replace_extension(".bag");
        } else {
            if (auto rootPath = std::filesystem::path(bagPath).parent_path();
                    !exists(rootPath) && !std::filesystem::create_directories(rootPath)) {
                bagPath = std::filesystem::path(vidPath).replace_extension(".bag");
            } else {
                bagPath = std::filesystem::canonical(std::filesystem::path(bagPath).parent_path()).string() + "/" +
                          std::filesystem::path(bagPath).filename().string();
            }
        }

        auto scale = prog.get<float>("--scale");
        if (scale <= 0.0 || scale > 1.0) {
            throw std::runtime_error("the input scale is out of range (0.0, 1.0]");
        }

        // display
        std::cout << "  the input video path: '" << vidPath << "'" << std::endl;
        std::cout << "the output rosbag path: '" << bagPath << "'" << std::endl;
        std::cout << "  the scale for images: '" << scale << "'" << std::endl;
    }
    catch (const std::runtime_error &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << prog;
    }
    ros::shutdown();
    return 0;
}