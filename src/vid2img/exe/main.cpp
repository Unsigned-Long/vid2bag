#include "vid2img.h"
#include "ros/ros.h"
#include "filesystem"
#include "argparse.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vid2bag_prog_node");

    argparse::ArgumentParser prog("vid2bag");
    prog.add_argument("video").help("the path of the video");
    prog.add_argument("--output", "-o").help("the folder path to output images").default_value("");
    prog.add_argument("--scale", "-s").help("the scale rate of image frames, range: (0.0, 1.0]").default_value(
            1.0f).scan<'f', float>();
    prog.add_argument("--gray", "-g").implicit_value(true).default_value(false).help(
            "convert color images to gray ones if they are");

    try {
        prog.parse_args(argc, argv);
        // get args
        auto vidPath = prog.get<std::string>("video");
        if (!std::filesystem::exists(vidPath)) {
            throw std::runtime_error("no video named '" + vidPath + "' exists");
        } else {
            vidPath = std::filesystem::canonical(vidPath);
        }

        auto imgPath = prog.get<std::string>("--output");
        if (imgPath.empty()) {
            // path not given
            imgPath = std::filesystem::path(vidPath).parent_path().string() + "/images";
            std::filesystem::create_directories(imgPath);
        } else {
            if (!std::filesystem::exists(imgPath)) {
                if (!std::filesystem::create_directories(imgPath)) {
                    // create path failed
                    imgPath = std::filesystem::path(vidPath).parent_path().string() + "/images";
                    std::filesystem::create_directories(imgPath);
                } else {
                    // create path succeed
                    imgPath = std::filesystem::canonical(imgPath).string();
                }
            } else {
                // path exists
                imgPath = std::filesystem::canonical(imgPath).string();
            }
        }

        auto scale = prog.get<float>("--scale");
        if (scale <= 0.0 || scale > 1.0) {
            throw std::runtime_error("the input scale is out of range (0.0, 1.0]");
        }

        auto toGrayImg = prog.get<bool>("--gray");

        // display
        std::cout << "  the input video path: '" << vidPath << "'" << std::endl;
        std::cout << "the output images path: '" << imgPath << "'" << std::endl;
        std::cout << "  the scale for images: '" << scale << "'" << std::endl;
        std::cout << "convert images to gray: '" << std::boolalpha << toGrayImg << "'" << std::endl;

        // process
        auto res = ns_v2i::Vid2Img::Create(vidPath, imgPath, scale, toGrayImg)->Process();

        if (res.first) {
            std::cout << "Process finished! See the output rosbag named '" << imgPath << "'." << std::endl;
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