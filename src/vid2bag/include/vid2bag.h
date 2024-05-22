#ifndef VID2BAG_VID2BAG_H
#define VID2BAG_VID2BAG_H

#include "rosbag/bag.h"

namespace ns_v2b {
    class Vid2Bag {
    public:
        using Ptr = std::shared_ptr<Vid2Bag>;

    private:
        std::string _vidPath;
        std::string _bagPath;
        float _scale;
        bool _toGray;
        int _flip;
    public:
        Vid2Bag(std::string videoPath, std::string rosbagPath, float scale, bool toGrayImg, int flip);

        static Ptr Create(const std::string &videoPath, const std::string &rosbagPath, float scale, bool toGrayImg, int flip);

        std::pair<bool, std::string> Process();
    };
}

#endif //VID2BAG_VID2BAG_H
