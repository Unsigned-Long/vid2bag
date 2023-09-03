#ifndef VID2BAG_VID2IMG_H
#define VID2BAG_VID2IMG_H

#include "rosbag/bag.h"

namespace ns_v2i {
    class Vid2Img {
    public:
        using Ptr = std::shared_ptr<Vid2Img>;

    private:
        std::string _vidPath;
        std::string _imgPath;
        float _scale;
        bool _toGray;
    public:
        Vid2Img(std::string vidPath, std::string imgPath, float scale, bool toGray);

        static Ptr Create(const std::string &videoPath, const std::string &imgPath, float scale, bool toGrayImg);

        std::pair<bool, std::string> Process();
    };
}


#endif //VID2BAG_VID2IMG_H
