#ifndef LIGHT_AND_ARMOR_STRUCT
#define LIGHT_AND_ARMOR_STRUCT
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <opencv2/core/types.hpp>
#include <vector>
struct Light{
    explicit Light(const cv::RotatedRect& rect)
    {
        cv::Point2f p[4];  //灯条四个点
        rect.points(p);  //rotatedrect的points函数可以获取四个点的坐标
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
        
        top = (p[0] + p[1]) / 2;//灯条顶部和底部中心点
        bottom = (p[2] + p[3]) / 2;

        this->center = rect.center;
        this->width = cv::norm(p[1]-p[0]);
        this->length = cv::norm(top - bottom);

    }

    cv::Point2f top, bottom;
    cv::Point2f center;

    double length;
    double width;
    enum Color { Red, Blue } color;
};



struct Armor{
    Armor(const Light& light1, const Light& light2):
        left(light1), right(light2)
    {
        if(light1.center.x>light2.center.x) std::swap(left, right);
        Lightcorners.resize(4);
        Lightcorners[0] = left.top;
        Lightcorners[1] = right.top;
        Lightcorners[2] = right.bottom;
        Lightcorners[3] = left.bottom;
    }
    Light left, right;
    double confidence;
    std::vector<cv::Point2f> Lightcorners; //装甲板四个顶点
    enum Type : int {base    = 0, hero     = 1, two   = 2,
                     three   = 3, four     = 4, guard = 5,
                     outpost = 6, negative = 7} type; 
};

struct ArmorPosi{
    cv::Point3d posi;
    cv::Point3d face;
    cv::Point3d toward;
    Armor::Type type;

    ArmorPosi(cv::Point3d posi, cv::Point3d face, cv::Point3d toward, Armor::Type type):
              posi(posi), face(face), toward(toward), type(type){}
};
#endif
