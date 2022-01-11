#ifndef ARMOUR_H
#define ARMOUR_H

#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Armour {
public:
    Armour();

    ~Armour();

    bool readCameraParameters(string filename);

    void img_pretreatment(const cv::Mat &src, cv::Mat &dst, int team);

    cv::RotatedRect Armor_Detector(const cv::Mat &src, cv::Point2f &tg_pt_L, cv::Point2f &tg_pt_R);

    void cal_angle(cv::Point2f &tg_pt_L, cv::Point2f &tg_pt_R,
                   double &angle_P, double &angle_Y, double &DIS);

    void draw_target(RotatedRect rect, Mat &src);

    bool ifShoot(double angle_p,double angle_y);
    cv::Mat camMatrix;
    cv::Mat distCoeffs;

    std::vector<cv::Point3f> corners;


};


#endif