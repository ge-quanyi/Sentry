#include "armour.h"

float armour_small_pt[5][2] = {
        0.0, 0.0,
        0.0675, 0.0,
        0.0, 0.0670,
        -0.0675, 0.0,
        0.0, -0.0670,
};
float armour_big_pt[5][2] = {
        0.0,0.0,       //世界坐标系原点
        0.1125,0.0,    //右
        0.0,0.1120,    //上
        -0.1125,0.0,   //左
        0.0,-0.1120,   //下
};
Armour::Armour() {


}


Armour::~Armour() {

}


bool Armour::readCameraParameters(string filename) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

void Armour::img_pretreatment(const cv::Mat &src, cv::Mat &dst, int team) {

    int gray_threshold_red  = 60;
    int separation_threshold_red = 180;
    int separation_threshold_green = 10;

    int gray_threshold_blue  = 60;
    int separation_threshold_blue = 60;

    int gray_threshold_purple = 200;

    std::vector<cv::Mat> src_split_;
    cv::Mat src_gray_, src_white_,src_separation_,src_separation_green;
    cv::split(src,src_split_);
    cv::cvtColor(src, src_gray_,CV_BGR2GRAY);
    cv::threshold(src_gray_, src_white_, 240, 255, cv::THRESH_BINARY);
    cv::bitwise_not(src_white_, src_white_);

    Mat element3 = getStructuringElement(MORPH_RECT,Size(3,5));
    Mat element5 = getStructuringElement(MORPH_RECT,Size(5,7));
    Mat element7 = getStructuringElement(MORPH_RECT,Size(7,9));

    if(team == 2){ //red
        cv::threshold(src_gray_, src_gray_, gray_threshold_red, 255, cv::THRESH_BINARY);
        cv::subtract(src_split_[2],src_split_[0],src_separation_);
        cv::threshold(src_separation_, src_separation_, separation_threshold_red,255, cv::THRESH_BINARY);

        cv::dilate(src_separation_,src_separation_,element3);
        cv::dilate(src_separation_,src_separation_,element7);

        cv::dilate(src_white_,src_white_,element3);
        cv::dilate(src_white_,src_white_,element3);
        dst = src_separation_ & src_gray_ &src_white_;

        cv::erode(dst,dst,element3);
        cv::dilate(dst, dst, element3);
        cv::dilate(dst,dst,element5);
        cv::erode(dst,dst,element5);

    }
    else{
        Mat src_purple_;
        cv::threshold(src_split_[2], src_purple_, gray_threshold_purple, 255, cv::THRESH_BINARY);
        cv::bitwise_not(src_purple_, src_purple_);
        cv::threshold(src_gray_, src_gray_, gray_threshold_blue, 255, cv::THRESH_BINARY);
        cv::subtract(src_split_[0],src_split_[2],src_separation_);
        cv::threshold(src_separation_, src_separation_, separation_threshold_blue,255, cv::THRESH_BINARY);

        //cv::dilate(src_separation_,src_separation_,element3);
        cv::dilate(src_separation_,src_separation_,element7);
        cv::erode(src_separation_,src_separation_,element3);

        dst = src_separation_ &src_gray_ ;
        erode(dst,dst,element3);
        dilate(dst,dst,element3);
        dilate(dst,dst,element7);
        erode(dst,dst,element7);
        //erode(dst,dst,element3);
    }

    //imshow("grey",src_gray_);
    // imshow("separation",src_separation_);
    //imshow("white",src_white_);
    // resize(dst,dst,Size(0,0), 0.5,0.5);
     //imshow("dst", dst);
    //waitKey(1);
}


cv::RotatedRect Armour::Armor_Detector(const Mat &src, cv::Point2f &tg_pt_L, cv::Point2f &tg_pt_R) {
    tg_pt_L = cv::Point2f(-1.0, -1.0);
    tg_pt_R = cv::Point2f(-1.0, -1.0);
    RotatedRect temp_rota_rect = RotatedRect(cv::Point2f(-1, -1), cv::Size2f(0, 0), 0);
    cv::RotatedRect lightBar, lightBar_fitEllipse, lightBar_minAreaRect;
    std::vector<cv::RotatedRect> v_lightBar;
    Mat src_contours;
    src_contours = src.clone();
    vector<vector<Point>> contours;
    vector<Vec4i> white_hierarchy;
    findContours(src_contours, contours, white_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    for (unsigned int i = 0; i < contours.size(); i++) {
        //std::cout<<"contours size is"<<contours[i].size()<<std::endl;
        if (contours[i].size() < 20)
            continue;
        lightBar_fitEllipse = fitEllipse(contours[i]);
        lightBar_minAreaRect = minAreaRect(contours[i]);

        lightBar.angle = lightBar_fitEllipse.angle;
        lightBar.center = lightBar_fitEllipse.center;
        if (lightBar_minAreaRect.size.width > lightBar_minAreaRect.size.height) {
            lightBar.size.height = lightBar_minAreaRect.size.width;
            lightBar.size.width = lightBar_minAreaRect.size.height;
        } else {
            lightBar.size.height = lightBar_minAreaRect.size.height;
            lightBar.size.width = lightBar_minAreaRect.size.width;
        }

        if ((lightBar.size.width / lightBar.size.height) > 0.8)
            continue;
        int x = lightBar.center.x - lightBar.size.width;
        if (x < 0)
            continue;
        int y = lightBar.center.y - lightBar.size.height;
        if (y < 0)
            continue;
        int w = lightBar.size.width + lightBar.size.width;
        if (w > src.cols - x)
            continue;
        int h = lightBar.size.height + lightBar.size.width;
        if (h > src.rows - y)
            continue;

        if ((lightBar.angle < 60 || lightBar.angle > 120) &&
            (lightBar.size.height > 5) && (lightBar.size.height < 150)) {
            v_lightBar.push_back(lightBar);
        }
    }

    if (v_lightBar.size() < 2)
        return temp_rota_rect;

    cv::RotatedRect temp_armour_rect;
    vector<cv::RotatedRect> v_armour_rects;

    cv::Point2f temp_pt_L;
    vector<cv::Point2f> v_Pts_L;

    cv::Point2f temp_pt_R;
    vector<cv::Point2f> v_Pts_R;

    for (unsigned int i = 0; i < v_lightBar.size() - 1; i++) {
        for (unsigned int j = i + 1; j < v_lightBar.size(); j++) {
            double height_diff = abs(v_lightBar[i].size.height - v_lightBar[j].size.height);//高度差
            double height_sum = v_lightBar[i].size.height + v_lightBar[j].size.height;//高度和
            double width_diff = abs(v_lightBar[i].size.width - v_lightBar[j].size.width);//宽度差
            double width_sum = v_lightBar[i].size.width + v_lightBar[j].size.width;//宽度和
            double angle_diff = fabs(v_lightBar[i].angle - v_lightBar[j].angle);//角度差
            double Y_diff = abs(v_lightBar[i].center.y - v_lightBar[j].center.y);//纵坐标差值
            double MH_diff = (min(v_lightBar[i].size.height, v_lightBar[j].size.height)) * 2 / 3;//高度差限幅
            double height_max = (max(v_lightBar[i].size.height, v_lightBar[j].size.height));//最大高度
            double X_diff = abs(v_lightBar[i].center.x - v_lightBar[j].center.x);//横坐标差值
            double lightBar_dis = sqrt((v_lightBar[i].center.x - v_lightBar[j].center.x)*(v_lightBar[i].center.x - v_lightBar[j].center.x)
                                       - (v_lightBar[i].center.y - v_lightBar[j].center.y)*(v_lightBar[i].center.y - v_lightBar[j].center.y));

            if (Y_diff < height_max && X_diff < MH_diff * 10 &&
                (angle_diff < 5 || 180 - angle_diff < 3) &&
                /*lightBar_dis / v_lightBar[i].size.height >0.5 &&*/
                height_diff / height_sum < 0.5 &&
                width_diff / width_sum < 0.4  &&
                X_diff / MH_diff > 2)//还可以加入高度差限制
            {
                temp_armour_rect.center.x = ((v_lightBar[i].center.x + v_lightBar[j].center.x) / 2);
                temp_armour_rect.center.y = ((v_lightBar[i].center.y + v_lightBar[j].center.y) / 2);
                temp_armour_rect.angle = (v_lightBar[i].angle + v_lightBar[j].angle) / 2;

                if (v_lightBar[i].center.x < v_lightBar[j].center.x) {
                    temp_pt_L = cv::Point2f(v_lightBar[i].center);
                    temp_pt_R = cv::Point2f(v_lightBar[j].center);
                } else {
                    temp_pt_L = cv::Point2f(v_lightBar[j].center);
                    temp_pt_R = cv::Point2f(v_lightBar[i].center);
                }
                if (180 - angle_diff < 3)
                    temp_armour_rect.angle += 90;
                int nL = (v_lightBar[i].size.height + v_lightBar[j].size.height) / 2; //装甲的高度
                int nW = sqrt((v_lightBar[i].center.x - v_lightBar[j].center.x) *
                              (v_lightBar[i].center.x - v_lightBar[j].center.x) +
                              (v_lightBar[i].center.y - v_lightBar[j].center.y) *
                              (v_lightBar[i].center.y - v_lightBar[j].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
                if (nL < nW) {
                    temp_armour_rect.size.height = nL;
                    temp_armour_rect.size.width = nW;
                } else {
                    temp_armour_rect.size.height = nW;
                    temp_armour_rect.size.width = nL;
                }
                if (Y_diff < nW / 3) {
                    v_armour_rects.push_back(temp_armour_rect);
                    v_Pts_L.push_back(temp_pt_L);
                    v_Pts_R.push_back(temp_pt_R);
                }
            }
        }
    }

    if (v_armour_rects.size() == 1) {
        temp_rota_rect = cv::RotatedRect(v_armour_rects[0]);
        tg_pt_L = cv::Point2f(v_Pts_L[0]);
        tg_pt_R = cv::Point2f(v_Pts_R[0]);
    } else if (v_armour_rects.size() > 1) {
        double min_dis = 10000;
        int min_idx = -1;
        for (int i = 0; i < v_armour_rects.size(); i++) {
            double dis = fabs(v_armour_rects[i].center.x - src.cols / 2) +
                         fabs(v_armour_rects[i].center.y - src.rows / 2);
            if (dis < min_dis) {
                min_dis = dis;
                min_idx = i;
            }
        }
        if (min_idx >= 0) {
            temp_rota_rect = cv::RotatedRect(v_armour_rects[min_idx]);
            tg_pt_L = cv::Point2f(v_Pts_L[min_idx]);
            tg_pt_R = cv::Point2f(v_Pts_R[min_idx]);
        }
    }
    return temp_rota_rect;
}


void Armour::cal_angle(Point2f &tg_pt_L, Point2f &tg_pt_R, double &angle_P, double &angle_Y, double &DIS) {
    const double cx = camMatrix.at<double>(0, 2);
    const double cy = camMatrix.at<double>(1, 2);
    const double fx = camMatrix.at<double>(0, 0);
    const double fy = camMatrix.at<double>(1, 1);
    //std::cout<<"cx:"<<cx<<"cy"<<cy<<"fx"<<fx<<"fy"<<fy<<std::endl;

    //得到pattern-cam坐标
    std::vector<cv::Point2f> observationPts;
    cv::Point2f tg_center;
    tg_center.x = (tg_pt_L.x + tg_pt_R.x) * 0.5;
    tg_center.y = (tg_pt_L.y + tg_pt_R.y) * 0.5;
    observationPts.push_back(tg_center);
    observationPts.push_back(tg_pt_R);

    cv::Point2f temp_pt_1(tg_pt_R.x - tg_center.x, tg_pt_R.y - tg_center.y);
    cv::Point2f temp_pt_2(-temp_pt_1.y, temp_pt_1.x);
    cv::Point2f tg_pt_down(temp_pt_2.x + tg_center.x, temp_pt_2.y + tg_center.y);
    observationPts.push_back(tg_pt_down);
    observationPts.push_back(tg_pt_L);
    cv::Point2f temp_pt_3(tg_pt_L.x - tg_center.x, tg_pt_L.y - tg_center.y);
    cv::Point2f temp_pt_4(-temp_pt_3.y, temp_pt_3.x);
    cv::Point2f tg_pt_up(temp_pt_4.x + tg_center.x, temp_pt_4.y + tg_center.y);
    observationPts.push_back(tg_pt_up);

    Mat rvec, tvec;
    cv::solvePnP(cv::Mat(corners), cv::Mat(observationPts), camMatrix, distCoeffs, rvec, tvec, false);
    //Rodrigues(rvec, r);
    //t = tvec;

    int img_center_x = 960/2;
    int img_center_y = 768/2;
    double Z = tvec.at<double>(2, 0) + 0.12;

    double X = (tg_center.x - cx) * Z / fx;
    double Y = (tg_center.y - cy) * Z / fy - 0.15;

    //double X = (tg_center.x - img_center_x) * Z / fx;
    //double Y = (tg_center.y - img_center_y) * Z / fy ;

    angle_Y = (float) (atan(X / Z) / 3.1415926 * 180);
    angle_P = (float) (atan(Y / Z) / 3.1415926 * 180);
    DIS = Z;
}

void Armour::draw_target(RotatedRect rect, Mat &src) {
    Point2f point[4];
    rect.points(point);

    //Rect rect = rotated.boundingRect();
    //cv::rectangle(src,Point(rect.x,rect.y),Point(rect.x + rect.width,rect.y + rect.height),Scalar(255,0,0),2,LINE_8);
    //cv::rectangle(src,point[1],point[3],Scalar(0,255,0),2,LINE_8);
    for (int i = 0; i < 4; i++) {
        line(src, point[i], point[(i + 1) % 4], Scalar(0, 255, 0), 2);
    }
}
bool Armour::ifShoot(double angle_p, double angle_y) {
    if(abs(angle_p) < 1.0 && abs(angle_y) < 1.0)
        return true;
    else
        return false;
}