#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <queue>
#include <thread>
#include <mutex>
#include <unistd.h>
#include "camera_api.h"
#include "tic_toc.h"
#include "armour.h"
#include "serial_port.h"
#include "Classifier.h"
#include "common.h"

bool time_flag_0 = false;
std::chrono::time_point<std::chrono::system_clock> start_0, end_0;

#define image_width 960//960
#define image_height 768//768
#define red_color 1
#define blue_color 2
char *pRGB24Buf_0 = new char[image_width * image_height * 3];
int cnt = 0;
std::queue<pair<std::chrono::time_point<std::chrono::system_clock>, cv::Mat>> img_buf;
std::mutex mImg_buf;

extern float armour_big_pt[5][2];
extern float armour_small_pt[5][2];

SerialPort port;


/**********************************
 * @descrip 回调采集
 * @param pFrame
 * @return
 * @date
 */
void Frame_0_ProcessRGB(GX_FRAME_CALLBACK_PARAM *pFrame) {

    if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
        cnt++;
        if (cnt > 10)
            cnt = 0;
        if (cnt != 1)
            return;
        start_0 = std::chrono::system_clock::now();
        /* if(!time_flag_0)
         {
             end_0 = start_0;
             time_flag_0 = true;
         }
         else
         {
             std::chrono::duration<double> elapsed_seconds = start_0 - end_0;
             std::cout << "the CAM_0 frame frequency is: " << 1/elapsed_seconds.count() << std::endl;
             end_0 = start_0;
         }*/
        //TicToc tic;
        //        printf("Image Time Stamp %ld\n",pFrame->nTimestamp);
        //        printf("Image Width %d\n",pFrame->nWidth);
        //        printf("Image Height %d\n",pFrame->nHeight);
        //        printf("Image_0 Frame ID %ld \n",pFrame->nFrameID);
        /*char *pRGB24Buf = new char[pFrame->nWidth * pFrame->nHeight * 3];           //输出图像RGB数据
        if (pRGB24Buf == NULL)
            return  ;
        else
            memset(pRGB24Buf,0,pFrame->nWidth * pFrame->nHeight * 3 * sizeof(char));    //缓冲区初始化*/

        memset(pRGB24Buf_0, 0, pFrame->nWidth * pFrame->nHeight * 3 * sizeof(char));    //缓冲区初始化*/
        DX_BAYER_CONVERT_TYPE cvtype = RAW2RGB_NEIGHBOUR;           //选择插值算法
        DX_PIXEL_COLOR_FILTER nBayerType = BAYERBG;                   //选择图像Bayer格式
        bool bFlip = false;
        VxInt32 DxStatus = DxRaw8toRGB24(const_cast<void *>(pFrame->pImgBuf), pRGB24Buf_0, pFrame->nWidth,
                                         pFrame->nHeight, cvtype, nBayerType, bFlip);
        if (DxStatus != DX_OK) {
            /*if (pRGB24Buf != NULL)
            {
                delete []pRGB24Buf;
                pRGB24Buf = NULL;
            }*/
            return;
        }
        cv::Mat image_rgb24(pFrame->nHeight, pFrame->nWidth, CV_8UC3);
        memcpy(image_rgb24.data, pRGB24Buf_0, pFrame->nHeight * pFrame->nWidth * 3);
        //cv::Mat image_bgr;
        //cv::cvtColor(image_rgb24,image_bgr,CV_RGB2BGR);
        mImg_buf.lock();
        img_buf.push(make_pair(start_0, image_rgb24));
        mImg_buf.unlock();
        //cv::imshow("img_show: ",image_rgb24);
        //cv::waitKey(1);
        /*if (pRGB24Buf != NULL)
        {
            delete []pRGB24Buf;
            pRGB24Buf = NULL;
        }*/
        //std::cout << "the time of read image_0 is: " << tic.toc() << std::endl;
    }
    return;
}

/***********************************************
 * @descrip 自瞄线程
 * @param
 * @return
 * @date
 *************************************************/
//#define SAVING_IMG

[[noreturn]] void armour_detect_process() {
    Armour armour;
    Classifier classifier;
    if (!armour.readCameraParameters("../config/cam_param.yaml")) {
        std::cout << "read camera param fail ..." << std::endl;
        return;
    }

#ifdef SAVING_IMG
    FileStorage fs1(PROJECT_DIR"/data/img_list.xml",FileStorage::WRITE);
    fs1 << "imgs"<<"[";
    int img_num = 0;
#endif
    uint8_t tg_num = 0;

    double ang_P_last = 0;
    double ang_Y_last = 0;
    int find_flag_last = 0;
    int autoaim = 0;
    int autoaim_fps = 0;

    ///auto aim loop
    while (1) {
        cv::Mat img;
        std::chrono::time_point<std::chrono::system_clock> t1;
        mImg_buf.lock();
        if (img_buf.size() > 0) {
            //std::cout << "the img_buf size: " << img_buf.size() << std::endl;
            t1 = img_buf.front().first;
            img = img_buf.front().second.clone();
            img_buf.pop();
        }
        mImg_buf.unlock();
        if (!img.data)
            continue;

#ifdef SAVING_IMG
        char key = waitKey(10);

        if (key == 'S' || key == 's') {
            img_num++;
            char Filename[100];
            sprintf(Filename, PROJECT_DIR"/data/%d.jpg", img_num);
            char Filename2[50];
            sprintf(Filename2, "%d.jpg", img_num);
            fs1 << Filename2;
            imwrite("../data/" + to_string(img_num) + ".jpg", img);
            std::cout << "img_num is :" << img_num << std::endl;
        } else if (key == ' ') {
            fs1 << "]";
            fs1.release();
            std::cout << "stop write xml" << std::endl;
        }
        //resize(img, img, Size(0, 0), 0.8, 0.8);
        imshow("capture", img);
        continue;
#endif
        cv::Mat img_bin;
        if (port.receive[2] == 'b') {   /// 我方是蓝色，寻找红色
            armour.img_pretreatment(img, img_bin, blue_color);
        }
        if (port.receive[2] == 'r') {
            armour.img_pretreatment(img, img_bin, blue_color);
        }

        cv::Point2f tg_pt_L, tg_pt_R;
        ///get result
        cv::RotatedRect tg_rect = armour.Armor_Detector(img_bin, tg_pt_L, tg_pt_R); //get armour area
        double ang_P, ang_Y, Dis;
        if (tg_pt_L.x >= 0) {
            armour.corners.clear();
            tg_num = 0;
            ///add armour id
            //if(id > 0)
            tg_num = 1;

            if (tg_num > 0) {

                autoaim = 1;
                find_flag_last = 1;

                armour.draw_target(tg_rect, img);
                float armour_width = tg_rect.boundingRect().width;
                float armour_height = tg_rect.boundingRect().height;
                Point3f tmp;
                if (armour_width / armour_height > 3.5) {
                    std::cout << "big armour" << std::endl;
                    for (int i = 0; i < 5; i++) {
                        tmp.x = armour_big_pt[i][0];
                        tmp.y = armour_big_pt[i][1];
                        tmp.z = 0;
                        armour.corners.push_back(tmp);
                    }
                    //std::cout<<armour.corners<<std::endl;
                } else {
                    std::cout << "small armour" << std::endl;
                    for (int i = 0; i < 5; i++) {
                        tmp.x = armour_small_pt[i][0];
                        tmp.y = armour_small_pt[i][1];
                        tmp.z = 0;
                        armour.corners.push_back(tmp);
                    }
                    //std::cout<<armour.corners<<std::endl;
                }
                armour.cal_angle(tg_pt_L, tg_pt_R, ang_P, ang_Y, Dis);
                ang_P_last = ang_P;
                ang_Y_last = ang_Y;
            } else {
                tg_num = 0;
                ang_P = ang_Y = Dis = 0;
            }
        } else {
            tg_num = 0;
            ang_P = ang_Y = Dis = 0;
        }

        if (tg_num == 0 && find_flag_last == 1) {
            autoaim_fps++;
        }
        if (autoaim_fps < 250 && autoaim_fps > 0) {
            autoaim_fps++;
            autoaim = 1;
        } else
            autoaim_fps = 0;

        if (tg_num == 0 && find_flag_last == 1) {
            ang_P = ang_P_last;
            ang_Y = ang_Y_last;
            find_flag_last = 0;
            autoaim = 1;
            std::cout << "loss" << std::endl;
        }

        char cmd;
        if (autoaim > 0 && armour.ifShoot(ang_P, ang_Y))
            cmd = 0x01;   //打弹  16
        else
            cmd = 0x00;   //不打  32

        *(signed char *) &port.buff_w_[0] = int16_t((100 * (ang_P))) >> 8;
        *(signed char *) &port.buff_w_[1] = int16_t(100 * (ang_P));
        *(signed char *) &port.buff_w_[2] = int16_t((100 * (ang_Y))) >> 8;
        *(signed char *) &port.buff_w_[3] = int16_t(100 * (ang_Y));
        *(signed char *) &port.buff_w_[4] = int16_t(100 * Dis) >> 8;
        *(signed char *) &port.buff_w_[5] = int16_t(100 * Dis);
        *(signed char *) &port.buff_w_[6] = int8_t(autoaim);   //是否开启自瞄模式
        *(signed char *) &port.buff_w_[7] = int8_t(cmd);
        port.SendBuff('c', port.buff_w_, 8);//std::cout <<"send success!!!"<< std::endl;

        //resize(img,img,Size(0,0),0.8,0.8);
        imshow("src", img);
        cv::waitKey(1);

        std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
        std::chrono::duration<double> dura = t2 - t1;
        std::cout << "the frame frequency is: " << 1 / dura.count() << std::endl;
        std::cout << "   === ang_P: " << ang_P
                  << "   === ang_Y: " << ang_Y
                  << "   === Dis: " << Dis
                  // << "   === ID: " << id
                  << "   === mode: " << autoaim
                  << "   === if shoot:" << to_string(cmd)
                  << std::endl;
    }
}

#define USB_CAM
int main() {

#ifndef USB_CAM
    GX_STATUS status = Config();
    if (status != GX_STATUS_SUCCESS) {
        std::cout << "config Camera Faile ..." << std::endl;
        return 0;
    }
    camera_config cam0_info;
    //cam0_info.sn_str = "KE0200100062";  //sentry down
    cam0_info.sn_str = "KE0200120159";  //sentry up
    cam0_info.SN = &cam0_info.sn_str[0];

    MercureDriver *cam0 = new MercureDriver(cam0_info);
    cam0->InitCamera();

    if (cam0->status != GX_STATUS_SUCCESS) {
        std::cout << "Initial Camera Faile ..." << std::endl;
        return 0;
    }

    status = GXRegisterCaptureCallback(cam0->hDevice_, NULL, Frame_0_ProcessRGB);
    status = GXSendCommand(cam0->hDevice_, GX_COMMAND_ACQUISITION_START);

    if (status != GX_STATUS_SUCCESS) {
        std::cout << "Cam0 Start Read Faile ..." << std::endl;
        return 0;
    }
#endif



    ///串口初始化
    //while (!port.PortInit(0, 115200));
    ///线程
    //std::thread serial_receive_thread(port_receive);
    std::thread process_armour_detect{armour_detect_process};

    while (1) {
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
    return 0;
}
