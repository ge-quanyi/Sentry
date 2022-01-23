#include "predictor.h"
#include <math.h>

Predictor::Predictor(){
    yaw_last = 0;
    w_last = 0;
    delta_t = 1 / 120;
    iters = 20;
}

Predictor::~Predictor(){
    
}

/***********************************************
 * @descrip yaw预测
 * @param
 * @return
 * @date
 *************************************************/

double Predictor::yaw_predict(double yaw_camera, double yaw_ptz, double frequency){
    float t;
    if(frequency > 0)
        t = 1 / frequency;
    else
        t = 0.000001;

    double yaw_now = yaw_camera + yaw_ptz;
    double yaw_delta = yaw_now - yaw_last;

    float w = yaw_delta / t;
    float a = (w - w_last) / t;

    double yaw_pre = w * delta_t + 0.5 * a * delta_t * delta_t ;

    yaw_last = yaw_now;
    w_last = w;

    return yaw_pre;
    
}

/***********************************************
 * @descrip 弹道补偿
 * @param
 * @return
 * @date
 *************************************************/
 #define PI 3.1415
double Predictor::pitch_predict(double pitch_camera, double pitch_ptz, double dis, float v){
    
    double angle = pitch_ptz + pitch_camera;
    double d_real = dis * cos(angle * PI / 180);
    double height = dis * sin(angle * PI / 180);
    

    for(int i = 0;i < iters; i++){
        double t = (sqrt(v*sin(angle* PI / 180)*v*sin(angle* PI / 180) + 2*9.8*height) - v*sin(angle* PI / 180)) / 9.8;
        double d_now = v*cos(angle* PI / 180) * t;
        double loss = d_now - d_real;

        if(abs(loss) <= 0.1)
            return angle;
        double dis_n = sqrt(height*height + d_now*d_now);

        double delta_pitch = acos((dis_n*dis_n + dis*dis - loss*loss) / (2 * dis *dis_n));
        delta_pitch = delta_pitch * 180 / PI;
        if(loss > 0){
            delta_pitch = delta_pitch;
        }else{
            delta_pitch = -delta_pitch;
        }

        angle = angle + delta_pitch;
        if(angle < 0)
            angle = 0;
        std::cout<<"loss:"<<loss<<" "
                 <<"delta_angle:"<<delta_pitch<<" "
                 <<"angle:"<<angle<<std::endl;
    }
}