#ifndef __PREDICTOR_H
#define __PREDICTOR_H

#include <iostream>

class Predictor{
public:

    Predictor();
    ~Predictor();

    double yaw_last;
    double w_last;
    double delta_t;

    int iters;
    double yaw_predict(double yaw_camera, double yaw_ptz, double frequency);
    double pitch_predict(double pitch_camera, double pitch_ptz, double dis, float v);

};
#endif