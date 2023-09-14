#ifndef _PID_H
#define _PID_H

#include <cmath>
#include <iostream>

class PID {
    public:
    PID();
    ~PID()=default;
    void PID_Position_init(double out_max,double i_max, double kp, double ki, double kd);
    void PID_Incremental_init(double out_max,double i_max, double kp, double ki, double kd);
    float PID_Incremental_calc(float ref, float fdb);
    float PID_Position_calc(float ref, float fdb);
    float LIMIT_MIN_MAX(float value,float min,float max);

    private:
    float Kp;//比例系数
    float Ki;//积分系数
    float Kd;//微分系数
    float Ref;//目标值
    float Fdb;//实际值
    float err[3];//误差
    float integral;//积分项

    float OUT_MAX;
    float I_MAX;

    float p_out;
    float i_out;
    float d_out;
    float output;
    float delta_output;

}
#endif