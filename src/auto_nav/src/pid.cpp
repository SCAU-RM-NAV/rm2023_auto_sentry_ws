#include "pid.h"

PID::PID()
{

}

float PID::LIMIT_MIN_MAX(float value,float min,float max)
{
    if(value>max)
        return max;
    else if (value<min)
        return min;
    else   
        return value;
}

void PID::PID_Position_init(double out_max,double i_max, double kp, double ki, double kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    OUT_MAX =out_max;
    I_MAX = i_max;
}

void PID::PID_Incremental_init(double out_max,double i_max, double kp, double ki, double kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    OUT_MAX =out_max;
    I_MAX = i_max;
}

float PID::PID_Position_calc(float ref, float fdb)
{
    Ref = ref;
    Fdb = fdb;
    err[1] = err[0];
    err[0] = Ref - Fdb;

    p_out = Kp *err[0];
    i_out += Ki *err[0];
    d_out = Kd *(err[0]-err[1]);
    LIMIT_MIN_MAX(i_out,-I_MAX,I_MAX);

    output = p_out + i_out + d_out;
    LIMIT_MIN_MAX(output,-OUT_MAX,OUT_MAX);
    return output;
}

float PID::PID_Incremental_calc(float ref, float fdb)
{
    Ref = ref;
    Fdb = fdb;
    err[2] = err[1];
    err[1] = err[0];
    err[0] = Ref - Fdb;
    p_out = Kp*(err[0]-err[1]);
    i_out = Ki*err[0];
    d_out = Kd*(err[0]-2*err[1]+err[2]);
    LIMIT_MIN_MAX(i_out,-I_MAX,I_MAX);

    delta_output = p_out + i_out + d_out;
    LIMIT_MIN_MAX(delta_output,-OUT_MAX,OUT_MAX);
    return delta_output;

}