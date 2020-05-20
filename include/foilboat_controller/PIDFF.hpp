#ifndef PIDFF_HPP
#define PIDFF_HPP

#include "ros/ros.h"

class PIDFF
{

public:

    struct PIDConfig{
        double kp;
        double ki;
        double kd;
        double kff;
        double imin;
        double imax;
        double min;
        double max;
    };

    PIDFF(double kp, double ki, double kd, double kff, double imin, double imax, double min, double max);
    PIDFF(PIDFF::PIDConfig pidConfig, ros::NodeHandle nh);
    PIDFF();
    ~PIDFF();

    // Initialize the PIDFF with coefficients and limits
    void init(double kp, double ki, double kd, double kff, double imin, double imax, double min, double max);
    void init(PIDConfig pidConfig);

    // Update PID with new error and elapsed time
    double update(double target, double error, double dt);

    // Reset the integrator
    void resetIntegrator();

private:
    // PID params
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double kff{0.0};
    double imin{0.0};
    double imax{0.0};
    double min{0.0};
    double max{0.0};

    // Integral for past values
    double integrator{0.0};

    // Track last error for derivative term
    double lastError{0.0};

    double lastUpdateTime{0.0};
    ros::NodeHandle n;

public:

    // Setters and getters
    void setKP(double kp){
        this->kp = kp;
    }

    void setKI(double ki){
        this->ki = ki;
    }

    void setKD(double kd){
        this->kd = kd;
    }

    void setKFF(double kff){
        this->kff = kff;
    }

    void setIMin(double imin){
        this->imin = imin;
    }

    void setIMax(double imax){
        this->imax = imax;
    }

    void setMin(double min){
        this->min = min;
    }

    void setMax(double max){
        this->max = max;
    }

    double getKP(){
        return kp;
    }

    double getKI(){
        return ki;
    }

    double getKD(){
        return kd;
    }

    double getKFF(){
        return kff;
    }

    double getIMin(){
        return imin;
    }

    double getIMax(){
        return imax;
    }

    double getMin(){
        return min;
    }

    double getMax(){
        return max;
    }
};

#endif