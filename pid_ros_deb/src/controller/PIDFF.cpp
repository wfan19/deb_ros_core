#include <pid_ros_deb/controller/PIDFF.hpp>

PIDFF::PIDFF(double p, double i, double d, double ff, double iMin, double iMax, double Min, double Max)
    : kp(p), ki(i), kd(d), kff(ff), imin(iMin), imax(iMax), min(Min), max(Max)
{
    resetIntegrator();
}

PIDFF::PIDFF(PIDFF::PIDConfig pidConfig, ros::NodeHandle nh)
    : kp(pidConfig.kp), ki(pidConfig.ki), kd(pidConfig.kd), kff(pidConfig.kff), imin(pidConfig.imin), imax(pidConfig.imax), min(pidConfig.min), max(pidConfig.max)
{
    resetIntegrator();
    n = nh;
}

PIDFF::PIDFF()
    : kp(1), ki(0), kd(0), kff(0), imin(-1), imax(1), min(-1), max(1)
{
    resetIntegrator();
}

PIDFF::~PIDFF()
{

}

void PIDFF::init(double kp, double ki, double kd, double kff, double imin, double imax, double min, double max)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kff = kff;
    this->imin = imin;
    this->imax = imax;
    this->min = min;
    this->max = max;
}

void PIDFF::init(PIDFF::PIDConfig pidConfig)
{
    this->kp = pidConfig.kp;
    this->ki = pidConfig.ki;
    this->kd = pidConfig.kd;
    this->kff = pidConfig.kff;
    this->imin = pidConfig.imin;
    this->imax = pidConfig.imax;
    this->min = pidConfig.min;
    this->max = pidConfig.max;
}

double PIDFF::update(double target, double current, double currentTime)
{
    double dt = currentTime - lastUpdateTime;
    float error = target - current;
    
    if(dt <= 0.0){
        return -1;
    }

    double p, d, ff;
    p = d = ff = 0;

    // Calculate proportional term
    p = this->kp * error; 
    
    // Calculate intergal term
    this->integrator += this->ki * error * dt;
    this->integrator = this->integrator > this->imax ? this->imax : this->integrator < this->imin ? this->imin : this->integrator; // anti-windup

    // Calculate derivative term
    d = this->kd * (error - lastError) / dt;
    
    // Calculate feedforward term
    ff = this->kff * target;

    lastUpdateTime = currentTime;

    // ROS_INFO("dt: %f, error: %f, p: %f, integrator: %f, d: %f, ff: %f", dt, error, p, integrator, d, ff);

    return p + integrator + d + ff;
}

void PIDFF::resetIntegrator()
{
    this->integrator = 0.0;
}