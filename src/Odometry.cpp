#include "Odometry.h"
#include <cmath>

namespace edu
{
    
Odometry::Odometry(OdometryMode odometry_mode, std::vector<double> kin_m0, std::vector<double> kin_m1, std::vector<double> kin_m2, std::vector<double> kin_m3)
{
    _odometry_mode = odometry_mode;

    // Invert kinematic description
    // ToDo: Validate for skid steering
    _inv_kinematics_0 = {1.0F/kin_m0.at(0)/4.0F, 1.0F/kin_m1.at(0)/4.0F, 1.0F/kin_m2.at(0)/4.0F, 1.0F/kin_m3.at(0)/4.0F};
    _inv_kinematics_1 = {1.0F/kin_m0.at(1)/4.0F, 1.0F/kin_m1.at(1)/4.0F, 1.0F/kin_m2.at(1)/4.0F, 1.0F/kin_m3.at(1)/4.0F};
    _inv_kinematics_2 = {1.0F/kin_m0.at(2)/4.0F, 1.0F/kin_m1.at(2)/4.0F, 1.0F/kin_m2.at(2)/4.0F, 1.0F/kin_m3.at(2)/4.0F};

    reset();
}

Odometry::~Odometry()
{

}

void Odometry::reset()
{
    _is_pos_init = false;
    _is_vel_init = false;

    _prev_p0 = 0;
    _prev_p0 = 0;
    _prev_p0 = 0;
    _prev_p0 = 0;

    _prev_time_ns = 0;

    _pose.x = 0;
    _pose.y = 0;
    _pose.theta = 0;
}

void Odometry::set_mode(OdometryMode odometry_mode)
{
    _odometry_mode = odometry_mode;
}

OdometryMode Odometry::get_mode(){
    return _odometry_mode;
}

bool Odometry::is_pos_init()
{
    return _is_pos_init;
}

bool Odometry::is_vel_init()
{
    return _is_vel_init;
}

Pose Odometry::get_pose()
{
    return _pose;
}

int Odometry::update(double p0, double p1, double p2, double p3)
{
    int status = 1;

    if((_odometry_mode == ODOMETRY_ABSOLUTE_MODE) || (_is_pos_init)){
        
        double dp0 = 0;
        double dp1 = 0;
        double dp2 = 0;
        double dp3 = 0;

        // In case absolute values are give, calculate the difference to prev. position
        if(_odometry_mode == ODOMETRY_RELATIVE_MODE){
            dp0 = p0 - _prev_p0;
            dp1 = p1 - _prev_p1;
            dp2 = p2 - _prev_p2;
            dp3 = p3 - _prev_p3;
        }else{
            dp0 = p0;
            dp1 = p1;
            dp2 = p2;
            dp3 = p3;
        }

        // Apply inverse kinematic model
        double dx     = _inv_kinematics_0.at(0) * dp0 + _inv_kinematics_0.at(1) * dp1 + _inv_kinematics_0.at(2) * dp2 + _inv_kinematics_0.at(3) * dp3;
        double dy     = _inv_kinematics_1.at(0) * dp0 + _inv_kinematics_1.at(1) * dp1 + _inv_kinematics_1.at(2) * dp2 + _inv_kinematics_1.at(3) * dp3;;
        double dtheta =  _inv_kinematics_2.at(0) * dp0 + _inv_kinematics_2.at(1) * dp1 + _inv_kinematics_2.at(2) * dp2 + _inv_kinematics_2.at(3) * dp3;;

        if(!std::isnan(dx) || std::isinf(dx)) status = -1;
        if(!std::isnan(dy) || std::isinf(dy)) status = -1;
        if(!std::isnan(dtheta) || std::isinf(dtheta)) status = -1;

        // Calculate new position in world ksys
        if(status == 1) status = propagate_position(dx, dy, dtheta);
    }
    
    
    if(status == 1){
        _is_pos_init = true;
        _prev_p0 = p0;
        _prev_p0 = p1;
        _prev_p0 = p2;
        _prev_p0 = p3;
    }

    return status;
}

int Odometry::update(uint64_t time_ns, double w0, double w1, double w2, double w3)
{
    int status = 1;

    if((_odometry_mode == ODOMETRY_RELATIVE_MODE) || (_is_vel_init)){

        // In case absolute values are give, calculate the difference to prev. time
        double dt_s = (_odometry_mode == ODOMETRY_ABSOLUTE_MODE) ? (double)(time_ns - _prev_time_ns) / 10e9F : (double)time_ns / 10e9F;
        
        // TODO: Find out where the error is and fix this urgently!
        // Magic number fix:
        dt_s *= 10;

        // Calculate change in motor position
        // Convert rpm to rad per sec
        double p0 = w0 * dt_s / 60.0F * 2 * M_PI;
        double p1 = w1 * dt_s / 60.0F * 2 * M_PI;
        double p2 = w2 * dt_s / 60.0F * 2 * M_PI;
        double p3 = w3 * dt_s / 60.0F * 2 * M_PI;

        // Apply inverse kinematic model
        double dx     = _inv_kinematics_0.at(0) * p0 + _inv_kinematics_0.at(1) * p1 + _inv_kinematics_0.at(2) * p2 + _inv_kinematics_0.at(3) * p3;
        double dy     = _inv_kinematics_1.at(0) * p0 + _inv_kinematics_1.at(1) * p1 + _inv_kinematics_1.at(2) * p2 + _inv_kinematics_1.at(3) * p3;;
        double dtheta =  _inv_kinematics_2.at(0) * p0 + _inv_kinematics_2.at(1) * p1 + _inv_kinematics_2.at(2) * p2 + _inv_kinematics_2.at(3) * p3;;

        if(std::isnan(dx) || std::isinf(dx)) status = -1;
        if(std::isnan(dy) || std::isinf(dy)) status = -1;
        if(std::isnan(dtheta) || std::isinf(dtheta)) status = -1;

        // Calculate new position in world ksys
        if(status == 1) status = propagate_position(dx, dy, dtheta);
    }

    if(status == 1){
        _is_vel_init = true;
        _prev_time_ns = time_ns;
    }

    return status;
}

int Odometry::propagate_position(double dx, double dy, double dtheta)
{
    int status = 1;

    // Euclidean distance of last move
    double ds = sqrt(dx*dx+dy*dy);

    // Direction of last move in world ksys
    double alpha = _pose.theta + atan2(dy, dx);

    // Turn radius of last move
    double r = ds/dtheta;

    // Calculate movement in world ksys
    double dx_world = 0;
    double dy_world = 0;
    if(dtheta < _straigt_line_threshold){
        // Straigt line move
        dx_world = ds * cos(alpha);
        dy_world = ds * sin(alpha);
    }else{
        // Move on curved path
        dx_world = r * (- sin(alpha) + sin(alpha + dtheta));
        dy_world = r * (+ cos(alpha) - cos(alpha + dtheta));
    }

    // Check for errors
    if(std::isnan(dx) || std::isinf(dx)) status = -2;
    if(std::isnan(dy) || std::isinf(dy)) status = -2;
    if(std::isnan(dtheta) || std::isinf(dtheta)) status = -2;

    if(status == 1){    
        // Sum up increments
        _pose.x += dx_world;
        _pose.y += dy_world;
        _pose.theta += dtheta;
    }

    return status;
}

} //namespace