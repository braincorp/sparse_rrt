/**
 * @file rally_car.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Original work Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * Modified work Copyright 2017 Oleg Y. Sinyavskiy
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Original authors: Zakary Littlefield, Kostas Bekris
 * Modifications by: Oleg Y. Sinyavskiy
 * 
 */


#include "systems/rally_car.hpp"
#include "utilities/random.hpp"
#include "image_creation/svg_image.hpp"


#define _USE_MATH_DEFINES


#include <cmath>
#include <limits>

#define M 1450
#define IZ 2740
#define LF 1.3
#define LR 1.4
#define R .3
#define IF 1.8 
#define IR 1.8
#define H .4    
#define B 7
#define C 1.6
#define D .52

#define CRBRAKE 700
#define CRACC 0
#define CFBRAKE 700
#define CFACC 1000

#define STATE_X 0
#define STATE_Y 1
#define STATE_VX 2
#define STATE_VY 3
#define STATE_THETA 4
#define STATE_THETADOT 5
#define STATE_WF 6
#define STATE_WR 7
#define CONTROL_STA 0
#define CONTROL_TF 1
#define CONTROL_TR 2

#define MIN_X -25
#define MAX_X 25
#define MIN_Y -35
#define MAX_Y 25


//void rally_car_t::random_state(double* state)
//{
//        state[0] = uniform_random(MIN_X,MAX_X);
//        state[1] = uniform_random(MIN_Y,MAX_Y);
//        // state[2] = uniform_random(-18,18);
//        // state[3] = uniform_random(-18,18);
//        // state[5] = uniform_random(-17,17);
//        // state[6] = uniform_random(-40,40);
//        // state[7] = uniform_random(-40,40);
//
//        //compute the angle that is created
//        double theta = atan2(state[1],state[0]);
//        theta+=M_PI/2;
//        theta = uniform_random(theta-M_PI/6.0,theta+M_PI/6.0);
//        if(theta > 2*M_PI)
//            theta -= 2*M_PI;
//        state[4]  = theta;
//        // state[4] = uniform_random(-M_PI,M_PI);
//}


double rally_car_t::propagate(
    const double* start_state, unsigned int state_dimension,
    const double* control, unsigned int control_dimension,
    int num_steps, double* result_state, double integration_step)
{
    temp_state[0] = start_state[0];
    temp_state[1] = start_state[1];
    temp_state[2] = start_state[2];
    temp_state[3] = start_state[3];
    temp_state[4] = start_state[4];
    temp_state[5] = start_state[5];
    temp_state[6] = start_state[6];
    temp_state[7] = start_state[7];
    for(int i=0;i<num_steps;i++)
    {
        update_derivative(control);
        temp_state[0] += integration_step*deriv[0];
        temp_state[1] += integration_step*deriv[1];
        temp_state[2] += integration_step*deriv[2];
        temp_state[3] += integration_step*deriv[3];
        temp_state[4] += integration_step*deriv[4];
        temp_state[5] += integration_step*deriv[5];
        temp_state[6] += integration_step*deriv[6];
        temp_state[7] += integration_step*deriv[7];
        enforce_bounds();
        if (!valid_state()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
    }
    result_state[0] = temp_state[0];
    result_state[1] = temp_state[1];
    result_state[2] = temp_state[2];
    result_state[3] = temp_state[3];
    result_state[4] = temp_state[4];
    result_state[5] = temp_state[5];
    result_state[6] = temp_state[6];
    result_state[7] = temp_state[7];
    double duration = num_steps*integration_step;
    return duration;
}

void rally_car_t::enforce_bounds()
{
// #x y xdot ydot theta thetadot wf wr
// state_space: 
//   min: [-25, -40, -18, -18, -3.14, -17, -40, -40]
//   max: [25, 25, 18, 18, 3.14, 17, 40, 40]
        if(temp_state[0]<MIN_X)
                temp_state[0]=MIN_X;
        else if(temp_state[0]>MAX_X)
                temp_state[0]=MAX_X;

        if(temp_state[1]<MIN_Y)
                temp_state[1]=MIN_Y;
        else if(temp_state[1]>MAX_Y)
                temp_state[1]=MAX_Y;

        if(temp_state[2]<-18)
                temp_state[2]=-18;
        else if(temp_state[2]>18)
                temp_state[2]=18;

        if(temp_state[3]<-18)
                temp_state[3]=-18;
        else if(temp_state[3]>18)
                temp_state[3]=18;

        if(temp_state[4]<-M_PI)
                temp_state[4]+=2*M_PI;
        else if(temp_state[4]>M_PI)
                temp_state[4]-=2*M_PI;

        if(temp_state[5]<-17)
                temp_state[5]=-17;
        else if(temp_state[5]>17)
                temp_state[5]=17;

        if(temp_state[6]<-40)
                temp_state[6]=-40;
        else if(temp_state[6]>40)
                temp_state[6]=40;

        if(temp_state[7]<-40)
                temp_state[7]=-40;
        else if(temp_state[7]>40)
                temp_state[7]=40;
}


bool rally_car_t::valid_state()
{
        bool obstacle_collision = false;
        //any obstacles need to be checked here
        for(unsigned i=0;i<obstacles.size() && !obstacle_collision;i++)
        {
                if(     temp_state[0]>obstacles[i].low_x-1 && 
                        temp_state[0]<obstacles[i].high_x+1 && 
                        temp_state[1]>obstacles[i].low_y-1 && 
                        temp_state[1]<obstacles[i].high_y+1)
                {
                        obstacle_collision = true;
                }
        }
        return !obstacle_collision;
}

std::tuple<double, double> rally_car_t::visualize_point(const double* state, unsigned int state_dimension) const
{
        double x = (state[0]-MIN_X)/(MAX_X-MIN_X);
        double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y);
        return std::make_tuple(x, y);
}

void rally_car_t::update_derivative(const double* control)
{
        double _vx = temp_state[2];
        double _vy = temp_state[3];
        double _theta = temp_state[4];
        double _thetadot = temp_state[5];
        double _wf = temp_state[6];
        double _wr = temp_state[7];

        double _sta = control[0];
        double _tf = control[0];
        double _tr = control[0];

        deriv[STATE_X] = _vx;
        deriv[STATE_Y] = _vy;
        deriv[STATE_THETA] = _thetadot;

        double V = sqrt(_vx*_vx+_vy*_vy);
        double beta = atan2(_vy,_vx) - _theta;
        double V_Fx = V*cos(beta-_sta) + _thetadot*LF*sin(_sta);
        double V_Fy = V*sin(beta-_sta) + _thetadot*LF*cos(_sta);
        double V_Rx = V*cos(beta);
        double V_Ry = V*sin(beta) - _thetadot*LR;

        double s_Fx = (V_Fx - _wf*R)/(_wf*R);
        double s_Fy = V_Fy/(_wf*R);
        double s_Rx = (V_Rx - _wr*R)/(_wr*R);
        double s_Ry = V_Ry/(_wr*R);

        double s_F = sqrt(s_Fx*s_Fx+s_Fy*s_Fy);
        double s_R = sqrt(s_Rx*s_Rx+s_Ry*s_Ry);

        double mu_F = D*sin(C*atan(B*s_F));
        double mu_R = D*sin(C*atan(B*s_R));
        double mu_Fx;
        double mu_Fy;
        if(std::isfinite(s_Fx))
                mu_Fx = -1*(s_Fx/s_F)*mu_F;
        else
                mu_Fx = -mu_F;
        if(std::isfinite(s_Fy))
                mu_Fy = -1*(s_Fy/s_F)*mu_F;
        else
                mu_Fy = -mu_F;
        double mu_Rx;
        double mu_Ry;
        if(std::isfinite(s_Rx))
                mu_Rx = -1*(s_Rx/s_R)*mu_R;
        else
                mu_Rx = -mu_R;
        if(std::isfinite(s_Ry))
                mu_Ry = -1*(s_Ry/s_R)*mu_R;
        else
                mu_Ry = -mu_R;

        double fFz = (LR*M*(9.8) - H*M*9.8*mu_Rx) / (LF+LR+H*(mu_Fx*cos(_sta)-mu_Fy*sin(_sta)-mu_Rx));
        double fRz = M*9.8 - fFz;

        double fFx = mu_Fx * fFz;
        double fFy = mu_Fy * fFz;
        double fRx = mu_Rx * fRz;
        double fRy = mu_Ry * fRz;;

        deriv[STATE_VX] = (fFx*cos(_theta+_sta)-fFy*sin(_theta+_sta)+fRx*cos(_theta)-fRy*sin(_theta) )/M;
        deriv[STATE_VY] = (fFx*sin(_theta+_sta)+fFy*cos(_theta+_sta)+fRx*sin(_theta)+fRy*cos(_theta) )/M;
        deriv[STATE_THETADOT] = ((fFy*cos(_sta)+fFx*sin(_sta))*LF - fRy*LR)/IZ;
        deriv[STATE_WF] = (_tf-fFx*R)/IF;
        deriv[STATE_WR] = (_tr-fRx*R)/IR;
}

std::string rally_car_t::visualize_obstacles(int image_width, int image_height) const
{
    svg::Dimensions dims(image_width, image_height);
    svg::DocumentBody doc(svg::Layout(dims, svg::Layout::BottomLeft));

    double temp[2];
    for(unsigned i=0;i<obstacles.size();i++)
    {
            temp[0] = obstacles[i].low_x;
            temp[1] = obstacles[i].high_y;
            double x, y;
            std::tie(x, y) = this->visualize_point(temp, 8);
            doc<<svg::Rectangle(svg::Point(x*dims.width, y*dims.height),
                                (obstacles[i].high_x-obstacles[i].low_x)/(MAX_X-MIN_X) * dims.width,
                                (obstacles[i].high_y-obstacles[i].low_y)/(MAX_Y-MIN_Y) * dims.height,
                                svg::Color::Red);
    }

    return doc.toString();
}

std::vector<std::pair<double, double> > rally_car_t::get_state_bounds() const {
        return {
                {MIN_X,MAX_X},
                {MIN_Y,MAX_Y},
                {-18,18},
                {-18,18},
                {-M_PI,M_PI},
                {-17,17},
                {-40,40},
                {-40,40}
        };
}

std::vector<std::pair<double, double> > rally_car_t::get_control_bounds() const {
        return {
                {-1.0472,1.0472},
                {-700,0},
                {-700,1200}
        };
}

std::vector<bool> rally_car_t::is_circular_topology() const {
    return {
            false,
            false,
            false,
            false,
            true,
            false,
            false,
            false
    };
}


