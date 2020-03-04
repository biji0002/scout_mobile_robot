/* 
 * scout_sim_params.hpp
 * 
 * Created on: Sep 27, 2019 15:08
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef SCOUT_SIM_PARAMS_HPP
#define SCOUT_SIM_PARAMS_HPP

#include <cstdint>

namespace wescore
{
#define RS_RGB_IMG_RES_X 1920
#define RS_RGB_IMG_RES_Y 1080

#define RS_DEP_IMG_RES_X 1280
#define RS_DEP_IMG_RES_Y 720

#define LASER_SCAN_RES_X 64
#define LASER_SCAN_RES_y 64

struct ScoutSimParams
{
    // static ScoutSimParams &GetParams()
    // {
    //     static ScoutSimParams params;
    //     return params;
    // };

    /* Scout Parameters */
    static constexpr double max_steer_angle = 30.0; // in degree

    static constexpr double track = 0.576;        // in meter (left & right wheel distance)
    static constexpr double wheelbase = 0.648;    // in meter (front & rear wheel distance)
    static constexpr double wheel_radius = 0.165; // in meter

    // from user manual v1.2.8 P18
    // max linear velocity: 1.5 m/s
    // max angular velocity: 0.7853 rad/s
    static constexpr double max_linear_speed = 1.5;  // in m/s
    static constexpr double max_angular_speed = 0.7853; // in rad/s
    static constexpr double max_speed_cmd = 10.0;       // in rad/s

    /* Sensor Parameters */
    static constexpr int32_t main_cam_res_x = RS_RGB_IMG_RES_X;
    static constexpr int32_t main_cam_res_y = RS_RGB_IMG_RES_Y;
};
} // namespace wescore


#endif /* SCOUT_SIM_PARAMS_HPP */
