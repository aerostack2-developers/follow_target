
/*!*******************************************************************************************
 *  \file       speed_controller.hpp
 *  \brief      This file contains the implementation of the Speed PID controller with speed output.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef __FT_SC_CONTROLLER_H__
#define __FT_SC_CONTROLLER_H__

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <unordered_map>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace follow_target_speed_controller
{
  using Vector3d = Eigen::Vector3d;

  struct UAV_state
  {
    Vector3d pos;
    Vector3d vel;
    tf2::Quaternion rot;
  };

  struct Control_ref
  {
    Vector3d pos;
    Vector3d vel;
    Vector3d yaw;
  };

  struct Control_command
  {
    Vector3d vel;
    Vector3d yaw;
  };

  class SpeedController
  {
  public:
    SpeedController();
    ~SpeedController(){};

  public:
    bool setParameter(const std::string &param, const double &value);
    bool getParameter(const std::string &param, double &value);
    bool isParameter(const std::string &param);
    bool setParametersList(const std::vector<std::pair<std::string, double>> &parameter_list);
    std::vector<std::pair<std::string, double>> getParametersList();

    Vector3d computePositionControl(
        const UAV_state &state,
        const Control_ref &ref,
        const double &dt);

    double computeYawSpeed(
        const double &yaw_angle_state,
        const double &yaw_angle_ref,
        const double &dt);

    Vector3d limitSpeed(
        const Vector3d &speed,
        const Vector3d &speed_limits_,
        const bool &proportional_limit);

    void resetError();

  private:
    Eigen::Vector3d position_accum_error_ = Eigen::Vector3d::Zero();
    double yaw_accum_error_ = 0.0;

    std::unordered_map<std::string, double> parameters_ = {
        {"antiwindup_cte", 5.0},
        {"alpha", 0.1},
        {"position_following.position_Kp.x", 1.0},
        {"position_following.position_Kp.y", 1.0},
        {"position_following.position_Kp.z", 1.0},
        {"position_following.position_Ki.x", 0.0},
        {"position_following.position_Ki.y", 0.0},
        {"position_following.position_Ki.z", 0.0},
        {"position_following.position_Kd.x", 0.0},
        {"position_following.position_Kd.y", 0.0},
        {"position_following.position_Kd.z", 0.0},
        {"yaw_speed_controller.Kp", 1.0},
        {"yaw_speed_controller.Ki", 1.0},
        {"yaw_speed_controller.Kd", 1.0}
    };

    float antiwindup_cte_ = 1.0f;
    double alpha_ = 0.1;

    Eigen::Matrix3d position_Kp_lin_mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d position_Ki_lin_mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d position_Kd_lin_mat_ = Eigen::Matrix3d::Identity();

    Eigen::Vector3d yaw_ang_mat_ = Eigen::Vector3d::Identity();

  private:
    void updateGains_();

  };
};

#endif
