/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 8/14/20.
//

#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <rm_gimbal_controllers/BulletSolverConfig.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/eigen_types.h>
#include <rm_common/ros_utilities.h>

namespace rm_gimbal_controllers
{
struct Config
{
  double resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18,
      resistance_coff_qd_30, g, delay, dt, timeout;
  // resistance_coff_qd_10: 阻尼系数 qd_10
  // resistance_coff_qd_15: 阻尼系数 qd_15,后面以此类推
  // g:重力加速度
  // delay:延迟时间
  // dt:时间步长
  // timeout:超时时间
};

class BulletSolver
{
public:
  explicit BulletSolver(ros::NodeHandle& controller_nh);

  bool solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw, double v_yaw,
             double r1, double r2, double dz, int armors_num);
  double getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw, double r1,
                        double r2, double dz, int armors_num, double yaw_real, double pitch_real, double bullet_speed);
  double getResistanceCoefficient(double bullet_speed) const;
  double getYaw() const
  {
    return output_yaw_;
  }
  double getPitch() const
  {
    return -output_pitch_;
  }
  void getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel,
                                 geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw,
                                 double r1, double r2, double dz, int armors_num);
  void bulletModelPub(const geometry_msgs::TransformStamped& odom2pitch, const ros::Time& time);
  void reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t);
  ~BulletSolver() = default;

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_desire_pub_; // 发布期望的目标位置标记
  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_real_pub_; // 发布实际的目标位置标记
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_; // 实时缓冲区，存储config_的实时副本
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>* d_srv_{};
  Config config_{};
  double max_track_target_vel_; // 最大跟随敌方目标的速度
  bool dynamic_reconfig_initialized_{}; // 动态重配置是否已经初始化的标志
  double output_yaw_{}, output_pitch_{}; // 输出的yaw，输出的pitch
  double bullet_speed_{}, resistance_coff_{}; // 子弹速度，阻力系数
  int selected_armor_; // 选中的装甲板编号
  bool track_target_; // 是否正在跟踪目标. true表示跟踪模式(低速)，false表示中心模式(高速)

  geometry_msgs::Point target_pos_{}; // 目标位置
  double fly_time_; // 子弹飞行时间
  visualization_msgs::Marker marker_desire_; // 期望目标位置的可视化标记
  visualization_msgs::Marker marker_real_; // 实际目标位置的可视化标记
};
}  // namespace rm_gimbal_controllers
