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
// Created by qiayuan on 1/16/21.
//
#include "rm_gimbal_controllers/gimbal_base.h"

#include <string>
#include <angles/angles.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

namespace rm_gimbal_controllers
{
// 初始化
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // 1.获取 Feedforward 参数
  XmlRpc::XmlRpcValue xml_rpc_value;
  bool enable_feedforward;
  enable_feedforward = controller_nh.getParam("feedforward", xml_rpc_value); // 用XmlRpcValue类探取feedforward命名空间的三个数据
  if (enable_feedforward)
  {
    ROS_ASSERT(xml_rpc_value.hasMember("mass_origin")); // 断言检查是否成功获取
    ROS_ASSERT(xml_rpc_value.hasMember("gravity"));
    ROS_ASSERT(xml_rpc_value.hasMember("enable_gravity_compensation"));
  }
  // 2.设置 Feedforward 相关参数
  mass_origin_.x = enable_feedforward ? (double)xml_rpc_value["mass_origin"][0] : 0.; // 在确认enable_feedforward拿到速度的前提下，给我们设置的参数赋上参数文件里面的值
  mass_origin_.z = enable_feedforward ? (double)xml_rpc_value["mass_origin"][2] : 0.;
  gravity_ = enable_feedforward ? (double)xml_rpc_value["gravity"] : 0.;
  enable_gravity_compensation_ = enable_feedforward && (bool)xml_rpc_value["enable_gravity_compensation"];

  // 3.获取阻力补偿参数
  ros::NodeHandle resistance_compensation_nh(controller_nh, "yaw/resistance_compensation"); // 在句柄controller_nh下，再建立句柄resistance_compensation_nh，拿的参数文件是在yaw/resistance_compensation命令空间下
  yaw_resistance_ = getParam(resistance_compensation_nh, "resistance", 0.); // 用resistance_compensation_nh句柄拿参数
  velocity_saturation_point_ = getParam(resistance_compensation_nh, "velocity_saturation_point", 0.);
  effort_saturation_point_ = getParam(resistance_compensation_nh, "effort_saturation_point", 0.);

  // 4.获取其他参数
  k_chassis_vel_ = getParam(controller_nh, "yaw/k_chassis_vel", 0.); // 直接用controller_nh拿参数
  ros::NodeHandle chassis_vel_nh(controller_nh, "chassis_vel"); // 在句柄controller_nh下，再建立句柄chassis_vel_nh，拿的参数文件是在chassis_vel命令空间下
  chassis_vel_ = std::make_shared<ChassisVel>(chassis_vel_nh); // 用 chassis_vel_nh 初始化 chassis_vel_ 对象
  ros::NodeHandle nh_bullet_solver = ros::NodeHandle(controller_nh, "bullet_solver"); // 在句柄controller_nh下，再建立句柄nh_bullet_solver，拿的参数文件是在bullet_solver命令空间下
  bullet_solver_ = std::make_shared<BulletSolver>(nh_bullet_solver);

  // 5.获取 Yaw 和 Pitch 相关参数
  ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw"); // 在句柄controller_nh下，再建立句柄nh_yaw和nh_pitch，拿的参数文件是在yaw和pitch命令空间下
  ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
  yaw_k_v_ = getParam(nh_yaw, "k_v", 0.); // 用nh_yaw和nh_pitch拿参数
  pitch_k_v_ = getParam(nh_pitch, "k_v", 0.);
  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>(); // 从 robot_hw 获取 EffortJointInterface 接口
  if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface, nh_pitch))
    return false; // (用 effort_joint_interface 和对应的命名空间句柄)初始化 ctrl_yaw_ 和 ctrl_pitch_

  // 6.获取机器人状态和 IMU 数据
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  if (!controller_nh.hasParam("imu_name")) // 检查imu_name参数,从rm_config看(默认有imu_name)
    has_imu_ = false;
  if (has_imu_)
  {
    // 给imu_name_起名gimbal_imu
    imu_name_ = getParam(controller_nh, "imu_name", static_cast<std::string>("gimbal_imu"));
    hardware_interface::ImuSensorInterface* imu_sensor_interface =
        robot_hw->get<hardware_interface::ImuSensorInterface>();
    imu_sensor_handle_ = imu_sensor_interface->getHandle(imu_name_); // 获取imu句柄
  }
  else
  {
    ROS_INFO("Param imu_name has not set, use motors' data instead of imu.");
  }

  // 7.设置 TF 变换
  gimbal_des_frame_id_ = ctrl_pitch_.joint_urdf_->child_link_name + "_des";
  odom2gimbal_des_.header.frame_id = "odom"; // 配置 odom2gimbal_des_、odom2pitch_ 和 odom2base_ 的帧 ID 和子帧 ID
  odom2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
  odom2gimbal_des_.transform.rotation.w = 1.;
  odom2pitch_.header.frame_id = "odom";
  odom2pitch_.child_frame_id = ctrl_pitch_.joint_urdf_->child_link_name;
  odom2pitch_.transform.rotation.w = 1.;
  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = ctrl_yaw_.joint_urdf_->parent_link_name;
  odom2base_.transform.rotation.w = 1.;

  // 8.订阅命令和跟踪数据
  cmd_gimbal_sub_ = controller_nh.subscribe<rm_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this); // 订阅/command和/track
  data_track_sub_ = controller_nh.subscribe<rm_msgs::TrackData>("/track", 1, &Controller::trackCB, this);
  publish_rate_ = getParam(controller_nh, "publish_rate", 100.); // 获取发布率
  error_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>(controller_nh, "error", 100));

  return true;
}

void Controller::starting(const ros::Time& /*unused*/)
{
  // 进入时state是RATE
  state_ = RATE;
  state_changed_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  // 1.读取实时缓冲区中的命令和跟踪数据
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT(); // 从话题拿到的信息会被存到buffer中，用cmd_gimbal_读取<rm_msgs::GimbalCmd> cmd_rt_buffer_的buffer
  data_track_ = *track_rt_buffer_.readFromNonRT(); // 用data_track_读取<rm_msgs::TrackData> track_rt_buffer_的buffer

  // 2.进行坐标变换
  try
  {
    // 将pitch和base_link转化为odom坐标系下面
    odom2pitch_ = robot_state_handle_.lookupTransform("odom", ctrl_pitch_.joint_urdf_->child_link_name, time);
    odom2base_ = robot_state_handle_.lookupTransform("odom", ctrl_yaw_.joint_urdf_->parent_link_name, time);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  // 3.更新底盘速度
  updateChassisVel();

  // 4.处理模式变化
  if (state_ != cmd_gimbal_.mode) // 有新命令时进入这个模式
  {
    state_ = cmd_gimbal_.mode;
    state_changed_ = true;
  }

  // 5.根据模式选择控制策略
  switch (state_)
  {
    case RATE:
      rate(time, period); // 速率控制
      break;
    case TRACK:
      track(time); // 跟踪控制
      break;
    case DIRECT:
      direct(time);
      break;
  }

  // 6.移动关节
  moveJoint(time, period);
}

void Controller::setDes(const ros::Time& time, double yaw_des, double pitch_des) // 更新云台位姿
{
  // 1.创建四元数实例
  tf2::Quaternion odom2base, odom2gimbal_des; // 创建Quaternion类的实例，这是tf类型的四元数(ros中四元数一种是msg类型，一种是tf类型)
                                              // odom2gimbal_des：odom 到 gimbal_des(云台目标坐标系)
  tf2::Quaternion base2gimbal_des;

  // 2.转换消息类型四元数
  tf2::fromMsg(odom2base_.transform.rotation, odom2base); // 调用官方接口，将msg类型的四元数odom2base_.transform.rotation转换为tf类型odom2base

  // 3.设置期望的姿态四元数
  odom2gimbal_des.setRPY(0, pitch_des, yaw_des); // 调用官方接口setRPY，通过欧拉角设置四元数的值

  // 4.计算相对姿态四元数
  base2gimbal_des = odom2base.inverse() * odom2gimbal_des;// 通过四元数矩阵相乘得到base2gimbal_des
                                                          // odom2base.inverse():返回odom2base这个旋转矩阵的逆矩阵

  // 5.提取欧拉角
  double roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw;
  quatToRPY(toMsg(base2gimbal_des), roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw); // 将tf类型的base2gimbal_des转化为msg类型，然后提取其对应的欧拉角

  // 6.检查并调整 pitch 角度
  double pitch_real_des, yaw_real_des;

  if (!setDesIntoLimit(pitch_real_des, pitch_des, base2gimbal_current_des_pitch, ctrl_pitch_.joint_urdf_)) // 检查 pitch_des 是否在关节的限位范围内
  {
    double yaw_temp;
    tf2::Quaternion base2new_des;
    // 获取 pitch 关节的上限和下限
    double upper_limit, lower_limit;
    upper_limit = ctrl_pitch_.joint_urdf_->limits ? ctrl_pitch_.joint_urdf_->limits->upper : 1e16;
    lower_limit = ctrl_pitch_.joint_urdf_->limits ? ctrl_pitch_.joint_urdf_->limits->lower : -1e16;
    base2new_des.setRPY(0,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, lower_limit)) ?
                            upper_limit :
                            lower_limit,
                        base2gimbal_current_des_yaw); // 设置新的四元数 base2new_des，使其 pitch 角度达到关节的极限值
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_real_des, yaw_temp); // 将新的四元数转换回欧拉角，获取实际的 pitch_real_des 和 yaw_temp
  }

  // 7.检查并调整 yaw 角度
  if (!setDesIntoLimit(yaw_real_des, yaw_des, base2gimbal_current_des_yaw, ctrl_yaw_.joint_urdf_))
  {
    double pitch_temp;
    tf2::Quaternion base2new_des;
    double upper_limit, lower_limit;
    upper_limit = ctrl_yaw_.joint_urdf_->limits ? ctrl_yaw_.joint_urdf_->limits->upper : 1e16;
    lower_limit = ctrl_yaw_.joint_urdf_->limits ? ctrl_yaw_.joint_urdf_->limits->lower : -1e16;
    base2new_des.setRPY(0, base2gimbal_current_des_pitch,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, lower_limit)) ?
                            upper_limit :
                            lower_limit);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_temp, yaw_real_des);
  }

  // 8.更新目标姿态
  odom2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0., pitch_real_des, yaw_real_des); // 使用实际的 pitch_real_des 和 yaw_real_des 创建一个新的msg类型四元数
  odom2gimbal_des_.header.stamp = time; // 同步时间戳
  robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers"); // 将更新后的 odom2gimbal_des_ 存储到机器人状态中
}

void Controller::rate(const ros::Time& time, const ros::Duration& period)
{
  // 1.初始状态处理
  if (state_changed_) // 若是首次进入rate，则进来
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter RATE");
    // 第一次切换到rate，会先用odom2pitch_给odom2gimbal_des_赋值，将初始的期望云台坐标系先设置成跟当前云台一样
    odom2gimbal_des_.transform.rotation = odom2pitch_.transform.rotation;
    odom2gimbal_des_.header.stamp = time;
    // 将odom2gimbal_des_的转换信息存放到tf中，但是没有发布，发布要用另外的函数
    robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
  }
  else // 若非首次rate
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw); // 将 odom2gimbal_des_ 的四元数转换为欧拉角
    setDes(time, yaw + period.toSec() * cmd_gimbal_.rate_yaw, pitch + period.toSec() * cmd_gimbal_.rate_pitch); // 设置期望坐标系的位置，rate模式下就是当前pitch/yaw，加上速度*时间，得到新的期望坐标系位置(角度 = 角速度 * 时间)
  }
}

void Controller::track(const ros::Time& time)
{
  // 打印[Gimbal] Enter TRACK
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRACK");
  }

  // 1.提取当前姿态的欧拉角
  double roll_real, pitch_real, yaw_real;
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real); // 把在pitch位置关系(四元素)转换为三个欧拉角
  double yaw_compute = yaw_real; // yaw_compute以yaw位置作为原点位置
  double pitch_compute = -pitch_real; // pitch_compute以pitch位置作为原点位置

  // 2.获取视觉数据
  geometry_msgs::Point target_pos = data_track_.position; // 用target_pos和target_vel从视觉读取目标中心位置target_pos和目标中心速度target_vel，均在相机坐标系下
  geometry_msgs::Vector3 target_vel = data_track_.velocity;

  // 3.转换到 odom 坐标系
  try
  {
    if (!data_track_.header.frame_id.empty())  // 如果视觉那边坐标系不是空的，就进去
    {
      // 定义好要交换的坐标系，就是把视觉发过来的数据都转化为odom坐标系下的数据，利用transform实现
      geometry_msgs::TransformStamped transform =
          robot_state_handle_.lookupTransform("odom", data_track_.header.frame_id, data_track_.header.stamp);
      // 用官方接口把position和velocity转换到odom坐标系
      tf2::doTransform(target_pos, target_pos, transform); // target_pos，target_vel都变为odom坐标下的敌方车中心位置和中心速度
      tf2::doTransform(target_vel, target_vel, transform);
    }
  }
  catch (tf2::TransformException& ex)  // 视觉坐标系是空的就报错
  {
    ROS_WARN("%s", ex.what());
  }

  // 4.相对于 pitch 和底盘的变换
  target_pos.x -= odom2pitch_.transform.translation.x; // target_pos变为相对于pitch的位置
  target_pos.y -= odom2pitch_.transform.translation.y;
  target_pos.z -= odom2pitch_.transform.translation.z;
  target_vel.x -= chassis_vel_->linear_->x(); // target_vel变为相对于底盘的速度
  target_vel.y -= chassis_vel_->linear_->y();
  target_vel.z -= chassis_vel_->linear_->z();

  // 5.解算目标姿态
  bool solve_success =
      bullet_solver_->solve(target_pos, target_vel, cmd_gimbal_.bullet_speed, data_track_.yaw, data_track_.v_yaw,
                            data_track_.radius_1, data_track_.radius_2, data_track_.dz, data_track_.armors_num);
  // target_pos:敌方车中心的位置(相对于pitch)
  // target_vel:敌方车中心速度(相对于底盘)
  // bullet_speed:子弹速度
  // data_track_.yaw:装甲板的朝向yaw值(相对我方机器人)
  // data_track_.v_yaw:装甲板旋转的角速度
  // data_track_.radius_1:r1半径1
  // data_track_.radius_2:r2半径2
  // data_track_.dz:相邻装甲板的高度差
  // data_track_.armors_num:装甲板数量
  // 解算成功了标志位给1

  // 6.发布误差信息
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)  // 若达到了发布频率(50)，则进来
  {
    if (error_pub_->trylock())
    {
      double error =
          bullet_solver_->getGimbalError(target_pos, target_vel, data_track_.yaw, data_track_.v_yaw,
                                         data_track_.radius_1, data_track_.radius_2, data_track_.dz,
                                         data_track_.armors_num, yaw_compute, pitch_compute, cmd_gimbal_.bullet_speed); // 计算目标位置与子弹实际飞行路径之间的误差，帮助系统调整枪管的 pitch 和 yaw
      // target_pos:敌方车中心的位置(相对于pitch)
      // target_vel:敌方车中心速度(相对于底盘)
      // yaw:装甲板的朝向(相对我方机器人)
      // v_yaw:装甲板旋转的角速度
      // r1,r2,dz,armors_num
      // yaw_compute:现在车yaw的位置
      // pitch_compute:现在车pitch的位置
      // bullet_speed:子弹速度
      error_pub_->msg_.stamp = time;  // 时间戳是ros_time
      // 解算成功就给算出来的gimbal_error，解算不成功gimbal_error直接等于1
      error_pub_->msg_.error = solve_success ? error : 1.0;
      error_pub_->unlockAndPublish();  // error_pub_不锁话题并且发布
    }
    // 发布模型
    bullet_solver_->bulletModelPub(odom2pitch_, time); // bulletModelPub():发布子弹轨迹的可视化信息
    last_publish_time_ = time; // 更新最后发布时间
  }

  // 7.更新目标姿态
  if (solve_success)  // 如果解算成功
    // bullet_solver_->getYaw()：现在我方头yaw需要偏向敌方车的角度
    // bullet_solver_->getPitch()：现在我方头pitch需要偏向敌方车的角度
    setDes(time, bullet_solver_->getYaw(), bullet_solver_->getPitch());
  else  // 解算失败
  {
    odom2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");  // 重置瞄准系统
  }
}

void Controller::direct(const ros::Time& time) // 将目标点从指定的坐标系转换到 odom 下，然后计算出相应的yaw和pitch
{
  // 1.检查状态变化
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter DIRECT");
  }
  // 2.获取目标点
  geometry_msgs::Point aim_point_odom = cmd_gimbal_.target_pos.point;
  // 3.坐标系变换
  try
  {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty()) // 若目标点的帧 ID 不为空，则进行坐标系变换
      tf2::doTransform(aim_point_odom, aim_point_odom,
                       robot_state_handle_.lookupTransform("odom", cmd_gimbal_.target_pos.header.frame_id,
                                                           cmd_gimbal_.target_pos.header.stamp));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  // 4.计算偏航角和俯仰角
  double yaw = std::atan2(aim_point_odom.y - odom2pitch_.transform.translation.y,
                          aim_point_odom.x - odom2pitch_.transform.translation.x);
  double pitch = -std::atan2(aim_point_odom.z - odom2pitch_.transform.translation.z,
                             std::sqrt(std::pow(aim_point_odom.x - odom2pitch_.transform.translation.x, 2) +
                                       std::pow(aim_point_odom.y - odom2pitch_.transform.translation.y, 2)));
  // 5.更新枪管姿态
  setDes(time, yaw, pitch);
}

bool Controller::setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                                 const urdf::JointConstSharedPtr& joint_urdf) // 检查给定的期望角度是否在关节的限位范围内
{
  // 1.获取关节的上下限
  double upper_limit, lower_limit;
  upper_limit = joint_urdf->limits ? joint_urdf->limits->upper : 1e16;
  lower_limit = joint_urdf->limits ? joint_urdf->limits->lower : -1e16;

  // 2.检查角度是否在限位范围内
  if ((base2gimbal_current_des <= upper_limit && base2gimbal_current_des >= lower_limit) ||
      (angles::two_pi_complement(base2gimbal_current_des) <= upper_limit &&
       angles::two_pi_complement(base2gimbal_current_des) >= lower_limit))
    real_des = current_des;
  else
    return false;
  return true;
}

void Controller::moveJoint(const ros::Time& time, const ros::Duration& period) // 在不同的控制模式下，计算并设置云台的 yaw 和 pitch 的速度指令
{
  // 1.获取传感器数据
  geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw;
  if (has_imu_) // 若启用 imu
  {
    // 获取三轴上的速度，存放到一个geometry_msgs::vector3（三维向量）类型的变量(gyro)中
    gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
    gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
    gyro.z = imu_sensor_handle_.getAngularVelocity()[2];
    try
    {
      // t_in是转换的输入，&是转换的输出，transform是转换关系
      // 这里通过imu到pitch/yaw的转换关系，把imu坐标下的三个轴的速度转换成pitch/yaw坐标下的三轴的速度，然后得到整个云台的速度
      tf2::doTransform(gyro, angular_vel_pitch,
                       robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
      tf2::doTransform(gyro, angular_vel_yaw,
                       robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }
  else // 若没有 imu
  {
    /*1.没有imu时，直接通过编码器来获取关节速度，精度会比imu低
      2.jointPositionController.joint是hardware_interface::JointHandle类型的变量；
      是用于读取和命令单个关节的句柄，可以通过这个句柄获取关节速度、位置等信息，也可以发送力矩指令*/
    angular_vel_yaw.z = ctrl_yaw_.joint_.getVelocity(); // 获取 yaw 关节的速度
    angular_vel_pitch.y = ctrl_pitch_.joint_.getVelocity(); // 获取 pitch 关节的速度
  }

  // 2.获取目标姿态
  geometry_msgs::TransformStamped base_frame2des;
  base_frame2des =
      robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->parent_link_name, gimbal_des_frame_id_, time); // 获取从odom到target的变换矩阵
  double roll_des, pitch_des, yaw_des;  // desired position
  quatToRPY(base_frame2des.transform.rotation, roll_des, pitch_des, yaw_des); // 转换成欧拉角

  // 3.计算期望速度
  double yaw_vel_des = 0., pitch_vel_des = 0.;
  if (state_ == RATE)
  {
    // rate模式下，期望速度即为指令中的速度
    yaw_vel_des = cmd_gimbal_.rate_yaw;
    pitch_vel_des = cmd_gimbal_.rate_pitch;
  }
  else if (state_ == TRACK)
  {
    // track模式时，期望速度通过下面的计算获得，getSelectedArmorPosAndVel在弹道解算里面
    geometry_msgs::Point target_pos;
    geometry_msgs::Vector3 target_vel;
    bullet_solver_->getSelectedArmorPosAndVel(target_pos, target_vel, data_track_.position, data_track_.velocity,
                                              data_track_.yaw, data_track_.v_yaw, data_track_.radius_1,
                                              data_track_.radius_2, data_track_.dz, data_track_.armors_num); // 获取目标装甲板的位置和速度
    tf2::Vector3 target_pos_tf, target_vel_tf;

    try
    {
      // 定义了是从自瞄坐标系到yaw坐标系的转换
      geometry_msgs::TransformStamped transform = robot_state_handle_.lookupTransform(
          ctrl_yaw_.joint_urdf_->parent_link_name, data_track_.header.frame_id, data_track_.header.stamp);
      // 转换到yaw坐标系下，得到yaw坐标系下的target_pos和target_vel
      tf2::doTransform(target_pos, target_pos, transform);
      tf2::doTransform(target_vel, target_vel, transform);
      // msg类型转换为tf类型
      tf2::fromMsg(target_pos, target_pos_tf);
      tf2::fromMsg(target_vel, target_vel_tf);

      yaw_vel_des = target_pos_tf.cross(target_vel_tf).z() / std::pow((target_pos_tf.length()), 2); // tf2::Vector3.cross()这个函数会返回该向量与另一个向量（cross函数的参数）之间的叉乘
                                                                                                         // z.()会返回这个向量的z的值，因为yaw只会绕着z轴转，计算方法不是很想看
      // 定义了是从自瞄坐标系到pitch坐标系的转换，计算方法和yaw的差不多
      transform = robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->parent_link_name,
                                                      data_track_.header.frame_id, data_track_.header.stamp);
      tf2::doTransform(target_pos, target_pos, transform);
      tf2::doTransform(target_vel, target_vel, transform);
      tf2::fromMsg(target_pos, target_pos_tf);
      tf2::fromMsg(target_vel, target_vel_tf);
      pitch_vel_des = target_pos_tf.cross(target_vel_tf).y() / std::pow((target_pos_tf.length()), 2);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }

  // 4.设置命令和更新控制器
  // 让yaw和pitch动
  ctrl_yaw_.setCommand(yaw_des, yaw_vel_des + ctrl_yaw_.joint_.getVelocity() - angular_vel_yaw.z); // 设置 yaw 关节的期望位置和速度(公式：期望速度 + 当前速度 - IMU 测得的角速度)
  ctrl_pitch_.setCommand(pitch_des, pitch_vel_des + ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);
  // yaw和picth更新数据
  ctrl_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
  double resistance_compensation = 0.; // 阻力补偿

  // 5.应用力矩补偿
  // 有速度就能进，一个速度一个力距
  if (std::abs(ctrl_yaw_.joint_.getVelocity()) > velocity_saturation_point_) // 若速度或命令值超过饱和点，设置固定的阻力补偿
    resistance_compensation = (ctrl_yaw_.joint_.getVelocity() > 0 ? 1 : -1) * yaw_resistance_;
  else if (std::abs(ctrl_yaw_.joint_.getCommand()) > effort_saturation_point_)
    resistance_compensation = (ctrl_yaw_.joint_.getCommand() > 0 ? 1 : -1) * yaw_resistance_;
  else // 否则，按比例设置阻力补偿
    resistance_compensation = ctrl_yaw_.joint_.getCommand() * yaw_resistance_ / effort_saturation_point_;
  // 设置最终命令
  // 给yaw设置速度指令，速度=命令-底盘本身有的速度+阻力增幅
  ctrl_yaw_.joint_.setCommand(ctrl_yaw_.joint_.getCommand() - k_chassis_vel_ * chassis_vel_->angular_->z() +
                              yaw_k_v_ * yaw_vel_des + resistance_compensation);
  // 给pitch设置速度指令，速度=命令速度+重力补偿+pitch本身有的速度
  ctrl_pitch_.joint_.setCommand(ctrl_pitch_.joint_.getCommand() + feedForward(time) + pitch_k_v_ * pitch_vel_des);
}

double Controller::feedForward(const ros::Time& time) // 计算pitch受到的重力补偿(通过向量叉乘计算得到)
{
  // 1.初始化重力向量
  Eigen::Vector3d gravity(0, 0, -gravity_); // 重力方向在 z 轴负方向上，大小为 -gravity_

  // 2.将重力向量从 odom 转换到 pitch 子坐标系
  tf2::doTransform(gravity, gravity,
                   robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name, "odom", time));

  // 3.计算质量原点相对于 pitch 子坐标系的叉乘
  Eigen::Vector3d mass_origin(mass_origin_.x, 0, mass_origin_.z); // mass_origin:质量中心位置
  double feedforward = -mass_origin.cross(gravity).y(); // 计算叉乘的结果向量的 y 分量

  // 4.应用重力补偿(计算方法同上)
  if (enable_gravity_compensation_) // 若启用重力补偿(计算方法同上)
  {
    Eigen::Vector3d gravity_compensation(0, 0, gravity_);
    tf2::doTransform(gravity_compensation, gravity_compensation,
                     robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name,
                                                         ctrl_pitch_.joint_urdf_->parent_link_name, time)); // 从 pitch 父坐标系 转换到 pitch 子坐标系
    feedforward -= mass_origin.cross(gravity_compensation).y();
  }
  return feedforward;
}

void Controller::updateChassisVel() // 更新底盘速度
{
  // 1.计算时间差
  double tf_period = odom2base_.header.stamp.toSec() - last_odom2base_.header.stamp.toSec(); // 读取时间，化成秒(当前时刻 - 上一时刻)

  // 2.计算线速度
  double linear_x = (odom2base_.transform.translation.x - last_odom2base_.transform.translation.x) / tf_period; // 当前时刻和上一时刻在 x 方向上的平移变化量除以时间间隔 = x 方向上的线速度
  double linear_y = (odom2base_.transform.translation.y - last_odom2base_.transform.translation.y) / tf_period;
  double linear_z = (odom2base_.transform.translation.z - last_odom2base_.transform.translation.z) / tf_period;

  // 3.转换四元数到欧拉角
  double last_angular_position_x, last_angular_position_y, last_angular_position_z, angular_position_x,
      angular_position_y, angular_position_z;
  quatToRPY(odom2base_.transform.rotation, angular_position_x, angular_position_y, angular_position_z); // 将此刻和上一时刻的xzy四元素转化成欧拉角
  quatToRPY(last_odom2base_.transform.rotation, last_angular_position_x, last_angular_position_y,
            last_angular_position_z);

  // 4.计算角速度
  double angular_x = angles::shortest_angular_distance(last_angular_position_x, angular_position_x) / tf_period; // 当前时刻和上一时刻在 x 方向上角度变化量除以时间间隔 = x 方向上的角速度
  double angular_y = angles::shortest_angular_distance(last_angular_position_y, angular_position_y) / tf_period;
  double angular_z = angles::shortest_angular_distance(last_angular_position_z, angular_position_z) / tf_period;

  // 5.存储线速度和角速度
  double linear_vel[3]{ linear_x, linear_y, linear_z }; // linear_vel数组存储三个线速度
  double angular_vel[3]{ angular_x, angular_y, angular_z }; // angular_vel数组存储三个角速度

  // 6.更新 chassis_vel_ 对象
  chassis_vel_->update(linear_vel, angular_vel, tf_period); // 调用接口，丢到chassis_vel_这个类实例中，头文件已经说明，要拿速度的时候也是调用类中的接口

  // 7.更新上一时刻数据
  last_odom2base_ = odom2base_; // 把现在时刻放入上一时刻,以便再次进入循环记录最新时刻数据
}

void Controller::commandCB(const rm_msgs::GimbalCmdConstPtr& msg)
{
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::trackCB(const rm_msgs::TrackDataConstPtr& msg)
{
  if (msg->id == 0) // 未识别到装甲板直接返回
    return;
  track_rt_buffer_.writeFromNonRT(*msg); // 将接收到的消息写入track_rt_buffer_
}

}  // namespace rm_gimbal_controllers

PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
