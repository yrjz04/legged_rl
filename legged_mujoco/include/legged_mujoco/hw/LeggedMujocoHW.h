/**
 * @file LeggedMujocoHW.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "legged_hw/LeggedHW.h"
#include "../src/mujoco/simulate.h"

#include <mujoco/mujoco.h>


namespace legged {

// struct to store motor data (for ros control)
struct MjJointData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

// struct to store imu data (for ros control)
struct MjImuData {
  std::string handle_name_;
  std::string frame_id_;
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class LeggedMujocoHW : public LeggedHW {
public:
    LeggedMujocoHW(mjModel *model, mjData *data, ::mujoco::Simulate & sim);
    
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

    void read(const ros::Time& time, const ros::Duration& period) override;

    void write(const ros::Time& time, const ros::Duration& period) override;

private:
    bool setupJoints();
    bool setupImu();

    std::vector<MjJointData> jointData_;
    std::vector<std::string> jointNames_;
    MjImuData imuData_;
    std::vector<double> jointTorqueLimits_;

    // mujoco
    mjData *mj_data_;
    mjModel *mj_model_;
    ::mujoco::Simulate & sim_;

    // model info
    int jointNum_;
    int jointSensorNum_;
    const int jointSensorCoeff_ = 3;

    // publisher for debug
    ros::Publisher imuPub_;
    ros::Publisher groundTruthPub_;

    std::string imuTopicName_;
    std::string groundTruthTopicName_;
    std::string baseLink_;
};



} // namespace legged_robot
