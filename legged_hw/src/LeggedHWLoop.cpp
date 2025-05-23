/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Qiayuan Liao
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

#include <legged_hw/LeggedHWLoop.h>


namespace legged
{

LeggedHWLoop::LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface)
    : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true) {
  // Create the controller manager
  controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

  // Load ros params
  int error = 0;
  int threadPriority = 0;
  ros::NodeHandle nhP("robot_config");
  error += static_cast<int>(!nhP.getParam("hw_loop/loop_frequency", loopHz_));
  error += static_cast<int>(!nhP.getParam("hw_loop/cycle_time_error_threshold", cycleTimeErrorThreshold_));
  error += static_cast<int>(!nhP.getParam("hw_loop/thread_priority", threadPriority));
  if (error > 0) {
    std::string error_message =
        "could not retrieve one of the required parameters: loop_hz or cycle_time_error_threshold or thread_priority";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // Get current time for use with first update
  lastTime_ = Clock::now();

  // Setup loop thread
  loopThread_ = std::thread([&]() {
    while (loopRunning_) {
      update();
    }
  });
  sched_param sched{.sched_priority = threadPriority};
  if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
    ROS_WARN(
        "Failed to set threads priority (one possible reason could be that the user and the group permissions "
        "are not set properly.).\n");
  }
}

void LeggedHWLoop::update() {
  const auto currentTime = Clock::now();
  // Compute desired duration rounded to clock decimation
  const Duration desiredDuration(1.0 / loopHz_);

  // Get change in time
  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  lastTime_ = currentTime;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  if (cycle_time_error > cycleTimeErrorThreshold_) {
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                               << "cycle time: " << elapsedTime_ << "s, "
                                                               << "threshold: " << cycleTimeErrorThreshold_ << "s");
  }

  // Input
  // get the hardware's state
  hardwareInterface_->read(ros::Time::now(), elapsedTime_);

  // Control
  // let the controller compute the new command (via the controller manager)
  controllerManager_->update(ros::Time::now(), elapsedTime_);

  // Output
  // send the new command to hardware
  hardwareInterface_->write(ros::Time::now(), elapsedTime_);

  // Sleep
  const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

LeggedHWLoop::~LeggedHWLoop() {
  loopRunning_ = false;
  if (loopThread_.joinable()) {
    loopThread_.join();
  }
}

} // namespace legged