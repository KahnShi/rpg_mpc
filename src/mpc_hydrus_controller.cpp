/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "rpg_mpc/mpc_hydrus_controller.h"

#include <ctime>

namespace rpg_mpc {

template <typename T>
MpcHydrusController<T>::MpcHydrusController(
  const ros::NodeHandle & nh, const ros::NodeHandle & pnh) :
  nh_(nh),
  pnh_(pnh),
  mpc_wrapper_(MpcWrapper<T>()),
  timing_feedback_(T(1e-3)),
  timing_preparation_(T(1e-3)),
  est_state_((Eigen::Matrix<T, kStateSize, 1>() <<
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished()),
  reference_states_(Eigen::Matrix<T, kStateSize, kSamples+1>::Zero()),
  reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples+1>::Zero()),
  predicted_states_(Eigen::Matrix<T, kStateSize, kSamples+1>::Zero()),
  predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()),
  point_of_interest_(Eigen::Matrix<T, 3, 1>::Zero())
{
  pub_predicted_trajectory_ =
    nh_.advertise<nav_msgs::Path>("/mpc/trajectory_predicted", 1);
  pub_reference_point_markers_ =
    nh_.advertise<visualization_msgs::MarkerArray>("/mpc/target_points", 1);
  pub_predicted_states_ =
    nh_.advertise<aerial_robot_msgs::MpcPredict>("/mpc/predict_states", 1);

  sub_mpc_command_ = nh_.subscribe("/mpc/command", 1,
                                   &MpcHydrusController<T>::mpcCommandCallback, this);
  sub_cog_odom_ = nh_.subscribe("/uav/cog/odom", 1,
                                   &MpcHydrusController<T>::cogOdomCallback, this);

  if(!params_.loadParameters(pnh_)) // todo
  {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
  setNewParams(params_);

  preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);
}

template <typename T>
void MpcHydrusController<T>::mpcCommandCallback(const aerial_robot_msgs::MpcCommandList::ConstPtr& msg)
{
  mpc_cmd_ = *msg;
  updateMpc();
}

template <typename T>
void MpcHydrusController<T>::cogOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  cog_odom_ = *msg;
}

template <typename T>
void MpcHydrusController<T>::updateMpc()
{
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();

  preparation_thread_.join();

  // todo: update online data
  updateRobotModel();
  setStateEstimate();
  setReference();

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  mpc_wrapper_.update(est_state_, do_preparation_step);
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

  // Publish the predicted trajectory.
  publishPrediction(predicted_states_, predicted_inputs_, call_time);

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&MpcHydrusController<T>::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ = 0.9*timing_feedback_ +
                     0.1* double(end - start)/CLOCKS_PER_SEC;
  if(params_.print_info_)
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
      timing_feedback_*1000, (timing_feedback_+timing_preparation_)*1000);

  // Return the input control command.
  updateControlCommand(predicted_states_.col(0),
                       predicted_inputs_.col(0),
                       call_time);
}

template <typename T>
void MpcHydrusController<T>::updateRobotModel(){
  Eigen::Matrix<T, 7, 1> inertia;
  inertia(0) =  mpc_cmd_.list.front().mass;
  for (int i = 0; i < 6; ++i)
    inertia(i + 1) = mpc_cmd_.list.front().inertia[i];
  mpc_wrapper_.setRobotInertia(inertia);
  Eigen::Matrix<T, 24, 1> rotors;
  for (int i = 0; i < 4; ++i){
    for (int j = 0; j < 3; ++j){
      rotors(6 * i + j) = mpc_cmd_.list.front().p_rotor[3 * i + j];
      rotors(6 * i + j + 3) = mpc_cmd_.list.front().n_rotor[3 * i + j];
    }
  }
  mpc_wrapper_.setRobotConfiguration(rotors);

}

template <typename T>
bool MpcHydrusController<T>::setStateEstimate()
{
  est_state_(kPosX) = cog_odom_.pose.pose.position.x;
  est_state_(kPosY) = cog_odom_.pose.pose.position.y;
  est_state_(kPosZ) = cog_odom_.pose.pose.position.z;
  est_state_(kOriW) = cog_odom_.pose.pose.orientation.w;
  est_state_(kOriX) = cog_odom_.pose.pose.orientation.x;
  est_state_(kOriY) = cog_odom_.pose.pose.orientation.y;
  est_state_(kOriZ) = cog_odom_.pose.pose.orientation.z;
  est_state_(kVelX) = cog_odom_.twist.twist.linear.x;
  est_state_(kVelY) = cog_odom_.twist.twist.linear.y;
  est_state_(kVelZ) = cog_odom_.twist.twist.linear.z;
  est_state_(kRateX) = cog_odom_.twist.twist.angular.x;
  est_state_(kRateY) = cog_odom_.twist.twist.angular.y;
  est_state_(kRateZ) = cog_odom_.twist.twist.angular.z;
  const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm()-1.0)<0.1;
  return quaternion_norm_ok;
}

template <typename T>
bool MpcHydrusController<T>::setReference()
{
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  Eigen::Quaternion<T> q_heading;
  Eigen::Quaternion<T> q_orientation;
  bool quaternion_norm_ok(true);
  // if(mpc_cmd_.list.size() == 1)
  {
    // todo: make use of end_stamp in mpc command
    Eigen::Matrix<T, kStateSize, 1> state;
    for (int i = 0; i < kStateSize; ++i)
      state(i) = mpc_cmd_.list.front().target.state[i];
    reference_states_ = state.replicate(1, kSamples+1);
    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() <<
                         mpc_cmd_.list.front().target.input[0],
                         mpc_cmd_.list.front().target.input[1],
                         mpc_cmd_.list.front().target.input[2],
                         mpc_cmd_.list.front().target.input[3]
      ).finished().replicate(1, kSamples+1);
    double period = (mpc_cmd_.list.front().end_stamp - mpc_cmd_.list.front().start_stamp).toSec();
    int end_state_id = std::round(period / dt);
    mpc_wrapper_.setCosts(end_state_id, 0.0, 0.0);
  }
  // else // todo
  // {
  //   std::list<quadrotor_common::TrajectoryPoint>::const_iterator iterator(
  //     reference_trajectory.points.begin());
  //   for(int i=0; i<kSamples+1; i++)
  //   {
  //     while(iterator->time_from_start.toSec() < i*dt &&
  //           iterator!=reference_trajectory.points.end())
  //     {
  //       iterator++;
  //     }
  //     q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
  //       iterator->heading, Eigen::Matrix<T,3,1>::UnitZ()));
  //     q_orientation = q_heading * iterator->orientation.template cast<T>();
  //     reference_states_.col(i) << iterator->position.template cast<T>(),
  //                                 q_orientation.w(),
  //                                 q_orientation.x(),
  //                                 q_orientation.y(),
  //                                 q_orientation.z(),
  //                                 iterator->velocity.template cast<T>(),
  //                                 iterator->bodyrates.template cast<T>();
  //     if(reference_states_.col(i).segment(kOriW,4).dot(
  //       est_state_.segment(kOriW,4))<0.0)
  //         reference_states_.block(kOriW,i,4,1) =
  //           -reference_states_.block(kOriW,i,4,1);
  //     acceleration << iterator->acceleration.template cast<T>() - gravity;
  //     reference_inputs_.col(i) << acceleration.norm() / 4.0,
  //       acceleration.norm() / 4.0,
  //       acceleration.norm() / 4.0,
  //       acceleration.norm() / 4.0;
  //     quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm()-1.0)<0.1;
  //   }
  // }
  return quaternion_norm_ok;
}

template <typename T>
void MpcHydrusController<T>::updateControlCommand(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
  ros::Time& time)
{
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();
  // Bound inputs for sanity.
  input_bounded(INPUT::kThrust1) = std::max(params_.min_thrust_,
    std::min(params_.max_thrust_, input_bounded(INPUT::kThrust1)));
  input_bounded(INPUT::kThrust2) = std::max(params_.min_thrust_,
    std::min(params_.max_thrust_, input_bounded(INPUT::kThrust2)));
  input_bounded(INPUT::kThrust3) = std::max(params_.min_thrust_,
    std::min(params_.max_thrust_, input_bounded(INPUT::kThrust3)));
  input_bounded(INPUT::kThrust4) = std::max(params_.min_thrust_,
    std::min(params_.max_thrust_, input_bounded(INPUT::kThrust4)));
}

template <typename T>
bool MpcHydrusController<T>::publishPrediction(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples+1>> states,
  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
  ros::Time& time)
{
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time;
  path_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  T dt = mpc_wrapper_.getTimestep();

  for(int i=0; i<kSamples; i++)
  {
    pose.header.stamp = time + ros::Duration(i*dt);
    pose.header.seq = i;
    pose.pose.position.x = states(kPosX,i);
    pose.pose.position.y = states(kPosY,i);
    pose.pose.position.z = states(kPosZ,i);
    pose.pose.orientation.w = states(kOriW,i);
    pose.pose.orientation.x = states(kOriX,i);
    pose.pose.orientation.y = states(kOriY,i);
    pose.pose.orientation.z = states(kOriZ,i);
    path_msg.poses.push_back(pose);
  }

  pub_predicted_trajectory_.publish(path_msg);

  aerial_robot_msgs::MpcPredict states_msg;
  states_msg.header = path_msg.header; // todo: which stamp to use
  states_msg.num = kSamples + 1;
  states_msg.time_step = dt;
  states_msg.predict.resize(states_msg.num);
  for (int i = 0; i < states_msg.num; ++i){
    for (int j = 0; j < 13; ++j)
      states_msg.predict[i].state[j] = states(j, i);
    if (i < states_msg.num - 1){
      for (int j = 0; j < 4; ++j)
        states_msg.predict[i + 1].input[j] = inputs(j, i);
    }
  }
  pub_predicted_states_.publish(states_msg);

  visualization_msgs::MarkerArray target_markers_msg;
  visualization_msgs::Marker marker_msg;
  target_markers_msg.markers.resize(2);
  target_markers_msg.markers[0].header.frame_id = "/world";
  target_markers_msg.markers[0].id = 0;
  target_markers_msg.markers[0].type = target_markers_msg.markers[0].SPHERE;
  target_markers_msg.markers[0].action = target_markers_msg.markers[0].ADD;
  target_markers_msg.markers[0].pose.position.x = states(kPosX, 0);
  target_markers_msg.markers[0].pose.position.y = states(kPosY, 0);
  target_markers_msg.markers[0].pose.position.z = states(kPosZ, 0);
  target_markers_msg.markers[0].scale.x = 0.1;
  target_markers_msg.markers[0].scale.y = 0.1;
  target_markers_msg.markers[0].scale.z = 0.1;
  target_markers_msg.markers[0].color.a = 0.6;
  target_markers_msg.markers[0].color.r = 0.0;
  target_markers_msg.markers[0].color.g = 0.0;
  target_markers_msg.markers[0].color.b = 1.0;
  target_markers_msg.markers[1] = target_markers_msg.markers[0];
  target_markers_msg.markers[1].id = 1;
  target_markers_msg.markers[1].color.r = 1.0;
  target_markers_msg.markers[1].color.g = 0.0;
  target_markers_msg.markers[1].color.b = 0.0;
  target_markers_msg.markers[1].pose.position.x = reference_states_(kPosX, 0);
  target_markers_msg.markers[1].pose.position.y = reference_states_(kPosY, 0);
  target_markers_msg.markers[1].pose.position.z = reference_states_(kPosZ, 0);
  pub_reference_point_markers_.publish(target_markers_msg);

  return true;
}

template <typename T>
void MpcHydrusController<T>::preparationThread()
{
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ = 0.9*timing_preparation_ +
                        0.1* double(end - start)/CLOCKS_PER_SEC;
}

template <typename T>
bool MpcHydrusController<T>::setNewParams(MpcParams<T>& params)
{
  mpc_wrapper_.setCosts(params.Q_, params.R_);
  mpc_wrapper_.setLimits(
    params.min_thrust_, params.max_thrust_);
  // mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
  params.changed_ = false;
  return true;
}


template class MpcHydrusController<float>;
template class MpcHydrusController<double>;

} // namespace rpg_mpc
