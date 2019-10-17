/* 
 * Copyright (c) 2018-2019, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <capture_walking/Controller.h>
#include <capture_walking/utils/ros.h>

namespace capture_walking
{
  namespace
  {
    constexpr double ARROW_HEAD_DIAM = 0.015;
    constexpr double ARROW_HEAD_LEN = 0.05;
    constexpr double ARROW_SHAFT_DIAM = 0.015;
    constexpr double FORCE_SCALE = 0.0015;
    constexpr double MARKER_LIFETIME = 0.05;
    constexpr double POINT_SCALE = 0.02;
    const std::map<char, std::vector<float>> COLORS =
    {
      {'r', {1.0, 0.0, 0.0}},
      {'g', {0.0, 1.0, 0.0}},
      {'b', {0.0, 0.0, 1.0}},
      {'y', {1.0, 0.5, 0.0}},
      {'c', {0.0, 0.5, 1.0}},
      {'m', {1.0, 0.0, 0.5}}
    };
  }

  visualization_msgs::Marker Controller::getArrowMarker(const std::string & frame_id, const Eigen::Vector3d & from, const Eigen::Vector3d & to, char color, double scale)
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.points.push_back(rosPoint(from));
    m.points.push_back(rosPoint(to));
    m.scale.x = scale * ARROW_SHAFT_DIAM;
    m.scale.y = scale * ARROW_HEAD_DIAM;
    m.scale.z = scale * ARROW_HEAD_LEN;
    m.color.a = 1.0;
    m.color.r = COLORS.at(color)[0];
    m.color.g = COLORS.at(color)[1];
    m.color.b = COLORS.at(color)[2];
    m.header.stamp = ros::Time();
    m.header.frame_id = frame_id;
    m.id = nextMarkerId_++;
    m.lifetime = ros::Duration(MARKER_LIFETIME);
    return m;
  }

  visualization_msgs::Marker Controller::getCoPMarker(const std::string & surfaceName, char color)
  {
    const std::shared_ptr<mc_tasks::force::CoPTask> & copTask = (surfaceName == "LeftFootCenter") ? stabilizer_.leftFootTask : stabilizer_.rightFootTask;
    Eigen::Vector2d cop_ = copTask->measuredCoP();
    Eigen::Vector3d cop(cop_(0), cop_(1), 0.);
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = cop(0);
    m.pose.position.y = cop(1);
    m.pose.position.z = cop(2);
    m.scale.x = 0.02;
    m.scale.y = 0.02;
    m.scale.z = 0.02;
    m.color.a = 1.0;
    m.color.r = COLORS.at(color)[0];
    m.color.g = COLORS.at(color)[1];
    m.color.b = COLORS.at(color)[2];
    m.header.stamp = ros::Time();
    m.header.frame_id = "control/surfaces/" + surfaceName;
    m.id = nextMarkerId_++;
    m.lifetime = ros::Duration(MARKER_LIFETIME);
    return m;
  }

  visualization_msgs::Marker Controller::getForceMarker(const std::string & surfaceName, char color)
  {
    const std::shared_ptr<mc_tasks::force::CoPTask> & copTask = (surfaceName == "LeftFootCenter") ? stabilizer_.leftFootTask : stabilizer_.rightFootTask;
    Eigen::Vector2d cop_ = copTask->measuredCoP();
    Eigen::Vector3d cop(cop_(0), cop_(1), 0.);
    Eigen::Vector3d force = copTask->measuredWrench().force();
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.points.push_back(rosPoint(cop));
    m.points.push_back(rosPoint(cop + FORCE_SCALE * force));
    m.scale.x = ARROW_SHAFT_DIAM;
    m.scale.y = ARROW_HEAD_DIAM;
    m.scale.z = ARROW_HEAD_LEN;
    m.color.a = 1.0;
    m.color.r = COLORS.at(color)[0];
    m.color.g = COLORS.at(color)[1];
    m.color.b = COLORS.at(color)[2];
    m.header.stamp = ros::Time();
    m.header.frame_id = "control/surfaces/" + surfaceName;
    m.id = nextMarkerId_++;
    m.lifetime = ros::Duration(MARKER_LIFETIME);
    return m;
  }

  visualization_msgs::Marker Controller::getPointMarker(const std::string & frame_id, const Eigen::Vector2d & pos, char color, double scale)
  {
    Eigen::Vector3d pos3D(pos(0), pos(1), 0.);
    return getPointMarker(frame_id, pos3D, color, scale);
  }

  visualization_msgs::Marker Controller::getPointMarker(const std::string & frame_id, const Eigen::Vector3d & pos, char color, double scale)
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = pos(0);
    m.pose.position.y = pos(1);
    m.pose.position.z = pos(2);
    m.scale.x = scale * POINT_SCALE;
    m.scale.y = scale * POINT_SCALE;
    m.scale.z = scale * POINT_SCALE;
    m.color.a = 1.0;
    m.color.r = COLORS.at(color)[0];
    m.color.g = COLORS.at(color)[1];
    m.color.b = COLORS.at(color)[2];
    m.header.stamp = ros::Time();
    m.header.frame_id = frame_id;
    m.id = nextMarkerId_++;
    m.lifetime = ros::Duration(MARKER_LIFETIME);
    return m;
  }

  visualization_msgs::Marker Controller::getContactMarker(const std::string & frame_id, char color)
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.points.push_back(rosPoint(Eigen::Vector3d(+sole.halfLength, +sole.halfWidth, 0.)));
    m.points.push_back(rosPoint(Eigen::Vector3d(+sole.halfLength, -sole.halfWidth, 0.)));
    m.points.push_back(rosPoint(Eigen::Vector3d(-sole.halfLength, -sole.halfWidth, 0.)));
    m.points.push_back(rosPoint(Eigen::Vector3d(-sole.halfLength, +sole.halfWidth, 0.)));
    m.points.push_back(rosPoint(Eigen::Vector3d(+sole.halfLength, +sole.halfWidth, 0.)));
    m.scale.x = 0.005;
    m.color.a = 1.0;
    m.color.r = COLORS.at(color)[0];
    m.color.g = COLORS.at(color)[1];
    m.color.b = COLORS.at(color)[2];
    m.header.stamp = ros::Time();
    m.header.frame_id = frame_id;
    m.id = nextMarkerId_++;
    m.lifetime = ros::Duration(MARKER_LIFETIME);
    return m;
  }

  visualization_msgs::MarkerArray Controller::getPendulumMarkerArray(const Pendulum & state, char color)
  {
    visualization_msgs::MarkerArray array;
    nextMarkerId_ = 0;
    array.markers.push_back(getArrowMarker("robot_map", state.zmp(), state.com(), color, 0.1));
    array.markers.push_back(getArrowMarker("robot_map", state.zmp(), state.zmp() + FORCE_SCALE * pendulumObserver_.contactForce(state), color));
    array.markers.push_back(getPointMarker("robot_map", state.com(), color));
    array.markers.push_back(getPointMarker("robot_map", state.zmp(), color));
    return array;
  }

  void Controller::publishMarkers()
  {
    Eigen::Vector3d controlDcm_ = controlCom_ + controlComd_ / pendulum().omega();
    Eigen::Vector3d realDcm_ = realCom_ + realComd_ / pendulum().omega();

    nextMarkerId_ = 0;
    visualization_msgs::MarkerArray sensorArray;
    sensorArray.markers.push_back(getCoPMarker("LeftFootCenter", 'g'));
    sensorArray.markers.push_back(getForceMarker("LeftFootCenter", 'g'));
    sensorArray.markers.push_back(getCoPMarker("RightFootCenter", 'g'));
    sensorArray.markers.push_back(getForceMarker("RightFootCenter", 'g'));
    sensorPublisher_.publish(sensorArray);

    nextMarkerId_ = 0;
    visualization_msgs::MarkerArray extraArray;
    extraArray.markers.push_back(getArrowMarker("robot_map", controlCom_, controlDcm_, 'b', 0.2));
    extraArray.markers.push_back(getPointMarker("robot_map", controlCom_, 'b'));
    extraArray.markers.push_back(getPointMarker("robot_map", controlDcm_, 'b', 0.5));
    extraArray.markers.push_back(getArrowMarker("robot_map", realCom_, realDcm_, 'g', 0.2));
    extraArray.markers.push_back(getPointMarker("robot_map", realCom_, 'g'));
    extraArray.markers.push_back(getPointMarker("robot_map", realDcm_, 'g', 0.5));
    extraArray.markers.push_back(getPointMarker("control/surfaces/LeftFootCenter", stabilizer_.leftFootTask->targetCoP(), 'm', 0.5));
    extraArray.markers.push_back(getPointMarker("control/surfaces/RightFootCenter", stabilizer_.rightFootTask->targetCoP(), 'm', 0.5));
    extraArray.markers.push_back(getPointMarker("robot_map", stabilizer_.distribZMP(), 'm'));
    extraPublisher_.publish(extraArray);

    pendulumObserverPublisher_.publish(getPendulumMarkerArray(pendulumObserver_, 'r'));
    pendulumPublisher_.publish(getPendulumMarkerArray(pendulum_, 'y'));

    nextMarkerId_ = 0;
    visualization_msgs::MarkerArray footstepArray;
    const auto & contacts = plan.contacts();
    for (unsigned i = 0; i < contacts.size(); i++)
    {
      auto & contact = contacts[i];
      double supportDist = (contact.p() - supportContact().p()).norm();
      double targetDist = (contact.p() - targetContact().p()).norm();
      constexpr double SAME_CONTACT_DIST = 0.005;
      if (supportDist > SAME_CONTACT_DIST && targetDist > SAME_CONTACT_DIST)
      {
        footstepArray.markers.push_back(getContactMarker("footstep_" + std::to_string(i), 'b'));
      }
    }
    footstepArray.markers.push_back(getContactMarker("support_contact", 'g'));
    footstepArray.markers.push_back(getContactMarker("target_contact", 'r'));
    footstepPublisher_.publish(footstepArray);
  }

  void Controller::publishTransforms()
  {
    ros::Time tm = ros::Time::now();
    std::vector<geometry_msgs::TransformStamped> transforms;
    const sva::PTransformd & X_0_base = robot().bodyPosW("base_link");
    sva::PTransformd X_0_inertial(robot().bodySensor().orientation().matrix(), X_0_base.translation());
    transforms.push_back(PT2TF(X_0_inertial, tm, std::string("robot_map"), "inertial", rosSeq_));
    transforms.push_back(PT2TF(supportContact().pose, tm, std::string("robot_map"), "support_contact", rosSeq_));
    transforms.push_back(PT2TF(targetContact().pose, tm, std::string("robot_map"), "target_contact", rosSeq_));
    const auto & contacts = plan.contacts();
    for (unsigned i = 0; i < contacts.size(); i++)
    {
      transforms.push_back(PT2TF(contacts[i].pose, tm, std::string("robot_map"), "footstep_" + std::to_string(i), rosSeq_));
    }
    tfBroadcaster_->sendTransform(transforms);
  }

  void Controller::spinner()
  {
    const auto & nodeHandle = mc_rtc::ROSBridge::get_node_handle();

    pendulumObserverPublisher_ = nodeHandle->advertise<visualization_msgs::MarkerArray>("/capture_walking/PendulumObserver", 1);
    extraPublisher_ = nodeHandle->advertise<visualization_msgs::MarkerArray>("/capture_walking/ExtraMarkers", 1);
    footstepPublisher_ = nodeHandle->advertise<visualization_msgs::MarkerArray>("/capture_walking/FootstepMarkers", 1);
    pendulumPublisher_ = nodeHandle->advertise<visualization_msgs::MarkerArray>("/capture_walking/PendulumReference", 1);
    sensorPublisher_ = nodeHandle->advertise<visualization_msgs::MarkerArray>("/capture_walking/FootForceSensors", 1);
    tfBroadcaster_ = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster());

    ros::Rate rate(200);
    while(ros::ok())
    {
      publishMarkers();
      publishTransforms();
      ros::spinOnce();
      rate.sleep();
      rosSeq_++;
    }
  }
}
