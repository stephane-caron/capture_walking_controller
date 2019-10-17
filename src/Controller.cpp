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

#include <mc_rbdyn/rpy_utils.h>

#include <capture_walking/Controller.h>
#include <capture_walking/utils/clamp.h>

namespace capture_walking
{
  Controller::Controller(std::shared_ptr<mc_rbdyn::RobotModule> robotModule, double dt, const mc_rtc::Configuration & config)
    : mc_control::fsm::Controller(robotModule, dt, config),
      halfSitPose(controlRobot().mbc().q),
      comVelFilter_(dt, /* cutoff period = */ 0.01),
      pendulumObserver_(dt),
      stabilizer_(controlRobot(), pendulum_, dt, getPostureTask(robot().name())),
      floatingBaseObserver_(controlRobot())
  {
    postureTask = getPostureTask(robot().name());

    // Set half-sitting pose for posture task
    const auto & halfSit = robotModule->stance();
    const auto & refJointOrder = robot().refJointOrder();
    for (unsigned i = 0; i < refJointOrder.size(); ++i)
    {
      if (robot().hasJoint(refJointOrder[i]))
      {
        halfSitPose[robot().jointIndexByName(refJointOrder[i])] = halfSit.at(refJointOrder[i]);
      }
    }

    // Read settings from configuration file
    plans_ = config("plans");
    hmpcConfig_ = config("hmpc");
    sole = config("sole");
    std::string initialPlan = plans_.keys()[0];
    config("initial_plan", initialPlan);
    if (config.has("stabilizer"))
    {
      stabilizer_.configure(config("stabilizer"));
    }

    loadFootstepPlan(initialPlan);
    updateRobotMass(robot().mass());
    stabilizer_.reset(robots());
    stabilizer_.wrenchFaceMatrix(sole);

    logger().addLogEntry("controlRobot_LeftFoot", [this]() { return controlRobot().surfacePose("LeftFoot"); });
    logger().addLogEntry("controlRobot_LeftFootCenter", [this]() { return controlRobot().surfacePose("LeftFootCenter"); });
    logger().addLogEntry("controlRobot_RightFoot", [this]() { return controlRobot().surfacePose("RightFoot"); });
    logger().addLogEntry("controlRobot_RightFootCenter", [this]() { return controlRobot().surfacePose("RightFootCenter"); });
    logger().addLogEntry("controlRobot_com", [this]() { return controlCom_; });
    logger().addLogEntry("controlRobot_comd", [this]() { return controlComd_; });
    logger().addLogEntry("controlRobot_comd_norm", [this]() { return controlComd_.norm(); });
    logger().addLogEntry("controlRobot_dcm", [this]() -> Eigen::Vector3d { return controlCom_ + controlComd_ / pendulum_.omega(); });
    logger().addLogEntry("controlRobot_posW", [this]() { return controlRobot().posW(); });
    logger().addLogEntry("cps_desired_step_time", [this]() { return cps.desiredStepTime(); });
    logger().addLogEntry("cps_init_contact", [this]() { return cps.initContact().p(); });
    logger().addLogEntry("cps_solution_step_time", [this]() { return cps.solution().stepTime(); });
    logger().addLogEntry("cps_target_cop", [this]() { return cps.targetCoP(); });
    logger().addLogEntry("error_com", [this]() -> Eigen::Vector3d { return controlCom_ - realCom_; });
    logger().addLogEntry("error_comd", [this]() -> Eigen::Vector3d { return controlComd_ - realComd_; });
    logger().addLogEntry("error_dcm", [this]() -> Eigen::Vector3d { return (controlCom_ - realCom_) + (controlComd_ - realComd_) / pendulum_.omega(); });
    logger().addLogEntry("error_zmp", [this]() -> Eigen::Vector3d { return stabilizer_.distribZMP() - pendulumObserver_.zmp(); });
    logger().addLogEntry("estimator_com", [this]() { return pendulumObserver_.com(); });
    logger().addLogEntry("estimator_comd",[this]() { return pendulumObserver_.comd(); });
    logger().addLogEntry("estimator_dcm", [this]() { return pendulumObserver_.dcm(); });
    logger().addLogEntry("estimator_zmp", [this]() { return pendulumObserver_.zmp(); });
    logger().addLogEntry("hmpc_failures", [this]() { return nbHMPCFailures_; });
    logger().addLogEntry("hmpc_pbstep", [this]() { return (preview) ? preview->playbackStep() : 0; });
    logger().addLogEntry("hmpc_pbtime", [this]() { return (preview) ? preview->playbackTime() : -0.42; });
    logger().addLogEntry("hmpc_updates", [this]() { return nbHMPCUpdates_; });
    logger().addLogEntry("hmpc_weights_jerk", [this]() { return hmpc.jerkWeight; });
    logger().addLogEntry("hmpc_weights_vel", [this]() { return hmpc.velWeights; });
    logger().addLogEntry("hmpc_weights_zmp", [this]() { return hmpc.zmpWeight; });
    logger().addLogEntry("left_foot_ratio", [this]() { return leftFootRatio_; });
    logger().addLogEntry("left_foot_ratio_measured", [this]() { return measuredLeftFootRatio(); });
    logger().addLogEntry("observers_kin_posW", [this]() { return floatingBaseObserver_.posW(); });
    logger().addLogEntry("pendulum_com", [this]() { return pendulum_.com(); });
    logger().addLogEntry("pendulum_comd", [this]() { return pendulum_.comd(); });
    logger().addLogEntry("pendulum_comdd", [this]() { return pendulum_.comdd(); });
    logger().addLogEntry("pendulum_dcm", [this]() { return pendulum_.dcm(); });
    logger().addLogEntry("pendulum_omega", [this]() { return pendulum_.omega(); });
    logger().addLogEntry("pendulum_zmp", [this]() { return pendulum_.zmp(); });
    logger().addLogEntry("plan_com_height", [this]() { return plan.comHeight(); });
    logger().addLogEntry("plan_double_support_duration", [this]() { return plan.doubleSupportDuration(); });
    logger().addLogEntry("plan_final_dsp_duration", [this]() { return plan.finalDSPDuration(); });
    logger().addLogEntry("plan_init_dsp_duration", [this]() { return plan.initDSPDuration(); });
    logger().addLogEntry("plan_landing_pitch", [this]() { return plan.landingPitch(); });
    logger().addLogEntry("plan_landing_ratio", [this]() { return plan.landingRatio(); });
    logger().addLogEntry("plan_ref_vel", [this]() { return plan.supportContact().refVel; });
    logger().addLogEntry("plan_single_support_duration", [this]() { return plan.singleSupportDuration(); });
    logger().addLogEntry("plan_swing_height", [this]() { return plan.swingHeight(); });
    logger().addLogEntry("plan_takeoff_offset", [this]() { return plan.takeoffOffset(); });
    logger().addLogEntry("plan_takeoff_pitch", [this]() { return plan.takeoffPitch(); });
    logger().addLogEntry("plan_takeoff_ratio", [this]() { return plan.takeoffRatio(); });
    logger().addLogEntry("realRobot_LeftFoot", [this]() { return realRobot().surfacePose("LeftFoot"); });
    logger().addLogEntry("realRobot_LeftFootCenter", [this]() { return realRobot().surfacePose("LeftFootCenter"); });
    logger().addLogEntry("realRobot_RightFoot", [this]() { return realRobot().surfacePose("RightFoot"); });
    logger().addLogEntry("realRobot_RightFootCenter", [this]() { return realRobot().surfacePose("RightFootCenter"); });
    logger().addLogEntry("realRobot_com", [this]() { return realCom_; });
    logger().addLogEntry("realRobot_comd", [this]() { return realComd_; });
    logger().addLogEntry("realRobot_dcm", [this]() -> Eigen::Vector3d { return realCom_ + realComd_ / pendulum_.omega(); });
    logger().addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });
    stabilizer_.addLogEntries(logger());

    if (gui_)
    {
      using namespace mc_rtc::gui;
      gui_->addElement(
        {"Sensors"},
        ArrayInput(
          "IMU", {"R [deg]", "P [deg]", "Y [deg]"},
          [this]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(realRobot().bodySensor().orientation().toRotationMatrix()) * 180. / M_PI; },
          [](const Eigen::Vector3d &) {}),
        Label(
          "Left foot pressure [N]",
          [this]() { return realRobot().forceSensor("LeftFootForceSensor").force()[2]; }),
        Label(
          "Right foot pressure [N]",
          [this]() { return realRobot().forceSensor("RightFootForceSensor").force()[2]; }));
      gui_->addElement(
        {"Walking", "Controller"},
        Button("# EMERGENCY STOP",
          [this]()
          {
            emergencyStop = true;
            this->interrupt();
          }),
        Button("Reset",
          [this]() { this->resume("Initial"); }),
        Label("Mass [kg]",
          [this]() { return std::round(robotMass_ * 100.) / 100.; }));
      gui_->addElement(
        {"Walking", "WPG"},
        ComboInput(
          "Method",
          {"CaptureProblem", "HorizontalMPC"},
          [this]()
          {
            switch (wpg)
            {
              case WalkingPatternGeneration::CaptureProblem:
                return "CaptureProblem";
              case WalkingPatternGeneration::HorizontalMPC:
                break;
            }
            return "HorizontalMPC";
          },
          [this](const std::string & label)
          {
            if (label == "CaptureProblem")
            {
              wpg = WalkingPatternGeneration::CaptureProblem;
            }
            else // (label == "HorizontalMPC")
            {
              wpg = WalkingPatternGeneration::HorizontalMPC;
            }
          }),
        NumberInput(
            "Preview update period [s]",
            [this]() { return previewUpdatePeriod; },
            [this](double period) { previewUpdatePeriod = clamp(period, 0., 1.); }),
        ArrayInput(
          "Capture CoP offset", {"x", "y"},
          [this]() { return cps.ankleToTargetCoP; },
          [this](const Eigen::Vector2d & cop) { cps.ankleToTargetCoP = cop; }),
        ArrayInput(
          "Horizontal MPC weights", {"jerk", "vel_x", "vel_y", "zmp"},
          [this]()
          {
            Eigen::VectorXd weights(4);
            weights[0] = hmpc.jerkWeight;
            weights[1] = hmpc.velWeights.x();
            weights[2] = hmpc.velWeights.y();
            weights[3] = hmpc.zmpWeight;
            return weights;
          },
          [this](const Eigen::VectorXd & weights)
          {
            hmpc.jerkWeight = weights[0];
            hmpc.velWeights.x() = weights[1];
            hmpc.velWeights.y() = weights[2];
            hmpc.zmpWeight = weights[3];
          }));
      gui_->addElement(
        {"Walking", "Plan"},
        Label(
          "Name",
          [this]() { return plan.name; }),
        NumberInput(
          "CoM height [m]",
          [this]() { return plan.comHeight(); },
          [this](double height) { plan.comHeight(height); }),
        NumberInput(
          "DSP duration [s]",
          [this]() { return plan.doubleSupportDuration(); },
          [this](double duration) { plan.doubleSupportDuration(duration); }),
        NumberInput(
          "SSP duration [s]",
          [this]() { return plan.singleSupportDuration(); },
          [this](double duration) { plan.singleSupportDuration(duration); }),
        NumberInput(
          "Initial DSP duration [s]",
          [this]() { return plan.initDSPDuration(); },
          [this](double duration) { plan.initDSPDuration(duration); }),
        NumberInput(
          "Final DSP duration [s]",
          [this]() { return plan.finalDSPDuration(); },
          [this](double duration) { plan.finalDSPDuration(duration); }),
        NumberInput(
          "Swing height [m]",
          [this]() { return plan.swingHeight(); },
          [this](double height) { plan.swingHeight(height); }),
        NumberInput(
          "Takeoff pitch [rad]",
          [this]() { return plan.takeoffPitch(); },
          [this](double pitch) { plan.takeoffPitch(pitch); }),
        NumberInput(
          "Landing pitch [rad]",
          [this]() { return plan.landingPitch(); },
          [this](double pitch) { plan.landingPitch(pitch); }),
        NumberInput(
          "Takeoff ratio",
          [this]() { return plan.takeoffRatio(); },
          [this](double ratio) { plan.takeoffRatio(ratio); }),
        NumberInput(
          "Landing ratio",
          [this]() { return plan.landingRatio(); },
          [this](double ratio) { plan.landingRatio(ratio); }));
      stabilizer_.addGUIElements(gui_);
    }

    LOG_SUCCESS("CaptureWalking controller init done " << this)
  }

  void Controller::updateRobotMass(double mass)
  {
    robotMass_ = mass;
    pendulumObserver_.mass(mass);
    stabilizer_.mass(mass);
    LOG_INFO("Robot mass updated to " << mass << " [kg]");
  }

  void Controller::internalReset()
  {
    if(!isSpinning_)
    {
      spinThread_ = std::thread(std::bind(&Controller::spinner, this));
      isSpinning_ = true;
    }

    auto X_0_fb = plan.computeInitialTransform(controlRobot());
    controlRobot().posW(X_0_fb);
    controlRobot().setBaseLinkVelocity(Eigen::Vector6d::Zero());
    realRobot().posW(X_0_fb);
    realRobot().setBaseLinkVelocity(Eigen::Vector6d::Zero());
    floatingBaseObserver_.reset(X_0_fb);

    Eigen::Vector3d initCom = controlRobot().com();

    comVelFilter_.reset(initCom);

    pendulumObserver_.reset(initCom);
    pendulum_.reset(initCom);
    plan.reset();
    postureTask->posture(halfSitPose);
    stabilizer_.reset(robots());
    stabilizer_.updateState(initCom, Eigen::Vector3d::Zero(), sva::ForceVecd{Eigen::Vector6d::Zero()});

    controlCom_ = initCom;
    controlComd_ = Eigen::Vector3d::Zero();
    leftFootRatioJumped_ = false;
    leftFootRatio_ = 0.5;
    nbHMPCFailures_ = 0;
    nbHMPCUpdates_ = 0;
    pauseWalking = false;
    realCom_ = initCom; // realRobot() may not be initialized yet
    realComd_ = Eigen::Vector3d::Zero();

    stopLogSegment();
  }

  void Controller::leftFootRatio(double ratio)
  {
    double maxRatioVar = 1.5 * timeStep / plan.doubleSupportDuration();
    if (std::abs(ratio - leftFootRatio_) > maxRatioVar)
    {
      LOG_WARNING("Left foot ratio jumped from " << leftFootRatio_ << " to " << ratio);
      leftFootRatioJumped_ = true;
    }
    leftFootRatio_ = clamp(ratio, 0., 1., "leftFootRatio");
  }

  bool Controller::run()
  {
    if (emergencyStop)
    {
      return false;
    }
    if (!mc_control::fsm::Controller::running())
    {
      return mc_control::fsm::Controller::run();
    }

    controlCom_ = controlRobot().com();
    controlComd_ = controlRobot().comVelocity();
    ctlTime_ += timeStep;

    // check contact state
    const auto & lfSensor = realRobot().forceSensor("LeftFootForceSensor");
    const auto & rfSensor = realRobot().forceSensor("RightFootForceSensor");
    constexpr double CONTACT_THRESHOLD = 30.; // [N]
    double leftFootPressure = lfSensor.force().z();
    double rightFootPressure = rfSensor.force().z();
    if (leftFootPressure < CONTACT_THRESHOLD && rightFootPressure < CONTACT_THRESHOLD)
    {
      if (!isInTheAir_)
      {
        LOG_WARNING("Robot is in the air");
        isInTheAir_ = true;
      }
    }
    else
    {
      if (isInTheAir_)
      {
        LOG_INFO("Robot is on the ground again");
        isInTheAir_ = false;
      }
    }

    // update kinematic observer
    floatingBaseObserver_.leftFootRatio(leftFootRatio_);
    floatingBaseObserver_.run(realRobot());

    // update realCom_
    floatingBaseObserver_.update(realRobot());
    realCom_ = realRobot().com();

    // update realComd_
    if (leftFootRatioJumped_)
    {
      comVelFilter_.updatePositionOnly(realCom_);
      leftFootRatioJumped_ = false;
    }
    else
    {
      comVelFilter_.update(realCom_);
    }
    realComd_ = comVelFilter_.vel();

    sva::ForceVecd contactWrench = measuredContactWrench();
    pendulumObserver_.update(/* comGuess = */ realCom_, contactWrench, supportContact());
    stabilizer_.updateState(realCom_, realComd_, contactWrench, leftFootRatio_);

    bool ret = mc_control::fsm::Controller::run();
    if (mc_control::fsm::Controller::running())
    {
      postureTask->posture(halfSitPose); // reset posture in case the FSM updated it
    }
    return ret;
  }

  sva::ForceVecd Controller::measuredContactWrench()
  {
    sva::ForceVecd netWrench{Eigen::Vector6d::Zero()};
    for (std::string sensorName : {"LeftFootForceSensor", "RightFootForceSensor"})
    {
      const auto & sensor = realRobot().forceSensor(sensorName);
      if (sensor.force()[2] > 1.)
      {
        netWrench += sensor.worldWrench(realRobot());
      }
    }
    return netWrench;
  }

  void Controller::loadFootstepPlan(std::string name)
  {
    plan = plans_(name);
    plan.complete(sole);
    plan.name = name;
    plan.reset();
    hmpc.configure(hmpcConfig_);
    if (plans_(name).has("hmpc"))
    {
      hmpc.configure(plans_(name)("hmpc"));
    }
    LOG_INFO("Loaded footstep plan \"" << name << "\"");
  }

  void Controller::startLogSegment(const std::string & label)
  {
    if (segmentName_.length() > 0)
    {
      stopLogSegment();
    }
    segmentName_ = "t_" + std::to_string(++nbLogSegments_).erase(0, 1) + "_" + label;
    logger().addLogEntry(segmentName_, [this]() { return ctlTime_; });
  }

  void Controller::stopLogSegment()
  {
    logger().removeLogEntry(segmentName_);
    segmentName_ = "";
  }

  bool Controller::updatePreviewCPS()
  {
    cps.initState(pendulum());
    cps.targetHeight(plan.comHeight());
    if (cps.solve())
    {
      preview.reset(new CaptureSolution(cps.solution()));
      nbCPSUpdates_++;
      return true;
    }
    else
    {
      nbCPSFailures_++;
      return false;
    }
  }

  bool Controller::updatePreviewHMPC()
  {
    hmpc.initState(pendulum());
    hmpc.comHeight(plan.comHeight());
    if (hmpc.solve())
    {
      preview.reset(new HorizontalMPCSolution(hmpc.solution()));
      nbHMPCUpdates_++;
      return true;
    }
    else
    {
      nbHMPCFailures_++;
      return false;
    }
  }
}

CONTROLLER_CONSTRUCTOR("CaptureWalking", capture_walking::Controller)
