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

#include "Initial.h"

namespace capture_walking
{
  namespace
  {
    constexpr double MAX_ROBOT_MASS = 42.; // [kg]
    constexpr double MIN_ROBOT_MASS = 35.; // [kg]
  }

  void states::Initial::start()
  {
    auto & ctl = controller();

    isWeighing_ = true;
    massEstimator_.reset();
    pleaseReWeigh_ = false;
    postureTaskIsActive_ = true;
    startStandingButton_ = false;
    startStanding_ = false;

    ctl.loadFootstepPlan(ctl.plan.name); // reload in case it was updated
    ctl.internalReset();

    logger().addLogEntry("walking_phase", []() { return -2.; });

    if (gui())
    {
      using namespace mc_rtc::gui;
      gui()->removeElement({"Walking", "Controller"}, "Go back to standing");
      gui()->removeElement({"Walking", "Controller"}, "Pause walking");
      gui()->addElement(
        {"Walking", "Controller"},
        Button(
          "Weigh robot",
          [this]()
          {
            massEstimator_.reset();
            isWeighing_ = true;
            pleaseReWeigh_ = false;
          }),
        ComboInput("Footstep plan",
          ctl.availablePlans(),
          [&ctl]() { return ctl.plan.name; },
          [&ctl](const std::string & name)
          {
            ctl.loadFootstepPlan(name);
            ctl.internalReset();
          }));
    }

    runState(); // don't wait till next cycle to update reference and tasks
  }

  void states::Initial::teardown()
  {
    logger().removeLogEntry("walking_phase");

    if (gui())
    {
      gui()->removeElement({"Walking", "Controller"}, "Footstep plan");
      gui()->removeElement({"Walking", "Controller"}, "Weigh robot");
      hideStartStandingButton();
    }
  }

  void states::Initial::runState()
  {
    auto & ctl = controller();
    postureTaskIsActive_ = (ctl.postureTask->speed().norm() > 1e-2);
    if (postureTaskIsActive_)
    {
      ctl.internalReset();
      hideStartStandingButton();
    }
    else if (isWeighing_)
    {
      weighRobot();
      hideStartStandingButton();
    }
    else if (pleaseReWeigh_)
    {
      hideStartStandingButton();
    }
    else
    {
      showStartStandingButton();
    }
  }

  bool states::Initial::checkTransitions()
  {
    if (startStanding_ && !postureTaskIsActive_ && !isWeighing_)
    {
      output("Standing");
      return true;
    }
    return false;
  }

  void states::Initial::weighRobot()
  {
    auto & ctl = controller();
    double LFz = ctl.stabilizer().leftFootTask->measuredWrench().force().z();
    double RFz = ctl.stabilizer().rightFootTask->measuredWrench().force().z();
    massEstimator_.add((LFz + RFz) / world::GRAVITY);
    if (massEstimator_.n() > 100)
    {
      if (massEstimator_.avg() < MIN_ROBOT_MASS || massEstimator_.avg() > MAX_ROBOT_MASS)
      {
        LOG_ERROR("Negative mass: robot is in the air?");
        pleaseReWeigh_ = true;
      }
      else
      {
        ctl.updateRobotMass(massEstimator_.avg());
      }
      isWeighing_ = false;
    }
  }

  void states::Initial::showStartStandingButton()
  {
    if (!startStandingButton_ && gui())
    {
      using namespace mc_rtc::gui;
      gui()->addElement(
        {"Walking", "Controller"},
        Button("Start standing", [this]() { startStanding_ = true; }));
      startStandingButton_ = true;
    }
  }

  void states::Initial::hideStartStandingButton()
  {
    if (startStandingButton_ && gui())
    {
      gui()->removeElement({"Walking", "Controller"}, "Start standing");
      startStandingButton_ = false;
    }
  }
}

EXPORT_SINGLE_STATE("Initial", capture_walking::states::Initial)
