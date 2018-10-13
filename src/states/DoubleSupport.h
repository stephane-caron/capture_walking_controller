/* Copyright 2018 CNRS-UM LIRMM
 *
 * \author Stéphane Caron
 *
 * This file is part of capture_walking_controller.
 *
 * capture_walking_controller is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * capture_walking_controller is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with capture_walking_controller. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <capture_walking/CaptureProblem.h>
#include <capture_walking/Controller.h>
#include <capture_walking/HorizontalMPCProblem.h>
#include <capture_walking/State.h>
#include <capture_walking/utils/Interval.h>

namespace capture_walking
{
  namespace states
  {
    struct DoubleSupport : State
    {
      /** Start state.
       *
       */
      void start() override;

      /** Teardown state.
       *
       */
      void teardown() override;

      /** Check transitions at beginning of control cycle.
       *
       */
      bool checkTransitions() override;

      /** Main state function, called if no transition at this cycle.
       *
       */
      void runState() override;

      /** Update MPC preview.
       *
       */
      void updatePreview();

      /** Update horizontal MPC preview.
       *
       */
      void updatePreviewHMPC();

      /** Update capturability preview.
       *
       */
      void updatePreviewCPS();

    private:
      bool stopDuringThisDSP_;
      double duration_;
      double initLeftFootRatio_;
      double remTime_;
      double stateTime_;
      double targetLeftFootRatio_;
      double timeSinceLastPreviewUpdate_;
    };
  }
}
