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

#include <capture_walking/Pendulum.h>
#include <capture_walking/defs.h>

namespace capture_walking
{
  /** Solution to a model predictive control problem.
   *
   */
  struct Preview
  {
    /** Integrate preview on a given inverted pendulum state.
     *
     * \param pendulum Inverted pendulum model.
     *
     * \param dt Duration.
     *
     */
    virtual void integrate(Pendulum & state, double dt) = 0;

    /** Get current playback step.
     *
     */
    inline unsigned playbackStep()
    {
      return playbackStep_;
    }

    /** Get current playback time.
     *
     */
    inline double playbackTime()
    {
      return playbackTime_;
    }

  protected:
    double playbackTime_ = 0.;
    unsigned playbackStep_ = 0;
  };
}
