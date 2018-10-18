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
#include <capture_walking/Contact.h>
#include <capture_walking/HorizontalMPC.h>
#include <capture_walking/Preview.h>

namespace capture_walking
{
  /** Solution to a horizontal model predictive control problem.
   *
   */
  struct HorizontalMPCSolution : public Preview
  {
    /** Default constructor.
     *
     */
    HorizontalMPCSolution() = default;

    /** Initialize a zero solution with a given initial state.
     *
     * \param initState Initial state.
     *
     */
    HorizontalMPCSolution(const Eigen::VectorXd & initState);

    /** Default copy constructor.
     *
     */
    HorizontalMPCSolution(const HorizontalMPCSolution &) = default;

    /** Initialize solution from trajectories.
     *
     * \param stateTraj State trajectory.
     *
     * \param jerkTraj CoM jerk trajectory.
     *
     */
    HorizontalMPCSolution(const Eigen::VectorXd & stateTraj, const Eigen::VectorXd & jerkTraj);

    /** Fill solution with zeros, except for initial state.
     *
     * \param initState Initial state.
     *
     */
    void zeroFrom(const Eigen::VectorXd & initState);

    /** Integrate playback on reference.
     *
     * \param state CoM state to integrate upon.
     *
     * \param dt Duration.
     *
     */
    void integrate(Pendulum & state, double dt) override;

    /** Playback integration of CoM state reference.
     *
     * \param state CoM state to integrate upon.
     *
     * \param dt Duration.
     *
     */
    void integratePlayback(Pendulum & state, double dt);

    /** Post-playback integration of CoM state reference.
     *
     * \param state CoM state to integrate upon.
     *
     * \param dt Duration.
     *
     */
    void integratePostPlayback(Pendulum & state, double dt);

    /** Get the CoM state trajectory.
     *
     */
    const Eigen::VectorXd & stateTraj()
    {
      return stateTraj_;
    }

    /** Get the CoM jerk (input) trajectory.
     *
     */
    const Eigen::VectorXd & jerkTraj()
    {
      return jerkTraj_;
    }

  private:
    Eigen::VectorXd jerkTraj_;
    Eigen::VectorXd stateTraj_;
  };
}
