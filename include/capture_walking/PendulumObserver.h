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

#include <SpaceVecAlg/SpaceVecAlg>

#include <capture_walking/Pendulum.h>
#include <capture_walking/Contact.h>
#include <capture_walking/defs.h>

namespace capture_walking
{
  /** IPM observer from force/torque measurements.
   *
   */
  struct PendulumObserver : Pendulum
  {
    /** Constructor.
     *
     * \param dt Controller time step.
     *
     */
    PendulumObserver(double dt);

    /** Update estimates based on the sensed net contact wrench.
     *
     * \param comGuess Guess for the CoM position.
     *
     * \param contactWrench Net contact wrench expressed at the origin of the
     * inertial frame.
     *
     * \param contact Support contact.
     *
     */
    void update(const Eigen::Vector3d & mb_com, const sva::ForceVecd & contactWrench, const Contact & contact);

    /** Get contact force.
     *
     */
    inline Eigen::Vector3d contactForce() const
    {
      double lambda = std::pow(omega_, 2);
      return mass_ * lambda * (com_ - zmp_);
    }

    /** Get contact force from other pendulum with current omega.
     *
     * \param other Other pendulum.
     *
     */
    Eigen::Vector3d contactForce(const Pendulum & other) const
    {
      double lambda = std::pow(other.omega(), 2);
      return mass_ * lambda * (other.com() - other.zmp());
    }

    /** Set robot mass.
     *
     */
    void mass(double mass)
    {
      mass_ = mass;
    }

  private:
    double mass_;
    double dt_;
  };
}
