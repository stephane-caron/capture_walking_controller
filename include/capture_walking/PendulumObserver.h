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
