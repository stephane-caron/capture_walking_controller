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

#include <capture_walking/PendulumObserver.h>

namespace capture_walking
{
  PendulumObserver::PendulumObserver(double dt)
    : Pendulum(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      dt_(dt)
  {
  }

  void PendulumObserver::update(const Eigen::Vector3d & comGuess, const sva::ForceVecd & contactWrench, const Contact & contact)
  {
    const Eigen::Vector3d & force = contactWrench.force();
    const Eigen::Vector3d & moment_0 = contactWrench.couple();
    Eigen::Vector3d moment_p = moment_0 - contact.p().cross(force);
    double fSquare = force.dot(force);
    if (fSquare < 42.)
    {
      return;
    }

    Eigen::Vector3d com_prev = com_;
    zmp_ = contact.p() + contact.n().cross(moment_p) / contact.n().dot(force);
    double closestCoMScaling = force.dot(comGuess - zmp_) / fSquare;
    // take the point on the measured central axis that is closest to the guess
    com_ = zmp_ + closestCoMScaling * force;
    comd_ = (com_ - com_prev) / dt_;
    comdd_ = force / mass_;

    double f_n = contact.n().dot(force);
    double lambda = f_n / (mass_ * contact.n().dot(com_ - zmp_));
    omega_ = std::sqrt(lambda);
    if ((contactForce() - force).norm() > 1e-5)
    {
      LOG_ERROR("error in estimated contact force computation");
    }
  }
}
