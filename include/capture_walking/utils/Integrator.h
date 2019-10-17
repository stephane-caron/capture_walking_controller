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

/** Integrator
 *
 * The output y(t) of this integrator w.r.t. its input x(t) follows
 *
 *    yd(t) = x(t) - decay * y(t)
 *
 * so that:
 *
 *    y(t) = int_{u=0}^t x(u) e^{decay * (u - t)} d{u}
 *
 * where decay > 0 is a reset frequency used for anti-windup.
 *
 */
struct Integrator
{
  /** Reset integral to zero.
   *
   */
  void reset()
  {
    integral_.setZero();
  }

  /** Add constant input for a fixed duration.
   *
   * \param value Constant input.
   *
   * \param dt Fixed duration.
   *
   */
  void add(const Eigen::Vector3d & value, double dt)
  {
    integral_ = (1. - decay_ * dt) * integral_ + dt * value;
    if (saturation_ > 0.)
    {
      saturate();
    }
  }

  /** Get decay frequency.
   *
   */
  double decay() const
  {
    return decay_;
  }

  /** Set decay frequency.
   *
   * \param decay New frequency.
   *
   */
  void decay(double decay)
  {
    decay_ = decay;
  }

  /** Evaluate the output of the integrator.
   *
   */
  const Eigen::Vector3d & eval() const
  {
    return integral_;
  }

  /** Set output saturation. Disable by providing a negative value.
   *
   * \param s Output will saturate between -s and +s.
   *
   */
  void saturation(double s)
  {
    saturation_ = s;
  }

private:
  void saturate()
  {
    for (unsigned i = 0; i < 3; i++)
    {
      if (integral_(i) < -saturation_)
      {
        integral_(i) = -saturation_;
      }
      else if (integral_(i) > saturation_)
      {
        integral_(i) = saturation_;
      }
    }
  }

private:
  Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
  double decay_ = 0.1;
  double saturation_ = -1.;
};
