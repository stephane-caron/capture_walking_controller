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

#include <mc_tasks/CoMTask.h>

#include <capture_walking/Contact.h>
#include <capture_walking/defs.h>

namespace capture_walking
{
  /** Inverted pendulum model.
   *
   */
  struct Pendulum
  {
    /** Initialize state.
     *
     * \param com Initial CoM position.
     *
     * \param comd Initial CoM velocity.
     *
     * \param comdd Initial CoM acceleration.
     *
     */
    Pendulum(const Eigen::Vector3d & com = Eigen::Vector3d::Zero(), const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(), const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

    /** Reset to a new state.
     *
     * \param com New CoM position.
     *
     * \param com New CoM velocity.
     *
     * \param comdd Initial CoM acceleration.
     *
     */
    void reset(const Eigen::Vector3d & com, const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(), const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

    /** Integrate in floating-base inverted pendulum mode with constant inputs.
     *
     * \param zmp Zero-tilting Moment Point, i.e. net force application point.
     *
     * \param lambda Normalized stiffness of the pendulum.
     *
     * \param dt Duration of integration step.
     *
     */
    void integrateIPM(Eigen::Vector3d zmp, double lambda, double dt);

    /** Integrate constant CoM jerk for a given duration.
     *
     * \param comddd CoM jerk.
     *
     * \param dt Integration step.
     *
     * \param floor Frame attached to the contact plane the CoP belongs to.
     *
     */
    void integrateCoMJerk(const Eigen::Vector3d & comddd, double dt);

    /** Reset CoM height above a given contact plane.
     *
     * \param height CoM height above contact plane.
     *
     * \param floor Contact plane.
     *
     */
    void resetCoMHeight(double height, const Contact & contact);

    /** Complete IPM inputs (ZMP and omega) from CoM and contact plane.
     *
     * \param plane Contact plane.
     *
     */
    void completeIPM(const Contact & plane);

    /** Centroidal Moment Pivot of last IPM integration.
     *
     */
    const Eigen::Vector3d & zmp() const
    {
      return zmp_;
    }

    /** Get CoM position of the inverted pendulum model.
     *
     */
    const Eigen::Vector3d & com() const
    {
      return com_;
    }

    /** Get CoM velocity of the inverted pendulum model.
     *
     */
    const Eigen::Vector3d & comd() const
    {
      return comd_;
    }

    /** Get CoM acceleration of the inverted pendulum.
     *
     */
    const Eigen::Vector3d & comdd() const
    {
      return comdd_;
    }

    /** Instantaneous Divergent Component of Motion.
     *
     */
    Eigen::Vector3d dcm() const
    {
      return com_ + comd_ / omega_;
    }

    /** Natural frequency of last IPM integration.
     *
     */
    double omega() const
    {
      return omega_;
    }

  protected:
    Eigen::Vector3d com_;
    Eigen::Vector3d comd_;
    Eigen::Vector3d comdd_;
    Eigen::Vector3d zmp_;
    double omega_;
  };
}
