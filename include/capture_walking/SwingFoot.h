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
#include <mc_rtc/log/Logger.h>

#include <capture_walking/utils/polynomials.h>

namespace capture_walking
{
  /** Swing foot interpolator.
   *
   */
  struct SwingFoot
  {
    /** Initialize with default values.
     *
     */
    SwingFoot();

    /** Recompute swing foot trajectory for a new pair of contacts.
     *
     * \param initPose Pose that the swing foot starts from.
     *
     * \param targetPose Target pose the swing foot lands onto.
     *
     * \param duration Duration of the trajectory.
     *
     * \param height Apex height of swing trajectory.
     *
     */
    void reset(const sva::PTransformd & initPose, const sva::PTransformd & targetPose, double duration, double height);

    /** Add swing foot entries to log.
     *
     * \param logger Logger.
     *
     */
    void addLogEntries(mc_rtc::Logger & logger);

    /** Remove swing foot entries from log.
     *
     * \param logger Logger.
     *
     */
    void removeLogEntries(mc_rtc::Logger & logger);

    /** Progress by dt along the swing foot trajectory.
     *
     * \param dt Integration step.
     *
     */
    void integrate(double dt);

    /** Timing remaining until heel strike.
     *
     */
    double remTime() const
    {
      return (duration_ - playback_);
    }

    /** Get current pose as Plucker transform.
     *
     */
    sva::PTransformd pose() const
    {
      return sva::PTransformd(ori_, pos_);
    }

    /** Get current velocity as motion vector.
     *
     */
    sva::MotionVecd vel() const
    {
      return sva::MotionVecd({0., 0., 0.}, vel_);
    }

    /** Get current acceleration as motion vector.
     *
     */
    sva::MotionVecd accel() const
    {
      return sva::MotionVecd({0., 0., 0.}, accel_);
    }

    /** Get current swing foot height.
     *
     */
    double height()
    {
      return pos_.z() - initPose_.translation().z();
    }

    /** Fraction at end of trajectory for landing.
     *
     */
    void landingRatio(double ratio)
    {
      landingRatio_ = ratio;
    }

    /** Upward pitch angle before landing.
     *
     * \param pitch Pitch angle.
     *
     */
    void landingPitch(double pitch)
    {
      landingPitch_ = pitch;
    }

    /** Offset applied to horizontal position after takeoff.
     *
     * \param offset Offset.
     *
     */
    void takeoffOffset(const Eigen::Vector3d & offset)
    {
      takeoffOffset_ = offset;
    }

    /** Downward pitch angle after takeoff.
     *
     * \param pitch Pitch angle.
     *
     */
    void takeoffPitch(double pitch)
    {
      takeoffPitch_ = pitch;
    }

    /** Fraction at beginning of trajectory for takeoff.
     *
     */
    void takeoffRatio(double ratio)
    {
      takeoffRatio_ = ratio;
    }

  private:
    /** Update pose to a given playback time.
     *
     * \param t Playback time.
     *
     */
    void updatePose(double t);

    /** Update altitude to a given playback time.
     *
     * \param t Playback time.
     *
     */
    void updateZ(double t);

    /** Update xy-position to a given playback time.
     *
     * \param t Playback time.
     *
     */
    void updateXY(double t);

    /** Update pitch angle to a given playback time.
     *
     * \param t Playback time.
     *
     */
    void updatePitch(double t);

  private:
    Eigen::Quaterniond ori_;
    Eigen::Vector3d accel_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    RetimedPolynomial<QuinticHermitePolynomial, Eigen::Vector2d> xyTakeoffChunk_;
    RetimedPolynomial<QuinticHermitePolynomial, Eigen::Vector2d> xyAerialChunk_;
    RetimedPolynomial<QuinticHermitePolynomial, double> pitchTakeoffChunk_;
    RetimedPolynomial<QuinticHermitePolynomial, double> pitchAerialChunk1_;
    RetimedPolynomial<QuinticHermitePolynomial, double> pitchAerialChunk2_;
    RetimedPolynomial<QuinticHermitePolynomial, double> pitchLandingChunk_;
    RetimedPolynomial<QuinticHermitePolynomial, double> zFirstChunk_;
    RetimedPolynomial<QuinticHermitePolynomial, double> zSecondChunk_;
    Eigen::Vector3d takeoffOffset_;
    double aerialStart_;
    double duration_;
    double height_;
    double landingPitch_;
    double landingRatio_;
    double pitch_;
    double playback_;
    double takeoffPitch_;
    double takeoffRatio_;
    sva::PTransformd airPose_;
    sva::PTransformd initPose_;
    sva::PTransformd targetPose_;
    sva::PTransformd touchdownPose_;
  };
}
