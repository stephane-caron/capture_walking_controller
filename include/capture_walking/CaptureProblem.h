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

#include <cps/Problem.h>
#include <cps/SQP.h>

#include <capture_walking/CaptureSolution.h>
#include <capture_walking/Contact.h>
#include <capture_walking/defs.h>
#include <capture_walking/utils/Interval.h>

namespace capture_walking
{
  /** General capture problem for the inverted pendulum mode.
   *
   */
  struct CaptureProblem
  {
    /** Initialize a new capture problem.
     *
     * \param nbSteps Number of discretization steps 
     *
     */
    CaptureProblem(unsigned nbSteps = 10);

    /** Reset contacts.
     *
     * \param initContact Contact used during the first phase of the motion.
     *
     * \param targetContact Contact used during the last phase of the motion.
     *
     */
    void contacts(Contact initContact, Contact targetContact);

    /** Print intervals of feasible values for the external parameter alpha.
     *
     */
    void logAlphaIntervals() const;

    /** Log the CPS RawProblem for further inspection.
     *
     */
    void logRawProblem() const;

    /** Log solver status for the last call to CPS.
     *
     * \param logSuccess Print log message even when solver is successful.
     *
     */
    void logSolverStatus(bool logSuccess = false) const;

    /** Solve the capture problem with an external optimization over alpha.
     *
     * \returns solutionFound Did the solver find a solution?
     *
     */
    bool solve();

    /** Get the vector of discretized square differences.
     *
     */
    inline const Eigen::VectorXd & delta() const
    {
      return pb_->delta();
    }

    /** Get desired step time.
     *
     */
    inline double desiredStepTime() const
    {
      return desiredStepTime_;
    }

    /** Get the initial CoM position.
     *
     */
    inline const Eigen::Vector3d & initCoM() const
    {
      return initCoM_;
    }

    /** Get the initial CoM velocity.
     *
     */
    inline const Eigen::Vector3d & initCoMVel() const
    {
      return initCoMd_;
    }

    /** Get initial contact.
     *
     */
    inline const Contact & initContact() const
    {
      return initContact_;
    }

    /** Set the initial CoM state.
     *
     * \param pendulum Inverted pendulum instance.
     *
     */
    inline void initState(const Pendulum & pendulum)
    {
      initCoM_ = pendulum.com();
      initCoMd_ = pendulum.comd();
    }

    /** Get the initial CoM height.
     *
     */
    inline double initZbar() const
    {
      return pb_->init_zbar();
    }

    /** Get the initial derivative of CoM height.
     *
     */
    inline double initZbarDeriv() const
    {
      return pb_->init_zbar_deriv();
    }

    /** Get the solution to the last problem to solve().
     *
     */
    inline const CaptureSolution & solution()
    {
      return solution_;
    }

    /** Set desired step time.
     *
     * \param desiredStepTime Time of contact switch.
     *
     */
    inline void stepTime(double desiredStepTime)
    {
      desiredStepTime_ = desiredStepTime;
    }

    /** Get target CoM in capture state.
     *
     */
    inline Eigen::Vector3d targetCoM() const
    {
      return targetCoP_ + targetHeight() * world::e_z;
    }

    /** Get target CoP in capture state.
     *
     */
    inline const Eigen::Vector3d & targetCoP() const
    {
      return targetCoP_;
    }

    /** Get target CoM height above contact in capture state.
     *
     */
    inline double targetHeight() const
    {
      return pb_->target_height();
    }

    /** Set target CoM height in capture state.
     *
     */
    inline void targetHeight(double height)
    {
      pb_->set_target_height(height);
    }

  private:
    /** Shallow check on problem feasibility.
     *
     * NB: to be removed when CPS checks are fixed.
     *
     */
    bool isObviouslyInfeasible_();

    /** Compute the intervals of feasible values for the external
     * parameter alpha given the current feasibility conditions.
     *
     */
    void recomputeAlphaIntervals();

    /** Update problem after any update to the pendulum state or external
     * parameter.
     *
     * \param alpha Value of the external parameter.
     *
     */
    void updateProblem_(double alpha);

    /** Compute step time for a given alpha.
     *
     * \param alpha Value of the external parameter.
     *
     */
    double solveStepTime(double alpha);

    /** Solve the capture problem for a given value of the external parameter.
     *
     * \param alpha Value of the external parameter.
     *
     * \returns solutionFound Did the solver find a solution?
     *
     */
    bool solveWithFixedAlpha(double alpha);

    /** Solve the capture problem with an external optimization over alpha.
     *
     * \param desiredAlpha Default value for alpha for regularization.
     *
     * \returns solutionFound Did the solver find a solution?
     *
     */
    bool solveWithVariableAlpha(double desiredAlpha = 0.5);

    /** Approximate step-time derivative by alpha.
     *
     * \param alpha Value of the external parameter.
     *
     * \param stepTime Step-time value at alpha.
     *
     * \param alphaStep Small alpha variation used in finite difference.
     *
     */
    double approxStepTimeDerivative(double alpha, double stepTime, double alphaStep);

    /** CoP target for capture problems.
     *
     * \param contact Capture contact.
     *
     */
    inline Eigen::Vector3d captureCoP(const Contact & contact) const
    {
      Eigen::Vector3d cop = contact.anklePos();
      cop += ankleToTargetCoP.x() * contact.t();
      double sign = (contact.surfaceName == "LeftFootCenter") ? -1. :
        (contact.surfaceName == "RightFootCenter") ? +1. : 0.;
      cop += sign * ankleToTargetCoP.y() * contact.b();
      return cop;
    }

  public:
    Eigen::Vector2d ankleToTargetCoP = {0.0, 0.025};

  private:
    CaptureSolution solution_;
    Contact initContact_;
    Eigen::Matrix<double, 4, 3> F_area_;
    Eigen::Vector3d initCoM_;
    Eigen::Vector3d initCoMd_;
    Eigen::Vector3d targetCoP_;
    Eigen::Vector4d p_area_;
    Eigen::Vector4d v_ineq_;
    cps::SQP sqp_;
    cps::SolverStatus status_;
    double alphaMax_;
    double alphaMin_;
    double desiredStepTime_;
    std::shared_ptr<cps::Problem> pb_;
    std::vector<Interval> alphaIntervals_;
  };
}
