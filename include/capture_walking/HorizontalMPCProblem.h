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

#include <copra/constraints.h>
#include <copra/costFunctions.h>
#include <copra/LMPC.h>
#include <copra/PreviewSystem.h>

#include <capture_walking/Contact.h>
#include <capture_walking/HorizontalMPC.h>
#include <capture_walking/HorizontalMPCSolution.h>
#include <capture_walking/defs.h>

namespace Eigen
{
  using HrepXd = std::pair<Eigen::MatrixXd, Eigen::VectorXd>;
}

namespace capture_walking
{
  /** Model Predictive Control problem for horizontal walking.
   *
   * This implementation is based on "Trajectory free linear model predictive
   * control for stable walking in the presence of strong perturbations"
   * (Wieber, Humanoids 2006) with the addition of terminal constraints.
   *
   */
  struct HorizontalMPCProblem
  {
    /** Initialize new problem.
     *
     */
    HorizontalMPCProblem();

    /** Read configuration from dictionary.
     *
     */
    void configure(const mc_rtc::Configuration &);

    /** Reset contacts.
     *
     * \param initContact Contact used during single-support phase.
     *
     * \param targetContact Contact used during double-support phases.
     *
     */
    void contacts(Contact initContact, Contact targetContact, Contact nextContact)
    {
      initContact_ = initContact;
      nextContact_ = nextContact;
      targetContact_ = targetContact;
    }

    /** Set the initial CoM state.
     *
     * \param state CoM state.
     *
     */
    void initState(const Pendulum & state)
    {
      initState_ = Eigen::VectorXd(6);
      initState_ << 
        state.com().head<2>(), 
        state.comd().head<2>(),
        state.comdd().head<2>();
    }

    /** Get solution vector.
     *
     */
    const HorizontalMPCSolution & solution()
    {
      return solution_;
    }

    /** Set duration of the initial single-support phase.
     *
     * \param initSupportDuration First SSP duration.
     *
     * \param doubleSupportDuration First DSP duration.
     *
     * \param targetSupportDuration Second SSP duration.
     *
     * Phase durations don't have to sum up to the total duration of the
     * preview horizon.
     *
     * If their sum is below total duration, there are two outcomes: if there
     * is a target support phase, a second DSP phase is added from the target
     * contact to the next (full preview mode); otherwise, the first DSP phase
     * is extended until the end of the preview horizon (half preview mode).
     *
     * If their sum exceeds total duration, phase durations are trimmed
     * starting from the last one.
     */
    void phaseDurations(double initSupportDuration, double doubleSupportDuration, double targetSupportDuration);

    /** Set the target CoM height.
     *
     */
    void comHeight(double height)
    {
      comHeight_ = height;
      zeta_ = height / world::GRAVITY;
      double omegaInv = std::sqrt(zeta_);
      dcmFromState_ << 
        1, 0, omegaInv, 0, 0, 0,
        0, 1, 0, omegaInv, 0, 0;
      zmpFromState_ <<
        1, 0, 0, 0, -zeta_, 0,
        0, 1, 0, 0, 0, -zeta_;
    }

    /** Solve the horizontal MPC problem.
     *
     * \returns solutionFound Did the solver find a solution?
     *
     */
    bool solve();

    /** Write problem and solution to Python script.
     *
     * \param suffix File name suffix.
     *
     */
    void writePython(const std::string & suffix = "");

  private:
    Eigen::HrepXd getSingleSupportHrep(const Contact & contact);

    void computeZMPRef();

    void updateTerminalConstraint();

    void updateZMPConstraint();

    void updateJerkCost();

    void updateVelCost();

    void updateZMPCost();

    void writePythonContact(const Contact & contact, const std::string & label);

    void writePythonSerializedVector(const Eigen::VectorXd & vec, const std::string & label, unsigned index, unsigned nbChunks);

    void writePythonSolution();

  public:
    Eigen::Vector2d velWeights = {10., 10.};
    double jerkWeight = 1.;
    double zmpWeight = 1000.;

  private:
    Contact initContact_;
    Contact nextContact_;
    Contact targetContact_;
    Eigen::HrepXd hreps_[4];
    Eigen::Matrix<double, 2 * (HorizontalMPC::NB_STEPS + 1), 1> velRef_;
    Eigen::Matrix<double, 2 * (HorizontalMPC::NB_STEPS + 1), 1> zmpRef_;
    Eigen::Matrix<double, 2, HorizontalMPC::STATE_SIZE> dcmFromState_;
    Eigen::Matrix<double, 2, HorizontalMPC::STATE_SIZE> velFromState_;
    Eigen::Matrix<double, 2, HorizontalMPC::STATE_SIZE> zmpFromState_;
    Eigen::VectorXd initState_;
    HorizontalMPCSolution solution_;
    double comHeight_;
    double zeta_;
    std::ofstream pyScript_;
    std::shared_ptr<copra::ControlCost> jerkCost_;
    std::shared_ptr<copra::PreviewSystem> previewSystem_;
    std::shared_ptr<copra::TrajectoryConstraint> termDCMCons_;
    std::shared_ptr<copra::TrajectoryConstraint> termZMPCons_;
    std::shared_ptr<copra::TrajectoryConstraint> zmpCons_;
    std::shared_ptr<copra::TrajectoryCost> velCost_;
    std::shared_ptr<copra::TrajectoryCost> zmpCost_;
    unsigned indexToHrep[HorizontalMPC::NB_STEPS + 1];
    unsigned nbDoubleSupportSteps_;
    unsigned nbInitSupportSteps_;
    unsigned nbNextDoubleSupportSteps_;
    unsigned nbTargetSupportSteps_;
    unsigned nbWritePythonCalls_ = 0;
  };
}
