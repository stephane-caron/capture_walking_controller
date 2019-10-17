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

#include <capture_walking/Pendulum.h>
#include <capture_walking/Preview.h>
#include <capture_walking/defs.h>

namespace capture_walking
{
  struct CaptureProblem;

  /** Solution to a capture optimization problem
   *
   */
  struct CaptureSolution : public Preview
  {
    /** Initialize a new solution
     *
     * \param nbSteps Number of spatial discretization steps
     *
     */
    CaptureSolution(unsigned nbSteps = 10);

    /** Default copy constructor.
     *
     */
    CaptureSolution(const CaptureSolution &) = default;

    /** Update solution.
     *
     * \param pb Capture problem solved
     *
     * \param alpha_ Value of the external parameter for this instance
     *
     * \param phi_1_n Solution vector found by the SQP
     *
     */
    void update(const CaptureProblem & pb, double alpha_, Eigen::VectorXd phi_1_n);

    /** Update solution.
     *
     * \param pb Capture problem solved
     *
     * \param alpha_ Value of the external parameter for this instance
     *
     * \param phi_1_n Solution vector found by the SQP
     *
     * \param stepTime Time of contact switch.
     *
     * \param cost Value of external cost function saved by solver.
     *
     */
    void update(const CaptureProblem & pb, double alpha_, Eigen::VectorXd phi_1_n, double stepTime, double cost);

    /** Compute the times :math:`t_j` where :math:`s(t_j) = s_j`.
     *
     */
    void computeSwitchTimes();

    /** Reset switch times.
     *
     */
    void resetSwitchTimes()
    {
      switchTimes[0] = -1.;
    }

    /** Invert the function :math:`s \\mapsto \\phi(s)`.
     *
     * \param phi Value of the function :math:`\\phi(s) = s \\omega(s)`.
     *
     * @return Index `s` such that `phi(s) = phi`.
     *
     * Given the index `j` such that :math:`\\phi_j \\leq \\phi < \\phi_{j+1}`,
     * the important formula behind this function is: 
     *
     *   \\phi(s) = \\sqrt{\\phi_j + \\lambda_j (s^2 - s_j^2)}
     * 
     * See the paper for derivation details.
     *
     */
    double sFromPhi(double phi);

    /** Compute the time corresponding to a given path index. 
     *
     * \param Path index `s` between 0 and 1.
     *
     * \returns Time `t(s) > 0` in [s].
     *
     * Given the index `j` such that :math:`s_j \\leq s < s_{j+1}`, the important
     * formula behind this function is: 
     *
     *   t(s) = t_{j+1} + \\frac{1}{\\sqrt{\\lambda_j}} \\log\\left(
     *       \\frac{
     *           \\sqrt{\\phi_{i+1}} + \\sqrt{\\lambda_j} s_{j+1}}{
     *           \\sqrt{\\phi_{i+1} - \\lambda_j (s_{j+1}^2 - s^2)}
     *       + \\sqrt{\\lambda_j} s} \\right)
     *
     * See the paper for a derivation of this formula.
     *
     */
    double tFromS(double s);

    /** Compute the CoP switch time for one-step problems.
     *
     */
    void computeStepTime();

    /** Compute complete CoM trajectory.
     *
     * \note This function only applies to one-step capture solutions.
     *
     */
    std::vector<Eigen::Vector3d> computeCoMTrajectory();

    /** Square cost in pendulum stiffness variations.
     *
     * \note The scale of this cost is affected by the number of samples
     * nbSteps.
     *
     */
    double varCost() const
    {
      double totalCost = 0.;
      for (long int j = 0; j < nbSteps - 1; j++)
      {
        totalCost += pow(lambda[j + 1] - lambda[j], 2);
      }
      return totalCost;
    }

    /** Integrate playback on reference.
     *
     * \param state CoM state to integrate upon.
     *
     * \param dt Duration.
     *
     * \note This function only applies to one-step capture solutions.
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

    /** Get the damping value for a given time.
     *
     * \param t Time, zero being the beginning of the solution.
     *
     */
    double omega(double t);

    /** Time of contact switch in capture solution.
     *
     */
    inline double stepTime() const
    {
      return stepTime_;
    }

    /** Value of external cost function saved by solver.
     *
     */
    inline double cost() const
    {
      return cost_;
    }

  private:
    /** Get the DCM frequency for a given time.
     *
     * \param t Time, zero being the beginning of the solution.
     *
     * \param j Index such that switchTimes[j] <= t < switchTimes[j + 1].
     *
     */
    double omega(double t, unsigned j);

  public:
    Eigen::Vector3d com_i;
    Eigen::Vector3d com_f;
    Eigen::Vector3d comd_i;
    Eigen::Vector3d cop_f;
    Eigen::Vector3d cop_i;
    Eigen::Vector3d dcm_i;
    Eigen::VectorXd lambda;
    double alpha;
    double lambda_i;
    double omega_i;
    std::vector<double> phi;
    std::vector<double> svec;
    std::vector<double> switchTimes;
    unsigned nbSteps;

  private:
    bool playbackIsOver_ = false;
    double cost_ = 1e5;
    double stepTime_ = -1.;
  };
}
