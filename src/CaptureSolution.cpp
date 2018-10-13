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

#include <capture_walking/CaptureSolution.h>
#include <capture_walking/CaptureProblem.h>

namespace capture_walking
{
  CaptureSolution::CaptureSolution(unsigned nbSteps)
    : lambda(nbSteps + 1),
      phi(nbSteps + 1),
      svec(nbSteps + 1),
      switchTimes(nbSteps),
      nbSteps(nbSteps)
  {
    double ds = 1. / nbSteps;
    for (unsigned i = 0; i <= nbSteps; i++)
    {
      svec[i] = i * ds;
    }
    phi[0] = 0.0;
    resetSwitchTimes();
  }

  void CaptureSolution::update(const CaptureProblem & pb, double alpha_, Eigen::VectorXd phi_1_n)
  {
    resetSwitchTimes();
    assert(phi_1_n.size() == nbSteps);
    Eigen::VectorXd::Map(&phi[1], phi_1_n.size()) = phi_1_n;
    for (unsigned int j = 0; j < nbSteps; j++)
    {
      lambda[j] = (phi[j + 1] - phi[j]) / pb.delta()[j];
    }
    lambda[nbSteps] = lambda[nbSteps - 1];

    alpha = alpha_;
    com_f = pb.targetCoM();
    com_i = pb.initCoM();
    comd_i = pb.initCoMVel();
    cop_f = pb.targetCoP();
    lambda_i = lambda[nbSteps];
    omega_i = std::sqrt(phi[nbSteps]);

    Eigen::Vector3d pos_proj = com_i - world::e_z * pb.initZbar();
    Eigen::Vector3d vel_proj = comd_i - world::e_z * pb.initZbarDeriv();
    dcm_i = pos_proj + vel_proj / omega_i - cop_f;
    cop_i = cop_f + dcm_i / (1. - alpha);
  }

  void CaptureSolution::update(const CaptureProblem & pb, double alpha_, Eigen::VectorXd phi_1_n, double stepTime, double cost)
  {
    update(pb, alpha_, phi_1_n);
    stepTime_ = stepTime;
    cost_ = cost;
  }

  void CaptureSolution::computeSwitchTimes()
  {
    switchTimes[0] = 0.;
    double curSwitchTime = 0.;
    for (unsigned j = nbSteps - 1; j > 0; j--)
    {
      double sqrt_lambda_j = std::sqrt(lambda[j]);
      double num = std::sqrt(phi[j + 1]) + sqrt_lambda_j * svec[j + 1];
      double denom = std::sqrt(phi[j]) + sqrt_lambda_j * svec[j];
      curSwitchTime += std::log(num / denom) / sqrt_lambda_j;
      switchTimes[nbSteps - j] = curSwitchTime;
    }
  }

  void CaptureSolution::computeStepTime()
  {
    if (alpha < 0)
    {
      LOG_WARNING("Solution is unset (alpha < 0), no step time to compute");
      stepTime_ = -1.;
      return;
    }
    if (switchTimes[0] < 0.)
    {
      computeSwitchTimes();
    }
    double phi_switch = pow(alpha * omega_i, 2);
    double s_switch = sFromPhi(phi_switch);
    stepTime_ = tFromS(s_switch);
  }

  double CaptureSolution::sFromPhi(double phiValue)
  {
    if (phiValue < -1e-5 || phiValue > phi[nbSteps])
    {
      LOG_ERROR("Value phi = " << phiValue << " out of range [0, " << phi[nbSteps] << "]");
      return -1.;
    }
    auto it = std::upper_bound(phi.begin(), phi.end(), phiValue);
    long j = std::distance(phi.begin(), it) - 1;
    assert (j >= 0 && phi[j] <= phiValue && (j == nbSteps || phiValue < phi[j + 1]));
    unsigned j_ = static_cast<unsigned>(j); // avoid conversion warning
    double s_sq = (svec[j_] * svec[j_]) + (phiValue - phi[j_]) / lambda[j];
    return std::sqrt(s_sq);
  }

  double CaptureSolution::tFromS(double s)
  {
    if (std::isnan(s) || s < -1e-5 || s > 1.)
    {
      LOG_ERROR("Value s = " << s << " out of range [0, 1]");
      return -1.;
    }
    auto it = std::upper_bound(svec.begin(), svec.end(), s);
    long j = std::distance(svec.begin(), it) - 1;
    assert (j >= 0 && svec[j] <= s && j < nbSteps && s < svec[j + 1]);
    unsigned j_ = static_cast<unsigned>(j); // avoid conversion warning
    double s_next = svec[j_ + 1];
    double t_next = switchTimes[nbSteps - (j_ + 1)];
    double sqrt_lambda_j = std::sqrt(lambda[j]);
    double num = std::sqrt(phi[j_ + 1]) + s_next * sqrt_lambda_j;
    double denom = std::sqrt(phi[j_ + 1] - lambda[j] * (std::pow(s_next, 2) - std::pow(s, 2))) + s * sqrt_lambda_j;
    return t_next + std::log(num / denom) / std::sqrt(lambda[j]);
  }

  std::vector<Eigen::Vector3d> CaptureSolution::computeCoMTrajectory()
  {
    if (alpha < 0.)
    {
      LOG_WARNING("Solution is unset (alpha < 0), no CoM trajectory");
      return {};
    }

    computeStepTime();

    double maxTime = switchTimes[nbSteps - 1] * 1.5;
    Pendulum state(com_i, comd_i);
    std::vector<Eigen::Vector3d> traj;

    //pendulum.cop(cop_i);
    traj.push_back(state.com());
    for (unsigned j = 0; j < nbSteps; j++)
    {
      double t_j = switchTimes[j];
      double lambda_j = lambda[nbSteps - j - 1];
      double t_next = (j < nbSteps - 1) ? switchTimes[j + 1] : maxTime;
      //pendulum.lambda(lambda_j);
      if (t_j <= stepTime_ && stepTime_ < t_next)
      {
        state.integrateIPM(cop_i, lambda_j, stepTime_ - t_j);
        traj.push_back(state.com());
        //pendulum.cop(cop_f);
        state.integrateIPM(cop_f, lambda_j, t_next - stepTime_);
      }
      else
      {
        state.integrateIPM(cop_i, lambda_j, t_next - t_j);
      }
      traj.push_back(state.com());
    }
    return traj;
  }

  double CaptureSolution::omega(double t)
  {
    auto it = std::upper_bound(switchTimes.begin(), switchTimes.end(), t);
    long j = std::distance(switchTimes.begin(), it) - 1;
    assert(j >= 0 && switchTimes[j] <= t && (j == nbSteps || t < switchTimes[j + 1]));
    return omega(t, static_cast<unsigned>(j));
  }

  double CaptureSolution::omega(double t, unsigned j)
  {
    // ASSUMPTION: switchTimes[j] <= t < switchTimes[j + 1]
    double lambda_j = lambda[nbSteps - j - 1];
    double omega_j = std::sqrt(phi[nbSteps - j]) / svec[nbSteps - j];
    double sqrt_lambda_j = std::sqrt(lambda_j);
    double x = sqrt_lambda_j / (t - switchTimes[j + 1]);
    double z = sqrt_lambda_j / omega_j;
    return sqrt_lambda_j * (1. - z * std::tanh(x)) / (z - std::tanh(x));
  }

  void CaptureSolution::integrate(Pendulum & state, double dt)
  {
    if (stepTime_ < 0.)
    {
      computeStepTime();
    }
    if (playbackIsOver_)
    {
      integratePostPlayback(state, dt);
    }
    else // playback remaining
    {
      integratePlayback(state, dt);
    }
  }

  void CaptureSolution::integratePlayback(Pendulum & state, double dt)
  {
    while (playbackStep_ < nbSteps - 1 && playbackTime_ >= switchTimes[playbackStep_ + 1])
    {
      playbackStep_++;
    }
    // switchTimes[playbackStep_] <= playbackTime_ < switchTimes[playbackStep_ + 1]
    double lambda_ = lambda[nbSteps - playbackStep_ - 1];
    if (playbackTime_ < stepTime_)
    {
      if (stepTime_ <= playbackTime_ + dt)
      {
        double toStep = stepTime_ - playbackTime_;
        state.integrateIPM(cop_i, lambda_, toStep);
        state.integrateIPM(cop_f, lambda_, dt - toStep);
      }
      else // (playbackTime_ + dt < stepTime_)
      {
        state.integrateIPM(cop_i, lambda_, dt);
      }
    }
    else // (playbackTime_ >= stepTime_)
    {
      if (playbackStep_ == nbSteps - 1 && state.comdd().dot(state.comd()) > -0.001)
      {
        playbackIsOver_ = true;
      }
      state.integrateIPM(cop_f, lambda_, dt);
    }
    playbackTime_ += dt;
  }

  void CaptureSolution::integratePostPlayback(Pendulum & state, double dt)
  {
    constexpr double s = 5., d = 2 * 2.23;
    Eigen::Vector3d comdd = s * (com_f - state.com()) - d * state.comd();
    double lambda_ = lambda[0];
    Eigen::Vector3d cop = state.com() + (world::gravity - comdd) / lambda_;
    state.integrateIPM(cop, lambda_, dt);
  }
}
