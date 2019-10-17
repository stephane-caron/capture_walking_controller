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

#include <fstream>

#include <capture_walking/HorizontalMPCSolution.h>

namespace capture_walking
{
  using namespace HorizontalMPC;

  HorizontalMPCSolution::HorizontalMPCSolution(const Eigen::VectorXd & initState)
  {
    jerkTraj_ = Eigen::VectorXd::Zero(NB_STEPS * INPUT_SIZE);
    stateTraj_ = Eigen::VectorXd::Zero((NB_STEPS + 1) * STATE_SIZE);
    stateTraj_.head<STATE_SIZE>() = initState;
  }

  HorizontalMPCSolution::HorizontalMPCSolution(const Eigen::VectorXd & stateTraj, const Eigen::VectorXd & jerkTraj)
  {
    if (stateTraj.size() / STATE_SIZE != 1 + jerkTraj.size() / INPUT_SIZE)
    {
      LOG_ERROR("Invalid state/input sizes, respectively " << stateTraj.size() << " and " << jerkTraj.size());
    }
    jerkTraj_ = jerkTraj;
    stateTraj_ = stateTraj;
  }

  void HorizontalMPCSolution::integrate(Pendulum & pendulum, double dt)
  {
    if (playbackStep_ < NB_STEPS)
    {
      integratePlayback(pendulum, dt);
    }
    else // (playbackStep_ >= NB_STEPS)
    {
      integratePostPlayback(pendulum, dt);
    }
  }

  void HorizontalMPCSolution::integratePlayback(Pendulum & pendulum, double dt)
  {
    Eigen::Vector3d comddd;
    comddd.head<INPUT_SIZE>() = jerkTraj_.segment<INPUT_SIZE>(INPUT_SIZE * playbackStep_);
    comddd.z() = 0.;
    playbackTime_ += dt;
    if (playbackTime_ >= (playbackStep_ + 1) * SAMPLING_PERIOD)
    {
      playbackStep_++;
    }
    pendulum.integrateCoMJerk(comddd, dt);
  }

  void HorizontalMPCSolution::integratePostPlayback(Pendulum & pendulum, double dt)
  {
    Eigen::Vector3d comddd;
    Eigen::VectorXd lastState = stateTraj_.segment<STATE_SIZE>(STATE_SIZE * NB_STEPS);
    Eigen::Vector2d comd_f = lastState.segment<2>(2);
    Eigen::Vector2d comdd_f = lastState.segment<2>(4);
    if (std::abs(comd_f.x() * comdd_f.y() - comd_f.y() * comdd_f.x()) > 1e-4)
    {
      LOG_WARNING("HMPC terminal condition is not properly fulfilled");
    }
    double omega_f = -comd_f.dot(comdd_f) / comd_f.dot(comd_f);
    double lambda_f = std::pow(omega_f, 2);
    comddd = -omega_f * pendulum.comdd() - lambda_f * pendulum.comd();
    comddd.z() = 0.;
    pendulum.integrateCoMJerk(comddd, dt);
  }
}
