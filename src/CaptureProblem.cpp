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

#include <chrono>

#include <capture_walking/CaptureProblem.h>
#include <capture_walking/CaptureSolution.h>
#include <capture_walking/defs.h>
#include <capture_walking/utils/Interval.h>

namespace capture_walking
{
  namespace
  {
    constexpr double ALPHA_MAX = 1.;
    constexpr double ALPHA_MIN = 0.;
    constexpr double LAMBDA_MAX = 2.0 * world::GRAVITY;
    constexpr double LAMBDA_MIN = 0.1 * world::GRAVITY;
    constexpr double SAMPLES_PER_INTERVAL = 10;
  }

  CaptureProblem::CaptureProblem(unsigned nbSteps)
    : solution_(nbSteps),
      sqp_(static_cast<int>(nbSteps))
  {
    const double ds = 1. / nbSteps;
    Eigen::VectorXd delta(nbSteps);
    cps::RawProblem raw;
    double curSquare = 0.;
    for (unsigned i = 0; i < nbSteps; i++)
    {
      const double s_next = (i + 1) * ds;
      const double nextSquare = s_next * s_next;
      delta(i) = nextSquare - curSquare;
      curSquare = nextSquare;
    }
    raw.delta = delta;
    pb_.reset(new cps::Problem(raw));
    pb_->set_lambda_max(LAMBDA_MAX);
    pb_->set_lambda_min(LAMBDA_MIN);
    assert(pb_->size() == nbSteps);
  }

  void CaptureProblem::contacts(Contact initContact, Contact targetContact)
  {
    targetCoP_ = captureCoP(targetContact); // r_f in the paper
    initContact_ = initContact;

    Eigen::Vector3d v_x = initContact_.b().cross(world::e_z);
    Eigen::Vector3d v_y = initContact_.t().cross(world::e_z);
    double X = initContact_.halfLength;
    double Y = initContact_.halfWidth;
    double X_n = X * world::e_z.dot(initContact_.n());
    double Y_n = Y * world::e_z.dot(initContact_.n());

    F_area_.row(0) = v_x;
    F_area_.row(1) = -v_x;
    F_area_.row(2) = v_y;
    F_area_.row(3) = -v_y;
    p_area_ = Eigen::Vector4d(X_n, X_n, Y_n, Y_n) + F_area_ * initContact_.p();
    v_ineq_ = p_area_ - F_area_ * targetCoP_;
  }

  void CaptureProblem::updateProblem_(double alpha)
  {
    Eigen::Vector4d u_alpha, v_alpha;
    u_alpha = (1. - alpha) * p_area_ + F_area_ * (alpha * targetCoP_ - initCoM_);
    v_alpha = F_area_ * initCoMd_;
    Interval omegaInterval(std::sqrt(LAMBDA_MIN), std::sqrt(LAMBDA_MAX));
    omegaInterval.reduce(u_alpha, v_alpha);  // u_alpha * omega_i >= v_alpha
    pb_->set_init_omega_max(omegaInterval.upper);
    pb_->set_init_omega_min(omegaInterval.lower);

    Eigen::Vector3d initNormal = initContact_.n();
    double initZbar = initNormal.dot(initCoM_ - alpha * targetCoP_ - (1. - alpha) * initContact_.p()) / initNormal(2);
    double initZbarDeriv = initNormal.dot(initCoMd_) / initNormal(2);
    pb_->set_init_zbar(initZbar);
    pb_->set_init_zbar_deriv(initZbarDeriv);
  }

  bool CaptureProblem::isObviouslyInfeasible_()
  {
    if (pb_->init_omega_max() < pb_->init_omega_min())
    {
      return true;
    }
    else if (pb_->init_zbar() < 0.)
    {
      return true;
    }
    else if (pb_->lambda_max() < pb_->lambda_min())
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool CaptureProblem::solveWithFixedAlpha(double alpha)
  {
    bool solutionFound;
    updateProblem_(alpha);
    if (isObviouslyInfeasible_())
    {
      status_ = cps::SolverStatus::NoLinearlyFeasiblePoint;
      solutionFound = false;
    }
    else
    {
      status_ = sqp_.solve(*pb_);
      solutionFound = (status_ != cps::SolverStatus::Fail && status_ != cps::SolverStatus::NoLinearlyFeasiblePoint);
    }
    if (solutionFound)
    {
      solution_.update(*this, alpha, sqp_.x());
    }
    return solutionFound;
  }

  bool CaptureProblem::solveWithVariableAlpha(double desiredAlpha)
  {
    recomputeAlphaIntervals();

    Eigen::VectorXd bestSQPSolution;
    double bestAlpha = -1.;
    double bestCost = 1e5;
    cps::SolverStatus bestStatus = cps::SolverStatus::Fail;
    for (const Interval & interval : alphaIntervals_)
    {
      double alphaStep = interval.width() / SAMPLES_PER_INTERVAL;
      for (double alpha = interval.lower; alpha < interval.upper; alpha += alphaStep)
      {
        if (solveWithFixedAlpha(alpha))
        {
          constexpr double ALPHA_WEIGHT = 0.01;
          constexpr double POS_DCM_WEIGHT = 1.;
          constexpr double VAR_WEIGHT = 1.;
          double cost = ALPHA_WEIGHT * std::abs(alpha - desiredAlpha) + \
                        POS_DCM_WEIGHT * solution_.dcm_i.norm() + \
                        VAR_WEIGHT * solution_.varCost();
          if (cost < bestCost)
          {
            bestAlpha = alpha;
            bestCost = cost;
            bestSQPSolution = sqp_.x();
            bestStatus = status_;
          }
        }
      }
    }
    updateProblem_(bestAlpha);
    solution_.update(*this, bestAlpha, bestSQPSolution);
    status_ = bestStatus;
    return (bestCost < 0.9999e5);
  }

  double CaptureProblem::solveStepTime(double alpha)
  {
    if (!solveWithFixedAlpha(alpha))
    {
      return -1.;
    }
    solution_.computeStepTime();
    return solution_.stepTime();
  }

  double CaptureProblem::approxStepTimeDerivative(double alpha, double stepTime, double alphaStep)
  {
    double leftTime = solveStepTime(alpha - alphaStep);
    double rightTime = solveStepTime(alpha + alphaStep);
    if (leftTime < 0. && rightTime < 0.)
    {
      LOG_ERROR("Cannot compute step-time derivative at alpha=" << alpha << ", alphaStep=" << alphaStep);
      return 0.; // next search direction will be arbitrary
    }
    else if (leftTime < 0. && rightTime > 0.)
    {
      return (rightTime - stepTime) / alphaStep;
    }
    else if (leftTime > 0. && rightTime < 0.)
    {
      return (stepTime - leftTime) / alphaStep;
    }
    else // (leftTime > 0. && rightTime > 0.)
    {
      return (rightTime - leftTime) / (2 * alphaStep);
    }
  }

  bool CaptureProblem::solve()
  {
    constexpr double SEARCH_STEP_TIME_PREC = 0.01;
    constexpr double SEARCH_MAX_ALPHA_PREC = 1e-3;

    recomputeAlphaIntervals();

    std::vector<Interval> searchIntervals = alphaIntervals_;
    std::vector<std::pair<double, double>> candidates;
    while (!searchIntervals.empty())
    {
      Interval alphaInterval = searchIntervals.back();
      searchIntervals.pop_back();
      double middleAlpha = alphaInterval.middle();
      double middleStepTime = solveStepTime(middleAlpha);
      if (middleStepTime < 0.)
      {
        continue;
      }
      if (std::abs(middleStepTime - desiredStepTime_) < SEARCH_STEP_TIME_PREC ||
          std::abs(middleAlpha - alphaInterval.lower) < SEARCH_MAX_ALPHA_PREC)
      {
        candidates.emplace_back(middleAlpha, middleStepTime);
        continue;
      }
      double alphaStep = std::min(1e-4, alphaInterval.width() / 3);
      double deriv = approxStepTimeDerivative(middleAlpha, middleStepTime, alphaStep);
      if (deriv * (desiredStepTime_ - middleStepTime) < 0)
      {
        searchIntervals.emplace_back(alphaInterval.lower, middleAlpha);
      }
      else // (deriv * (desiredStepTime_ - middleStepTime) > 0)
      {
        searchIntervals.emplace_back(middleAlpha, alphaInterval.upper);
      }
    }

    Eigen::VectorXd bestSQPSolution;
    cps::SolverStatus bestStatus = cps::SolverStatus::Fail;
    double bestAlpha = -1.;
    double bestCost = 1e5;
    double bestStepTime = -1.;
    for (auto & pair : candidates)
    {
      double alpha = pair.first;
      double stepTime = pair.second;
      if (solveWithFixedAlpha(alpha))
      {
        constexpr double TIME_WEIGHT = 10.;
        constexpr double VAR_WEIGHT = 1.;
        double cost = TIME_WEIGHT * std::abs(desiredStepTime_ - stepTime) + \
                      VAR_WEIGHT * solution_.varCost();
        if (cost < bestCost)
        {
          bestAlpha = alpha;
          bestCost = cost;
          bestSQPSolution = sqp_.x();
          bestStatus = status_;
          bestStepTime = stepTime;
        }
      }
    }

    updateProblem_(bestAlpha);
    solution_.update(*this, bestAlpha, bestSQPSolution, bestStepTime, bestCost);
    status_ = bestStatus;
    return (bestCost < 0.9999e5);
  }

  void CaptureProblem::recomputeAlphaIntervals()
  {
    alphaIntervals_.clear();

    Eigen::Vector4d u_ineq, w_ineq;
    u_ineq = p_area_ - F_area_ * initCoM_;
    w_ineq = F_area_ * initCoMd_;

    // (u - alpha * v) * omega_i >= w     -- Equation (75) in the paper
    Eigen::VectorXd u(6);
    Eigen::VectorXd v(6);
    Eigen::VectorXd w(6);
    u << u_ineq, Eigen::Vector2d(+1., -1.);
    v << v_ineq_, Eigen::Vector2d(0., 0.);
    w << w_ineq, Eigen::Vector2d(+std::sqrt(LAMBDA_MIN), -std::sqrt(LAMBDA_MAX));

    const auto nbRows = u.size();
    std::vector<double> roots;
    roots.reserve(size_t(nbRows + 2));
    roots.push_back(ALPHA_MIN);
    roots.push_back(ALPHA_MAX);
    for (auto j = 0; j < nbRows; j++)
    {
      if (std::abs(u[j]) < std::abs(v[j]))  // use the fact that 0 < alpha < 1
      {
        const double root = u[j] / v[j];
        if (ALPHA_MIN < root && root < ALPHA_MAX)
        {
          roots.push_back(root);
        }
      }
    }
    std::sort(roots.begin(), roots.end());

    alphaMin_ = 1.;
    alphaMax_ = 0.;
    for (long rootId = long(roots.size()) - 2; rootId >= 0; rootId--)
    {
      unsigned uRootId = static_cast<unsigned>(rootId);
      Interval alphaInterval(roots[uRootId], roots[uRootId + 1]);
      double alphaSample = alphaInterval.middle();
      std::vector<long> lowerBounds, upperBounds;
      for (auto i = 0; i < nbRows; i++)
      {
        if (u[i] - alphaSample * v[i] >= 0)
        {
          lowerBounds.push_back(i);
        }
        else
        {
          upperBounds.push_back(i);
        }
      }
      std::vector<double> u_w, v_w;
      u_w.reserve(lowerBounds.size() * upperBounds.size());
      v_w.reserve(lowerBounds.size() * upperBounds.size());
      for (auto i : lowerBounds)
      {
        for (auto j : upperBounds)
        {
          v_w.push_back(v[i] * w[j] - v[j] * w[i]);
          u_w.push_back(u[i] * w[j] - u[j] * w[i]);
        }
      }
      // CAUTION: v_w * alpha >= u_w
      alphaInterval.reduce(v_w, u_w);
      //alphaInterval.pad(0.005);
      //alphaInterval.shrink(0.95);
      if (!alphaInterval.isEmpty())
      {
        alphaIntervals_.push_back(alphaInterval);
        if (alphaInterval.lower < alphaMin_)
        {
          alphaMin_ = alphaInterval.lower;
        }
        if (alphaInterval.upper > alphaMax_)
        {
          alphaMax_ = alphaInterval.upper;
        }
      }
    }
    if (alphaMin_ > alphaMax_)
    {
      alphaMax_ = -1.;
      alphaMin_ = -1.;
    }
  }

  void CaptureProblem::logAlphaIntervals() const
  {
    std::string s = "Alpha intervals:";
    for (auto it = alphaIntervals_.begin(); it != alphaIntervals_.end(); it++)
    {
      const auto interval = *it;
      s += " [" + std::to_string(interval.lower) + ", " + std::to_string(interval.upper) + "]";
    }
    LOG_INFO(s);
  }

  void CaptureProblem::logRawProblem() const
  {
    LOG_INFO("delta = [" << pb_->delta().transpose() << "];");
    LOG_INFO("init_omega_max = " << pb_->init_omega_max() << ";");
    LOG_INFO("init_omega_min = " << pb_->init_omega_min() << ";");
    LOG_INFO("init_zbar = " << pb_->init_zbar() << ";");
    LOG_INFO("init_zbar_deriv = " << pb_->init_zbar_deriv() << ";");
    LOG_INFO("lambda_max = " << pb_->lambda_max() << ";");
    LOG_INFO("lambda_min = " << pb_->lambda_min() << ";");
    LOG_INFO("target_height = " << pb_->target_height() << ";");
  }

  void CaptureProblem::logSolverStatus(bool logSuccess) const
  {
    switch (status_)
    {
      case cps::SolverStatus::Converge:
        if (logSuccess)
        {
          LOG_INFO("CaptureProblem: CPS converged");
        }
        break;
      case cps::SolverStatus::MaxIteration:
        LOG_WARNING("CaptureProblem: CPS reached maximum number of iterations");
        break;
      case cps::SolverStatus::LineSearchFailed:
        LOG_WARNING("CaptureProblem: SQP line search failed");
        break;
      case cps::SolverStatus::NoLinearlyFeasiblePoint:
        LOG_ERROR("CaptureProblem: problem not linearly feasible");
        break;
      case cps::SolverStatus::NumericallyEquivalentIterates:
        if (logSuccess)
        {
          LOG_INFO("CaptureProblem: CPS returned with numerically equivalent iterates");
        }
        break;
      case cps::SolverStatus::Fail:
        LOG_ERROR("CaptureProblem: CPS returned with fail status");
        break;
      default:
        break;
    }
  }
}
