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

/** Online estimator for the average and standard deviation of a time series of
 * scalar values.
 *
 */
struct AvgStdEstimator
{
  /** Reset estimator to empty series.
   *
   */
  void reset()
  {
    max_ = 0.;
    min_ = 0.;
    n_ = 0;
    squareTotal_ = 0.;
    total_ = 0.;
  }

  /* Add a new value of the time series.
   *
   * \param x New value.
   *
   */
  void add(double x)
  {
    n_++;
    total_ += x;
    squareTotal_ += x * x;
    if (x > max_)
    {
      max_ = x;
    }
    if (x < min_)
    {
      min_ = x;
    }
  }

  /** Number of samples.
   *
   */
  unsigned n()
  {
    return n_;
  }

  /** Average of the time series.
   *
   */
  double avg()
  {
    if (n_ < 1)
    {
      return 0.;
    }
    return total_ / n_;
  }

  /** Standard deviation of the time series.
   *
   */
  double std()
  {
    if (n_ <= 1)
    {
      return 0.;
    }
    double unbiased = std::sqrt(double(n_) / (n_ - 1));
    return unbiased * std::sqrt(squareTotal_ / n_ - pow(avg(), 2));
  }

  /** Printout series statistics.
   *
   */
  std::string str()
  {
    std::ostringstream ss;
    ss << avg() << " +/- " << std() << " (max: " << max_ << ", min: " << min_ << " over " << n_ << " items)";
    return ss.str();
  }

private:
  double max_ = 0.;
  double min_ = 0.;
  double squareTotal_ = 0.;
  double total_ = 0.;
  unsigned n_ = 0;
};
