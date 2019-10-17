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

#include <Eigen/Dense>

namespace capture_walking
{
  /** Interval of real numbers.
   *
   */
  struct Interval
  {
    /** Default constructor.
     *
     */
    Interval(double lower, double upper)
      : lower(lower), upper(upper)
    {
    }

    /** Return true if the interval is empty.
     *
     */
    bool isEmpty() const
    {
      return (lower > upper);
    }

    /** Return the center of the interval.
     *
     */
    double middle() const
    {
      return 0.5 * (lower + upper);
    }

    /** Pad interval bounds by a small amount.
     *
     * \param epsilon Padding value.
     *
     */
    void pad(double epsilon)
    {
      lower += epsilon;
      upper -= epsilon;
    }

    /** Shrink from the middle to a given amount.
     *
     * \param amount Shrinking amount, between 0 and 1.
     *
     */
    void shrink(double amount)
    {
      assert(0. < amount && amount < 1.);
      double m = middle();
      double d = 0.5 * amount * width();
      lower = m - d;
      upper = m + d;
    }

    /** Apply a set of inequalities `u * x >= v` to the interval `lower <= x <=
     * upper`, resulting in a new interval `new_x_min <= x <= new_x_max`.
     *
     * \param u Vector of left-hand side inequality coordinates
     *
     * \param v Vector of right-hand side inequality coordinates
     *
     */
    template<typename T>
    void reduce(const T & u, const T & v)
    {
      constexpr double EPSILON = 1e-3;
      const size_t nbRows = u.size();
      for (size_t i = 0; i < nbRows; i++)
      {
        if (u[i] > +EPSILON)
        {
          lower = std::max(lower, v[i] / u[i]);
        }
        else if (u[i] < -EPSILON)
        {
          upper = std::min(upper, v[i] / u[i]);
        }
        else if (v[i] > +EPSILON)
        {
          // u[i] is almost 0, therefore the interval is empty unless v[i] is negative
          lower = 1.0;
          upper = 0.0;
          break;
        }
      }
    }

    /** Return the width of the interval.
     *
     */
    double width() const
    {
      return (upper - lower);
    }

  public:
    double lower;
    double upper;
  };
}
