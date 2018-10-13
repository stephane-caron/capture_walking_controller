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
