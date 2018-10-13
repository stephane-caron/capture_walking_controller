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

namespace capture_walking
{
  /** Foot sole properties.
   *
   */
  struct Sole
  {
    double friction = 0.7;
    double halfLength = 0.112; // [m]
    double halfWidth = 0.065; // [m]
  };
}

namespace mc_rtc
{
  template<>
  struct ConfigurationLoader<capture_walking::Sole>
  {
    static capture_walking::Sole load(const mc_rtc::Configuration & config)
    {
      capture_walking::Sole sole;
      config("friction", sole.friction);
      config("half_length", sole.halfLength);
      config("half_width", sole.halfWidth);
      return sole;
    }

    static mc_rtc::Configuration save(const capture_walking::Sole & sole)
    {
      mc_rtc::Configuration config;
      config.add("friction", sole.friction);
      config.add("half_length", sole.halfLength);
      config.add("half_width", sole.halfWidth);
      return config;
    }
  };
}
