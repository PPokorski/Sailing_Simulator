/*********************************************************************
*
* BSD 3-Clause License
*
*  Copyright (c) 2019, Piotr Pokorski
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "sailing_simulator/physics/sailing_boat.hpp"

#include "sailing_simulator/physics/world.hpp"

namespace sailing_simulator {
namespace physics {

SailingBoat::SailingBoat(World& world,
                         const b2PolygonShape& shape,
                         const b2Vec2& position,
                         const b2EdgeShape& axis,
                         const std::vector<std::pair<double, double>>& polar_plot)
    : DynamicObject(world, shape, position, axis),
      polar_plot_(polar_plot) {
  if (polar_plot.empty()) {
    throw std::invalid_argument("Polar plot cannot be empty.");
  }
}

void SailingBoat::process(World& world) {
  DynamicObject::process(world);

  useSails(*world.getWind());
  useSteer();
}

void SailingBoat::useSails(const BaseWind& wind) {
  b2Vec2 wind_vector = wind.getWind(getPosition());

  double wind_force = getForceForAngle(getAngleToWind(wind_vector));

  b2Vec2 local_vector(wind_force, 0.0f);
  b2Vec2 global_force = body_->GetWorldVector(local_vector);

  body_->ApplyForce(global_force, body_->GetWorldCenter(), true);
}

void SailingBoat::useSteer() {
}

double SailingBoat::getAngleToWind(const b2Vec2& wind) {
  double inverse_wind_direction = std::atan2(-wind.y, -wind.x) + M_PI;

  return std::fmod(std::abs(getOrientation() - inverse_wind_direction), M_2_PI);
}

double SailingBoat::getForceForAngle(double angle) {
  if (angle < polar_plot_.front().first) {
    return polar_plot_.front().second;
  }

  for (auto angle_force = polar_plot_.begin(); angle_force != polar_plot_.end() - 1; ++angle_force) {
    double current_angle = angle_force->first;
    double next_angle = (angle_force + 1)->first;
    if (angle > current_angle && angle < next_angle) {
      double coefficient = (angle - current_angle) / (next_angle - current_angle);

      double current_force = angle_force->second;
      double next_force = (angle_force + 1)->second;

      return current_force + coefficient * (next_force - current_force);
    }
  }

  return polar_plot_.back().second;
}
}  // namespace physics {
}  // namespace sailing_simulator