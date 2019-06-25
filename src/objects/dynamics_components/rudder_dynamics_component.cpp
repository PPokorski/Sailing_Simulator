/*********************************************************************
*
* BSD 3-Clause License
*
*  Copyright (c) 2019, Piotr Pokorski, Piotr Rzewnicki
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

#include "sailing_simulator/constants.hpp"
#include "sailing_simulator/objects/dynamics_components/rudder_dynamics_component.hpp"

namespace sailing_simulator {
namespace objects {
RudderDynamicsComponent::RudderDynamicsComponent(DynamicBodyComponent::Ptr body,
                                                 double force_multiplier,
                                                 double rudder_length,
                                                 const b2Vec2& rudder_position)
    : body_(body),
      force_multiplier_(force_multiplier),
      rudder_length_(rudder_length),
      rudder_position_(rudder_position),
      rudder_orientation_(0.0) {}

void RudderDynamicsComponent::update(GameObject& object, World& world) {
  if (auto body_ptr = body_.lock()) {
    b2Vec2 relative_rudder_midpoint(-rudder_length_ * std::cos(rudder_orientation_ / 2.0),
                                    -rudder_length_ * std::sin(rudder_orientation_ / 2.0));

    b2Vec2 force_position = rudder_position_ + relative_rudder_midpoint;
    b2Vec2 midpoint_rudder_velocity = body_ptr->getLinearVelocityFromLocalPoint(force_position);
    if (midpoint_rudder_velocity.LengthSquared() < SMALL_VELOCITY) {
      return;
    }

    double force_value = force_multiplier_ *
                         rudder_length_ *
                         midpoint_rudder_velocity.LengthSquared() *
                         sinBetweenVectors(relative_rudder_midpoint, midpoint_rudder_velocity);
    double force_orientation = rudder_orientation_ + M_PI;
    b2Vec2 local_force(force_value * std::sin(force_orientation),
                       force_value * std::cos(force_orientation));

    body_ptr->applyLocalForce(local_force, force_position);
  }
}

double RudderDynamicsComponent::sinBetweenVectors(const b2Vec2& one, const b2Vec2& two) const {
  // From https://www.wikihow.com/Find-the-Angle-Between-Two-Vectors
  double cos = b2Dot(one, two) / (one.Length() * two.Length());

  return std::sqrt(1 - cos * cos);
}
}  // namespace objects
}  // namespace sailing_simulator