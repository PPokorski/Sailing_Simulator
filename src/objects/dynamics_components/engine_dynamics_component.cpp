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

#include "sailing_simulator/objects/dynamics_components/engine_dynamics_component.hpp"

#include "sailing_simulator/constants.hpp"
#include "sailing_simulator/objects/world.hpp"

namespace sailing_simulator {
namespace objects {
EngineDynamicsComponent::EngineDynamicsComponent(DynamicBodyComponent::Ptr body,
                                                 double max_thrust_forward,
                                                 double max_thrust_backwards,
                                                 const b2Vec2& thrust_position)
    : body_(body),
      max_thrust_forward_(max_thrust_forward),
      max_thrust_backwards_(max_thrust_backwards),
      thrust_position_(thrust_position),
      current_thrust_(0.0),
      engine_orientation_(0.0) {}


void EngineDynamicsComponent::update(GameObject& object, World& world) {
  if (auto body_ptr = body_.lock()) {
    b2Vec2 thrust_vector(std::cos(engine_orientation_) * current_thrust_,
                         std::sin(engine_orientation_) * current_thrust_);

    body_ptr->applyLocalForce(thrust_vector, thrust_position_);
  }
}
}  // namespace objects
}  // namespace sailing_simulator
