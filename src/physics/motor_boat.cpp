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

#include "sailing_simulator/physics/motor_boat.hpp"

namespace sailing_simulator {
namespace physics {

MotorBoat::MotorBoat(World& world,
                     const b2PolygonShape& shape,
                     const b2Vec2& position,
                     const b2EdgeShape& axis,
                     const b2Vec2& thrust_position,
                     double thrust_force)
    : DynamicObject(world, shape, position, axis),
      thrust_(false),
      thrust_position_(thrust_position),
      thrust_force_(thrust_force) {}

void MotorBoat::process(World& world) {
  DynamicObject::process(world);

  if (thrust_) {
    useThurst();
  }
}

void MotorBoat::useThurst() {
  b2Vec2 thrust_vector(std::cos(steer_position_) * thrust_force_,
                       std::sin(steer_position_) * thrust_force_);

  b2Vec2 thrust_position = body_->GetWorldPoint(thrust_position_);
  b2Vec2 thrust_force = body_->GetWorldVector(thrust_vector);

  body_->ApplyForce(thrust_force, thrust_position, true);
}
}  // namespace physics {
}  // namespace sailing_simulator
