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

#include "include/sailing_simulator/constants.hpp"
#include "include/sailing_simulator/physics/world.hpp"

namespace sailing_simulator {
namespace physics {
World::World()
    : World(1 / FRAMERATE, VELOCITY_ITERATIONS, POSITION_ITERATIONS, nullptr) {}

World::World(const BaseWind::Ptr& wind) : World(1 / FRAMERATE, VELOCITY_ITERATIONS, POSITION_ITERATIONS, wind) {}

World::World(double time_step, int velocity_iterations, int position_iterations, const BaseWind::Ptr& wind)
    : world_(b2Vec2(0.0f, 0.0f)),
      time_step_(time_step),
      velocity_iterations_(velocity_iterations),
      position_iterations_(position_iterations),
      wind_(wind) {
  b2BodyDef ground_definition;
  ground_definition.position.SetZero();
  ground_body_ = world_.CreateBody(&ground_definition);
}

void World::step() {
  for (auto& object : objects_) {
    object->process(*this);
  }
  world_.Step(time_step_, velocity_iterations_, position_iterations_);
}
}  // namespace physics {
}  // namespace sailing_simulator
