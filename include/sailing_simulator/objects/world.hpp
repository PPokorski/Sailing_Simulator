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

#ifndef SAILING_SIMULATOR_OBJECTS_WORLD_HPP
#define SAILING_SIMULATOR_OBJECTS_WORLD_HPP

#include <Box2D/Box2D.h>

#include "sailing_simulator/physics/base_wind.hpp"

namespace sailing_simulator {
namespace objects {
class World {
 public:
  World();

  World(double time_step, int velocity_iterations, int position_iterations);

  void step();

  const b2World& getWorld() const {
    return world_;
  }

  b2World& getWorld() {
    return world_;
  }

  const b2Body& getGroundBody() const {
    return *ground_body_;
  }

  b2Body& getGroundBody() {
    return *ground_body_;
  }

  physics::BaseWind::ConstPtr getWind() const {
    return wind_;
  }

  double getTimeStep() const {
    return time_step_;
  }

  void setTimeStep(double time_step) {
    time_step_ = time_step;
  }

  int getVelocityIterations() const {
    return velocity_iterations_;
  }

  void setVelocityIterations(int velocity_iterations) {
    velocity_iterations_ = velocity_iterations;
  }

  int getPositionIterations() const {
    return position_iterations_;
  }

  void setPositionIterations(int position_iterations) {
    position_iterations_ = position_iterations;
  }

 private:
  b2World world_;
  b2Body* ground_body_;

  physics::BaseWind::Ptr wind_;

  double time_step_;
  int velocity_iterations_;
  int position_iterations_;
};

}  // namespace objects
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_OBJECTS_WORLD_HPP
