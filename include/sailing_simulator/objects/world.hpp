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

#ifndef SAILING_SIMULATOR_OBJECTS_WORLD_HPP
#define SAILING_SIMULATOR_OBJECTS_WORLD_HPP

#include <Box2D/Box2D.h>
#include <include/sailing_simulator/objects/dynamics_components/engine_dynamics_component.hpp>
#include <include/sailing_simulator/objects/dynamics_components/rudder_dynamics_component.hpp>

#include "sailing_simulator/objects/game_object.hpp"
#include "sailing_simulator/objects/body_components/dynamic_body_component.hpp"
#include "sailing_simulator/objects/dynamics_components/composite_dynamics_component.hpp"
#include "sailing_simulator/objects/dynamics_components/wind_dynamics_component.hpp"
#include "sailing_simulator/objects/input_components/qt_motor_boat_input_component.hpp"
#include "sailing_simulator/objects/wind/base_wind.hpp"

namespace sailing_simulator {
namespace objects {
class World {
 public:
  World();

  World(double time_step, int velocity_iterations, int position_iterations);

  void step();

  void addMotorBoat(const b2PolygonShape& shape,
                    const b2Vec2& position) {
    auto body = std::make_shared<DynamicBodyComponent>(*this, shape, position);
    auto dynamics = std::make_shared<WindDynamicsComponent>(body);
    objects_.emplace_back(body, dynamics, nullptr);
  }

  void addMotorBoat(DynamicBodyComponent::Ptr body,
                    EngineDynamicsComponent::Ptr engine,
                    InputComponent::Ptr input) {
    auto wind_dynamics = std::make_shared<WindDynamicsComponent>(body);
    std::vector<DynamicsComponent::Ptr> dynamic_components = {wind_dynamics, engine};
    auto dynamics = std::make_shared<CompositeDynamicsComponent>(dynamic_components);
    objects_.emplace_back(body, dynamics, input);
  }

  void addSailingBoat(DynamicBodyComponent::Ptr body,
                      RudderDynamicsComponent::Ptr rudder,
                      InputComponent::Ptr input) {
    auto wind_dynamics = std::make_shared<WindDynamicsComponent>(body);
    std::vector<DynamicsComponent::Ptr> dynamic_components = {wind_dynamics, rudder};
    auto dynamics = std::make_shared<CompositeDynamicsComponent>(dynamic_components);
    objects_.emplace_back(body, dynamics, input);
  }

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

  BaseWind::ConstPtr getWind() const {
    return wind_;
  }

  void setWind(BaseWind::Ptr wind) {
    wind_ = wind;
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

  std::vector<GameObject> objects_;

  BaseWind::Ptr wind_;

  double time_step_;
  int velocity_iterations_;
  int position_iterations_;
};

}  // namespace objects
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_OBJECTS_WORLD_HPP
