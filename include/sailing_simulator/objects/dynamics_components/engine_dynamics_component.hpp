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

#ifndef SAILING_SIMULATOR_OBJECTS_DYNAMICS_COMPONENTS_ENGINE_DYNAMICS_COMPONENT_HPP
#define SAILING_SIMULATOR_OBJECTS_DYNAMICS_COMPONENTS_ENGINE_DYNAMICS_COMPONENT_HPP

#include "sailing_simulator/objects/dynamics_components/dynamics_component.hpp"
#include "sailing_simulator/objects/body_components/dynamic_body_component.hpp"

namespace sailing_simulator {
namespace objects {
class EngineDynamicsComponent : public DynamicsComponent {
 public:
  using Ptr = std::shared_ptr<EngineDynamicsComponent>;
  using ConstPtr = std::shared_ptr<EngineDynamicsComponent>;

  static constexpr double MAX_ENGINE_ORIENTATION_ = M_PI_2;

  EngineDynamicsComponent(DynamicBodyComponent& body,
                          double max_thrust_forward,
                          double max_thrust_backwards,
                          const b2Vec2& thrust_position);

  void update(GameObject& object, World& world) override;

  double getMaxThrustForward() const {
    return max_thrust_forward_;
  }

  void setMaxThrustForward(double max_thrust_forward) {
    max_thrust_forward_ = max_thrust_forward;
  }

  double getMaxThrustBackwards() const {
    return max_thrust_backwards_;
  }

  void setMaxThrustBackwards(double max_thrust_backwards) {
    max_thrust_backwards_ = max_thrust_backwards;
  }

  const b2Vec2& getThrustPosition() const {
    return thrust_position_;
  }

  void setThrustPosition(const b2Vec2& thrust_position) {
    thrust_position_ = thrust_position;
  }

  double getCurrentThrust() const {
    return current_thrust_;
  }

  void setCurrentThrust(double current_thrust) {
    current_thrust_ = current_thrust;
    limitEngineThrust();
  }

  void changeThrust(double current_thrust_change) {
    current_thrust_ += current_thrust_change;
    limitEngineThrust();
  }

  double getEngineOrientation() const {
    return engine_orientation_;
  }

  void setEngineOrientation(double engine_orientation) {
    engine_orientation_ = engine_orientation;
    limitEngineOrientation();
  }

  void rotateEngine(double engine_orientation_change) {
    engine_orientation_ += engine_orientation_change;
    limitEngineOrientation();
  }

 protected:
  void limitEngineThrust() {
    current_thrust_ = b2Clamp(current_thrust_, -max_thrust_backwards_, max_thrust_forward_);
  }

  void limitEngineOrientation() {
    engine_orientation_ = b2Clamp(engine_orientation_, -MAX_ENGINE_ORIENTATION_, MAX_ENGINE_ORIENTATION_);
  }

  DynamicBodyComponent& body_;

  double max_thrust_forward_;
  double max_thrust_backwards_;

  b2Vec2 thrust_position_;

  double current_thrust_;
  double engine_orientation_;
};

}  // namespace objects
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_OBJECTS_DYNAMICS_COMPONENTS_ENGINE_DYNAMICS_COMPONENT_HPP
