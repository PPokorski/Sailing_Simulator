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

#ifndef SAILING_SIMULATOR_OBJECTS_DYNAMICS_COMPONENTS_RUDDER_DYNAMICS_COMPONENT_HPP
#define SAILING_SIMULATOR_OBJECTS_DYNAMICS_COMPONENTS_RUDDER_DYNAMICS_COMPONENT_HPP

#include "sailing_simulator/objects/dynamics_components/dynamics_component.hpp"
#include "sailing_simulator/objects/body_components/dynamic_body_component.hpp"

namespace sailing_simulator {
namespace objects {
class RudderDynamicsComponent : public DynamicsComponent {
 public:
  using Ptr = std::shared_ptr<RudderDynamicsComponent>;
  using ConstPtr = std::shared_ptr<RudderDynamicsComponent>;

  static constexpr double MAX_RUDDER_ORIENTATION = M_PI_2;

  RudderDynamicsComponent(DynamicBodyComponent::Ptr body,
                          double force_multiplier,
                          double rudder_length,
                          const b2Vec2& rudder_position);

  void update(GameObject& object, World& world) override;

  double getRudderOrientation() const {
    return rudder_orientation_;
  }

  void setRudderOrientation(double rudder_orientation) {
    rudder_orientation_ = rudder_orientation;
    limitRudderOrientation();
  }

  void rotateRudder(double rudder_orientation_change) {
    rudder_orientation_ += rudder_orientation_change;
    limitRudderOrientation();
  }

 protected:
  void limitRudderOrientation() {
    rudder_orientation_ = b2Clamp(rudder_orientation_, -MAX_RUDDER_ORIENTATION, MAX_RUDDER_ORIENTATION);
  }
  double sinBetweenVectors(const b2Vec2& one, const b2Vec2& two) const;

  std::weak_ptr<DynamicBodyComponent> body_;

  double force_multiplier_;
  double rudder_length_;

  b2Vec2 rudder_position_;

  double rudder_orientation_;
};
}  // namespace objects
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_OBJECTS_DYNAMICS_COMPONENTS_RUDDER_DYNAMICS_COMPONENT_HPP
