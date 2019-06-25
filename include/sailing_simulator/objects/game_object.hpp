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

#ifndef SAILING_SIMULATOR_OBJECTS_GAME_OBJECT_HPP
#define SAILING_SIMULATOR_OBJECTS_GAME_OBJECT_HPP

#include "include/sailing_simulator/objects/body_components/body_component.hpp"
#include "include/sailing_simulator/objects/dynamics_components/dynamics_component.hpp"
#include "include/sailing_simulator/objects/input_components/input_component.hpp"

namespace sailing_simulator {
namespace objects {
class GameObject {
 public:
  using Ptr = std::shared_ptr<GameObject>;
  using ConstPtr = std::shared_ptr<GameObject>;

  GameObject(BodyComponent::Ptr body_component,
             DynamicsComponent::Ptr dynamics_component,
             InputComponent::Ptr input_component)
    : body_component_(std::move(body_component)),
      dynamics_component_(std::move(dynamics_component)),
      input_component_(std::move(input_component)) {}

  virtual void update(World& world) {
    if (body_component_) {
      body_component_->update(*this, world);
    }

    if (dynamics_component_) {
      dynamics_component_->update(*this, world);
    }

    if (input_component_) {
      input_component_->update(*this);
    }
  }

  virtual ~GameObject() = default;
 protected:
  BodyComponent::Ptr body_component_;
  DynamicsComponent::Ptr dynamics_component_;
  InputComponent::Ptr input_component_;
};
}  // namespace objects
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_OBJECTS_GAME_OBJECT_HPP
