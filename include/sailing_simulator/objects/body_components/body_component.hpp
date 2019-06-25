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

#ifndef SAILING_SIMULATOR_OBJECTS_BODY_COMPONENT_HPP
#define SAILING_SIMULATOR_OBJECTS_BODY_COMPONENT_HPP

#include <vector>

#include "Box2D/Box2D.h"

namespace sailing_simulator {
namespace objects {
class GameObject;
class World;

class BodyComponent {
 public:
  using Ptr = std::shared_ptr<BodyComponent>;
  using ConstPtr = std::shared_ptr<BodyComponent>;

  virtual void update(GameObject& object, World& world) = 0;

  virtual ~BodyComponent() = default;

  virtual void applyWorldForce(const b2Vec2& world_force, const b2Vec2& world_position) {
    body_->ApplyForce(world_force, world_position, true);
  }

  virtual void applyLocalForce(const b2Vec2& local_force, const b2Vec2& local_position) {
    b2Vec2 global_force = body_->GetWorldVector(local_force);
    b2Vec2 global_position = body_->GetWorldPoint(local_position);

    applyWorldForce(global_force, global_position);
  }

  b2Vec2 getGlobalVector(const b2Vec2& local_vector) const {
    return body_->GetWorldVector(local_vector);
  }

  b2Vec2 getLocalVector(const b2Vec2& global_vector) const {
    return body_->GetLocalVector(global_vector);
  }

  b2Vec2 getGlobalPosition(const b2Vec2& local_position) const {
    return body_->GetWorldPoint(local_position);
  }

  b2Vec2 getLocalPosition(const b2Vec2& global_position) const {
    return body_->GetLocalPoint(global_position);
  }

  b2Vec2 getPosition() const {
    return body_->GetPosition();
  }

  double getOrientation() const {
    return body_->GetAngle();
  }

  b2Vec2 getLinearVelocity() const {
    return body_->GetLinearVelocity();
  }

  void setLinearVelocity(const b2Vec2& linear_velocity) {
    body_->SetLinearVelocity(linear_velocity);
  }

  b2Vec2 getLinearVelocityFromLocalPoint(const b2Vec2& local_point) const {
    return body_->GetLinearVelocityFromLocalPoint(local_point);
  }

  b2Vec2 getLinearVelocityFromGlobalPoint(const b2Vec2& global_point) const {
    return body_->GetLinearVelocityFromWorldPoint(global_point);
  }

  double getAngularVelocity() const {
    return body_->GetAngularVelocity();
  }

  void setAngularVelocity(double angular_velocity) {
    body_->SetAngularVelocity(angular_velocity);
  }

  std::vector<b2Vec2> getShape() const {
    return shape_;
  }

 protected:
  b2Body* body_;
  b2Fixture* fixture_;
  std::vector<b2Vec2> shape_;
};
}  // namespace objects
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_OBJECTS_BODY_COMPONENT_HPP
