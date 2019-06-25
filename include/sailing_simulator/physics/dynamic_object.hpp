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

#ifndef SAILING_SIMULATOR_PHYSICS_DYNAMIC_OBJECT_HPP
#define SAILING_SIMULATOR_PHYSICS_DYNAMIC_OBJECT_HPP

#include "sailing_simulator/physics/base_wind.hpp"
#include "sailing_simulator/physics/object.hpp"

namespace sailing_simulator {
namespace physics {

b2Vec2 getWindForcePoint(const std::vector<b2Vec2>& polygon, const b2Vec2& wind_vector, const b2EdgeShape& axis);

class DynamicObject : public Object {
 public:
  using Ptr = std::shared_ptr<DynamicObject>;
  using ConstPtr = std::shared_ptr<const DynamicObject>;

  static constexpr double MAX_STEER_POSITION_ = M_PI_2;

  DynamicObject(World& world,
                const b2PolygonShape& shape,
                const b2Vec2& position,
                const b2EdgeShape& axis);

  void process(World& world) override;

  void moveSteer(double steer_position_change) {
    steer_position_ += steer_position_change;
    steer_position_ = b2Clamp(steer_position_, -MAX_STEER_POSITION_, MAX_STEER_POSITION_);
  }

  b2Vec2 getLinearVelocity() const {
    return body_->GetLinearVelocity();
  }

  float32 getAngularVelocity() const {
    return body_->GetAngularVelocity();
  }

  double getSteerPosition() const {
    return steer_position_;
  }

  void setSteerPosition(double steer_position) {
    steer_position_ = steer_position;
  }

  virtual ~DynamicObject() = default;

 protected:
  void createFrictionJoint(World& world);

  b2EdgeShape axis_;
  double steer_position_;
};
}  // namespace physics {
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_PHYSICS_DYNAMIC_OBJECT_HPP
