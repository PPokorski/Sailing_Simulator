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

#ifndef TEST_TEST_DYNAMIC_BODY_COMPONENT_STUB_HPP
#define TEST_TEST_DYNAMIC_BODY_COMPONENT_STUB_HPP

#include "sailing_simulator/objects/body_components/dynamic_body_component.hpp"

namespace test {
class DynamicBodyComponentStub : public sailing_simulator::objects::DynamicBodyComponent {
 public:
  using Ptr = std::shared_ptr<DynamicBodyComponentStub>;
  using ConstPtr = std::shared_ptr<DynamicBodyComponentStub>;

  DynamicBodyComponentStub(sailing_simulator::objects::World& world,
                           const b2PolygonShape& shape,
                           const b2Vec2& position)
    : DynamicBodyComponent(world, shape, position) {}

  void applyWorldForce(const b2Vec2& world_force, const b2Vec2& world_position) override {
    last_global_force_ = world_force;
    last_global_force_position_ = world_position;
  }

  void applyLocalForce(const b2Vec2& local_force, const b2Vec2& local_position) override {
    last_local_force_ = local_force;
    last_local_force_position_ = local_position;

    DynamicBodyComponent::applyLocalForce(local_force, local_position);
  }

  b2Vec2 last_local_force_ = b2Vec2_zero, last_global_force_ = b2Vec2_zero;
  b2Vec2 last_local_force_position_ = b2Vec2_zero, last_global_force_position_ = b2Vec2_zero;
};
}  // namespace test

#endif //TEST_TEST_DYNAMIC_BODY_COMPONENT_STUB_HPP
