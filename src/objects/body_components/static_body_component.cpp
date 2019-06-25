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

#include "include/sailing_simulator/objects/body_components/static_body_component.hpp"

#include "sailing_simulator/objects/world.hpp"

namespace sailing_simulator {
namespace objects {
StaticBodyComponent::StaticBodyComponent(World& world, const b2PolygonShape& shape, const b2Vec2& position) {
  b2BodyDef body_def;
  body_def.type = b2_staticBody;
  body_def.position = position;
  body_def.active = true;

  body_ = world.getWorld().CreateBody(&body_def);

  b2FixtureDef fixture_def;
  fixture_def.shape = &shape;

  fixture_ = body_->CreateFixture(&fixture_def);

  shape_.resize(shape.m_count);
  for (int i = 0; i < shape.m_count; ++i) {
    shape_.at(i) = shape.m_vertices[i];
  }
}

void StaticBodyComponent::update(GameObject& object, World& world) {
}
}  // namespace objects
}  // namespace sailing_simulator
