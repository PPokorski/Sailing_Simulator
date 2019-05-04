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

#include <gtest/gtest.h>

#include "sailing_simulator/objects/world.hpp"
#include "sailing_simulator/objects/body_components/dynamic_body_component.hpp"

#include "test/comparisions.hpp"

TEST(DynamicBodyComponentTest, AccessorsTest) {
  sailing_simulator::objects::World world;

  b2PolygonShape shape;
  b2Vec2 edges[4] {
    b2Vec2(0.0f, 0.0f),
    b2Vec2(1.0f, 0.0f),
    b2Vec2(1.0f, 1.0f),
    b2Vec2(0.0f, 1.0f)
  };
  shape.Set(edges, 4);

  b2Vec2 position(2.0f, 2.0f);

  sailing_simulator::objects::DynamicBodyComponent component(world, shape, position);

  EXPECT_EQ(position, component.getPosition());
  auto body_shape = component.getShape();

  EXPECT_EQ(0.0, component.getOrientation());

  ASSERT_EQ(shape.m_count, body_shape.size());
  for (int i = 0; i < body_shape.size(); ++i) {
    EXPECT_EQ(shape.m_vertices[i], body_shape.at(i));
  }

  EXPECT_EQ(b2Vec2_zero, component.getLinearVelocity());
  EXPECT_EQ(0.0, component.getAngularVelocity());
}
