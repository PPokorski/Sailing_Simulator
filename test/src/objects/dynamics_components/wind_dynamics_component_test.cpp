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
#include "sailing_simulator/objects/dynamics_components/wind_dynamics_component.hpp"
#include "sailing_simulator/objects/wind/constant_wind.hpp"

#include "test/comparisions.hpp"
#include "test/dynamic_body_component_stub.hpp"

using namespace sailing_simulator::objects;

class WindDynamicsComponentTest : public testing::Test {
 protected:
  WindDynamicsComponentTest()
      : Test(),
        world_(),
        game_object_(nullptr, nullptr, nullptr) {}

  World world_;
  GameObject game_object_;
  test::DynamicBodyComponentStub::Ptr body_component_;
  WindDynamicsComponent::Ptr dynamic_component_;
};

TEST_F(WindDynamicsComponentTest, ZeroWindTest) {
  b2PolygonShape shape;
  b2Vec2 edges[4] {
      b2Vec2(0.0f, 0.0f),
      b2Vec2(1.0f, 0.0f),
      b2Vec2(1.0f, 1.0f),
      b2Vec2(0.0f, 1.0f)
  };
  shape.Set(edges, 4);

  b2Vec2 position(0.0f, 0.0f);

  body_component_ = std::make_shared<test::DynamicBodyComponentStub>(world_, shape, position);
  dynamic_component_ = std::make_shared<WindDynamicsComponent>(body_component_);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2_zero));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2_zero, body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2_zero, body_component_->last_local_force_position_);
}

TEST_F(WindDynamicsComponentTest, NewtonsPerMeterTest) {
  b2PolygonShape shape;
  b2Vec2 edges[4] {
      b2Vec2(0.0f, 0.0f),
      b2Vec2(1.0f, 0.0f),
      b2Vec2(1.0f, 1.0f),
      b2Vec2(0.0f, 1.0f)
  };
  shape.Set(edges, 4);

  b2Vec2 position(0.0f, 0.0f);

  body_component_ = std::make_shared<test::DynamicBodyComponentStub>(world_, shape, position);
  dynamic_component_ = std::make_shared<WindDynamicsComponent>(body_component_, 1.0);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2(1.0, 0.0)));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(1.0, 0.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.5), body_component_->last_local_force_position_);

  dynamic_component_->setNewtonsPerMeter(4.0);
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(4.0, 0.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.5), body_component_->last_local_force_position_);
}

TEST_F(WindDynamicsComponentTest, DifferentWindsTest) {
  b2PolygonShape shape;
  b2Vec2 edges[4] {
      b2Vec2(0.0f, 0.0f),
      b2Vec2(1.0f, 0.0f),
      b2Vec2(1.0f, 1.0f),
      b2Vec2(0.0f, 1.0f)
  };
  shape.Set(edges, 4);

  b2Vec2 position(0.0f, 0.0f);

  body_component_ = std::make_shared<test::DynamicBodyComponentStub>(world_, shape, position);
  dynamic_component_ = std::make_shared<WindDynamicsComponent>(body_component_, 1.0);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2(0.0, 1.0)));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(0.0, 1.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.5), body_component_->last_local_force_position_);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2(1.0, 1.0)));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(1.0, 1.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.5), body_component_->last_local_force_position_);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2(0.0, -1.0)));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(0.0, -1.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.5), body_component_->last_local_force_position_);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2(-1.0, 0.0)));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.5), body_component_->last_local_force_position_);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2(-3.0, 5.0)));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(-3.0, 5.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.5), body_component_->last_local_force_position_);
}

TEST_F(WindDynamicsComponentTest, TriangleShapeTest) {
  b2PolygonShape shape;
  b2Vec2 edges[3] {
      b2Vec2(0.0f, 1.0f),
      b2Vec2(-0.25f, 0.0f),
      b2Vec2(0.25f, 0.0f)
  };
  shape.Set(edges, 3);

  b2Vec2 position(0.0f, 0.0f);

  body_component_ = std::make_shared<test::DynamicBodyComponentStub>(world_, shape, position);
  dynamic_component_ = std::make_shared<WindDynamicsComponent>(body_component_, 1.0);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2(0.0, 1.0)));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(0.0, 1.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.0, 0.5), body_component_->last_local_force_position_);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2(0.5, -1.0)));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(0.125, -1.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.25, 0.0), body_component_->last_local_force_position_);
}

TEST_F(WindDynamicsComponentTest, NonZeroVelocityTest) {
  b2PolygonShape shape;
  b2Vec2 edges[4] {
      b2Vec2(0.0f, 0.0f),
      b2Vec2(1.0f, 0.0f),
      b2Vec2(1.0f, 1.0f),
      b2Vec2(0.0f, 1.0f)
  };
  shape.Set(edges, 4);

  b2Vec2 position(0.0f, 0.0f);

  body_component_ = std::make_shared<test::DynamicBodyComponentStub>(world_, shape, position);
  dynamic_component_ = std::make_shared<WindDynamicsComponent>(body_component_, 1.0);

  world_.setWind(std::make_shared<ConstantWind>(b2Vec2(1.0, 0.0)));
  body_component_->setLinearVelocity(b2Vec2(1.0, 0.0));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(0.0, 0.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.0, 0.0), body_component_->last_local_force_position_);

  body_component_->setLinearVelocity(b2Vec2(5.0, -2.0));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(4.0, -2.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.5), body_component_->last_local_force_position_);

  body_component_->setLinearVelocity(b2Vec2(-1.0, 2.0));
  dynamic_component_->update(game_object_, world_);

  EXPECT_PRED2(approxEqual, b2Vec2(2.0, 2.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.5), body_component_->last_local_force_position_);
}
