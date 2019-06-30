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

#include <gtest/gtest.h>

#include "sailing_simulator/objects/world.hpp"

#include "test/comparisions.hpp"
#include "test/dynamic_body_component_stub.hpp"

using namespace sailing_simulator::objects;

class EngineDynamicsComponentTest : public testing::Test {
 protected:
  EngineDynamicsComponentTest()
    : Test(),
      world_(),
      game_object_(nullptr, nullptr, nullptr) {
    b2PolygonShape shape;
    b2Vec2 edges[4] {
      b2Vec2(-1.0, -1.0),
      b2Vec2(-1.0,  1.0),
      b2Vec2( 1.0,  1.0),
      b2Vec2( 1.0, -1.0)
    };
    shape.Set(edges, 4);
    body_component_ = std::make_shared<test::DynamicBodyComponentStub>(world_, shape, b2Vec2_zero);
  }

  World world_;
  GameObject game_object_;
  test::DynamicBodyComponentStub::Ptr body_component_;
  EngineDynamicsComponent::Ptr dynamic_component_;
};

TEST_F(EngineDynamicsComponentTest, OrientationTest) {
  dynamic_component_ = std::make_shared<EngineDynamicsComponent>(body_component_,
      1.0,
      0.5,
      b2Vec2(-1.0, 0.0));

  EXPECT_NEAR(0.0, dynamic_component_->getEngineOrientation(), comparisions::PRECISION<double>);
  dynamic_component_->setEngineOrientation(-1.0);
  EXPECT_NEAR(-1.0, dynamic_component_->getEngineOrientation(), comparisions::PRECISION<double>);
  dynamic_component_->setEngineOrientation(-2.0);
  EXPECT_NEAR(-EngineDynamicsComponent::MAX_ENGINE_ORIENTATION_,
              dynamic_component_->getEngineOrientation(),
              comparisions::PRECISION<double>);

  dynamic_component_->setEngineOrientation(1.0);
  EXPECT_NEAR(1.0, dynamic_component_->getEngineOrientation(), comparisions::PRECISION<double>);
  dynamic_component_->setEngineOrientation(2.0);
  EXPECT_NEAR(EngineDynamicsComponent::MAX_ENGINE_ORIENTATION_,
              dynamic_component_->getEngineOrientation(),
              comparisions::PRECISION<double>);

  dynamic_component_->setEngineOrientation(0.0);
  dynamic_component_->rotateEngine(0.5);
  EXPECT_NEAR(0.5, dynamic_component_->getEngineOrientation(), comparisions::PRECISION<double>);
  dynamic_component_->rotateEngine(0.5);
  EXPECT_NEAR(1.0, dynamic_component_->getEngineOrientation(), comparisions::PRECISION<double>);
  dynamic_component_->rotateEngine(0.5);
  EXPECT_NEAR(1.5, dynamic_component_->getEngineOrientation(), comparisions::PRECISION<double>);
  dynamic_component_->rotateEngine(0.5);
  EXPECT_NEAR(EngineDynamicsComponent::MAX_ENGINE_ORIENTATION_,
              dynamic_component_->getEngineOrientation(),
              comparisions::PRECISION<double>);

  dynamic_component_->setEngineOrientation(0.0);
  dynamic_component_->rotateEngine(-0.5);
  EXPECT_NEAR(-0.5, dynamic_component_->getEngineOrientation(), comparisions::PRECISION<double>);
  dynamic_component_->rotateEngine(-0.5);
  EXPECT_NEAR(-1.0, dynamic_component_->getEngineOrientation(), comparisions::PRECISION<double>);
  dynamic_component_->rotateEngine(-0.5);
  EXPECT_NEAR(-1.5, dynamic_component_->getEngineOrientation(), comparisions::PRECISION<double>);
  dynamic_component_->rotateEngine(-0.5);
  EXPECT_NEAR(-EngineDynamicsComponent::MAX_ENGINE_ORIENTATION_,
              dynamic_component_->getEngineOrientation(),
              comparisions::PRECISION<double>);
}

TEST_F(EngineDynamicsComponentTest, ThrustTest) {
  dynamic_component_ = std::make_shared<EngineDynamicsComponent>(body_component_,
       1.0,
       0.5,
       b2Vec2(-1.0, 0.0));

  EXPECT_NEAR(1.0, dynamic_component_->getMaxThrustForward(), comparisions::PRECISION<double>);
  dynamic_component_->setMaxThrustForward(2.0);
  EXPECT_NEAR(2.0, dynamic_component_->getMaxThrustForward(), comparisions::PRECISION<double>);

  EXPECT_NEAR(0.5, dynamic_component_->getMaxThrustBackwards(), comparisions::PRECISION<double>);
  dynamic_component_->setMaxThrustBackwards(1.0);
  EXPECT_NEAR(1.0, dynamic_component_->getMaxThrustBackwards(), comparisions::PRECISION<double>);

  EXPECT_NEAR(0.0, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);
  dynamic_component_->setCurrentThrust(1.0);
  EXPECT_NEAR(1.0, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);
  dynamic_component_->setCurrentThrust(3.0);
  EXPECT_NEAR(2.0, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);
  dynamic_component_->setCurrentThrust(-0.5);
  EXPECT_NEAR(-0.5, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);
  dynamic_component_->setCurrentThrust(-1.5);
  EXPECT_NEAR(-1.0, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);

  dynamic_component_->setCurrentThrust(0.0);
  dynamic_component_->changeThrust(0.8);
  EXPECT_NEAR(0.8, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);
  dynamic_component_->changeThrust(0.8);
  EXPECT_NEAR(1.6, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);
  dynamic_component_->changeThrust(0.8);
  EXPECT_NEAR(2.0, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);

  dynamic_component_->setCurrentThrust(0.0);
  dynamic_component_->changeThrust(-0.4);
  EXPECT_NEAR(-0.4, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);
  dynamic_component_->changeThrust(-0.4);
  EXPECT_NEAR(-0.8, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);
  dynamic_component_->changeThrust(-0.4);
  EXPECT_NEAR(-1.0, dynamic_component_->getCurrentThrust(), comparisions::PRECISION<double>);
}

TEST_F(EngineDynamicsComponentTest, ThrustPositionTest) {
  dynamic_component_ = std::make_shared<EngineDynamicsComponent>(body_component_,
       1.0,
       0.5,
       b2Vec2(-1.0, 0.0));

  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), dynamic_component_->getThrustPosition());
  dynamic_component_->setThrustPosition(b2Vec2(2.0, -3.0));
  EXPECT_PRED2(approxEqual, b2Vec2(2.0, -3.0), dynamic_component_->getThrustPosition());
}

TEST_F(EngineDynamicsComponentTest, ForceTest) {
  dynamic_component_ = std::make_shared<EngineDynamicsComponent>(body_component_,
       1.0,
       1.0,
       b2Vec2(-1.0, 0.0));

  dynamic_component_->update(game_object_, world_);
  EXPECT_PRED2(approxEqual, b2Vec2_zero, body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_position_);

  dynamic_component_->setCurrentThrust(1.0);
  dynamic_component_->update(game_object_, world_);
  EXPECT_PRED2(approxEqual, b2Vec2(1.0, 0.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_position_);

  dynamic_component_->setCurrentThrust(0.5);
  dynamic_component_->update(game_object_, world_);
  EXPECT_PRED2(approxEqual, b2Vec2(0.5, 0.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_position_);

  dynamic_component_->setCurrentThrust(-0.5);
  dynamic_component_->update(game_object_, world_);
  EXPECT_PRED2(approxEqual, b2Vec2(-0.5, 0.0), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_position_);

  dynamic_component_->setCurrentThrust(1.0);
  dynamic_component_->setEngineOrientation(M_PI_4);
  dynamic_component_->update(game_object_, world_);
  EXPECT_PRED2(approxEqual, b2Vec2(M_SQRT1_2l, M_SQRT1_2l), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_position_);

  dynamic_component_->setEngineOrientation(M_PI / 3.0);
  dynamic_component_->update(game_object_, world_);
  EXPECT_PRED2(approxEqual, b2Vec2(std::sqrt(3.0) / 2.0, 0.5), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_position_);

  dynamic_component_->setCurrentThrust(0.5);
  dynamic_component_->update(game_object_, world_);
  EXPECT_PRED2(approxEqual, b2Vec2(std::sqrt(3.0) / 4.0, 0.25), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_position_);

  dynamic_component_->setCurrentThrust(1.0);
  dynamic_component_->setEngineOrientation(-M_PI_4);
  dynamic_component_->update(game_object_, world_);
  EXPECT_PRED2(approxEqual, b2Vec2(M_SQRT1_2l, -M_SQRT1_2l), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_position_);

  dynamic_component_->setEngineOrientation(-M_PI / 3.0);
  dynamic_component_->update(game_object_, world_);
  EXPECT_PRED2(approxEqual, b2Vec2(std::sqrt(3.0) / 2.0, -0.5), body_component_->last_local_force_);
  EXPECT_PRED2(approxEqual, b2Vec2(-1.0, 0.0), body_component_->last_local_force_position_);

  dynamic_component_->setCurrentThrust(-1.0);
}