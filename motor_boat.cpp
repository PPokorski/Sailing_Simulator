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

#include "motor_boat.hpp"

b2Vec2 getWindForcePoint(const std::vector<b2Vec2>& polygon, const b2Vec2& wind_vector, const b2EdgeShape& axis) {
  b2Vec2 unit_wind = wind_vector;
  unit_wind.Normalize();
  // The line parallel to wind_vector
  Eigen::Hyperplane<double, 2> line;
  line.coeffs() << unit_wind.x, unit_wind.y, 0.0;
  Eigen::Vector2d eigen_point, projected_point;
  // We project all points of the polygon onto the line and search for the longest line segment
  Eigen::Vector2d min_point, max_point;
  // The lower end of the segment
  min_point.setConstant(+std::numeric_limits<double>::infinity());
  // The upper end of the segment
  max_point.setConstant(-std::numeric_limits<double>::infinity());
  int index = std::abs(line.coeffs()(0)) < std::abs(line.coeffs()(1)) ? 0 : 1;
  for (const auto& point : polygon)
  {
    eigen_point << point.x, point.y;
    projected_point = line.projection(eigen_point);

    if (projected_point(index) < min_point(index))
    {
      min_point = projected_point;
    }
    if (projected_point(index) > max_point(index))
    {
      max_point = projected_point;
    }
  }
  // After finding a segment we need the midpoint
  Eigen::Vector2d midpoint = (min_point + max_point) / 2.0;
  // The we need line which is parallel to the wind vector and passes through the midpoint
  line.coeffs() << -unit_wind.y, unit_wind.x, unit_wind.y * midpoint(0) - unit_wind.x * midpoint(1);
  Eigen::Hyperplane<double, 2> line_2 = Eigen::Hyperplane<double, 2>::Through(
      Eigen::Vector2d(axis.m_vertex1.x, axis.m_vertex1.y),
      Eigen::Vector2d(axis.m_vertex2.x, axis.m_vertex2.y));

  // Resulting point is the intersection of axis segment and the line parallel to wind and passing through midpoint
  Eigen::Vector2d resulting_point = line.intersection(line_2);

  return b2Vec2(resulting_point(0), resulting_point(1));
}

MotorBoat::MotorBoat(b2World& world, b2Body* ground_body) :
    max_speed_(10.0),
    max_steer_position_(b2_pi / 4.0),
    steer_position_(0.0),
    thrust_(50.0),
    points_({
        b2Vec2(1.0, 0.0),
        b2Vec2(-0.5, 0.5),
        b2Vec2(-0.5, -0.5)
    }) {
    b2PolygonShape shape;
    shape.Set(points_.data(), points_.size());

    b2FixtureDef fixture_def;
    fixture_def.shape = &shape;
    fixture_def.density = 1.0f;
    fixture_def.friction = 0.3f;

    b2BodyDef body_def;
    body_def.type = b2_dynamicBody;
    body_def.position.Set(5.0f, 5.0f);
    body_def.active = true;
    body_def.linearDamping = 0.6f;
    body_def.angularDamping = 2.0f;

    body_ = world.CreateBody(&body_def);

    body_->CreateFixture(&fixture_def);
    float32 gravity = 10.0f;
    float32 mass = body_->GetMass();

    b2FrictionJointDef joint_def;
    joint_def.localAnchorA.SetZero();
    joint_def.localAnchorB.SetZero();
    joint_def.bodyA = ground_body;
    joint_def.bodyB = body_;
    joint_def.collideConnected = true;
    joint_def.maxForce = mass * gravity;
    joint_def.maxTorque = 0.02 * mass * gravity;

    world.CreateJoint(&joint_def);
}

void MotorBoat::process(b2World& world) {
    b2Vec2 velocity = body_->GetLinearVelocity();
    float32 speed = velocity.Length();

    if (speed > max_speed_)
    {
        velocity *= (max_speed_ / speed);
        body_->SetLinearVelocity(velocity);
    }

    b2Vec2 wind_force(0.0f, 15.0f);
    b2Vec2 local_wind_force = body_->GetLocalVector(wind_force);
    b2EdgeShape boat_axis;
    boat_axis.Set({-0.5f, 0.0f}, {1.0f, 0.0f});
    b2Vec2 local_point = getWindForcePoint({points_.begin(), points_.end()},
            local_wind_force, boat_axis);

    wind_position_ = body_->GetWorldPoint(local_point);
    body_->ApplyForce(wind_force, wind_position_, true);
}

void MotorBoat::useThrust() {
    b2Vec2 thrust_vector(std::cos(steer_position_) * thrust_,
                         std::sin(steer_position_) * thrust_);

    b2Vec2 thrust_position = body_->GetWorldPoint(b2Vec2(-0.5, 0.0));
    b2Vec2 thrust_force = body_->GetWorldVector(thrust_vector);

    body_->ApplyForce(thrust_force, thrust_position, true);
}
