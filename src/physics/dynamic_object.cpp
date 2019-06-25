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

#include "sailing_simulator/physics/dynamic_object.hpp"

#include <Eigen/Geometry>

#include "sailing_simulator/physics/world.hpp"

#include "sailing_simulator/constants.hpp"

#include <iostream>

namespace sailing_simulator {
namespace physics {
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

DynamicObject::DynamicObject(World& world,
                             const b2PolygonShape& shape,
                             const b2Vec2& position,
                             const b2EdgeShape& axis)
    : Object(world, shape, b2_dynamicBody, position),
      axis_(axis),
      steer_position_(0.0) {
  body_->SetLinearDamping(LINEAR_DAMPING);
  body_->SetAngularDamping(ANGULAR_DAMPING);

  fixture_->SetDensity(DENSITY);
  fixture_->SetFriction(FRICTION);
  body_->ResetMassData();

  createFrictionJoint(world);
}

void DynamicObject::process(World& world) {
  Entity::process(world);

  b2Vec2 wind_force = world.getWind()->getWind(getPosition());
  if (wind_force.Length() < 0.1) {
    return;
  }
  b2Vec2 local_wind_force = body_->GetLocalVector(wind_force);
  b2Vec2 local_point = getWindForcePoint(shape_, local_wind_force, axis_);

  b2Vec2 wind_position = body_->GetWorldPoint(local_point);

  body_->ApplyForce(wind_force, wind_position, true);
}

void DynamicObject::createFrictionJoint(World& world) {
  b2FrictionJointDef joint_definition;
  joint_definition.localAnchorA.SetZero();
  joint_definition.localAnchorB.SetZero();
  joint_definition.bodyA = world.getGroundBody();
  joint_definition.bodyB = body_;
  joint_definition.collideConnected = true;
  joint_definition.maxForce = MAX_FORCE;
  joint_definition.maxTorque = MAX_TORQUE;

  world.getWorld().CreateJoint(&joint_definition);
}
}  // namespace physics {
}  // namespace sailing_simulator
