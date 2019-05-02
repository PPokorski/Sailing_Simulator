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

#include "sailing_simulator/objects/dynamics_components/wind_dynamics_component.hpp"

#include <Eigen/Geometry>

#include "sailing_simulator/constants.hpp"
#include "sailing_simulator/objects/world.hpp"

namespace sailing_simulator {
namespace objects {
void projectPolygonOntoLine(const std::vector<b2Vec2>& polygon,
                            const internal::Line2D& line,
                            internal::Point2D& min_point,
                            internal::Point2D& max_point) {
  internal::Point2D eigen_point, projected_point;
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
}

double getProjectionLength(const std::vector<b2Vec2>& polygon, const internal::Line2D& line) {
  internal::Point2D min_point, max_point;
  projectPolygonOntoLine(polygon, line, min_point, max_point);

  return (max_point - min_point).norm();
}

internal::Line2D getLineParallelToWind(const b2Vec2& wind_vector) {
  b2Vec2 unit_wind = wind_vector;
  unit_wind.Normalize();
  internal::Line2D line;
  line.coeffs() << unit_wind.x, unit_wind.y, 0.0;
  return line;
}

b2Vec2 getWindForcePoint(const std::vector<b2Vec2>& polygon, const b2Vec2& wind_vector, const b2EdgeShape& axis) {
  b2Vec2 unit_wind = wind_vector;
  unit_wind.Normalize();
  // The line parallel to wind_vector
  internal::Line2D line;
  line.coeffs() << unit_wind.x, unit_wind.y, 0.0;
  // We project all points of the polygon onto the line and search for the longest line segment
  internal::Point2D min_point, max_point;
  projectPolygonOntoLine(polygon, line, min_point, max_point);

  // After finding a segment we need the midpoint
  internal::Point2D midpoint = (min_point + max_point) / 2.0;
  // The we need line which is parallel to the wind vector and passes through the midpoint
  line.coeffs() << -unit_wind.y, unit_wind.x, unit_wind.y * midpoint(0) - unit_wind.x * midpoint(1);
  internal::Line2D line_2 = internal::Line2D::Through(
      internal::Point2D(axis.m_vertex1.x, axis.m_vertex1.y),
      internal::Point2D(axis.m_vertex2.x, axis.m_vertex2.y));

  // Resulting point is the intersection of axis segment and the line parallel to wind and passing through midpoint
  internal::Point2D resulting_point = line.intersection(line_2);

  return b2Vec2(resulting_point(0), resulting_point(1));
}

WindDynamicsComponent::WindDynamicsComponent(DynamicBodyComponent& body_)
    : body_(body_) {}

void WindDynamicsComponent::update(GameObject& object, World& world) {
  b2Vec2 global_wind = world.getWind()->getWind(body_.getPosition());
  b2Vec2 local_wind = body_.getLocalVector(global_wind);

  b2Vec2 wind_local_position = getWindForcePoint(body_.getShape(), local_wind, body_.getAxis());

  local_wind *= NEWTONS_PER_M * getProjectionLength(body_.getShape(), getLineParallelToWind(local_wind));

  body_.applyLocalForce(local_wind, wind_local_position);
}
}  // namespace objects
}  // namespace sailing_simulator