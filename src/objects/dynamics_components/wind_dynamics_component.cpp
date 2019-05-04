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
  for (const auto& point : polygon) {
    eigen_point << point.x, point.y;
    projected_point = line.projection(eigen_point);

    if (projected_point(index) < min_point(index)) {
      min_point = projected_point;
    }
    if (projected_point(index) > max_point(index)) {
      max_point = projected_point;
    }
  }
}

double getProjectionLength(const std::vector<b2Vec2>& polygon, const internal::Line2D& line) {
  internal::Point2D min_point, max_point;
  projectPolygonOntoLine(polygon, line, min_point, max_point);

  return (max_point - min_point).norm();
}

internal::Line2D getLinePerpendicularToWind(const b2Vec2& wind_vector) {
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

b2EdgeShape getAxisOfPolygon(const std::vector<b2Vec2>& polygon) {
  int polygon_size = polygon.size();
  int extended_polygon_size = 2 * polygon.size();
  std::vector<b2Vec2> edges_midpoints(extended_polygon_size);
  for (int i = 0; i < polygon_size - 1; ++i) {
    edges_midpoints.at(2 * i) = polygon.at(i);
    edges_midpoints.at(2 * i + 1) = 0.5 * (polygon.at(i) + polygon.at(i + 1));
  }
  edges_midpoints.at(extended_polygon_size - 2) = polygon.back();
  edges_midpoints.at(extended_polygon_size - 1) = 0.5 * (polygon.back() + polygon.front());

  auto wrap_index = [extended_polygon_size](int index) {
    int ret_value = index % extended_polygon_size;
    return (ret_value < 0) ? ret_value + extended_polygon_size : ret_value;};


  b2EdgeShape axis;
  float32 nan = std::numeric_limits<float>::quiet_NaN();
  axis.Set(b2Vec2(nan, nan), b2Vec2(nan, nan));
  double longest_length_squared = 0.0;
  for (int i = 0; i < polygon_size; ++i) {
    const auto& axis_one = edges_midpoints.at(i);
    const auto& axis_two = edges_midpoints.at(i + polygon_size);

    double axis_length_squared = (axis_one - axis_two).LengthSquared();
    if (axis_length_squared <= longest_length_squared) {
      continue;
    }

    bool is_axis = true;
    for (int j = 1; j < polygon_size; ++j) {
      const auto& point_one = edges_midpoints.at(wrap_index(i - j));
      const auto& point_two = edges_midpoints.at(wrap_index(i + j));

      double distance_one_one = (axis_one - point_one).LengthSquared();
      double distance_one_two = (axis_one - point_two).LengthSquared();
      double distance_two_one = (axis_two - point_one).LengthSquared();
      double distance_two_two = (axis_two - point_two).LengthSquared();

      static constexpr double PRECISION = 0.001;
      if (std::abs(distance_one_one - distance_one_two) > PRECISION ||
          std::abs(distance_two_one - distance_two_two) > PRECISION) {
        is_axis = false;
        break;
      }
    }

    if (is_axis) {
      axis.Set(axis_one, axis_two);
      longest_length_squared = axis_length_squared;
    }
  }

  return axis;
}

WindDynamicsComponent::WindDynamicsComponent(DynamicBodyComponent::Ptr body)
    : body_(body) {
  if (auto body_ptr = body_.lock()) {
    symmetry_axis_ = getAxisOfPolygon(body_ptr->getShape());
    if (!symmetry_axis_.m_vertex1.IsValid() ||
        !symmetry_axis_.m_vertex2.IsValid()) {
      throw std::invalid_argument("Given body must have at least one axis of symmetry.");
    }
  }
}

void WindDynamicsComponent::update(GameObject& object, World& world) {
  if (auto body_ptr = body_.lock())
  {
    b2Vec2 global_wind = world.getWind()->getWind(body_ptr->getPosition());
    if (global_wind.Length() < SMALL_WIND) {
      return;
    }

    b2Vec2 local_wind = body_ptr->getLocalVector(global_wind);

    b2Vec2 wind_local_position = getWindForcePoint(body_ptr->getShape(), local_wind, symmetry_axis_);

    local_wind *= NEWTONS_PER_M * getProjectionLength(body_ptr->getShape(), getLinePerpendicularToWind(local_wind));

    body_ptr->applyLocalForce(local_wind, wind_local_position);
  }
}
}  // namespace objects
}  // namespace sailing_simulator