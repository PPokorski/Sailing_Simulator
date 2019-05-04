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

#ifndef SAILING_SIMULATOR_OBJECTS_DYNAMICS_COMPONENTS_WIND_DYNAMICS_COMPONENT_HPP
#define SAILING_SIMULATOR_OBJECTS_DYNAMICS_COMPONENTS_WIND_DYNAMICS_COMPONENT_HPP

#include <vector>

#include <Box2D/Box2D.h>

#include <Eigen/Geometry>

#include "sailing_simulator/objects/dynamics_components/dynamics_component.hpp"
#include "sailing_simulator/objects/body_components/dynamic_body_component.hpp"

namespace sailing_simulator {
namespace objects {
namespace internal {
using Point2D = Eigen::Vector2d;
using Line2D = Eigen::Hyperplane<double, 2>;
}  // namespace internal

void projectPolygonOntoLine(const std::vector<b2Vec2>& polygon,
                            const internal::Line2D& line,
                            internal::Point2D& min_point,
                            internal::Point2D& max_point);

double getProjectionLength(const std::vector<b2Vec2>& polygon,
                           const internal::Line2D& line);

internal::Line2D getLinePerpendicularToWind(const b2Vec2& wind_vector);

b2Vec2 getWindForcePoint(const std::vector<b2Vec2>& polygon, const b2Vec2& wind_vector, const b2EdgeShape& axis);

b2EdgeShape getAxisOfPolygon(const std::vector<b2Vec2>& polygon);

class WindDynamicsComponent : public DynamicsComponent {
 public:
  using Ptr = std::shared_ptr<WindDynamicsComponent>;
  using ConstPtr = std::shared_ptr<WindDynamicsComponent>;

  WindDynamicsComponent(DynamicBodyComponent::Ptr body);

  void update(GameObject& object, World& world) override;

 protected:
  std::weak_ptr<DynamicBodyComponent> body_;
  b2EdgeShape symmetry_axis_;
};

}  // namespace objects
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_OBJECTS_DYNAMICS_COMPONENTS_WIND_DYNAMICS_COMPONENT_HPP
