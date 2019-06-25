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

#include <cmath>

#include <gtest/gtest.h>

#include "sailing_simulator/objects/world.hpp"
#include "sailing_simulator/objects/dynamics_components/wind_dynamics_component.hpp"

#include "test/comparisions.hpp"

using namespace sailing_simulator::objects;

TEST(WindDynamicsComponentHelpersTest, ProjectPolygonOntoLineTest) {
  std::vector<b2Vec2> polygon {
    b2Vec2(0.0f, 0.0f),
    b2Vec2(1.0f, 1.0f),
    b2Vec2(1.5f, 2.0f),
    b2Vec2(2.0f, 1.5f)
  };

  internal::Line2D line;
  line.coeffs() << 1.0, 0.0, 0.0;

  internal::Point2D min, max;
  projectPolygonOntoLine(polygon, line, min, max);

  internal::Point2D expected_min, expected_max;
  expected_min << 0.0f, 0.0f;
  expected_max << 0.0f, 2.0f;
  EXPECT_TRUE(expected_min.isApprox(min, comparisions::PRECISION<double>))
      << "Expected value:\n" << expected_min << std::endl
      << "Actual value:\n" << min;
  EXPECT_TRUE(expected_max.isApprox(max, comparisions::PRECISION<double>))
      << "Expected value:\n" << expected_max
      << "Actual value:\n" << max;

  // x + y + 1 = 0
  line.coeffs() << M_SQRT2 / 2.0, M_SQRT2 / 2.0, M_SQRT2 / 2.0;
  projectPolygonOntoLine(polygon, line, min, max);

  expected_min << -0.25, -0.75;
  expected_max << -0.75, -0.25;
  EXPECT_TRUE(expected_min.isApprox(min, comparisions::PRECISION<double>))
      << "Expected value:\n" << expected_min << std::endl
      << "Actual value:\n" << min;
  EXPECT_TRUE(expected_max.isApprox(max, comparisions::PRECISION<double>))
      << "Expected value:\n" << expected_max << std::endl
      << "Actual value:\n" << max;
}

TEST(WindDynamicsComponentHelpersTest, GetProjectionLengthTest) {
  std::vector<b2Vec2> polygon {
      b2Vec2(0.0f, 0.0f),
      b2Vec2(1.0f, 1.0f),
      b2Vec2(1.5f, 2.0f),
      b2Vec2(2.0f, 1.5f)
  };

  internal::Line2D line;
  line.coeffs() << 1.0, 0.0, 0.0;
  EXPECT_NEAR(2.0, getProjectionLength(polygon, line), comparisions::PRECISION<double>);

  // x + y + 1 = 0
  line.coeffs() << M_SQRT2 / 2.0, M_SQRT2 / 2.0, M_SQRT2 / 2.0;
  EXPECT_NEAR(M_SQRT2 / 2.0, getProjectionLength(polygon, line), comparisions::PRECISION<double>);
}

TEST(WindDynamicsComponentHelpersTest, GetLineParallelToWindTest) {
  b2Vec2 wind(2.0f, 0.0f);
  internal::Line2D line = getLinePerpendicularToWind(wind);
  internal::Line2D expected_line;
  expected_line.coeffs() << 1.0, 0.0, 0.0;
  EXPECT_TRUE(expected_line.coeffs().isApprox(line.coeffs(), comparisions::PRECISION<double>))
            << "Expected value:\n" << expected_line.coeffs() << std::endl
            << "Actual value:\n" << line.coeffs();

  wind.Set(-1.0f, 2.0f);
  line = getLinePerpendicularToWind(wind);
  expected_line.coeffs() << (-1.0 / std::sqrt(5)), (2 / std::sqrt(5)), 0.0;

  EXPECT_TRUE(expected_line.coeffs().isApprox(line.coeffs(), comparisions::PRECISION<double>))
            << "Expected value:\n" << expected_line.coeffs() << std::endl
            << "Actual value:\n" << line.coeffs();
}

TEST(WindDynamicsComponentHelpersTest, GetAxisOfPolygonTest) {
  std::vector<b2Vec2> polygon {
    b2Vec2(0.0f, 0.0f),
    b2Vec2(1.0f, 0.0f),
    b2Vec2(1.0f, 1.0f),
    b2Vec2(0.0f, 1.0f)
  };

  auto axis = getAxisOfPolygon(polygon);
  b2EdgeShape expected_axis;
  expected_axis.Set(b2Vec2(0.0f, 0.0f), b2Vec2(1.0f, 1.0f));
  EXPECT_EQ(expected_axis, axis);

  polygon = {
      b2Vec2(1.0f, 0.0f),
      b2Vec2(3.0f, 1.0f),
      b2Vec2(4.0f, 0.0f),
      b2Vec2(3.0f, -1.0f)
  };
  axis = getAxisOfPolygon(polygon);
  expected_axis.Set(b2Vec2(1.0f, 0.0f), b2Vec2(4.0f, 0.0f));
  EXPECT_EQ(expected_axis, axis);

  polygon = {
      b2Vec2(1.0f, 0.00f),
      b2Vec2(0.00f, 0.25f),
      b2Vec2(-0.5f, 0.20f),
      b2Vec2(-0.5f, -0.20f),
      b2Vec2(0.00f, -0.25f)
  };
  axis = getAxisOfPolygon(polygon);
  expected_axis.Set(b2Vec2(1.0f, 0.0f), b2Vec2(-0.5f, 0.0f));
  EXPECT_EQ(expected_axis, axis);

  polygon = {
      b2Vec2(-1.0f, 0.0f),
      b2Vec2(1.0f, 0.0f),
      b2Vec2(1.0f, 1.0f),
      b2Vec2(0.0f, 1.0f)
  };
  axis = getAxisOfPolygon(polygon);
  EXPECT_FALSE(axis.m_vertex1.IsValid());
  EXPECT_FALSE(axis.m_vertex2.IsValid());
}
