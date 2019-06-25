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

#ifndef TEST_TEST_COMPARISIONS_HPP
#define TEST_TEST_COMPARISIONS_HPP

#include <Box2D/Box2D.h>

#include <gtest/gtest.h>

namespace comparisions {

template<class T>
constexpr static T PRECISION = T(0.0001);

}  // namespace comparisions

inline bool approxEqual(const b2Vec2& lhs, const b2Vec2& rhs) {
  return std::fabs(lhs.x - lhs.x) <= comparisions::PRECISION<double> &&
         std::fabs(lhs.y - lhs.y) <= comparisions::PRECISION<double>;
}

inline bool operator==(const b2EdgeShape& lhs, const b2EdgeShape& rhs) {
  return approxEqual(lhs.m_vertex1, rhs.m_vertex1) &&
         approxEqual(lhs.m_vertex2, rhs.m_vertex2);
}

inline std::ostream& operator<<(std::ostream& os, const b2Vec2& vec) {
  return os << "[" << vec.x << ", " << vec.y << "]";
}

#endif //TEST_TEST_COMPARISIONS_HPP
