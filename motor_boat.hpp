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

#ifndef _MOTOR_BOAT_HPP
#define _MOTOR_BOAT_HPP

#include <vector>

#include <Box2D/Box2D.h>
#include <Eigen/Geometry>

/**
 * @brief Function which, for a body with given polygon shape and axis of symmetry,
 * finds the point to which wind force should be applied
 * @param polygon The physical shape of the object
 * @param wind_vector Vector corresponding to direction and force of the wind
 * @param axis Axis of symmetry of the body
 * @return Point of the axis of symmetry, to which force of wind should be applied
 */
b2Vec2 getWindForcePoint(const std::vector<b2Vec2>& polygon, const b2Vec2& wind_vector, const b2EdgeShape& axis);

class MotorBoat {
 public:
    MotorBoat(b2World& world, b2Body* ground_body);

    void moveSteer(double steer_position_change) {
        steer_position_ += steer_position_change;
        steer_position_ = std::clamp(steer_position_, -max_steer_position_, max_steer_position_);
    }

    b2Vec2 getPosition() const {
        return body_->GetPosition();
    }

    b2Vec2 getLinearVelocity() const {
        return body_->GetLinearVelocity();
    }

    b2Vec2 getWindForcePosition() const {
        return wind_position_;
    }

    float32 getAngularVelocity() const {
        return body_->GetAngularVelocity();
    }

    double getSteerPosition() const {
        return steer_position_;
    }

    std::array<b2Vec2, 3> getCurrentShape() const {
        std::array<b2Vec2, 3> shape;
        for (int i = 0; i < points_.size(); ++i)
        {
            shape.at(i) = body_->GetWorldPoint(points_.at(i));
        }

        return shape;
    }

    /**
     * @brief Apply thrust to the body
     */
    void useThrust();

    /**
     * @brief Limit max speed and apply wind force
     * @param world
     */
    void process(b2World& world);

 private:
    b2Vec2 wind_position_;
    double max_speed_;
    double max_steer_position_;
    double steer_position_;
    double thrust_;
    std::array<b2Vec2, 3> points_;
    b2Body* body_;
};


#endif //_MOTOR_BOAT_HPP
