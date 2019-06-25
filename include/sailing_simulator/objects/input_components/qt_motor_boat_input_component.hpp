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

#ifndef SAILING_SIMULATOR_OBJECTS_INPUT_COMPONENTS_QT_MOTOR_BOAT_INPUT_COMPONENT_HPP
#define SAILING_SIMULATOR_OBJECTS_INPUT_COMPONENTS_QT_MOTOR_BOAT_INPUT_COMPONENT_HPP

#include <QObject>

#include "sailing_simulator/objects/input_components/input_component.hpp"
#include "sailing_simulator/objects/dynamics_components/engine_dynamics_component.hpp"

namespace sailing_simulator {
namespace objects {
class QtMotorBoatInputComponent : public QObject, public InputComponent {
  Q_OBJECT

 public:
  using Ptr = std::shared_ptr<QtMotorBoatInputComponent>;
  using ConstPtr = std::shared_ptr<QtMotorBoatInputComponent>;

  explicit QtMotorBoatInputComponent(EngineDynamicsComponent::Ptr engine)
      : QObject(nullptr),
        engine_(engine) {}

  void update(GameObject& object) override {
  }

 public slots:
  void rotateEngine(double engine_orientation_change) {
    if (auto engine_ptr = engine_.lock()) {
      engine_ptr->rotateEngine(engine_orientation_change);
    }
  }

  void changeThrust(double current_thrust_change) {
    if (auto engine_ptr = engine_.lock()) {
      engine_ptr->changeThrust(current_thrust_change);
    }
  }

  void setCurrentThrust(double current_thrust) {
    if (auto engine_ptr = engine_.lock()) {
      engine_ptr->setCurrentThrust(current_thrust);
    }
  }

 protected:
  std::weak_ptr<EngineDynamicsComponent> engine_;

};
}  // namespace objects
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_OBJECTS_INPUT_COMPONENTS_QT_MOTOR_BOAT_INPUT_COMPONENT_HPP
