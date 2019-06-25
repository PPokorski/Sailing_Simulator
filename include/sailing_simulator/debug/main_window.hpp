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

#ifndef SAILING_SIMULATOR_DEBUG_MAIN_WINDOW_HPP
#define SAILING_SIMULATOR_DEBUG_MAIN_WINDOW_HPP

#include <memory>

#include <QMainWindow>

#include <Box2D/Common/b2Draw.h>
#include <QtWidgets/QLabel>

#include "sailing_simulator/objects/world.hpp"

namespace Ui {
class MainWindow;
}  // namespace Ui

namespace sailing_simulator {
namespace debug {
class MainWindow : public QMainWindow, public b2Draw {
 Q_OBJECT
 public:
  explicit MainWindow(QWidget* parent = nullptr);

  void paintEvent(QPaintEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;

  void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override;
  void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override;
  void DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) override;
  void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) override;
  void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) override;
  void DrawTransform(const b2Transform& xf) override;
  void DrawPoint(const b2Vec2& p, float32 size, const b2Color& color) override;

  ~MainWindow();

 public slots:
  void updateEngine();

 signals:
  void rotateRudder(double rudder_orientation_change);

  void rotateEngine(double engine_orientation_change);

  void changeThrust(double current_thrust_change);

  void setCurrentThrust(double current_thrust);

 private:
  QPointF fromSimulation(const b2Vec2& vector) {
    constexpr int RATIO = 20;
    return {vector.x * RATIO, vector.y * RATIO};
  }

  QColor fromB2color(const b2Color& color) {
    return QColor::fromRgbF(color.r, color.g, color.b, color.a);
  }

  std::pair<QPolygonF, QColor> polygon_;
  std::pair<QPolygonF, QColor> solid_polygon_;
  std::pair<QRectF, QColor> circle_;
  std::pair<QRectF, QColor> solid_circle_;
  std::pair<QLineF, QColor> line_;
  std::pair<QRectF, QColor> point_;

  Ui::MainWindow* ui_;
  QTimer* timer_;
  QLabel* label_;

  objects::World world_;
  objects::EngineDynamicsComponent::Ptr engine_;
  objects::RudderDynamicsComponent::Ptr rudder_;
  bool turn_left_, turn_right_;
};
}  // namespace debug
}  // namespace sailing_simulator

#endif //SAILING_SIMULATOR_DEBUG_MAIN_WINDOW_HPP
