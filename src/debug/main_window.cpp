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

#include "sailing_simulator/debug/main_window.hpp"
#include "ui/ui_main_window.h"

#include <QDebug>
#include <QKeyEvent>
#include <QPainter>
#include <QTimer>
#include <include/sailing_simulator/objects/input_components/qt_sailing_boat_input_component.hpp>

namespace sailing_simulator {
namespace debug {
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui_(new Ui::MainWindow) {
  ui_->setupUi(this);

  world_.getWorld().SetDebugDraw(this);
  SetFlags(b2Draw::e_shapeBit);

  b2PolygonShape shape;
  b2Vec2 edges[5] {
      b2Vec2(1.0f, 0.00f),
      b2Vec2(0.00f, 0.25f),
      b2Vec2(-0.5f, 0.20f),
      b2Vec2(-0.5f, -0.20f),
      b2Vec2(0.00f, -0.25f)
  };
  shape.Set(edges, 5);
  b2Vec2 position(5.0f, 5.0f);
  b2EdgeShape edge_shape;
  edge_shape.Set(b2Vec2(1.0f, 0.0f), b2Vec2(-0.5f, 0.0f));
  b2Vec2 engine_position(-0.5f, 0.0f);
  double thrust = 20.0;

  auto body = std::make_shared<objects::DynamicBodyComponent>(world_, shape, position);
  auto engine = std::make_shared<objects::EngineDynamicsComponent>(body, thrust, thrust, engine_position);
  engine_ = engine;
  auto input = std::make_shared<objects::QtMotorBoatInputComponent>(engine);

  QObject::connect(this, &MainWindow::rotateEngine, input.get(), &objects::QtMotorBoatInputComponent::rotateEngine);
  QObject::connect(this, &MainWindow::changeThrust, input.get(), &objects::QtMotorBoatInputComponent::changeThrust);

//  world_.addMotorBoat(shape, position);
//  world_.addMotorBoat(body, engine, input);

  auto rudder = std::make_shared<objects::RudderDynamicsComponent>(body, 0.01, 0.5, engine_position);
  rudder_ = rudder;
  auto sail_input = std::make_shared<objects::QtSailingBoatInputComponent>(rudder);
  QObject::connect(this, &MainWindow::rotateRudder, sail_input.get(), &objects::QtSailingBoatInputComponent::rotateRudder);
  world_.addSailingBoat(body, rudder, sail_input);

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateEngine()));

  label_ = new QLabel("", this);

  timer_->start(1000 * world_.getTimeStep());

  turn_left_ = false;
  turn_right_ = false;
}

void MainWindow::paintEvent(QPaintEvent* event) {
  QWidget::paintEvent(event);
  QPainter painter(this);

  painter.setBrush(polygon_.second);
  painter.drawPolygon(polygon_.first);

  QPainterPath painter_path;
  painter_path.addPolygon(solid_polygon_.first);
  painter.fillPath(painter_path, solid_polygon_.second);

  painter.setBrush(circle_.second);
  painter.drawEllipse(circle_.first);

  QPainterPath painter_path1;
  painter_path1.addEllipse(solid_circle_.first);
  painter.fillPath(painter_path1, solid_circle_.second);

  painter.setBrush(line_.second);
  painter.drawLine(line_.first);

  painter.setBrush(point_.second);
  painter.drawRect(point_.first);

//  QString string = QString("Steer position: %1\nThrust: %2").
//      arg(engine_->getEngineOrientation()).
//      arg(engine_->getCurrentThrust());
  QString string = QString("Steer position: %1").
      arg(rudder_->getRudderOrientation());
//  QString string = QString("Linear velocity: %1 %2\nAngular velocity: %3\nSteer position: %4").
//      arg(world_.getMotorBoat()->getLinearVelocity().x).
//      arg(world_.getMotorBoat()->getLinearVelocity().y).
//      arg(world_.getMotorBoat()->getAngularVelocity()).
//      arg(world_.getMotorBoat()->getSteerPosition());
  label_->setText(string);
  label_->adjustSize();
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
  QWidget::keyPressEvent(event);

  switch (event->key())
  {
    case Qt::Key_Up:
      emit changeThrust(1.0);
//      world_.getMotorBoat()->thrustOn();
      break;
    case Qt::Key_Down:
      emit changeThrust(-1.0);
      break;
    case Qt::Key_Left:
      emit rotateEngine(-0.02);
      emit rotateRudder(-0.02);
//      turn_left_ = true;
      break;
    case Qt::Key_Right:
      emit rotateEngine(+0.02);
      emit rotateRudder(0.02);
//      turn_right_ = true;
      break;
  }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event) {
  QWidget::keyReleaseEvent(event);

  switch (event->key())
  {
//    case Qt::Key_Up:
//      world_.getMotorBoat()->thrustOff();
//      break;
//    case Qt::Key_Left:
//      turn_left_ = false;
//      break;
//    case Qt::Key_Right:
//      turn_right_ = false;
//      break;
  }
}

void MainWindow::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
  polygon_.first.clear();
  for (int i = 0; i < vertexCount; ++i) {
    QPointF point_f = fromSimulation(vertices[i]);
    polygon_.first.push_back(point_f);
  }

  polygon_.second = fromB2color(color);

  repaint();
}

void MainWindow::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
  solid_polygon_.first.clear();
  for (int i = 0; i < vertexCount; ++i) {
    QPointF point_f = fromSimulation(vertices[i]);
    solid_polygon_.first.push_back(point_f);
  }

  solid_polygon_.second = fromB2color(color);

  repaint();
}

void MainWindow::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) {
  circle_.first.moveTo(fromSimulation(center));
  circle_.first.setWidth(2 * radius);
  circle_.first.setHeight(2 * radius);

  circle_.second = fromB2color(color);

  repaint();
}

void MainWindow::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) {
  qDebug() << "Draw solid circle";
  solid_circle_.first.moveTo(fromSimulation(center));
  solid_circle_.first.setWidth(2 * radius);
  solid_circle_.first.setHeight(2 * radius);

  solid_circle_.second = fromB2color(color);

  repaint();
}

void MainWindow::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) {
  qDebug() << "Draw segment";
  line_.first.setP1(fromSimulation(p1));
  line_.first.setP2(fromSimulation(p2));

  line_.second = fromB2color(color);

  repaint();
}

void MainWindow::DrawTransform(const b2Transform& xf) {
  qDebug() << "Draw transform";
  repaint();
}

void MainWindow::DrawPoint(const b2Vec2& p, float32 size, const b2Color& color) {
  point_.first.moveTo(fromSimulation(p));
  point_.first.setWidth(2 * size);
  point_.first.setHeight(2 * size);

  point_.second = fromB2color(color);

  repaint();
}

void MainWindow::updateEngine() {
  constexpr double STEER_CHANGE = 0.02;
  if (turn_left_) {
//    world_.getMotorBoat()->moveSteer(STEER_CHANGE);
  }
  if (turn_right_) {
//    world_.getMotorBoat()->moveSteer(-STEER_CHANGE);
  }

  world_.step();
  world_.getWorld().DrawDebugData();
}

MainWindow::~MainWindow() {
}
}  // namespace debug
}  // namespace sailing_simulator
