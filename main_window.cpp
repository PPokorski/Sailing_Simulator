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

#include "main_window.hpp"
#include "ui_main_window.h"

#include <QDebug>
#include <QtGui/QPainter>
#include <QKeyEvent>

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        thrust_(false),
        turn_left_(false),
        turn_right_(false),
        ui_(new Ui::MainWindow),
        world_(b2Vec2(0.0f, 0.0f))
{
    ui_->setupUi(this);

    label_ = new QLabel("", this);

    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), this, SLOT(updateEngine()));
    timer_->start(1000.0 / 60.0);

    b2BodyDef ground_def;
    ground_def.position.Set(0.0f, 0.0f);
    b2Body* ground_body = world_.CreateBody(&ground_def);

    motor_boat_ = std::make_unique<MotorBoat>(world_, ground_body);
}

void MainWindow::paintEvent(QPaintEvent* event) {
    QMainWindow::paintEvent(event);

    QPainter painter(this);
    painter.setPen(Qt::blue);

    constexpr double PROPORTION = 10.0;

    std::array<b2Vec2, 3> points = motor_boat_->getCurrentShape();
    QPointF point_f[3] = {
            {PROPORTION * points.at(0).x, PROPORTION * points.at(0).y},
            {PROPORTION * points.at(1).x, PROPORTION * points.at(1).y},
            {PROPORTION * points.at(2).x, PROPORTION * points.at(2).y}
    };

    painter.drawConvexPolygon(point_f, 3);
    qDebug() << "x: " << motor_boat_->getPosition().x << " y: " << motor_boat_->getPosition().y;
    qDebug() << "Wind x: " << motor_boat_->getWindForcePosition().x << " Wind y: " << motor_boat_->getWindForcePosition().y;

    QPointF point_f1(
            PROPORTION * motor_boat_->getWindForcePosition().x,
            PROPORTION * motor_boat_->getWindForcePosition().y);
    painter.drawEllipse(point_f1, 5, 5);

    QString string = QString("Linear velocity: %1 %2\nAngular velocity: %3\nSteer position: %4").
            arg(motor_boat_->getLinearVelocity().x).
            arg(motor_boat_->getLinearVelocity().y).
            arg(motor_boat_->getAngularVelocity()).
            arg(motor_boat_->getSteerPosition());
    label_->setText(string);
    label_->adjustSize();
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    QWidget::keyPressEvent(event);

    switch (event->key())
    {
        case Qt::Key_Up:
            thrust_ = true;
            break;
        case Qt::Key_Left:
            turn_left_ = true;
            break;
        case Qt::Key_Right:
            turn_right_ = true;
            break;
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event) {
    QWidget::keyReleaseEvent(event);

    switch (event->key())
    {
        case Qt::Key_Up:
            thrust_ = false;
            break;
        case Qt::Key_Left:
            turn_left_ = false;
            break;
        case Qt::Key_Right:
            turn_right_ = false;
            break;
    }
}

MainWindow::~MainWindow()
{
    delete ui_;
    delete label_;
    delete timer_;
}

void MainWindow::updateEngine() {
    constexpr double STEER_CHANGE = 0.02;
    if (thrust_)
    {
        motor_boat_->useThrust();
    }
    if (turn_left_)
    {
        motor_boat_->moveSteer(STEER_CHANGE);
    }
    if (turn_right_)
    {
        motor_boat_->moveSteer(-STEER_CHANGE);
    }

    motor_boat_->process(world_);
    world_.Step(1.0f / 60.0f, 6, 2);
    repaint();
}