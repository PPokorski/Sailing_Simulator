#include <iostream>

#include <QApplication>

#include "main_window.hpp"

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box2D is meant to be used
// with your rendering engine in your game engine.
int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    MainWindow w;
    w.show();

    return app.exec();
}
