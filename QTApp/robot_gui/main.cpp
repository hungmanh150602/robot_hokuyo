#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Initialize Qt application
    QApplication app(argc, argv);

    // Create main window
    MainWindow w(&app);

    w.show();

    // Run Qt event loop
    int ret = app.exec();

    // Shutdown ROS2
    rclcpp::shutdown();

    return ret;
}
