#include "mainwindow.h"
#include "receiver.cpp"
#include <QApplication>
#include <QtGui/QApplication>
#include <QVTKWidget.h>


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow window;
    boost::asio::io_service io_service;

    receiver r(io_service,
               boost::asio::ip::address::from_string("0.0.0.0"),
               boost::asio::ip::address::from_string("239.255.0.1"));
    boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service));
    sleep(1);

    window.show();
    sleep(1);

    window.StartThread();
    return app.exec();
}
