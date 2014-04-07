#include "mainwindow.h"
#include "receiver.cpp"
#include <QApplication>
#include <QtGui/QApplication>
#include <QVTKWidget.h>



boost::asio::io_service io_service;
receiver r(io_service,
           boost::asio::ip::address::from_string("0.0.0.0"),
           boost::asio::ip::address::from_string("239.255.0.1"));

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow window;
    window.mWebThread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

    sleep(1);

    window.show();
    sleep(1);

    window.StartThread();
    return app.exec();
}
