#-------------------------------------------------
#
# Project created by QtCreator 2014-03-23T23:39:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Client
TEMPLATE = app

LIBS += -L/usr/lib -lboost_thread \
        -L/usr/lib -lboost_system \
        -lpthread \
        -lboost_serialization \
        -lQVTK \

#Point Cloud
LIBS += -LC/usr/lib \
-lpcl_common \
-lpcl_features \
-lpcl_kdtree \
-lpcl_io \
-lpcl_io_ply \
-lpcl_visualization \

#VTK (For PCL)
LIBS += -LC/usr/lib \
-lvtkCommon \
-lvtkFiltering \
-lvtkIO \
-lvtkRendering \

INCLUDEPATH += /usr/include/boost \
               /usr/include/pcl-1.7 \
               /usr/include/eigen3 \
               /usr/include/vtk-5.8 \
               /usr/include/flann \



SOURCES += main.cpp\
        mainwindow.cpp \
        receiver.cpp

HEADERS  += mainwindow.h \
    receiver.h

FORMS    += mainwindow.ui
