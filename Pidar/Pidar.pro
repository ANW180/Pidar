TEMPLATE = app
CONFIG += console
CONFIG -= qt

LIBS += -L/usr/local/lib -lurg_c \
        -L/usr/lib -lboost_thread \
        -L/usr/lib -lboost_system \
        -L/usr/lib -ltinyxml \
        -L/usr/local/lib -lopencv_core \
        -L/usr/local/lib -lopencv_calib3d \
        -L/usr/local/lib -lopencv_highgui \
        -L/usr/local/lib -lwiringPi \
        -L/usr/local/lib -ldxl \
        -L/usr/local/lib -lrt

INCLUDEPATH += /usr/local/include/urg_c \
               /usr/include/boost \
               /usr/local/include/opencv2 \
               /usr/local/include/dxl \

HEADERS += \
    hokuyo.hpp \
    dynamixel.hpp

SOURCES += hokuyo.cpp \
           dynamixel.cpp \
           hokuyoexample.cpp
