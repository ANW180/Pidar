TEMPLATE = app
CONFIG += console
CONFIG -= qt

LIBS += -L/usr/local/lib -lurg_c \
        -L/usr/lib -lboost_thread \
        -L/usr/lib -ltinyxml \
        -L/usr/local/lib -lopencv_core \
        -L/usr/local/lib -lopencv_calib3d \
        -L/usr/local/lib -lopencv_highgui
INCLUDEPATH += /usr/local/include/urg_c \
               /usr/include/boost \
               /usr/local/include/opencv2

HEADERS += \
    hokuyo.h

SOURCES += main.cpp \
    hokuyo.cpp
