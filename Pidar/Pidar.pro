TEMPLATE = app
CONFIG += console
CONFIG += c++11
CONFIG -= qt

target.path=/home/pi/software/deploy
INSTALLS+=target
#Hokuyo
LIBS += -LC/usr/local/lib \
-lurg_c \

#Boost
LIBS += -LC/usr/local/local \
-lboost_thread \
-lboost_system \
-lboost_serialization \
-lpthread

#WiringPi
LIBS += -LC/usr/lib \
-lwiringPi \

#Dynamixel
LIBS += -LC/usr/local/lib \
-ldxl \

#Real-Time Timing
LIBS += -LC/usr/local/lib \
-lrt \


INCLUDEPATH += /usr/local/include/urg_c \
               /usr/include/boost \
               /usr/local/include/dxl \
               /usr/local/include

HEADERS += \
    hokuyo.hpp \
    control.hpp \
    global.hpp \
    dynamixel.hpp \
    point_structs.hpp \
    point3d.hpp \
    command_receiver.hpp \
    server.hpp \

SOURCES += \
    hokuyo.cpp \
    dynamixel.cpp \
    control.cpp \
    main.cpp \
    server.cpp \
    command_receiver.cpp \

