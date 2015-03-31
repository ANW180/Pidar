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
    Hokuyo.hpp \
    Control.hpp \
    Global.hpp \
    Dynamixel.hpp \
    PointStructs.hpp \
    Point3D.hpp \
    CommandReceiver.hpp \
    Server.hpp \

SOURCES += \
    Hokuyo.cpp \
    Dynamixel.cpp \
    Control.cpp \
    Main.cpp \
    Server.cpp \
    CommandReceiver.cpp \

