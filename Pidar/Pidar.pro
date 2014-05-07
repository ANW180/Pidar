TEMPLATE = app
CONFIG += console
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

#TinyXML
LIBS += -LC/usr/lib \
-ltinyxml \

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
    pointstructs.hpp \
    point3d.hpp

SOURCES +=  hokuyo.cpp \
            dynamixel.cpp \
            control.cpp \
            main.cpp \
            server.cpp \
    commands.cpp
