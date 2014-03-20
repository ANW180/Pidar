TEMPLATE = app
CONFIG += console
CONFIG -= qt

#Hokuyo
LIBS += -LC/usr/local/lib \
-lurg_c \

#Boost
LIBS += -LC/usr/local/local \
-lboost_thread \
-lboost_system \
-lboost_serialization \

#TinyXML
LIBS += -LC/usr/lib \
-ltinyxml \

#WiringPi
LIBS += -LC/usr/local/lib \
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
            connection.cpp
