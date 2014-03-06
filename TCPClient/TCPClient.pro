TEMPLATE = app
CONFIG += console
CONFIG -= qt

LIBS += \
        -L/usr/lib -lboost_thread \
        -L/usr/lib -lboost_system \
        -lpthread \
        -lboost_serialization \


INCLUDEPATH += \
               /usr/include/boost \

HEADERS += \
    ../Pidar/connection.hpp \
    ../Pidar/pointcloud.hpp \

SOURCES += main.cpp \
           client.cpp


