TEMPLATE = app
CONFIG += console
CONFIG -= qt

LIBS += \
        -L/usr/lib -lboost_thread \
        -L/usr/lib -lboost_system \
        -lpthread \


INCLUDEPATH += \
               /usr/include/boost \


SOURCES += main.cpp

