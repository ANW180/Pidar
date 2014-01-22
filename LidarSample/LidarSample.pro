TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += main.cpp
LIBS += -L/usr/local/lib -lurg_c
INCLUDEPATH += /usr/local/include/urg_c
