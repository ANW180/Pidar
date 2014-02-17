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

#TinyXML
LIBS += -LC/usr/lib \
-ltinyxml \

#OpenCV
LIBS += -LC/usr/local/lib \
-lopencv_core \
-lopencv_calib3d \
-lopencv_highgui \

#WiringPi
LIBS += -LC/usr/local/lib \
-lwiringPi \

#Dynamixel
LIBS += -LC/usr/local/lib \
-ldxl \

#Real-Time Timing
LIBS += -LC/usr/local/lib \
-lrt \

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
-lvtksys \


INCLUDEPATH += /usr/local/include/urg_c \
               /usr/include/boost \
               /usr/local/include/opencv2 \
               /usr/local/include/dxl \
               /usr/include/pcl-1.7 \
               /usr/include/eigen3 \
               /usr/include/vtk-5.8 \
               /usr/include/flann \

HEADERS += \
    hokuyo.hpp \
    dynamixel.hpp

SOURCES += hokuyo.cpp \
           dynamixel.cpp \
           main.cpp
