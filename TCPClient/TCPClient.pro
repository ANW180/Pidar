TEMPLATE = app
CONFIG += console
CONFIG -= qt

LIBS += \
        -L/usr/lib -lboost_thread \
        -L/usr/lib -lboost_system \
        -lpthread \
        -lboost_serialization \

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
    ../Pidar/connection.hpp \
    ../Pidar/pointcloud.hpp \
    visual.hpp \
    ../Pidar/pointstructs.hpp

SOURCES += main.cpp \
           client.cpp \
           visual.cpp \


