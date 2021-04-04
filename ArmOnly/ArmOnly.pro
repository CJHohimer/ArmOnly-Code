QT += core
QT -= gui

CONFIG += c++11

TARGET = ArmOnly
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    robotarm.cpp \
    test3.cpp

HEADERS += \
    robotarm.h

INCLUDEPATH += \\usr\\local\\include \

LIBS += -LC:\\usr\\local\\lib \
-lopencv_calib3d \
-lopencv_core \
-lopencv_xfeatures2d \
-lopencv_features2d \
-lopencv_ximgproc \
-fopenmp \
-lopencv_highgui \
-lopencv_imgcodecs \
-lopencv_imgproc \
-lopencv_photo \
-lpmdaccess2 \
-lPvAPI \
-lImagelib \
-lQt5Widgets \
-lQt5Gui \
-lQt5Test \
-lQt5Concurrent \
-lQt5Core \
-ldxl_x64_cpp \

#LIBS += -L//opt/Qt5.6.0/5.6/gcc_64/lib -lQt5Widgets  -lQt5Gui -lQt5Test -lQt5Concurrent  -lQt5Core

INCLUDEPATH += \\usr\\include \

LIBS += -LC:\\usr\\lib \
-lglog \
-lgflags \
-lprotobuf\
-lphidget21 \
-lflycapture \
-lflycapturegui \
-ltriclops \
-lpnmutils \
-lflycapture2bridge \

INCLUDEPATH += /home/abhi/caffe-master/distribute/include
LIBS += -L/home/abhi/caffe-master/distribute/lib -lcaffe

INCLUDEPATH += /usr/local/cuda-8.0/include

LIBS += -L/usr/local/cuda-8.0/lib64 -lcublas -lcudart -lcudnn

INCLUDEPATH += /usr/include/boost

LIBS += -L/usr/lib/x86_64-linux-gnu \
-lboost_system \
-lpcl_visualization \
-lpcl_common \
-lvtkRenderingCore-6.2 \
-lvtkCommonDataModel-6.2 \
-lvtkCommonMath-6.2 \
-lvtkCommonCore-6.2 \

INCLUDEPATH += /opt/Qt5.6.0/5.6/Src/qtactiveqt/include/
INCLUDEPATH += /usr/include/pcl-1.7/
INCLUDEPATH += /usr/include/vtk-6.2/
#INCLUDEPATH += /opt/Qt5.6.0/5.6/gcc_64/include
#LIBS += -L//opt/Qt5.6.0/5.6/gcc_64/lib -lQt5Widgets  -lQt5Gui -lQt5Test -lQt5Concurrent  -lQt5Core
