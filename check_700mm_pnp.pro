TEMPLATE = app

CONFIG += console c++11

CONFIG -= app_bundle

SOURCES += main.cpp \
    couple_red.cpp

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_* \
        /home/hzh/camera_rcx/linuxSDK/lib/x64/libMVSDK.so

HEADERS += \
    couple_red.h
