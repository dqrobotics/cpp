# win32 {
#     SOURCES += WinSystem.cpp
#     HEADERS += WinSystem.h
# }


unix {

QT -= core
QT -= gui

VERSION = 0.1
CONFIG += c++11

DESTDIR=bin/linux64
OBJECTS_DIR=build/linux64

CONFIG(debug, debug|release) {
    CONFIG -= debug release
    CONFIG += debug
    TARGET = dqrobotics.d
}
CONFIG(release, debug|release) {
    CONFIG -= debug release
    CONFIG += release
    TARGET = dqrobotics
}
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = lib

SOURCES += \
    DQ.cpp \
    DQ_kinematics.cpp

HEADERS += \
    DQ.h \
    DQ_controller.h \
    DQ_kinematics.h

}


