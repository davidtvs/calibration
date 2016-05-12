#-------------------------------------------------
#
# Project created by QtCreator 2016-04-26T15:20:57
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = gui
TEMPLATE = app


SOURCES += src/gui_main.cpp\
        src/gui_mainwindow.cpp \
    src/gui_myrviz.cpp \
    src/gui_calibration_node.cpp \
    src/gui_options.cpp \
    src/gui_supportedsensors.cpp \
    src/gui_QProgressIndicator.cpp

HEADERS  += include/calibration_gui/gui_mainwindow.h \
    include/calibration_gui/gui_myrviz.h \
    include/calibration_gui/gui_calibration_node.hpp \
    include/calibration_gui/gui_options.h \
    include/calibration_gui/gui_options.h \
    include/calibration_gui/gui_supportedsensors.h \
    include/calibration_gui/gui_QProgressIndicator.h

FORMS    += ui/mainwindow.ui \
    ui/options.ui

INCLUDEPATH += $$PWD/include
