#-------------------------------------------------
#
# Project created by QtCreator 2018-03-25T16:22:27
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DGPS_Message_Generator
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
    UbloxInterface.cpp \
    NavMessageDecoder.cpp \
    PositionAveraging.cpp \
    MainWindow.cpp \
    DGPSMesGen.cpp \
    XBeeInterface.cpp \
    RefPosDialog.cpp \
    Debug.cpp

HEADERS += \
    ThreadQueue.h \
    TypeConversion.h \
    UbloxInterface.h \
    UbloxMessages.h \
    NavMessageDecoder.h \
    GPSDataTypes.h \
    PositionAveraging.h \
    MainWindow.h \
    DGPSMesGen.h \
    XBeeInterface.h \
    RTCMEncode.h \
    RefPosDialog.h \
    Debug.h

FORMS += \
        mainwindow.ui \
    refposdialog.ui

SUBDIRS += \
    DGPS_Message_Generator.pro

# Set c++11 compiler
QMAKE_CXXFLAGS += -std=c++11

DISTFILES += \
    system_model.qmodel
