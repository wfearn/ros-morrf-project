QT += core
QT += gui

CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++11

TARGET = CostmapGenerator
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_highgui

SOURCES += main.cpp \
    generator.cpp \
    point.cpp

HEADERS += \
    generator.h \
    point.h \
    img_load_util.h

DISTFILES += \
    outputVal.txt \
    world_solids.png \
    world_boundaries.png \
    image1.png \
    safeCostMap.png \
    expectedSafeCostMap.png \
    stealthCostMap.png \
    safeCostMap.png \
    stealthOutputVals.txt \
    safeOutputVals.txt \
    world_boundaries_new.png
