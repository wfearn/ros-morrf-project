QT += core
QT += gui

CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++11

TARGET = CostmapGenerator
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    generator.cpp \
    point.cpp

HEADERS += \
    generator.h \
    point.h

DISTFILES += \
    outputVal.txt \
    world_solids.png \
    world_boundaries.png \
    image.png
