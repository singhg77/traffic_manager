#-------------------------------------------------
#
# Project created by QtCreator 2014-12-24T19:42:00
#
#-------------------------------------------------
#TEMPLATE = lib

#CONFIG += shared

#TARGET = lib/mrplanner

TARGET = bin/mrplanner

HEADERS  += include/djik.h \
         include/TrafficManager.h \
         ../db/db.h

SOURCES  += src/TrafficManager.cpp \
		src/djik.cpp

QT       += sql widgets quick

INCLUDEPATH += include/ \
    /usr/local/include/ \
    ../db/ \
   ../scheduler/include/

LIBS += -lpthread

QMAKE_CXXFLAGS += -g
