QT += testlib
QT -= gui

INCLUDEPATH += ../
DEPENDPATH += ../

CONFIG += qt console warn_on depend_includepath testcase
CONFIG -= app_bundle

TEMPLATE = app

SOURCES +=  tst_testrobot.cpp /
            my_math.cpp
HEADERS += my_math.h
