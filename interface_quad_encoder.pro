TEMPLATE = app
CONFIG += console c++11

CONFIG -= qt

HEADERS += \
    src/interface.h \
    src/ros_node.h \
    src/rpi_interface.h

SOURCES += \
    src/interface.cpp \
    src/main_rpi.cpp \
    src/ros_node.cpp \
    src/rpi_interface.cpp

INCLUDEPATH += \
    /opt/ros/melodic/include \
    ../../devel/include

DISTFILES += \
    CMakeLists.txt \
    package.xml
