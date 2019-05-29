TEMPLATE = app
CONFIG += console c++11

CONFIG -= qt

HEADERS += \
    src/driver.h \
    src/ros_node.h \
    src/rpi_driver.h

SOURCES += \
    src/driver.cpp \
    src/main_rpi.cpp \
    src/ros_node.cpp \
    src/rpi_driver.cpp

INCLUDEPATH += \
    /opt/ros/melodic/include \
    ../../devel/include

DISTFILES += \
    CMakeLists.txt \
    package.xml
