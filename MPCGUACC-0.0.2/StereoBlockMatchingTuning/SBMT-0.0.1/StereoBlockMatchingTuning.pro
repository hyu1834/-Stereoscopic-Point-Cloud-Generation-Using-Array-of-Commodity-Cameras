#-------------------------------------------------
#
# Project created by QtCreator 2017-03-06T15:24:11
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SBMT
TEMPLATE = app

INCLUDEPATH += $$PWD/include
INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib -L/usr/local/share/OpenCV/3rdparty/lib -lopencv_contrib -lopencv_stitching -lopencv_nonfree -lopencv_superres -lopencv_ocl -lopencv_ts -lopencv_videostab -lopencv_gpu -lopencv_photo -lopencv_objdetect -lopencv_legacy -lopencv_video -lopencv_ml -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lIlmImf -lopencv_imgproc -lopencv_flann -lopencv_core -lQt5OpenGL -lQt5Concurrent -lQt5Test -lQt5Widgets -lQt5Gui -lQt5Core -lswscale-ffmpeg -lavutil-ffmpeg -lavformat-ffmpeg -lavcodec-ffmpeg -ljasper -ltiff -lpng -ljpeg -lGL -lGLU -lrt -lpthread -lm -ldl -lstdc++ -lz
LIBS += -ljsoncpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES +=  src/main.cpp\
            src/mainwindow.cpp\
            src/opencv_utils.cpp\
            src/stereo_camera_utils.cpp\
            src/stereo_camera_parameters.cpp\
            src/stereo_camera_calibration_parameters.cpp\
            src/stereo_camera_sbm_parameters.cpp\
            src/stereo_camera_sgbm_parameters.cpp\
            src/stereo_camera_disparity.cpp

HEADERS  += include/mainwindow.h\
            include/opencv_utils.h\
            include/stereo_camera_utils.h\
            include/stereo_camera_parameters.h\
            include/stereo_camera_calibration_parameters.h\
            include/stereo_camera_sbm_parameters.h\
            include/stereo_camera_sgbm_parameters.h\
            include/stereo_camera_disparity.h

FORMS    += forms/mainwindow.ui
