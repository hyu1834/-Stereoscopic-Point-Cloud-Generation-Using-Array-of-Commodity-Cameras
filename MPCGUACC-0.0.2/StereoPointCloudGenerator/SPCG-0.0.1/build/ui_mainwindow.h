/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "glwidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionGenerate;
    QAction *actionExit;
    QAction *actionHelp;
    QAction *actionAbout;
    QAction *actionExport;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QLabel *menuLabel;
    QFrame *line;
    QLabel *calibrationParamsLabel;
    QHBoxLayout *horizontalLayout_2;
    QLineEdit *calibrationParamsPath;
    QToolButton *calibrationParamsBrowseButton;
    QLabel *blockMatchingLabel;
    QHBoxLayout *horizontalLayout_6;
    QLineEdit *blockMatchingPath;
    QToolButton *blockMatchingBrowseButton;
    QFrame *line_4;
    QLabel *leftInageLabel;
    QHBoxLayout *horizontalLayout_3;
    QLineEdit *leftImagePath;
    QToolButton *leftImageBrowseButton;
    QLabel *rightImageLabel;
    QHBoxLayout *horizontalLayout_4;
    QLineEdit *rightImagePath;
    QToolButton *rightImageBrowseButton;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_5;
    QGridLayout *gridLayout;
    QRadioButton *uhd2KRadio;
    QRadioButton *vga480Radio;
    QRadioButton *hd1080Radio;
    QRadioButton *hd720Radio;
    QRadioButton *uhd4KRadio;
    QHBoxLayout *horizontalLayout_7;
    QVBoxLayout *verticalLayout_3;
    QLabel *minDistLabel;
    QDoubleSpinBox *minSpinBox;
    QVBoxLayout *verticalLayout_4;
    QLabel *maxDistLabel;
    QDoubleSpinBox *maxSpinBox;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_8;
    QRadioButton *sgbmRadio;
    QRadioButton *sbmRadio;
    QPushButton *generatePCButton;
    QSpacerItem *verticalSpacer;
    QFrame *line_3;
    QLabel *resultPCLabel;
    QHBoxLayout *horizontalLayout_5;
    QLineEdit *resultPCPath;
    QToolButton *resultPCBrowseButton;
    QPushButton *exportPCButton;
    GLWidget *openGLWidget;
    QMenuBar *menuBar;
    QMenu *menuAbout;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1371, 818);
        actionGenerate = new QAction(MainWindow);
        actionGenerate->setObjectName(QStringLiteral("actionGenerate"));
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionHelp = new QAction(MainWindow);
        actionHelp->setObjectName(QStringLiteral("actionHelp"));
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionExport = new QAction(MainWindow);
        actionExport->setObjectName(QStringLiteral("actionExport"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        menuLabel = new QLabel(centralWidget);
        menuLabel->setObjectName(QStringLiteral("menuLabel"));
        menuLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(menuLabel);

        line = new QFrame(centralWidget);
        line->setObjectName(QStringLiteral("line"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(line->sizePolicy().hasHeightForWidth());
        line->setSizePolicy(sizePolicy);
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        calibrationParamsLabel = new QLabel(centralWidget);
        calibrationParamsLabel->setObjectName(QStringLiteral("calibrationParamsLabel"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(calibrationParamsLabel->sizePolicy().hasHeightForWidth());
        calibrationParamsLabel->setSizePolicy(sizePolicy1);
        calibrationParamsLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(calibrationParamsLabel);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        calibrationParamsPath = new QLineEdit(centralWidget);
        calibrationParamsPath->setObjectName(QStringLiteral("calibrationParamsPath"));
        calibrationParamsPath->setEnabled(false);
        sizePolicy.setHeightForWidth(calibrationParamsPath->sizePolicy().hasHeightForWidth());
        calibrationParamsPath->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(calibrationParamsPath);

        calibrationParamsBrowseButton = new QToolButton(centralWidget);
        calibrationParamsBrowseButton->setObjectName(QStringLiteral("calibrationParamsBrowseButton"));

        horizontalLayout_2->addWidget(calibrationParamsBrowseButton);


        verticalLayout->addLayout(horizontalLayout_2);

        blockMatchingLabel = new QLabel(centralWidget);
        blockMatchingLabel->setObjectName(QStringLiteral("blockMatchingLabel"));
        blockMatchingLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(blockMatchingLabel);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        blockMatchingPath = new QLineEdit(centralWidget);
        blockMatchingPath->setObjectName(QStringLiteral("blockMatchingPath"));
        blockMatchingPath->setEnabled(false);
        sizePolicy.setHeightForWidth(blockMatchingPath->sizePolicy().hasHeightForWidth());
        blockMatchingPath->setSizePolicy(sizePolicy);

        horizontalLayout_6->addWidget(blockMatchingPath);

        blockMatchingBrowseButton = new QToolButton(centralWidget);
        blockMatchingBrowseButton->setObjectName(QStringLiteral("blockMatchingBrowseButton"));
        blockMatchingBrowseButton->setEnabled(false);

        horizontalLayout_6->addWidget(blockMatchingBrowseButton);


        verticalLayout->addLayout(horizontalLayout_6);

        line_4 = new QFrame(centralWidget);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_4);

        leftInageLabel = new QLabel(centralWidget);
        leftInageLabel->setObjectName(QStringLiteral("leftInageLabel"));
        leftInageLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(leftInageLabel);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        leftImagePath = new QLineEdit(centralWidget);
        leftImagePath->setObjectName(QStringLiteral("leftImagePath"));
        leftImagePath->setEnabled(false);
        sizePolicy.setHeightForWidth(leftImagePath->sizePolicy().hasHeightForWidth());
        leftImagePath->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(leftImagePath);

        leftImageBrowseButton = new QToolButton(centralWidget);
        leftImageBrowseButton->setObjectName(QStringLiteral("leftImageBrowseButton"));
        leftImageBrowseButton->setEnabled(false);

        horizontalLayout_3->addWidget(leftImageBrowseButton);


        verticalLayout->addLayout(horizontalLayout_3);

        rightImageLabel = new QLabel(centralWidget);
        rightImageLabel->setObjectName(QStringLiteral("rightImageLabel"));
        rightImageLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(rightImageLabel);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        rightImagePath = new QLineEdit(centralWidget);
        rightImagePath->setObjectName(QStringLiteral("rightImagePath"));
        rightImagePath->setEnabled(false);
        sizePolicy.setHeightForWidth(rightImagePath->sizePolicy().hasHeightForWidth());
        rightImagePath->setSizePolicy(sizePolicy);

        horizontalLayout_4->addWidget(rightImagePath);

        rightImageBrowseButton = new QToolButton(centralWidget);
        rightImageBrowseButton->setObjectName(QStringLiteral("rightImageBrowseButton"));
        rightImageBrowseButton->setEnabled(false);

        horizontalLayout_4->addWidget(rightImageBrowseButton);


        verticalLayout->addLayout(horizontalLayout_4);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setAlignment(Qt::AlignCenter);
        verticalLayout_5 = new QVBoxLayout(groupBox_2);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        uhd2KRadio = new QRadioButton(groupBox_2);
        uhd2KRadio->setObjectName(QStringLiteral("uhd2KRadio"));
        uhd2KRadio->setEnabled(false);

        gridLayout->addWidget(uhd2KRadio, 1, 1, 1, 1);

        vga480Radio = new QRadioButton(groupBox_2);
        vga480Radio->setObjectName(QStringLiteral("vga480Radio"));
        vga480Radio->setEnabled(false);

        gridLayout->addWidget(vga480Radio, 0, 0, 1, 1);

        hd1080Radio = new QRadioButton(groupBox_2);
        hd1080Radio->setObjectName(QStringLiteral("hd1080Radio"));

        gridLayout->addWidget(hd1080Radio, 1, 0, 1, 1);

        hd720Radio = new QRadioButton(groupBox_2);
        hd720Radio->setObjectName(QStringLiteral("hd720Radio"));
        hd720Radio->setChecked(true);

        gridLayout->addWidget(hd720Radio, 0, 1, 1, 1);

        uhd4KRadio = new QRadioButton(groupBox_2);
        uhd4KRadio->setObjectName(QStringLiteral("uhd4KRadio"));
        uhd4KRadio->setEnabled(false);

        gridLayout->addWidget(uhd4KRadio, 2, 0, 1, 1);


        verticalLayout_5->addLayout(gridLayout);


        verticalLayout->addWidget(groupBox_2);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        minDistLabel = new QLabel(centralWidget);
        minDistLabel->setObjectName(QStringLiteral("minDistLabel"));
        minDistLabel->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(minDistLabel);

        minSpinBox = new QDoubleSpinBox(centralWidget);
        minSpinBox->setObjectName(QStringLiteral("minSpinBox"));
        minSpinBox->setMinimum(50);

        verticalLayout_3->addWidget(minSpinBox);


        horizontalLayout_7->addLayout(verticalLayout_3);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        maxDistLabel = new QLabel(centralWidget);
        maxDistLabel->setObjectName(QStringLiteral("maxDistLabel"));
        maxDistLabel->setAlignment(Qt::AlignCenter);

        verticalLayout_4->addWidget(maxDistLabel);

        maxSpinBox = new QDoubleSpinBox(centralWidget);
        maxSpinBox->setObjectName(QStringLiteral("maxSpinBox"));
        maxSpinBox->setMinimum(3000);

        verticalLayout_4->addWidget(maxSpinBox);


        horizontalLayout_7->addLayout(verticalLayout_4);


        verticalLayout->addLayout(horizontalLayout_7);

        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setAlignment(Qt::AlignCenter);
        horizontalLayout_8 = new QHBoxLayout(groupBox);
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        sgbmRadio = new QRadioButton(groupBox);
        sgbmRadio->setObjectName(QStringLiteral("sgbmRadio"));
        sgbmRadio->setChecked(true);

        horizontalLayout_8->addWidget(sgbmRadio);

        sbmRadio = new QRadioButton(groupBox);
        sbmRadio->setObjectName(QStringLiteral("sbmRadio"));
        sbmRadio->setEnabled(false);

        horizontalLayout_8->addWidget(sbmRadio);


        verticalLayout->addWidget(groupBox);

        generatePCButton = new QPushButton(centralWidget);
        generatePCButton->setObjectName(QStringLiteral("generatePCButton"));
        generatePCButton->setEnabled(false);

        verticalLayout->addWidget(generatePCButton);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        line_3 = new QFrame(centralWidget);
        line_3->setObjectName(QStringLiteral("line_3"));
        sizePolicy.setHeightForWidth(line_3->sizePolicy().hasHeightForWidth());
        line_3->setSizePolicy(sizePolicy);
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_3);

        resultPCLabel = new QLabel(centralWidget);
        resultPCLabel->setObjectName(QStringLiteral("resultPCLabel"));
        resultPCLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(resultPCLabel);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        resultPCPath = new QLineEdit(centralWidget);
        resultPCPath->setObjectName(QStringLiteral("resultPCPath"));
        resultPCPath->setEnabled(false);
        sizePolicy.setHeightForWidth(resultPCPath->sizePolicy().hasHeightForWidth());
        resultPCPath->setSizePolicy(sizePolicy);

        horizontalLayout_5->addWidget(resultPCPath);

        resultPCBrowseButton = new QToolButton(centralWidget);
        resultPCBrowseButton->setObjectName(QStringLiteral("resultPCBrowseButton"));
        resultPCBrowseButton->setEnabled(false);

        horizontalLayout_5->addWidget(resultPCBrowseButton);


        verticalLayout->addLayout(horizontalLayout_5);

        exportPCButton = new QPushButton(centralWidget);
        exportPCButton->setObjectName(QStringLiteral("exportPCButton"));
        exportPCButton->setEnabled(false);

        verticalLayout->addWidget(exportPCButton);


        horizontalLayout->addLayout(verticalLayout);

        openGLWidget = new GLWidget(centralWidget);
        openGLWidget->setObjectName(QStringLiteral("openGLWidget"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(openGLWidget->sizePolicy().hasHeightForWidth());
        openGLWidget->setSizePolicy(sizePolicy2);

        horizontalLayout->addWidget(openGLWidget);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1371, 22));
        menuAbout = new QMenu(menuBar);
        menuAbout->setObjectName(QStringLiteral("menuAbout"));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuAbout->menuAction());
        menuAbout->addAction(actionHelp);
        menuAbout->addSeparator();
        menuAbout->addAction(actionAbout);
        menuFile->addAction(actionGenerate);
        menuFile->addAction(actionExport);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        mainToolBar->addAction(actionGenerate);
        mainToolBar->addAction(actionExport);
        mainToolBar->addSeparator();
        mainToolBar->addAction(actionHelp);
        mainToolBar->addAction(actionAbout);
        mainToolBar->addSeparator();
        mainToolBar->addAction(actionExit);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Stereo Point Cloud Generator", 0));
        actionGenerate->setText(QApplication::translate("MainWindow", "Generate", 0));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0));
        actionHelp->setText(QApplication::translate("MainWindow", "Help", 0));
        actionAbout->setText(QApplication::translate("MainWindow", "About", 0));
        actionExport->setText(QApplication::translate("MainWindow", "Export", 0));
        menuLabel->setText(QApplication::translate("MainWindow", "Menu", 0));
        calibrationParamsLabel->setText(QApplication::translate("MainWindow", "Calibration Parameters", 0));
        calibrationParamsBrowseButton->setText(QApplication::translate("MainWindow", "...", 0));
        blockMatchingLabel->setText(QApplication::translate("MainWindow", "Block Matching Parameters", 0));
        blockMatchingBrowseButton->setText(QApplication::translate("MainWindow", "...", 0));
        leftInageLabel->setText(QApplication::translate("MainWindow", "Left Image", 0));
        leftImageBrowseButton->setText(QApplication::translate("MainWindow", "...", 0));
        rightImageLabel->setText(QApplication::translate("MainWindow", "Right Image", 0));
        rightImageBrowseButton->setText(QApplication::translate("MainWindow", "...", 0));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Image Resolution", 0));
        uhd2KRadio->setText(QApplication::translate("MainWindow", "2K", 0));
        vga480Radio->setText(QApplication::translate("MainWindow", "480P/VGA", 0));
        hd1080Radio->setText(QApplication::translate("MainWindow", "1080P", 0));
        hd720Radio->setText(QApplication::translate("MainWindow", "720P", 0));
        uhd4KRadio->setText(QApplication::translate("MainWindow", "4K", 0));
        minDistLabel->setText(QApplication::translate("MainWindow", "Min Distance (MM)", 0));
        maxDistLabel->setText(QApplication::translate("MainWindow", "Max Distance (MM)", 0));
        groupBox->setTitle(QApplication::translate("MainWindow", "Block Matching Algorithm", 0));
        sgbmRadio->setText(QApplication::translate("MainWindow", "SGBM", 0));
        sbmRadio->setText(QApplication::translate("MainWindow", "SBM", 0));
        generatePCButton->setText(QApplication::translate("MainWindow", "Generate", 0));
        resultPCLabel->setText(QApplication::translate("MainWindow", "Result Point Cloud Path", 0));
        resultPCBrowseButton->setText(QApplication::translate("MainWindow", "...", 0));
        exportPCButton->setText(QApplication::translate("MainWindow", "Export", 0));
        menuAbout->setTitle(QApplication::translate("MainWindow", "About", 0));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
