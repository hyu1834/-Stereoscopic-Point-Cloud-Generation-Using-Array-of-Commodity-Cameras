/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
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
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionHelp;
    QAction *actionAbout;
    QAction *actionExit;
    QAction *actionGenerate;
    QAction *actionExport;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label_2;
    QFrame *line_2;
    QLabel *label_6;
    QHBoxLayout *horizontalLayout_4;
    QLineEdit *calibrationParamsPath;
    QToolButton *calibrationParamsBrowseButton;
    QLabel *label;
    QHBoxLayout *horizontalLayout_2;
    QLineEdit *leftImagePath;
    QToolButton *leftImageBrowseButton;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_3;
    QLineEdit *rightImagePath;
    QToolButton *rightImageBrowseButton;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_2;
    QRadioButton *image480PRB;
    QRadioButton *image720PRB;
    QRadioButton *image1080PRB;
    QRadioButton *image2KRB;
    QRadioButton *image4KRB;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_5;
    QRadioButton *sgbmRadio;
    QRadioButton *sbmRadio;
    QPushButton *computeButton;
    QSpacerItem *verticalSpacer;
    QPushButton *exportButton;
    QFrame *line;
    QVBoxLayout *verticalLayout_3;
    QLabel *imageViewerLabel;
    QLabel *imageViewer;
    QFrame *line_4;
    QLabel *bmSettingLabel;
    QScrollArea *settingScrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *verticalLayout_9;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_7;
    QSpinBox *preFilterCapSpinBox;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_8;
    QSpinBox *sadWindowSizeSpinBox;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_9;
    QSpinBox *minDisparitySpinBox;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_10;
    QSpinBox *numOfDisparitySpinBox;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_11;
    QSpinBox *uniquenessRatioSpinBox;
    QVBoxLayout *verticalLayout_10;
    QHBoxLayout *horizontalLayout_20;
    QLabel *label_14;
    QSpinBox *speckleWindowSizeSpinBox;
    QHBoxLayout *horizontalLayout_19;
    QLabel *label_17;
    QSpinBox *speckleRangeSpinBox;
    QHBoxLayout *horizontalLayout_17;
    QLabel *label_16;
    QSpinBox *disp12MaxDiffSpinBox;
    QHBoxLayout *horizontalLayout_18;
    QLabel *label_15;
    QSpinBox *fullDPSpinBox;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuAbout;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1306, 672);
        actionHelp = new QAction(MainWindow);
        actionHelp->setObjectName(QStringLiteral("actionHelp"));
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionGenerate = new QAction(MainWindow);
        actionGenerate->setObjectName(QStringLiteral("actionGenerate"));
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
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(label_2);

        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_2);

        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(label_6);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        calibrationParamsPath = new QLineEdit(centralWidget);
        calibrationParamsPath->setObjectName(QStringLiteral("calibrationParamsPath"));
        calibrationParamsPath->setEnabled(false);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(calibrationParamsPath->sizePolicy().hasHeightForWidth());
        calibrationParamsPath->setSizePolicy(sizePolicy);

        horizontalLayout_4->addWidget(calibrationParamsPath);

        calibrationParamsBrowseButton = new QToolButton(centralWidget);
        calibrationParamsBrowseButton->setObjectName(QStringLiteral("calibrationParamsBrowseButton"));

        horizontalLayout_4->addWidget(calibrationParamsBrowseButton);


        verticalLayout->addLayout(horizontalLayout_4);

        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout->addWidget(label);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        leftImagePath = new QLineEdit(centralWidget);
        leftImagePath->setObjectName(QStringLiteral("leftImagePath"));
        leftImagePath->setEnabled(false);
        sizePolicy.setHeightForWidth(leftImagePath->sizePolicy().hasHeightForWidth());
        leftImagePath->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(leftImagePath);

        leftImageBrowseButton = new QToolButton(centralWidget);
        leftImageBrowseButton->setObjectName(QStringLiteral("leftImageBrowseButton"));
        leftImageBrowseButton->setEnabled(false);

        horizontalLayout_2->addWidget(leftImageBrowseButton);


        verticalLayout->addLayout(horizontalLayout_2);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        verticalLayout->addWidget(label_3);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        rightImagePath = new QLineEdit(centralWidget);
        rightImagePath->setObjectName(QStringLiteral("rightImagePath"));
        rightImagePath->setEnabled(false);
        sizePolicy.setHeightForWidth(rightImagePath->sizePolicy().hasHeightForWidth());
        rightImagePath->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(rightImagePath);

        rightImageBrowseButton = new QToolButton(centralWidget);
        rightImageBrowseButton->setObjectName(QStringLiteral("rightImageBrowseButton"));
        rightImageBrowseButton->setEnabled(false);

        horizontalLayout_3->addWidget(rightImageBrowseButton);


        verticalLayout->addLayout(horizontalLayout_3);

        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setAlignment(Qt::AlignCenter);
        verticalLayout_2 = new QVBoxLayout(groupBox);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        image480PRB = new QRadioButton(groupBox);
        image480PRB->setObjectName(QStringLiteral("image480PRB"));
        image480PRB->setChecked(false);

        verticalLayout_2->addWidget(image480PRB);

        image720PRB = new QRadioButton(groupBox);
        image720PRB->setObjectName(QStringLiteral("image720PRB"));
        image720PRB->setChecked(true);

        verticalLayout_2->addWidget(image720PRB);

        image1080PRB = new QRadioButton(groupBox);
        image1080PRB->setObjectName(QStringLiteral("image1080PRB"));
        image1080PRB->setChecked(false);

        verticalLayout_2->addWidget(image1080PRB);

        image2KRB = new QRadioButton(groupBox);
        image2KRB->setObjectName(QStringLiteral("image2KRB"));
        image2KRB->setEnabled(false);

        verticalLayout_2->addWidget(image2KRB);

        image4KRB = new QRadioButton(groupBox);
        image4KRB->setObjectName(QStringLiteral("image4KRB"));
        image4KRB->setEnabled(false);

        verticalLayout_2->addWidget(image4KRB);


        verticalLayout->addWidget(groupBox);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setAlignment(Qt::AlignCenter);
        horizontalLayout_5 = new QHBoxLayout(groupBox_2);
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        sgbmRadio = new QRadioButton(groupBox_2);
        sgbmRadio->setObjectName(QStringLiteral("sgbmRadio"));
        sgbmRadio->setChecked(true);

        horizontalLayout_5->addWidget(sgbmRadio);

        sbmRadio = new QRadioButton(groupBox_2);
        sbmRadio->setObjectName(QStringLiteral("sbmRadio"));
        sbmRadio->setEnabled(false);

        horizontalLayout_5->addWidget(sbmRadio);


        verticalLayout->addWidget(groupBox_2);

        computeButton = new QPushButton(centralWidget);
        computeButton->setObjectName(QStringLiteral("computeButton"));

        verticalLayout->addWidget(computeButton);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        exportButton = new QPushButton(centralWidget);
        exportButton->setObjectName(QStringLiteral("exportButton"));
        exportButton->setEnabled(false);

        verticalLayout->addWidget(exportButton);

        line = new QFrame(centralWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);


        horizontalLayout->addLayout(verticalLayout);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        imageViewerLabel = new QLabel(centralWidget);
        imageViewerLabel->setObjectName(QStringLiteral("imageViewerLabel"));
        imageViewerLabel->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(imageViewerLabel);

        imageViewer = new QLabel(centralWidget);
        imageViewer->setObjectName(QStringLiteral("imageViewer"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(imageViewer->sizePolicy().hasHeightForWidth());
        imageViewer->setSizePolicy(sizePolicy1);
        imageViewer->setAutoFillBackground(true);

        verticalLayout_3->addWidget(imageViewer);

        line_4 = new QFrame(centralWidget);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line_4);

        bmSettingLabel = new QLabel(centralWidget);
        bmSettingLabel->setObjectName(QStringLiteral("bmSettingLabel"));
        bmSettingLabel->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(bmSettingLabel);

        settingScrollArea = new QScrollArea(centralWidget);
        settingScrollArea->setObjectName(QStringLiteral("settingScrollArea"));
        settingScrollArea->setEnabled(false);
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(settingScrollArea->sizePolicy().hasHeightForWidth());
        settingScrollArea->setSizePolicy(sizePolicy2);
        settingScrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 1100, 186));
        verticalLayout_8 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        label_7 = new QLabel(scrollAreaWidgetContents);
        label_7->setObjectName(QStringLiteral("label_7"));

        horizontalLayout_7->addWidget(label_7);

        preFilterCapSpinBox = new QSpinBox(scrollAreaWidgetContents);
        preFilterCapSpinBox->setObjectName(QStringLiteral("preFilterCapSpinBox"));
        preFilterCapSpinBox->setValue(63);

        horizontalLayout_7->addWidget(preFilterCapSpinBox);


        verticalLayout_9->addLayout(horizontalLayout_7);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        label_8 = new QLabel(scrollAreaWidgetContents);
        label_8->setObjectName(QStringLiteral("label_8"));

        horizontalLayout_9->addWidget(label_8);

        sadWindowSizeSpinBox = new QSpinBox(scrollAreaWidgetContents);
        sadWindowSizeSpinBox->setObjectName(QStringLiteral("sadWindowSizeSpinBox"));
        sadWindowSizeSpinBox->setValue(3);

        horizontalLayout_9->addWidget(sadWindowSizeSpinBox);


        verticalLayout_9->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        label_9 = new QLabel(scrollAreaWidgetContents);
        label_9->setObjectName(QStringLiteral("label_9"));

        horizontalLayout_10->addWidget(label_9);

        minDisparitySpinBox = new QSpinBox(scrollAreaWidgetContents);
        minDisparitySpinBox->setObjectName(QStringLiteral("minDisparitySpinBox"));
        minDisparitySpinBox->setValue(5);

        horizontalLayout_10->addWidget(minDisparitySpinBox);


        verticalLayout_9->addLayout(horizontalLayout_10);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QStringLiteral("horizontalLayout_11"));
        label_10 = new QLabel(scrollAreaWidgetContents);
        label_10->setObjectName(QStringLiteral("label_10"));

        horizontalLayout_11->addWidget(label_10);

        numOfDisparitySpinBox = new QSpinBox(scrollAreaWidgetContents);
        numOfDisparitySpinBox->setObjectName(QStringLiteral("numOfDisparitySpinBox"));
        numOfDisparitySpinBox->setMaximum(99999);
        numOfDisparitySpinBox->setSingleStep(16);
        numOfDisparitySpinBox->setValue(240);

        horizontalLayout_11->addWidget(numOfDisparitySpinBox);


        verticalLayout_9->addLayout(horizontalLayout_11);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        label_11 = new QLabel(scrollAreaWidgetContents);
        label_11->setObjectName(QStringLiteral("label_11"));

        horizontalLayout_12->addWidget(label_11);

        uniquenessRatioSpinBox = new QSpinBox(scrollAreaWidgetContents);
        uniquenessRatioSpinBox->setObjectName(QStringLiteral("uniquenessRatioSpinBox"));
        uniquenessRatioSpinBox->setValue(10);

        horizontalLayout_12->addWidget(uniquenessRatioSpinBox);


        verticalLayout_9->addLayout(horizontalLayout_12);


        horizontalLayout_6->addLayout(verticalLayout_9);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setSpacing(6);
        horizontalLayout_20->setObjectName(QStringLiteral("horizontalLayout_20"));
        label_14 = new QLabel(scrollAreaWidgetContents);
        label_14->setObjectName(QStringLiteral("label_14"));

        horizontalLayout_20->addWidget(label_14);

        speckleWindowSizeSpinBox = new QSpinBox(scrollAreaWidgetContents);
        speckleWindowSizeSpinBox->setObjectName(QStringLiteral("speckleWindowSizeSpinBox"));
        speckleWindowSizeSpinBox->setMaximum(99999);
        speckleWindowSizeSpinBox->setSingleStep(1);
        speckleWindowSizeSpinBox->setValue(100);

        horizontalLayout_20->addWidget(speckleWindowSizeSpinBox);


        verticalLayout_10->addLayout(horizontalLayout_20);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setObjectName(QStringLiteral("horizontalLayout_19"));
        label_17 = new QLabel(scrollAreaWidgetContents);
        label_17->setObjectName(QStringLiteral("label_17"));

        horizontalLayout_19->addWidget(label_17);

        speckleRangeSpinBox = new QSpinBox(scrollAreaWidgetContents);
        speckleRangeSpinBox->setObjectName(QStringLiteral("speckleRangeSpinBox"));
        speckleRangeSpinBox->setValue(32);

        horizontalLayout_19->addWidget(speckleRangeSpinBox);


        verticalLayout_10->addLayout(horizontalLayout_19);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setSpacing(6);
        horizontalLayout_17->setObjectName(QStringLiteral("horizontalLayout_17"));
        label_16 = new QLabel(scrollAreaWidgetContents);
        label_16->setObjectName(QStringLiteral("label_16"));

        horizontalLayout_17->addWidget(label_16);

        disp12MaxDiffSpinBox = new QSpinBox(scrollAreaWidgetContents);
        disp12MaxDiffSpinBox->setObjectName(QStringLiteral("disp12MaxDiffSpinBox"));
        disp12MaxDiffSpinBox->setValue(1);

        horizontalLayout_17->addWidget(disp12MaxDiffSpinBox);


        verticalLayout_10->addLayout(horizontalLayout_17);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setSpacing(6);
        horizontalLayout_18->setObjectName(QStringLiteral("horizontalLayout_18"));
        label_15 = new QLabel(scrollAreaWidgetContents);
        label_15->setObjectName(QStringLiteral("label_15"));

        horizontalLayout_18->addWidget(label_15);

        fullDPSpinBox = new QSpinBox(scrollAreaWidgetContents);
        fullDPSpinBox->setObjectName(QStringLiteral("fullDPSpinBox"));
        fullDPSpinBox->setValue(1);

        horizontalLayout_18->addWidget(fullDPSpinBox);


        verticalLayout_10->addLayout(horizontalLayout_18);


        horizontalLayout_6->addLayout(verticalLayout_10);


        verticalLayout_8->addLayout(horizontalLayout_6);

        settingScrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout_3->addWidget(settingScrollArea);


        horizontalLayout->addLayout(verticalLayout_3);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1306, 22));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuAbout = new QMenu(menuBar);
        menuAbout->setObjectName(QStringLiteral("menuAbout"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuAbout->menuAction());
        menuFile->addAction(actionGenerate);
        menuFile->addAction(actionExport);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuAbout->addAction(actionHelp);
        menuAbout->addSeparator();
        menuAbout->addAction(actionAbout);
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Stereo Block Matching Tuner", Q_NULLPTR));
        actionHelp->setText(QApplication::translate("MainWindow", "Help", Q_NULLPTR));
        actionAbout->setText(QApplication::translate("MainWindow", "About", Q_NULLPTR));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", Q_NULLPTR));
        actionGenerate->setText(QApplication::translate("MainWindow", "Generate", Q_NULLPTR));
        actionExport->setText(QApplication::translate("MainWindow", "Export", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "Menu", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "Calibration Parameters", Q_NULLPTR));
        calibrationParamsBrowseButton->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "Left Image:", Q_NULLPTR));
        leftImageBrowseButton->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "Right Image:", Q_NULLPTR));
        rightImageBrowseButton->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("MainWindow", "Resolution", Q_NULLPTR));
        image480PRB->setText(QApplication::translate("MainWindow", "480P", Q_NULLPTR));
        image720PRB->setText(QApplication::translate("MainWindow", "720P", Q_NULLPTR));
        image1080PRB->setText(QApplication::translate("MainWindow", "1080P", Q_NULLPTR));
        image2KRB->setText(QApplication::translate("MainWindow", "2K", Q_NULLPTR));
        image4KRB->setText(QApplication::translate("MainWindow", "4K", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Block Matching Alg", Q_NULLPTR));
        sgbmRadio->setText(QApplication::translate("MainWindow", "SGBM", Q_NULLPTR));
        sbmRadio->setText(QApplication::translate("MainWindow", "SBM", Q_NULLPTR));
        computeButton->setText(QApplication::translate("MainWindow", "Compute", Q_NULLPTR));
        exportButton->setText(QApplication::translate("MainWindow", "Export", Q_NULLPTR));
        imageViewerLabel->setText(QApplication::translate("MainWindow", "SGBM", Q_NULLPTR));
        imageViewer->setText(QString());
        bmSettingLabel->setText(QApplication::translate("MainWindow", "SGBM Setting", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "PreFilterCap: ", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindow", "SADWindowSize:", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindow", "MinDisparity", Q_NULLPTR));
        label_10->setText(QApplication::translate("MainWindow", "NumOfDisparities", Q_NULLPTR));
        label_11->setText(QApplication::translate("MainWindow", "UniquenessRatio", Q_NULLPTR));
        label_14->setText(QApplication::translate("MainWindow", "SpeckleWindowSize:", Q_NULLPTR));
        label_17->setText(QApplication::translate("MainWindow", "SpeckleRange:", Q_NULLPTR));
        label_16->setText(QApplication::translate("MainWindow", "Disp12MaxDiff:", Q_NULLPTR));
        label_15->setText(QApplication::translate("MainWindow", "FullDP:", Q_NULLPTR));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", Q_NULLPTR));
        menuAbout->setTitle(QApplication::translate("MainWindow", "About", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
