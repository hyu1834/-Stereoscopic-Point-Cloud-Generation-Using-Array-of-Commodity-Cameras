#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/*
	Standard Libraries
*/
#include <cstdlib>
#include <iostream>
// #include <fstream>
#include <string>
// #include <sstream>
// #include <thread> 

/*
	3rd Party Libraries
*/
//QT
#include <QTime>
#include <QtGui>
#include <QFile>
#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QDesktopWidget>
#include <QCoreApplication>

//PCL
// #include <pcl/common/common_headers.h>

/*
	Local Libraries
*/
#include "debugging.h"
#include "../build/ui_mainwindow.h"
#include "str_utils.h"
#include "opencv_utils.h"
#include "point_cloud_utils.h"
#include "point_cloud_container.h"
#include "stereo_camera_calibration_parameters.h"
#include "stereo_camera_sgbm_parameters.h"
#include "stereo_camera_point_cloud.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

	public:
		explicit MainWindow(QWidget *parent = 0);
		~MainWindow();

	private slots:
		/*
			Clicked Signal Methods
		*/
		void on_calibrationParamsBrowseButton_clicked();
		void on_blockMatchingBrowseButton_clicked();
		void on_leftImageBrowseButton_clicked();
		void on_rightImageBrowseButton_clicked();
		void on_resultPCBrowseButton_clicked();
		void on_generatePCButton_clicked();
        void on_exportPCButton_clicked();

		/*
			Trigger Signal Methods
		*/
		void on_actionGenerate_triggered();
		void on_actionExport_triggered();
		void on_actionExit_triggered();
		void on_actionHelp_triggered();
		void on_actionAbout_triggered();

	   

	private:
		Ui::MainWindow *ui;

		/*
			Point Cloud Variables
		*/
		PointCloudContainer* pcc;

		/*
			Stereo Camera Instance
		*/
		StereoCameraCalibrationParameters* sccp;
		StereoCameraSGBMParameters* scsgbmp;
		StereoCameraPointCloud* scpc; 

		/*
			Class Methods
		*/
		void generatePointCloud(std::string calibrationPath, std::string bmPath, std::string leftIMPath,
								std::string rightIMPath, IMAGE_RESOLUTION imageResolution);
		void exportPoints(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);
};

#endif // MAINWINDOW_H
