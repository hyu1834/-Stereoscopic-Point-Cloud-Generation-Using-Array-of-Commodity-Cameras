#ifndef MAINWINDOW_H
#define MAINWINDOW_H
/*
	Standard Libraries
*/

/*
	3rd Party Libraries
*/
//QT
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
	Local Libraries
*/
#include "ui_mainwindow.h"
#include "opencv_utils.h"
#include "stereo_camera_calibration_parameters.h"
#include "stereo_camera_sgbm_parameters.h"
#include "stereo_camera_disparity.h"

/*
	Enum
*/

/*
	MACRO
*/



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow	{
	Q_OBJECT

	public:
		explicit MainWindow(QWidget *parent = 0);
		~MainWindow();

	private slots:
		/*
			Clicked Signal Methods
		*/
		void on_calibrationParamsBrowseButton_clicked();
		void on_leftImageBrowseButton_clicked();
		void on_rightImageBrowseButton_clicked();
        void on_computeButton_clicked();
		void on_exportButton_clicked();
        void on_image480PRB_clicked();
        void on_image720PRB_clicked();
        void on_image1080PRB_clicked();
        void on_image2KRB_clicked();
        void on_image4KRB_clicked();
        void on_sgbmRadio_clicked();
        void on_sbmRadio_clicked();


		/*
			Trigger Signal Methods
		*/
		void on_actionHelp_triggered();
		void on_actionAbout_triggered();
		void on_actionExit_triggered();




        void on_actionGenerate_triggered();

        void on_actionExport_triggered();

private:
		Ui::MainWindow *ui;

		// OpenCV mat variable
		cv::Mat leftImage;
		cv::Mat rightImage;
		cv::Mat disparityMap;

		// Stereo Camera instance
		StereoCameraCalibrationParameters* sccp;
		// Stereo SGBM instance
		StereoCameraSGBMParameters* scsgbmp;
		//Stereo Disparity instance
		StereoCameraDisparity* scd;

		/*
			Class Methods
		*/
		void computeDisparity();

};

#endif // MAINWINDOW_H
