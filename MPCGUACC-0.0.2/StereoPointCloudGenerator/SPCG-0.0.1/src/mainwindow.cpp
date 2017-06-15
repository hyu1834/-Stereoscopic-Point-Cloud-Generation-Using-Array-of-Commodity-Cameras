#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)	{

	ui->setupUi(this);
	// Set window to be maximum size
	QWidget::showMaximized();

	pcc = PointCloudContainer::init(1);

	// Stereo Camera Instance
	sccp = new StereoCameraCalibrationParameters(SIDE_BY_SIDE);
	scsgbmp = new StereoCameraSGBMParameters();
	scpc = new StereoCameraPointCloud(sccp, CALIBRATED);
}

MainWindow::~MainWindow()	{
	delete ui;
}

/*
	Clicked Signal Methods
*/
void MainWindow::on_calibrationParamsBrowseButton_clicked()	{
	QString filePath = QFileDialog::getOpenFileName(this, tr("Select Stereo Camera Calibration file"), QString(),
														  tr("XML Files (*.xml);;All Files (*.*)"));
	if(filePath != "")	{
		// set camera parameter path
		sccp->setCalibrationFP(filePath.toUtf8().constData());
		// read the parameters
		if(!sccp->readParams())	{
			QMessageBox::critical(this, "Error!", "Error: Unable to parse stereo camera calibration parameters");
			return;
		}
		// set line edit content
		ui->calibrationParamsPath->setText(filePath);
		ui->blockMatchingBrowseButton->setEnabled(true);
	}
}

void MainWindow::on_blockMatchingBrowseButton_clicked()	{
	QString filePath = QFileDialog::getOpenFileName(this, tr("Select Block Matching file"), QString(),
														  tr("Json Files (*.json);;All Files (*.*)"));
	if(filePath != "")	{
		// set block matching parameter path
		scsgbmp->setBMPPath(filePath.toUtf8().constData());
		// set line edit content
		ui->blockMatchingPath->setText(filePath);
		ui->leftImageBrowseButton->setEnabled(true);
	}
}

void MainWindow::on_leftImageBrowseButton_clicked()	{
	QString filePath = QFileDialog::getOpenFileName(this, tr("Select Image file"), QString(),
														  tr("BMP Files (*.bmp);;JPEG Files (*.jpeg);;PNG Files (*.png);;All Files (*.*)"));
	if(filePath != "")	{
		// set line edit content
		ui->leftImagePath->setText(filePath);
		ui->rightImageBrowseButton->setEnabled(true);
	}
}

void MainWindow::on_rightImageBrowseButton_clicked()	{
	QString filePath = QFileDialog::getOpenFileName(this, tr("Select Image file"), QString(),
														  tr("BMP Files (*.bmp);;JPEG Files (*.jpeg);;PNG Files (*.png);;All Files (*.*)"));
	if(filePath != "")	{
		// set line edit content
		ui->rightImagePath->setText(filePath);
		// enable generate button
		ui->generatePCButton->setEnabled(true);
	}
}

void MainWindow::on_resultPCBrowseButton_clicked()	{
	QString filePath = QFileDialog::getSaveFileName(this, tr("Save Point Cloud File"), QString(),
														  tr("Text Files (*.txt);;All Files (*.*)"));

	if(filePath != "")	{
		ui->resultPCPath->setText(filePath);
		//enable export pc button
		ui->exportPCButton->setEnabled(true);
	}
}

void MainWindow::on_generatePCButton_clicked()	{
	// Make sure all required field are filled
	if(ui->calibrationParamsPath->text() == "")	{
		QMessageBox::critical(this, "Error!", "Error: Stereo camera calibration parameters is not filled!!");
		return;
	}
	if(ui->blockMatchingPath->text() == "")	{
		QMessageBox::critical(this, "Error!", "Error: Stereo camera block matching parameters is not filled!!");
		return;
	}
	if(ui->leftImagePath->text() == "")	{
		QMessageBox::critical(this, "Error!", "Error: Left image path is not filled!!");
		return;
	}
	if(ui->rightImagePath->text() == "")	{
		QMessageBox::critical(this, "Error!", "Error: Right image path is not filled!!");
		return;
	}

	// Get image reaolution
	IMAGE_RESOLUTION imageResolution;
    if(ui->vga480Radio->isChecked())	{
		imageResolution = IMR_480;
	}
	else if(ui->hd720Radio->isChecked())	{
		imageResolution = IMR_720;
	}
	else if(ui->hd1080Radio->isChecked())	{
		imageResolution = IMR_1080;
	}
    else if(ui->uhd2KRadio->isChecked())	{
//		imageResolution = IMR_5MP;
	}
	else	{
//		imageResolution = IMR_8MP;
	}

	// Generate point cloud
	generatePointCloud(ui->calibrationParamsPath->text().toUtf8().constData(),
					   ui->blockMatchingPath->text().toUtf8().constData(),
					   ui->leftImagePath->text().toUtf8().constData(),
					   ui->rightImagePath->text().toUtf8().constData(),
					   imageResolution);

	// enable export browse button
	ui->resultPCBrowseButton->setEnabled(true);
}

void MainWindow::on_exportPCButton_clicked()	{
	if(ui->resultPCPath->text() == "")	{
		QMessageBox::critical(this, "Error!", "Error: Result Point Cloud Path is not filled");
		return;
	}

	// Export point cloud to given path
	exportPoints(ui->resultPCPath->text().toUtf8().constData(), pcc->getPCContainer3D(0));
}


/*
	Trigger Signal Methods
*/
void MainWindow::on_actionGenerate_triggered()	{
	on_generatePCButton_clicked();
}

void MainWindow::on_actionExport_triggered()	{
	on_exportPCButton_clicked();
}


void MainWindow::on_actionExit_triggered()	{
	QApplication::quit();
}

void MainWindow::on_actionHelp_triggered()	{
	QString helpStr = QString::fromStdString(std::string("Usage:\n") +
											 "1. Select stereo camera calibration parameters (.xml)\n" +
											 "2. Select stereo block matching algorithm parameters (.json)\n" +
											 "3. Select left image (.bmp, .jpeg, .png)\n" +
											 "4. Select right image (.bmp, .jpeg, .png)\n" +
											 "5. Select image resolution\n" +
											 "6. Click \"Generate\" to generate point cloud\n" +
											 "7/8. Optional: select exported path and click export to export generated point cloud"
											);
	QMessageBox::information(this, "Help", helpStr);
}

void MainWindow::on_actionAbout_triggered()	{
	QString aboutStr = QString::fromStdString(std::string("Program Title: Stereo Point Cloud Generator\n") + 
											  "Developer: Hiu Hong Yu\n" +
											  "Developer Email: hiuyu@ucdavis.edu\n" +
											  "Organization: Advanced Highway Maintenance and Construction Research Center (AHMCT), University of California, Davis\n" +
											  "Copy Right 2017"
											 );
	QMessageBox::information(this, "About", aboutStr);
}


/*
	Class Methods
*/
void MainWindow::generatePointCloud(std::string calibrationPath, std::string bmPath, std::string leftIMPath,
									std::string rightIMPath, IMAGE_RESOLUTION imageResolution)	{
	/* 
		Generate point cloud
	*/
	// Clear the point cloud container
	pcc->getPCContainer3D(0)->clear();

	// Parse block matching parameters
	if(!scsgbmp->readParams(imageResolution))	{
		QMessageBox::critical(this, "Error!", "Error: Unable to parse stereo camera block matching parameters");
		return;
	}

	// Load image into memory
	cv::Mat image1 = loadMatImage(leftIMPath, CV_LOAD_IMAGE_COLOR);
	cv::Mat image2 = loadMatImage(rightIMPath, CV_LOAD_IMAGE_COLOR);
	// Make sure both image are same size and resolution
	if(!isEqualResolution(image1, image2))	{
		QMessageBox::critical(this, "Error!", "Error: Image size are not equal");
		return;
	}

	// Init Point Cloud instance
	if(!scpc->init(image1.size(), scsgbmp))	{
		QMessageBox::critical(this, "Error!", "Error: Unable to init StereoPointCloudGenerator\nPlease restart the program or contract developer");
		return;
	}

	QMessageBox::information(this, "Message", 
						  QString::fromStdString(std::string("Generating point cloud with the following setting:\n") +
						  						 "BM Algorithm: SGBM\n"
						  						));
	// Generate 3D point cloud in world coordinate
	scpc->generateWorldPCLPointCloud(image1, image2, ui->minSpinBox->value(), ui->maxSpinBox->value(), CV_16U,
									 pcc->getPCContainer3D(0));

	QMessageBox::information(this, "Message", 
						  QString::fromStdString("Generated: " + formatNumber<int>(pcc->getPCContainer3D(0)->size()) + " points"));
}

void MainWindow::exportPoints(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc)	{
	exportPointCloud<pcl::PointXYZRGB>(path, pc);
	QMessageBox::information(this, "Message", 
							 QString::fromStdString("Exported: " + formatNumber<int>(pc->size()) + " Points"));
}
