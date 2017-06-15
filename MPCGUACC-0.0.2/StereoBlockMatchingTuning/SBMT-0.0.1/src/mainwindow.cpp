#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)	{
    ui->setupUi(this);
    // Stereo instance
    sccp = new StereoCameraCalibrationParameters(SIDE_BY_SIDE);
    scsgbmp = new StereoCameraSGBMParameters();
    scd = new StereoCameraDisparity(sccp, true);

    QWidget::showMaximized();

    //init viewer size
    ui->imageViewer->setFixedSize(QSize(1280, 720));
}

MainWindow::~MainWindow()	{
    delete scd;
    delete scsgbmp;
    delete sccp;
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
		ui->leftImageBrowseButton->setEnabled(true);
	}
}

void MainWindow::on_leftImageBrowseButton_clicked()	{
	QString filePath = QFileDialog::getOpenFileName(this, tr("Select Image file"), QString(),
														  tr("BMP Files (*.bmp);;JPEG Files (*.jpeg);;PNG Files (*.png);;All Files (*.*)"));
	if(filePath != "")	{
		// load images
		leftImage = loadMatImage(filePath.toUtf8().constData(), CV_LOAD_IMAGE_COLOR);

		// set line edit content
		ui->leftImagePath->setText(filePath);
		ui->rightImageBrowseButton->setEnabled(true);
	}
}

void MainWindow::on_rightImageBrowseButton_clicked()	{
	QString filePath = QFileDialog::getOpenFileName(this, tr("Select Image file"), QString(),
														  tr("BMP Files (*.bmp);;JPEG Files (*.jpeg);;PNG Files (*.png);;All Files (*.*)"));
	if(filePath != "")	{
		// load images
		rightImage = loadMatImage(filePath.toUtf8().constData(), CV_LOAD_IMAGE_COLOR);
		// set line edit content
		ui->rightImagePath->setText(filePath);
		// enable generate button
		ui->exportButton->setEnabled(true);
        ui->settingScrollArea->setEnabled(true);
	}
}

void MainWindow::on_computeButton_clicked()    {
    computeDisparity();
}

void MainWindow::on_exportButton_clicked()	{
	QString filePath = QFileDialog::getSaveFileName(this, tr("Select Image file"), QString(),
														  tr("BMP Files (*.bmp);;JPEG Files (*.jpeg);;PNG Files (*.png);;All Files (*.*)"));
	if(filePath != "")	{
		saveImage(filePath.toUtf8().constData(), disparityMap);
	}
}

void MainWindow::on_image480PRB_clicked()   {
    ui->imageViewer->setFixedSize(QSize(640, 480));
}

void MainWindow::on_image720PRB_clicked()   {
    ui->imageViewer->setFixedSize(QSize(1280, 720));
}

void MainWindow::on_image1080PRB_clicked()  {
    ui->imageViewer->setFixedSize(QSize(1920, 1080));
}

void MainWindow::on_image2KRB_clicked() {
    ui->imageViewer->setFixedSize(QSize(2048, 1200));
}

void MainWindow::on_image4KRB_clicked() {
    ui->imageViewer->setFixedSize(QSize(3840,2160));
}

void MainWindow::on_sgbmRadio_clicked() {
    ui->imageViewerLabel->setText("SGBM");
    ui->bmSettingLabel->setText("SGBM Setting");
}

void MainWindow::on_sbmRadio_clicked()   {
    ui->imageViewerLabel->setText("SBM");
    ui->bmSettingLabel->setText("SBM Setting");
}


/*
	Trigger Signal Methods
*/
void MainWindow::on_actionHelp_triggered()	{
	QString helpStr = QString::fromStdString(std::string("\n") +
											 ""
											);
	QMessageBox::information(this, "Help", helpStr);
}

void MainWindow::on_actionAbout_triggered()	{
	QString aboutStr = QString::fromStdString(std::string("Program Title: Stereo Block Matching Tuning\n") + 
											  "Developer: Hiu Hong Yu\n" +
											  "Developer Email: hiuyu@ucdavis.edu\n" +
											  "Organization: Advanced Highway Maintenance and Construction Research Center (AHMCT), University of California, Davis\n" +
											  "Copy Right 2017"
											 );
	QMessageBox::information(this, "About", aboutStr);
}

void MainWindow::on_actionExit_triggered()	{
	QApplication::quit();
}

void MainWindow::on_actionGenerate_triggered()  {
    on_computeButton_clicked();
}

void MainWindow::on_actionExport_triggered()    {
    on_exportButton_clicked();
}


/*
	Class Methods
*/
void MainWindow::computeDisparity()	{
    // First update the sgbm parameters
    scsgbmp->updateParams(ui->preFilterCapSpinBox->value(), ui->sadWindowSizeSpinBox->value(), ui->minDisparitySpinBox->value(),
                          ui->numOfDisparitySpinBox->value(), ui->uniquenessRatioSpinBox->value(), ui->speckleWindowSizeSpinBox->value(),
                          ui->speckleRangeSpinBox->value(), ui->disp12MaxDiffSpinBox->value(), ui->fullDPSpinBox->value());
    scsgbmp->printParams(std::cout);

    // Check if 2 input images are same resolution
    if(!isEqualResolution(leftImage, rightImage))   {
        QMessageBox::critical(this, "Error!", "Error: Input images are not same");
        return;
    }

    cv::Size imageSize = leftImage.size();
    // Perform one time init process
    if(!scd->disparityInit(imageSize, scsgbmp))	{
        QMessageBox::critical(this, "Error!", "Error: Unable to Init Disparity Map");
        return;
    }

    if(!scd->computeDisparityMap(leftImage, rightImage, imageSize, TRIPLE)) {
        QMessageBox::critical(this, "Error!", "Error: Unable to Compute Disparity Map");
        return;
    }

    disparityMap = scd->getNormDisparityMap(CV_16U);
//    cv::cvtColor(disparityMap, disparityMap, CV_GRAY2RGB);

    QImage qDisparityMap((uchar*)disparityMap.data, disparityMap.cols, disparityMap.rows, disparityMap.step, QImage::Format_Indexed8);
    ui->imageViewer->setPixmap(QPixmap::fromImage(qDisparityMap));

}
