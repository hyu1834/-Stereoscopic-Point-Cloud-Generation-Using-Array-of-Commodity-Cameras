#ifndef GLWIDGET_H
#define GLWIDGET_H
/*
 *  Standard Libraries
*/
#include <cstdlib>
#include <vector>

/*
 * 3rd Party Libraries
*/
//QT
#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

//OpenGL
#include <GL/freeglut.h>
#include <GL/glut.h>

//Eigen
#include <Eigen/Dense>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

/*
 * Local Libraries
*/
#include "point_cloud_utils.h"
#include "point_cloud_container.h"

/*
	MACRO
*/
#define CROSS_MARK_SIZE 0.02
#define TRANSLATE_SPEED_2D 2
#define TRANSLATE_SPEED_3D 2
#define ZOOM_SPEED 1.1
#define ENABLE true
#define DISABLE false
#define MIN_POINT_SIZE 1.0
#define MAX_POINT_SIZE 4.0

/*
	Struct
*/
struct Coordinate2D	{
	double x;
	double y;
};

struct Coordinate3D	{
	double x;
	double y;
	double z;
};

struct Color	{
	unsigned r;
	unsigned g;
	unsigned b;
};


class GLWidget : public QOpenGLWidget
{
	Q_OBJECT
	private:
		// Variables for View Mode
		bool enable3DView;

		// Widget Variables
		int widgetWidth;
		int widgetHeight;
		int widgetCenterX;
		int widgetCenterY;

		//Point Cloud Variables
		float pointSize;
		PointCloudContainer* pointClouds;


		// View variables
		Coordinate3D zoomView1;
		Coordinate3D translateView1;
		Coordinate3D positionView1;
		Coordinate3D centerView1;
		Coordinate3D upView1;

		// Variables for keeping track
		bool viewDrag1;
		int prevDragX;
		int prevDragY;
		GLdouble* modelViewMatrix1;

	public:
		explicit GLWidget(QWidget *parent = 0);

		/*
			Overloading Class Methods
		*/
		void initializeGL();
		void paintGL();
		void resizeGL(int w, int h);
		void mouseMoveEvent(QMouseEvent *event);
		void mousePressEvent(QMouseEvent *event);
		void mouseReleaseEvent(QMouseEvent *event);
		void wheelEvent(QWheelEvent * event);
		void keyPressEvent(QKeyEvent* event);
		
		/*
			2D Class Methods
		*/
		void drawCrossMark2D(double size, double viewX, double viewY, double width, double height,
							 double clipLeft, double clipRight, double clipBottom, double clipTop);
		void drawPointCloud2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, 
							  double viewX, double viewY, double width, double height,
							  double clipLeft, double clipRight, double clipBottom, double clipTop,
							  Coordinate3D zoom, Coordinate3D translate, GLdouble* modelViewMatrix);

		/*
			3D Class Methods
		*/
		void drawCrossMark3D(double size, double viewX, double viewY, double width, double height,
							 Coordinate3D positionView, Coordinate3D centerView, Coordinate3D upView);
		void drawPointCloud3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, 
							  double viewX, double viewY, double width, double height,
							  Coordinate3D positionView, Coordinate3D centerView, Coordinate3D upView,
							  Coordinate3D zoom, Coordinate3D translate);

		/*
			Enabler/Disabler Class Methods
		*/
		void enable3DViewer(bool mode);

		/*
			Class Methods
		*/
};

#endif // GLWIDGET_H
