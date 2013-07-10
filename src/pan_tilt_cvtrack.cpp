/*
 * Programa para seguir una bola de color
 * Gran parte del código fuente ha sido extraido de
 * // http://opencv-srf.blogspot.com.es/2010/09/object-detection-using-color-seperation.html
 */

#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

#define MIN_AREA 400

using namespace cv;
using namespace std;

RNG rng(12345);
int lastX = -1;
int lastY = -1;

Mat coloredBallFilter(const Mat* src, Scalar inColorMinHSV, Scalar inColorMaxHSV);
void trackBall(Mat* input, Scalar inColorMinHSV, Scalar inColorMaxHSV);


int main( int argc, char** argv ) {

	VideoCapture theCapture(1);
	Mat theFrame;

	// Rangos de representación HSV en OpenCV: 0<=H<=180, 0<=S<=255, 0<=V<=255
	// http://www.colorpicker.com/

	Scalar HSV_NARANJA_MIN(32*180/360,60*255/100,75*255/100);
	Scalar HSV_NARANJA_MAX(37*180/360,255,255);

	if(!theCapture.isOpened()) {
		cout << "Error abriendo captura de imagen" << endl;
		return -1;
	}

	while(1) {

		theCapture >> theFrame;

		trackBall(&theFrame, HSV_NARANJA_MIN, HSV_NARANJA_MAX);

		cout << "X: " << lastX << ", Y: " << lastY << endl;

	}

	return 0;

}



Mat coloredBallFilter(const Mat* src, Scalar inColorMinHSV, Scalar inColorMaxHSV) {

	Mat hsv, redBallOnly;

	// Comprobamos que el formato de imagen es el esperado

	assert(src->type() == CV_8UC3);

	// Obtenemos la representación HSV de la imagen BGR (ojo, BGR y no RGB!!)

	cvtColor(*src,hsv,CV_BGR2HSV);

	// Filtrado en HSV

	inRange(hsv, inColorMinHSV, inColorMaxHSV, redBallOnly);

	// Definimos kernel para operación de dilatación

	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(25, 25));

	// Invertir
	//bitwise_not(redBallOnly, redBallOnly);

	// Erosión

	erode(redBallOnly, redBallOnly, Mat());

	// Dilatación

	dilate(redBallOnly, redBallOnly, element);

	return redBallOnly;

}

void trackBall(Mat* input, Scalar inColorMinHSV, Scalar inColorMaxHSV) {


	// Aplicamos filtro de búsqueda de bolas rojas

	Mat redOnly = coloredBallFilter(input, inColorMinHSV, inColorMaxHSV);

	// Obtenemos el contorno

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(redOnly, contours, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	// Comprobamos el número de contornos detectados.
	// En caso de haber más de uno nos quedamos con el de mayor área

	float area = 0.0;
	Moments mu;
	Point2f mc;
	int contourNum = 0;
	double moment10 = mu(1);
	double moment01 = mu(2);

	for( int i = 0; i < contours.size(); i++ ) {
		float tmpArea = contourArea(contours[i]);
		if (tmpArea > area) {
			contourNum = i;
			area = tmpArea;
		}
	}

	// Obtenemos el centro de masa a partir del vector de momentos

	if (area > MIN_AREA) {
		mu = moments(contours[contourNum], false);
		mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

		lastX = moment10/area;
		lastY = moment01/area;

		/*
		// Representamos el contorno

		Mat drawing = Mat::zeros(redOnly.size(), CV_8UC3 );
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, contourNum, color, 2, 8, hierarchy, 0, Point() );
		circle( drawing, mc, 4, color, -1, 8, 0 );

		namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		imshow( "Contours", drawing );
		*/


	} else {

		lastX = -1;
		lastY = -1;

	}

}
