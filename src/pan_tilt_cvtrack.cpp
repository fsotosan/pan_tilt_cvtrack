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

#define MIN_AREA 1000

using namespace cv;
using namespace std;

Mat coloredBallFilter(const Mat* src, Scalar inColorMinHSV, Scalar inColorMaxHSV);
Mat trackBall(Mat* input, Scalar inColorMinHSV, Scalar inColorMaxHSV, Point2f* outCentro, float* outArea);


int main( int argc, char** argv ) {

	VideoCapture theCapture(1);
	Mat theFrame;
	Mat theFilteredFrame;
	Point2f theCentro;
	float theArea = 0.0;

	// Rangos de representación HSV en OpenCV: 0<=H<=180, 0<=S<=255, 0<=V<=255
	// http://www.colorpicker.com/

	/*
	Scalar HSV_NARANJA_MIN(32*180/360,60*255/100,75*255/100);
	Scalar HSV_NARANJA_MAX(37*180/360,255,255);
	*/

	Scalar HSV_NARANJA_MIN(30*180/360,30*255/100,25*255/100);
	Scalar HSV_NARANJA_MAX(44*180/360,90*255/100,90*255/100);

	if(!theCapture.isOpened()) {
		cout << "Error abriendo captura de imagen" << endl;
		return -1;
	}

	namedWindow("Video",CV_WINDOW_AUTOSIZE);
	namedWindow("Ball",CV_WINDOW_AUTOSIZE);

	while(1) {

		theCapture >> theFrame;

		theFilteredFrame = trackBall(&theFrame, HSV_NARANJA_MIN, HSV_NARANJA_MAX,&theCentro,&theArea);

		cout << "X: " << theCentro.x << ", Y: " << theCentro.y << ", area: " << theArea << endl;

		imshow("Video", theFrame);
		imshow("Ball", theFilteredFrame);

		waitKey();

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

	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(30, 30));

	// Invertir
	//bitwise_not(redBallOnly, redBallOnly);

	// Erosión

	erode(redBallOnly, redBallOnly, Mat());

	// Dilatación

	dilate(redBallOnly, redBallOnly, element);

	return redBallOnly;

}

Mat trackBall(Mat* input, Scalar inColorMinHSV, Scalar inColorMaxHSV, Point2f* outCentro, float* outArea) {


	// Aplicamos filtro de búsqueda de bolas rojas

	Mat redOnly = coloredBallFilter(input, inColorMinHSV, inColorMaxHSV);

	// Obtenemos el contorno

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(redOnly, contours, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	// Comprobamos el número de contornos detectados.
	// En caso de haber más de uno nos quedamos con el de mayor área

	CvMoments m;

	int contourNum = 0;
	*outArea = 0.0;

	for( int i = 0; i < contours.size(); i++ ) {
		float tmpArea = contourArea(contours[i]);
		if (tmpArea > *outArea) {
			contourNum = i;
			*outArea = tmpArea;
		}
	}

	// Obtenemos el centro de masa a partir del vector de momentos

	if (*outArea > MIN_AREA) {
		m = moments(contours[contourNum], false);
		*outCentro = Point2f(m.m10/m.m00,m.m01/m.m00);
	} else {
		*outCentro = Point2f(-1,-1);
	}

	return redOnly;

}
