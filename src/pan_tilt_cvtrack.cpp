/*
 * Programa para seguir una bola de color.
 * Autor: Fernando Soto.
 * A partir de las siguientes referencias y ejemplos de código:
 *  - http://opencv-srf.blogspot.com.es/2010/09/object-detection-using-color-seperation.html
 *  - Real-Time Object Tracking Using OpenCV: https://www.youtube.com/watch?v=bSeFrPrqZ2A
 */

#include <stdio.h>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

#define FRAME_HEIGHT	480
#define FRAME_WIDTH		640

using namespace cv;
using namespace std;

Mat colorFilter(const Mat* src, Scalar inColorMinHSV, Scalar inColorMaxHSV);
vector< vector<Point> > findObjects(Mat* inFrame, Scalar inColorMinHSV, Scalar inColorMaxHSV, Point2f* outLargestMatchCenter, float* outLargestMatchArea);
string intToString(int number);

int main( int argc, char** argv ) {

	VideoCapture theCapture(1);
	Mat theFrame;
	vector< vector<Point> > theBallContours;
	Point2f theCentro;
	float theArea = 0.0;

	Scalar theTargetIndicatorColor(0,0,255);

	// Rangos de representación HSV en OpenCV: 0<=H<=180, 0<=S<=255, 0<=V<=255
	// http://www.colorpicker.com/

	Scalar HSV_NARANJA_MIN(30*180/360,30*255/100,25*255/100);
	Scalar HSV_NARANJA_MAX(44*180/360,90*255/100,90*255/100);

	if(!theCapture.isOpened()) {
		cout << "Error abriendo captura de imagen" << endl;
		return -1;
	}

	namedWindow("Video",CV_WINDOW_AUTOSIZE);

	while(1) {

		// Obtenemos una imagen

		theCapture >> theFrame;

		// Ejecutamos función de detección.
		// La función devolverá un vector de contornos con todos los candidatos
		// Las variables theCentro y theArea contienen las coordenadas (en pixeles) del candidato con mayor área

		theBallContours = findObjects(&theFrame, HSV_NARANJA_MIN, HSV_NARANJA_MAX,&theCentro,&theArea);

		if (theArea > 0) {

			// Si la detección ha sido satisfactoria
			// marcamos el objetivo con un círculo y una cruz en el centroide

			circle(theFrame,theCentro,20,theTargetIndicatorColor);
			line(theFrame,Point(theCentro.x-10, theCentro.y), Point(theCentro.x+10, theCentro.y), theTargetIndicatorColor);
			line(theFrame,Point(theCentro.x, theCentro.y-10), Point(theCentro.x, theCentro.y+10), theTargetIndicatorColor);

			// Añadimos texto con las coordenadas

			putText(theFrame,intToString(theCentro.x)+","+intToString(theCentro.y),Point(theCentro.x,theCentro.y+30),1,1,Scalar(0,255,0),2);

			// Representamos el contorno de todos los candidatos

			drawContours(theFrame,theBallContours,-1,theTargetIndicatorColor);

			cout << "X: " << theCentro.x << ", Y: " << theCentro.y << ", area: " << theArea << endl;

		}

		// Mostramos por pantalla la imagen modificada con la información de detección

		imshow("Video", theFrame);

		// Pausa hasta que el usuario pulse tecla

		waitKey();

	}

	return 0;

}

Mat colorFilter(const Mat* inFrame, Scalar inColorMinHSV, Scalar inColorMaxHSV) {

	Mat theHsvFrame, outThresholdedFrame;

	// Comprobamos que el formato de imagen es el esperado

	assert(inFrame->type() == CV_8UC3);

	// Obtenemos la representación HSV de la imagen BGR (ojo, BGR y no RGB!!)
	// ya que HSV presenta mejores características para operaciones de filtrado de color

	cvtColor(*inFrame,theHsvFrame,CV_BGR2HSV);

	// Filtrado en HSV

	inRange(theHsvFrame, inColorMinHSV, inColorMaxHSV, outThresholdedFrame);

	// Definimos kernel para operaciones de erosión y dilatación
	// Los elementos rectangulares implican menor carga computacional

	Mat theErodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat theDilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	// Invertir
	//bitwise_not(theThresholdedFrame, theThresholdedFrame);

	// Erosión.
	// Eliminamos el ruido (conicidencias dispersas y aisladas de pequeño tamaño)

	erode(outThresholdedFrame, outThresholdedFrame, theErodeElement);

	// Dilatación
	// Reforzamos las detecciones que han sobrevivido a la erosión

	dilate(outThresholdedFrame, outThresholdedFrame, theDilateElement);

	return outThresholdedFrame;

}

vector< vector<Point> > findObjects(Mat* inFrame, Scalar inColorMinHSV, Scalar inColorMaxHSV, Point2f* outCentro, float* outArea) {

	vector<vector<Point> > outContours;
	vector<Vec4i> theHierarchy;
	vector<Point> theLargestTargetContour;
	CvMoments theMoments;
	int theTargetContourNum = 0;
	double theMaxArea = 0.0;

	// Aplicamos filtro de búsqueda de bolas del co

	Mat theThresholdedImage = colorFilter(inFrame, inColorMinHSV, inColorMaxHSV);

	// Buscamos contornos

	findContours(theThresholdedImage, outContours, theHierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	// Comprobamos el número de contornos detectados.
	// En caso de haber más de uno establecemos el blanco en el de mayor área.
	// Si el número de detecciones es mayor que 5 entendemos que es debido a una detección errónea

	int theNumObjects = theHierarchy.size();

	if ((theNumObjects > 0)&&(theNumObjects < 5)) {

		// Iteramos sobre los objetos detectados

		for (int theIndex = 0; theIndex >= 0; theIndex = theHierarchy[theIndex][0]) {
			float theTmpArea = contourArea(outContours[theIndex]);
			if (theTmpArea > theMaxArea) {
				theTargetContourNum = theIndex;
				theMaxArea = theTmpArea;
			}
		}
	}

	// Si se ha seleccionado un objetivo con un area aceptable (area de la imagen entre 32)
	// obtenemos su centro de masa a partir del vector de momentos
	// Si no, devolvemos area 0 y punto -1,-1 para indicar que no consideramos detección

	if (theMaxArea > FRAME_HEIGHT*FRAME_WIDTH/32) {
		theLargestTargetContour = outContours[theTargetContourNum];
		theMoments = moments(theLargestTargetContour, false);
		*outCentro = Point2f(theMoments.m10/theMoments.m00,theMoments.m01/theMoments.m00);
		*outArea = theMaxArea;
	} else {
		*outCentro = Point2f(-1,-1);
		*outArea = 0.0;
	}

	return outContours;

}

string intToString(int number) {

	std::stringstream ss;
	ss << number;
	return ss.str();

}
