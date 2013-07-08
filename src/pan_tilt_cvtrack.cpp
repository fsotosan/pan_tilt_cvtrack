#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

using namespace cv;
using namespace std;

char thePicPath[] = "./pic.jpg";

Mat redBallFilter(const Mat& src) {

	Mat redBallOnly;

	// Comprobamos que el formato de imagen es el esperado

	assert(src.type() == CV_8UC3);

	// Eliminamos aquellos pixeles que no tengan un valor de rojo suficiente
	// y aquellos que tengan demasiada componente verde o azul

	inRange(src, Scalar(0, 0, 120), Scalar(100, 100, 255), redBallOnly);

	// Definimos kernel para operación de dilatación

	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(10, 10));

	// Invertir
	//bitwise_not(redBallOnly, redBallOnly);

	// Erosión

	erode(redBallOnly, redBallOnly, Mat());

	// Dilatación

	dilate(redBallOnly, redBallOnly, element);

	return redBallOnly;

}

int main( int argc, char** argv ) {

	// Leemos una imagen

	Mat input = imread(thePicPath);

	// La mostramos por pantalla

	imshow("input",input);

	// Esperamos pulsación de tecla

	waitKey();

	// Aplicamos filtro de búsqueda de bolas rojas

	Mat redOnly = redBallFilter(input);

	// Mostramos resultado filtrado

	imshow("redOnly", redOnly);

	// Esperamos pulsación de tecla

	waitKey();

	return 0;

}
