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
#include <std_msgs/Float64.h>

#define PI 				3.1415927

#define FRAME_HEIGHT	480
#define FRAME_WIDTH		640

#define CALIBRATION_FILE_PATH "../camera.yaml"

using namespace cv;
using namespace std;

Mat colorFilter(const Mat* src, Scalar inColorMinHSV, Scalar inColorMaxHSV);
vector< vector<Point> > findObjects(Mat* inFrame, Scalar inColorMinHSV, Scalar inColorMaxHSV, Point2f* outLargestMatchCenter, float* outLargestMatchArea);
string intToString(int number);
string floatToString(float number);

Scalar HsvMin;
Scalar HsvMax;

std_msgs::Float64 thePanMsg, theTiltMsg;

// Funciones callback para ajuste de límites por pantalla

void onHsvHMin(int theSliderValue,void*) { HsvMin[0] = theSliderValue*180/360; }
void onHsvHMax(int theSliderValue,void*) { HsvMax[0] = theSliderValue*180/360; }
void onHsvSMin(int theSliderValue,void*) { HsvMin[1] = theSliderValue*255/100; }
void onHsvSMax(int theSliderValue,void*) { HsvMax[1] = theSliderValue*255/100; }
void onHsvVMin(int theSliderValue,void*) { HsvMin[2] = theSliderValue*255/100; }
void onHsvVMax(int theSliderValue,void*) { HsvMax[2] = theSliderValue*255/100; }

int theFrameCounter = 0;

int main( int argc, char** argv ) {

	VideoCapture theCapture(1);
	Mat theFrame, theFrame2;
	vector< vector<Point> > theBallContours;
	Point2f theCentro;
	float theArea = 0.0;

	// Leemos los parámetros intrínsecos de la cámara
	// y los coeficientes de distorsión del archivo de calibración

	/*
	// Problema con el formato generado por ROS.
	// No coincide con el esperado por OpenCV a través de FileStorage

	Mat theCameraMatrix, theDistCoeffs;
	FileStorage theCalibrationFile(CALIBRATION_FILE_PATH, FileStorage::READ);
	theCalibrationFile["cameraMatrix"] >> theCameraMatrix;
	theCalibrationFile["distCoeffs"] >> theDistCoeffs;
	theCalibrationFile.release();
	*/

	float theCM[] = {1247.1132043625, 0, 268.031780862028, 0, 1245.77083050807, 197.854525292817, 0, 0, 1};
	float theDC[] = {0.156027008694038, 0.414861176927745, -0.00461630436049559, -0.012986972246621, 0};
	float thePC[] = {1268.92993164062, 0, 265.47622626249, 0, 0, 1268.49304199219, 197.223631488687, 0, 0, 0, 1, 0};

	Mat theCameraMatrix =  Mat(3, 3, CV_32F, theCM).clone();
	Mat theDistCoeffs = Mat(1, 5, CV_32F, theDC).clone();
	Mat theProjectionMatrix = Mat(3, 4, CV_32F, thePC).clone();

	float fx = theCameraMatrix.at<float>(0,0);
	float fy = theCameraMatrix.at<float>(1,1);
	float cx = theCameraMatrix.at<float>(0,2);
	float cy = theCameraMatrix.at<float>(1,2);

	cout << "Parámetros intrínsecos: c=(" << cx << "," << cy << "), fx=" << fx << ", fy=" << fy << endl;

	Scalar theTargetIndicatorColor(255,0,00); // BGR Azul

	// Rangos de representación HSV en OpenCV: 0<=H<=180, 0<=S<=255, 0<=V<=255

	HsvMin[0] = 4;		// H min
	HsvMin[1] = 34;		// S min
	HsvMin[2] = 169;	// V min

	HsvMax[0] = 18;		// H min
	HsvMax[1] = 218;	// S min
	HsvMax[2] = 255;	// V min

	if(!theCapture.isOpened()) {
		cout << "Error abriendo captura de imagen" << endl;
		return -1;
	}

	ros::init(argc, argv, "pan_tilt_ptu_46");
	ros::NodeHandle theNodeHandle;
	ros::Publisher theTiltPublisher = theNodeHandle.advertise<std_msgs::Float64>("/tilt_joint/command",10);
	ros::Publisher thePanPublisher = theNodeHandle.advertise<std_msgs::Float64>("/pan_joint/command",10);

	thePanMsg.data = 0.0;
	thePanPublisher.publish(thePanMsg);

	theTiltMsg.data = 0.0;
	theTiltPublisher.publish(theTiltMsg);

	usleep(500000);

	namedWindow("Video",CV_WINDOW_AUTOSIZE);

	createTrackbar("H min", "Video", NULL, 360, &onHsvHMin);
	createTrackbar("H max", "Video", NULL, 360, &onHsvHMax);
	createTrackbar("S min", "Video", NULL, 100, &onHsvSMin);
	createTrackbar("S max", "Video", NULL, 100, &onHsvSMax);
	createTrackbar("V min", "Video", NULL, 100, &onHsvVMin);
	createTrackbar("V max", "Video", NULL, 100, &onHsvVMax);

	setTrackbarPos("H min", "Video", 213);
	setTrackbarPos("H max", "Video", 322);
	setTrackbarPos("S min", "Video", 12);
	setTrackbarPos("S max", "Video", 98);
	setTrackbarPos("V min", "Video", 17);
	setTrackbarPos("V max", "Video", 100);

	ros::Rate r(10); // 10 hz

	while(theNodeHandle.ok()) {

		// Obtenemos una imagen

		theCapture >> theFrame2;

		// Corregimos la imagen a partir de los parámetros de calibración conocidos

		//undistort(theFrame2, theFrame, theCameraMatrix, theDistCoeffs);
		theFrame = theFrame2.clone();

		// Ejecutamos función de detección.
		// La función devolverá un vector de contornos con todos los candidatos
		// Las variables theCentro y theArea contienen las coordenadas (en pixeles) del candidato con mayor área


		theBallContours = findObjects(&theFrame, HsvMin, HsvMax,&theCentro,&theArea);

		if (theArea > 0) {

			// Si la detección ha sido satisfactoria
			// marcamos el objetivo con un círculo y una cruz en el centroide

			//circle(theFrame,theCentro,sqrt(theArea/PI),theTargetIndicatorColor);
			circle(theFrame,theCentro,20,theTargetIndicatorColor);
			line(theFrame,Point(theCentro.x-10, theCentro.y), Point(theCentro.x+10, theCentro.y), theTargetIndicatorColor);
			line(theFrame,Point(theCentro.x, theCentro.y-10), Point(theCentro.x, theCentro.y+10), theTargetIndicatorColor);

			// Señalar el centro cx,cy con una cruz
			line(theFrame,Point(cx-10, cy), Point(cx+10, cy), Scalar(0,0,0));
			line(theFrame,Point(cx, cy-10), Point(cx, cy+10), Scalar(0,0,0));
			//line(theFrame,Point(FRAME_WIDTH/2-10, FRAME_HEIGHT/2), Point(FRAME_WIDTH/2+10, FRAME_HEIGHT/2), Scalar(0,0,0));
			//line(theFrame,Point(FRAME_WIDTH/2, FRAME_HEIGHT/2-10), Point(FRAME_WIDTH/2, FRAME_HEIGHT/2+10), Scalar(0,0,0));

			// Añadimos texto con las coordenadas

			putText(theFrame,intToString(theCentro.x)+","+intToString(theCentro.y),Point(theCentro.x,theCentro.y+30),1,1,Scalar(0,255,0),2);

			// Representamos el contorno de todos los candidatos

			drawContours(theFrame,theBallContours,-1,theTargetIndicatorColor);

			//cout << "X: " << theCentro.x << ", Y: " << theCentro.y << ", area: " << theArea << endl;

			// Tratar de entender cómo obtener la corrección en radianes
			// http://stackoverflow.com/questions/13957150/opencv-computing-camera-position-rotation


			// Ajuste de ángulos pan y tilt:
			// El centro de la imagen (en pixels) viene dado por los parámetros intrínsecos cx y cy
			// (en caso de no disponer de información de calibración tomaríamos FRAME_WIDTH/2 y FRAME_HEIGHT/2)
			// La distancia focal (en pixels) viene dado por los parámetros de calibración fx y fy

			// Ajuste del ángulo PAN
			// a partir de la coordenada X y los parámetros cx y fx: atan2(x-cx,fx)

			// Ajuste del ángulo TILT
			// a partir de la coordenada Y y los parámetros cy y fx: atan2(y-cy,fy)

			// Limitar a una acción de control cada 10 frames para no saturar el bus de motores
			if (theFrameCounter >= 10) {

				theFrameCounter = 0;

				if (abs((int)cx - (int)theCentro.x) > 25) {
					thePanMsg.data = -atan2(theCentro.x-cx,fx);
					//thePanMsg.data = -atan2(theCentro.x-FRAME_WIDTH/2,fx);
					//thePanMsg.data = - 0.9*atan2(theCentro.x-FRAME_WIDTH/2,fx);
					//cout << "Publishing Pan " << thePanMsg.data << endl;
					putText(theFrame,"PAN: "+floatToString(thePanMsg.data),Point(FRAME_WIDTH/2 - 20, 10),1,1,Scalar(0,255,0),2);
					thePanPublisher.publish(thePanMsg);
					usleep(200000);

				}


				if (abs((int)cy - (int)theCentro.y) > 25) {
					theTiltMsg.data = atan2(theCentro.y-cy,fy);
					//theTiltMsg.data = atan2(theCentro.y-FRAME_HEIGHT/2,fy);
					//theTiltMsg.data = 0.9*atan2(theCentro.y-FRAME_HEIGHT/2,fy);
					//cout << "Publishing Tilt " << thePanMsg.data << endl;
					putText(theFrame,"TILT: "+floatToString(thePanMsg.data),Point(10,FRAME_HEIGHT/2 - 20),1,1,Scalar(0,255,0),2);
					theTiltPublisher.publish(theTiltMsg);
					//usleep(100000);

				}

			}

			//usleep(100000);

		}

		// Mostramos por pantalla la imagen modificada con la información de detección

		imshow("Video", theFrame);

		// Pausa hasta que el usuario pulse tecla

		waitKey(50);

		ros::spinOnce();

		theFrameCounter++;

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

	for(int i=0;i<2;i++)
		erode(outThresholdedFrame, outThresholdedFrame, theErodeElement);

	// Dilatación
	// Reforzamos las detecciones que han sobrevivido a la erosión

	for(int i=0;i<2;i++)
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

	//if ((theNumObjects > 0)&&(theNumObjects < 15)) {
	if ((theNumObjects > 0)) {

		// Iteramos sobre los objetos detectados

		for (int theIndex = 0; theIndex >= 0; theIndex = theHierarchy[theIndex][0]) {
			float theTmpArea = contourArea(outContours[theIndex]);
			if (theTmpArea > theMaxArea) {
				theTargetContourNum = theIndex;
				theMaxArea = theTmpArea;
			}
		}
	}

	// Si se ha seleccionado un objetivo con un area aceptable (area de la imagen entre 64)
	// obtenemos su centro de masa a partir del vector de momentos
	// Si no, devolvemos area 0 y punto -1,-1 para indicar que no consideramos detección

	if (theMaxArea > FRAME_HEIGHT*FRAME_WIDTH/128) {
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

string floatToString(float number) {

	std::stringstream ss;
	ss << number;
	return ss.str();

}

