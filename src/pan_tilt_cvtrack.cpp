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
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/MotorState.h>

#define PI 				3.1415927

#define FRAME_HEIGHT	480
#define FRAME_WIDTH		640

#define CALIBRATION_FILE_PATH "../camera.yaml"

using namespace cv;
using namespace std;

double currentPanPosRad = 0.0;
double currentTiltPosRad = 0.0;

Mat colorFilter(const Mat* src, Scalar inColorMinHSV, Scalar inColorMaxHSV);
vector< vector<Point> > findObjects(Mat* inFrame, Scalar inColorMinHSV, Scalar inColorMaxHSV, Point2f* outLargestMatchCenter, float* outLargestMatchArea);
string intToString(int number);
string floatToString(float number);
double digiPos2Deg(int inPos);

bool IsTracking = false;

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

void motorStatesCallback(const dynamixel_msgs::MotorStateList& inMsg);

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

	float theCM[] = {1107.46545955351, 0, 321.746543160967, 0, 1109.96438787448, 291.303399490763, 0, 0, 1};
	float theDC[] = {-0.0974330899674774, 0.479179593078898, 0.0207048507870403, 0.00531414762112178, 0};
	float thePC[] = {1102.72436035156, 0, 322.944612208022, 0, 0, 1098.22875, 295.148907870075, 0, 0, 0, 1, 0};

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
	ros::Subscriber theMotorStatesSubscriber = theNodeHandle.subscribe("/motor_states/pan_tilt_port", 10, motorStatesCallback);

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


	//createButton("Track",&onChkTrackChanged,NULL,CV_CHECKBOX,0); // Qt NO soportado por la instalación de OpenCV de ROS

	setTrackbarPos("H min", "Video", 213);
	setTrackbarPos("H max", "Video", 322);
	setTrackbarPos("S min", "Video", 12);
	setTrackbarPos("S max", "Video", 98);
	setTrackbarPos("V min", "Video", 17);
	setTrackbarPos("V max", "Video", 100);

	float theLoopRate = 30;

	//int theRefCenterX = (int)cx;
	int theRefCenterX = FRAME_WIDTH/2;
	//int theRefCenterY = (int)cy;
	int theRefCenterY = FRAME_HEIGHT/2;

	ros::Rate theRosRate(theLoopRate); // 30 hz

	while(theNodeHandle.ok()) {

		// Obtenemos una imagen

		theCapture >> theFrame;

		// Corregimos la imagen a partir de los parámetros de calibración conocidos

		//undistort(theFrame2, theFrame, theCameraMatrix, theDistCoeffs);
		//theFrame = theFrame2.clone();

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
			line(theFrame,Point(theRefCenterX-10, theRefCenterY), Point(theRefCenterX+10, theRefCenterY), Scalar(0,0,0));
			line(theFrame,Point(theRefCenterX, theRefCenterY-10), Point(theRefCenterX, theRefCenterY+10), Scalar(0,0,0));
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

			double thePanCorrection = -atan2(theCentro.x-theRefCenterX,fx);

			thePanMsg.data = currentPanPosRad + thePanCorrection;
			//thePanMsg.data = -atan2(theCentro.x-FRAME_WIDTH/2,fx);
			putText(theFrame,"PAN: "+intToString(round(thePanCorrection*180/PI)),Point(FRAME_WIDTH/2 - 20, 10),1,1,Scalar(0,255,0),2);

			// Ajuste del ángulo TILT
			// a partir de la coordenada Y y los parámetros cy y fx: atan2(y-cy,fy)

			double theTiltCorrection = atan2(theCentro.y-theRefCenterY,fy);

			theTiltMsg.data = currentTiltPosRad + theTiltCorrection;

			//theTiltMsg.data = atan2(theCentro.y-FRAME_HEIGHT/2,fy);
			putText(theFrame,"TILT: "+intToString(round(theTiltCorrection*180/PI)),Point(10,FRAME_HEIGHT/2 - 20),1,1,Scalar(0,255,0),2);

			// Indicar si estamos en modo tracking

			if (IsTracking) putText(theFrame,"TRACKING",Point(FRAME_WIDTH - 100, FRAME_HEIGHT - 10),1,1,Scalar(0,255,0),2);

			// Limitar las acciones de control para no saturar el bus de motores

			if (theFrameCounter >= 5) {

				theFrameCounter = 0;

				if ((IsTracking)&&(abs(theRefCenterX - (int)theCentro.x) > FRAME_WIDTH/20)) {
					ROS_INFO_STREAM("Comando PAN: " << thePanMsg.data << " rad / " << round(thePanMsg.data*180/PI) << " grados aprox.");
					thePanPublisher.publish(thePanMsg);
					usleep(200000);
				}

				if ((IsTracking)&&(abs(theRefCenterY - (int)theCentro.y) > FRAME_HEIGHT/20)) {
					ROS_INFO_STREAM("Comando TILT: " << theTiltMsg.data << " rad / " << round(theTiltMsg.data*180/PI) << " grados aprox.");
					theTiltPublisher.publish(theTiltMsg);
				}

			}

		}

		// Mostramos por pantalla la imagen modificada con la información de detección

		imshow("Video", theFrame);

		// Pausa hasta que el usuario pulse tecla o transcurran 20 milisegundos
		// Solo importa la letra 't' para hacer un toggle del modo tracking
		if (waitKey(1000/theLoopRate) == 116) {
			IsTracking = (IsTracking)?false:true;
		}

		theRosRate.sleep();

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

double digiPos2Deg(int inPos) {

	return -150.0 + (float)inPos*300.0/1023.0;

}

void motorStatesCallback(const dynamixel_msgs::MotorStateList& inMsg) {

	static int i = 0;

	// Descartamos 9 de cada 10 mensajes para no ralentizar el proceso
	if (i >= 10) {

		i = 0;

		for(int i=0;i<2;i++) {
			dynamixel_msgs::MotorState theMotorState = inMsg.motor_states[i];
			float thePosDeg = digiPos2Deg(theMotorState.position);
			float thePosRad = thePosDeg*PI/180.0;
			float theGoalDeg = digiPos2Deg(theMotorState.goal);
			float theGoalRad = theGoalDeg*PI/180.0;
			int isMoving = theMotorState.moving;
			string theMotorName = (i==0)?"PAN":"TILT";

			if (i==0) {
				currentPanPosRad = thePosRad;
			} else {
				currentTiltPosRad = thePosRad;
			}

			ROS_INFO_STREAM("Motor " << theMotorName << ". Pos actual: " << thePosRad << " rad (" << thePosDeg << " grados). Goal: " << theGoalRad << " rad ( " << theGoalDeg << " grados). " << ((isMoving == 0)?"Parado":"En movimiento"));
		}

	}

	i++;

}

