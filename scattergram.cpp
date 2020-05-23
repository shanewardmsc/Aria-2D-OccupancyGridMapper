//#include <iostream>
//#include <stdlib.h>
//#include <math.h>
//#include <Aria.h>
//#include <Eigen/Dense>
//
//#include "scattergram.h"
//
//using namespace Eigen;
//// Implementation
//
//// Constructor
//scattergram::scattergram() : ArAction("scattergram")
//{
//	ArLog::init(ArLog::File, ArLog::Normal, "sg.xls"); // Makes ArLog write to sg.xls
//}
//
//// Body of action
//ArActionDesired * scattergram::fire(ArActionDesired d)
//{
//	desiredState.reset(); // reset the desired state (must be done)
//
//	// Get ARIA odometer readings 
//	double robotX = myRobot->getX();
//	double robotY = myRobot->getY();
//	double robotTh = myRobot->getTh(); // (in degrees)
//
//	double leftAngle; // (in degrees)
//	double leftDistance = myRobot->checkRangeDevicesCurrentPolar(0.0, 90.0, &leftAngle);
//	double rightAngle; // (in degrees)
//	double rightDistance = myRobot->checkRangeDevicesCurrentPolar(-90.0, 0.0, &rightAngle);
//
//	// Left Sensor //
//	double convertToRad = M_PI / 180;
//	double Theta_R = robotTh * convertToRad;
//	double Theta_S_left = leftAngle * convertToRad;
//
//	double X_S_left = cos(Theta_S_left)*(leftDistance);
//	double Y_S_left = sin(Theta_S_left)*(leftDistance);
//
//	Matrix<float, 2, 1> orig_Coordinate_left;
//	orig_Coordinate_left << X_S_left, Y_S_left;
//
//	Matrix<float, 2, 2> rotate_Matrix;
//	rotate_Matrix << cos(Theta_R), -sin(Theta_R),
//		sin(Theta_R), cos(Theta_R);
//
//	MatrixXf rotated_Coordinates_left = rotate_Matrix * orig_Coordinate_left;
//
//	double translate_X_left = rotated_Coordinates_left(0) + robotX;
//	double translate_Y_left = rotated_Coordinates_left(1) + robotY;
//
//	// Right Sensor //
//	double Theta_S_right = rightAngle * convertToRad;
//
//	double X_S_right = cos(Theta_S_right)*(rightDistance);
//	double Y_S_right = sin(Theta_S_right)*(rightDistance);
//
//	Matrix<float, 2, 1> orig_Coordinate_right;
//	orig_Coordinate_right << X_S_right, Y_S_right;
//
//	MatrixXf rotated_Coordinates_right = rotate_Matrix * orig_Coordinate_right;
//
//	double translate_X_right = rotated_Coordinates_right(0) + robotX;
//	double translate_Y_right = rotated_Coordinates_right(1) + robotY;
//
//	printf("Left Translated X: %f... Left Translated Y: %f... Right Translated X: %f... Right Translated Y: %f\n", translate_X_left, translate_Y_left, translate_X_right, translate_Y_right);
//
//	return &desiredState; // give the desired state to the robot for actioning
//}