//#include "Aria.h"
//#include <stdio.h>
//#include "SonarModel.h"
//#include "SFML\Window.hpp"
//#include "SFML\Graphics.hpp"
//#include "SFML\System.hpp"
//#include <math.h>
//#include <iostream>
//#include <Eigen\Dense>
//#include <fstream>
//#include <iostream>
//#include <chrono>
//#include <SFML/Graphics.hpp>
//#include <SFML/OpenGL.hpp>
//
//// Functionality for a sonar model //
//
//using namespace Eigen;
//
//// Define Occupancy Grid //
//double occupancyGrid[100][100]; // memory for Occupancy Grid
//double previousGrid[100][100]; // Previous memory for Occupancy Grid
//double resolution = 50; // Resolution of the occ grid
//int iterationCount = 0;
//int executedCount = 0;
//
//SonarModel::SonarModel() : ArAction("SonarModel")
//{
//	ArLog::init(ArLog::File, ArLog::Normal, "sg.xls"); // Makes ArLog write to sg.xls
//}
//
//// Body of action
//ArActionDesired * SonarModel::fire(ArActionDesired d)
//{
//	desiredState.reset(); // reset the desired state (must be done)
//
//	// Get Robot Readings //  
//	double robotX = myRobot->getX();
//	double robotY = myRobot->getY();
//	double robotTh = myRobot->getTh(); // (in degrees)
//	double robotRad = myRobot->getRobotRadius();
//
//	if(robotX != 0 && robotY != 0 && robotTh != 0)
//	{ 
//		// Get Robot Readings //  
//		double robotX = myRobot->getX();
//		double robotY = myRobot->getY();
//		double robotTh = myRobot->getTh(); // (in degrees)
//		double robotRad = myRobot->getRobotRadius();
//
//		// Get Sensor 1 Angle and Distance //
//		double rightAngle; // (in degrees)
//		double rightDistance = myRobot->checkRangeDevicesCurrentPolar(90.0, 0.0, &rightAngle);
//		
//		// Sensor 1 Characteristics // 
//		double sensorRange = 2000;
//		double sensorX = robotX + robotRad;
//		double sensorY = robotY + robotRad;
//		double sensorTh = rightAngle;
//		double Beta = 10;
//		double Theta = degrees(atan(sensorY / sensorX));
//
//		// Left Sensor - Rotate and Translate to Global Map //
//		double Theta_S_left = radians(rightAngle);
//
//		double X_S_left = cos(Theta_S_left)*(rightDistance);
//		double Y_S_left = sin(Theta_S_left)*(rightDistance);
//
//		Matrix<float, 2, 1> orig_Coordinate_left;
//		orig_Coordinate_left << X_S_left, Y_S_left;
//
//		Matrix<float, 2, 2> rotate_Matrix;
//		rotate_Matrix << cos(robotTh), -sin(robotTh),
//			sin(robotTh), cos(robotTh);
//
//		MatrixXf rotated_Coordinates_left = rotate_Matrix * orig_Coordinate_left;
//
//		double translate_X_left = rotated_Coordinates_left(0) + robotX;
//		double translate_Y_left = rotated_Coordinates_left(1) + robotY;
//
//		// Define Bounding Box - Only update within this box // 
//		double x_1 = robotX + robotRad;
//		double y_1 = robotY + robotRad;
//
//		double x_2 = sensorRange /*Or is this the sensor reading??*/ * cos(sensorTh - Beta) + x_1;
//		double y_2 = sensorRange /*Or is this the sensor reading??*/ * sin(sensorTh - Beta) + y_1;
//
//		double x_4 = sensorRange /*Or is this the sensor reading??*/ * cos(sensorTh + Beta) + x_1;
//		double y_4 = sensorRange /*Or is this the sensor reading??*/ * sin(sensorTh + Beta) + y_1;
//
//		double r2 = sensorRange /*Or is this the sensor reading??*/ / cos(Beta);
//
//		double x_3 = r2 * cos(sensorTh) + x_1;
//		double y_3 = r2 * sin(sensorTh) + y_1;
//
//		double x_lower = min(x_1, x_2, x_3, x_4);
//		double y_lower = min(y_1, y_2, y_3, y_4);
//
//		double x_upper = max(x_1, x_2, x_3, x_4);
//		double y_upper = max(y_1, y_2, y_3, y_4);
//
//		// Set initial probabilities to 0.5 //
//		double P_H_previousTime = 0.5;
//		double P_H_previousTime_minus = 1 - P_H_previousTime;
//
//		if(iterationCount == 0)
//		{ 
//			int m, n;
//		
//			for (m = 0; m < 100; m++)
//			{
//				for (n = 0; n < 100; n++)
//				{
//					previousGrid[m][n] = 0.5f;
//				}
//			}
//		}
//		else if (iterationCount >= 1)
//		{
//			int o, p;
//
//			for (o = 0; o < 100; o++)
//			{
//				for (p = 0; p < 100; p++)
//				{
//					previousGrid[o][p] = occupancyGrid[o][p];
//				}
//			}
//		}
//
//
//		int i, j;
//		
//		for (i = 0; i < 100; i++)
//		{
//			for (j = 0; j < 100; j++)
//			{
//				int i_int = i * resolution - 2500.0;
//				int j_int = j * resolution - 2500.0;
//
//				double P_occ = (probability(i_int, j_int, sensorX, sensorY, sensorTh, rightDistance, sensorRange));
//				double P_1_occ = (1 - probability(i_int, j_int, sensorX, sensorY, sensorTh, rightDistance, sensorRange));
//				double P_previous_occ = previousGrid[i][j];
//				double P_1_previous_occ = (1 - previousGrid[i][j]);
//
//				if (i > x_lower && i < x_upper && j > y_lower && j < y_upper)
//				{
//					// Bayes Recursive Theorem //
//					double Bayes_Recursive_Result = (P_occ * P_previous_occ) / ((P_occ *P_previous_occ) + (P_1_occ *P_1_previous_occ));
//
//					occupancyGrid[i][j] = 0.5 + (0.5 * Bayes_Recursive_Result);
//
//				}
//			}
//		}
//
//	
//
//	if (executedCount > 500)
//	{
//
//		float scale = 0.025;
//		float angle_x = 340.f;
//		float angle_y = 55.f;
//		// Request a 32-bits depth buffer when creating the window
//		sf::ContextSettings contextSettings;
//		contextSettings.depthBits = 32;
//
//		// Create the main window
//		sf::RenderWindow window(sf::VideoMode(800, 600), "Sonar Model Visualisation Program", sf::Style::Default, contextSettings);
//		window.setVerticalSyncEnabled(true);
//		window.setActive();
//
//		// Enable Z-buffer read and write
//		glEnable(GL_DEPTH_TEST);
//		glDepthMask(GL_TRUE);
//		glClearDepth(1.f);
//
//		// Disable lighting
//		glDisable(GL_LIGHTING);
//
//		// Configure the viewport (the same size as the window)
//		glViewport(0, 0, window.getSize().x, window.getSize().y);
//
//		// Setup a perspective projection
//		glMatrixMode(GL_PROJECTION);
//		glLoadIdentity();
//		GLfloat ratio = static_cast<float>(window.getSize().x) / window.getSize().y;
//		glFrustum(-ratio, ratio, -1.f, 1.f, 1.f, 500.f);
//
//		glClearColor(1.f, 1.f, 1.f, 1.f);
//		// Start game loop
//		while (window.isOpen())
//		{
//			// Process events
//			sf::Event event;
//			while (window.pollEvent(event))
//			{
//				// Close window: exit
//				if (event.type == sf::Event::Closed)
//				window.close();
//
//				// Escape key: exit
//				if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Escape))
//					window.close();
//
//				// Adjust the viewport when the window is resized
//				if (event.type == sf::Event::Resized)
//					glViewport(0, 0, event.size.width, event.size.height);
//
//				// Process otehr keypresses
//				if (event.type == sf::Event::KeyPressed)
//				{
//			 
//					// Movement of the model
//					if (event.key.code == sf::Keyboard::Up)
//					{
//						angle_y += 5.0f;
//						if (angle_y > 360.0f) angle_y -= 360.f;
//					}
//					if (event.key.code == sf::Keyboard::Down)
//					{
//						angle_y -= 5.0f;
//						if (angle_y < 0.0f) angle_y += 360.f;
//					}
//					if (event.key.code == sf::Keyboard::Right)
//					{
//						angle_x += 5.0f;
//						if (angle_x > 360.0f) angle_x -= 360.f;
//					}
//					if (event.key.code == sf::Keyboard::Left)
//					{
//						angle_x -= 5.0f;
//						if (angle_x < 0.0f) angle_x += 360.f;
//					}
//					if (event.key.code == sf::Keyboard::Add || (event.key.code == sf::Keyboard::Equal && event.key.shift)) scale += 0.002;
//					if (event.key.code == sf::Keyboard::Subtract || event.key.code == sf::Keyboard::Dash) scale -= 0.002;
//				}
//			}
//
//			// Clear the depth buffer
//			glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
//
//			// We get the position of the mouse cursor, so that we can move the box accordingly
//			float x = sf::Mouse::getPosition(window).x * 200.f / window.getSize().x - 100.f;
//			float y = -sf::Mouse::getPosition(window).y * 200.f / window.getSize().y + 100.f;
//
//			// Apply some transformations
//			glMatrixMode(GL_MODELVIEW);
//			glLoadIdentity();
//			glTranslatef(0, 0, -100.f);
//
//			// rotate and scale
//			glRotatef(angle_x, 0.0, 1.0, 0.0);
//			glRotatef(angle_y, 1.0, 1.0, 0.0);
//			glScalef(scale, 1.0, scale);
//
//			// draw Xaxis 
//			glBegin(GL_LINES);
//			// Red
//			glColor3f(1.0, 0.0, 0.0);
//			glVertex3f(0, 25, 0.0);
//			glVertex3f(2500, 25, 0.0);
//			// Green
//			glColor3f(0.0, 1.0, 0.0);
//			// z-axis (y-axis of the occ grid)
//			glVertex3f(0.0, 25, 0.0);
//			glVertex3f(0.0, 25, 2500.0);
//
//			glEnd();
//
//			// Black
//			glColor3f(0.0, 0.0, 0.0);
//			// Draw occ grid
//			int i, j;
//			for (i = 0; i < 100; i++)
//			{
//				glBegin(GL_LINE_STRIP);
//				for (j = 0; j < 100; j++)
//				{
//					glVertex3f(resolution*i - 2500, occupancyGrid[i][j] * 50.0, resolution*j - 2500);
//				}
//				glEnd();
//			}
//
//			for (i = 0; i < 100; i++)
//			{
//				glBegin(GL_LINE_STRIP);
//				for (j = 0; j < 100; j++)
//				{
//					glVertex3f(resolution*j - 2500, occupancyGrid[j][i] * 50., resolution*i - 2500);
//				}
//				glEnd();
//			}
//
//			// Finally, display the rendered frame on screen
//			window.display();
//
//		}
//	}
//	
//}
//
//executedCount = executedCount + 1;
//iterationCount = iterationCount + 1;
//
//return &desiredState; // give the desired state to the robot for actioning
//
//}
//
//double SonarModel::radians(double angle)
//{
//       return (3.141 / 180.0) * angle;
//}
//
//double SonarModel::degrees(double angle)
//{
//       return (180.0  / 3.141) * angle;
//}
//
//double SonarModel::probability(double x, double y, double sX, double sY, double sTh, double sRead, double sRange)
//{
//	double prob_val;
//	double rightDistance = sRead;
//	double sensorRange = sRange;
//
//	if (rightDistance <= sensorRange)
//	{
//		// Sonar Model // 
//		double m = 0;
//		double e = 0;
//		double grid_X = x;
//		double grid_Y = y;
//		double sensorX = sX;
//		double sensorY = sY;
//		double sensorTh = sTh;
//
//		double local_X = sensorX - grid_X;
//		double local_Y = sensorY - grid_Y;
//		double sensor_Th = radians(sensorTh);
//		double sensor_Reading = rightDistance;
//		double max_angle = 10; //Region I
//		double max_angle_range = sensor_Th + max_angle;
//		double max_occupied = 0.98;
//		
//		double theta = degrees(atan(local_Y / local_X));
//
//		if (local_X < 0)
//		{
//			if (theta <= 0)
//			{
//				theta = theta + 180;
//			}
//			else if (theta > 0)
//			{
//				theta = theta - 180;
//			}
//		}
//
//		if (sensorTh > 180 - max_angle && theta < 0)
//		{
//			theta = theta + 360;
//		}
//		else if (sensorTh < -180 + max_angle && theta > 0)
//		{
//			theta = theta - 360;
//		}
//
//		double tolerance = 100;
//
//		double local_theta = sensor_Th - theta;
//
//		local_theta = fabs(local_theta);
//
//		// Get distance r //
//		double dist_to_cell = sqrt((pow(grid_X - sensorX, 2)) + pow(grid_Y - sensorY, 2));
//
//		prob_val = 0.5;
//
//		if (dist_to_cell < rightDistance + tolerance && local_theta < max_angle) // Region I
//		{
//			m = 0.5 + (dist_to_cell - (rightDistance - 150))* (0.5 / 150);
//			e = 0.5 + (dist_to_cell - (rightDistance - 150))* (0.1 / 150);
//			prob_val = e + (local_theta - max_angle) *((m - e) / -max_angle);
//		}
//		else
//		{
//			prob_val = 0.5;
//		}
//	}
//	else
//	{
//		prob_val = 0.5;
//	}
//
//	return prob_val;
//}
//
////double SonarModel::initialGrid(double grid_previous)
////{
////
////}