#include "Aria.h"
#include "scattergram.h"
#include "SonarModel.h"
#include "SFML\Window.hpp"
#include "SFML\Graphics.hpp"
#include "SFML\System.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <Eigen\Dense>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <chrono>

/* Required Namespaces */
using namespace std;
using namespace Eigen;							// Eigen Matrix Library

/* Variable Declarations */
const int window_width = 1000;					// SFML Window width
const int window_height = 600;					// SFML Window height
sf::Color sfml_windowBackgroundColor
= sf::Color(127.5, 127.5, 127.5);				// Grey Background colour for SFML Window
double occupancyGrid[100][100];					// Occupancy Grid Memory
double previousGrid[100][100];					// Previous Memory for Occupancy Grid
double resolution = 100;						// Resolution of Occupancy Grid
double probability(double x, double y, 
	double sX, double sY, double sTh, 
	double sRead, double sRange);				// Probability function "Sonar Model" 
double radians(double angle);					// Convert to Radians function
double degrees(double angle);					// Convert to Degrees function
//SonarModel sonarModel;						// External ArAction of the Sonar Model - Not used..

/* Main Function */
int main(int argc, char **argv)
{
	Aria::init();								// Initialize Aria Libraries
	ArArgumentParser argParser(&argc, argv);	
	argParser.loadDefaultArguments();			
	ArRobot robot;
	ArRobotConnector robotConnector(&argParser, &robot);
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

	argParser.addDefaultArgument("-connectLaser");

	if(!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if(argParser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);						// Exit if connection to robot cannot be made
		}
	}

	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);							// Trigger for argument parsing
	}

	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);

	puts("This program will make the robot wander around. It uses some avoidance\n"
	"actions if obstacles are detected, otherwise it just has a\n"
	"constant forward velocity.\n\nPress CTRL-C or Escape to exit.");
  
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	robot.runAsync(true);

	robot.enableMotors();						// Enable robot motors
	robot.comInt(ArCommands::SOUNDTOG, 0);		// Disable sound

	/* Aria Actions */
	ArActionStallRecover recover;
	ArActionBumpers bumpers;
	ArActionAvoidFront avoidFrontNear("Avoid Front Near", 225, 0);
	ArActionAvoidFront avoidFrontFar;
	ArActionConstantVelocity constantVelocity("Constant Velocity",100);
	//SonarModel SonarModel;

	/* Priority for Aria Actions */
	robot.addAction(&recover, 100);
	robot.addAction(&bumpers, 75);
	robot.addAction(&avoidFrontNear, 50);
	robot.addAction(&avoidFrontFar, 49);
	robot.addAction(&constantVelocity, 25);
	//robot.addAction(&SonarModel, 50);
  
	robot.requestEncoderPackets();

	int iterationCount = 0;
	int executedCount = 0;

	/* Mapping Functionality */
	while(true)
	{ 

		/* Get Robot Readings */  
		double robotX = robot.getX();
		double robotY = robot.getY();
		double robotTh = robot.getTh();			// Degrees
		double robotRad = robot.getRobotRadius();

		/* Get Sensor 1 Angle and Distance */
		double rightAngle;						// Degrees
		double rightDistance = robot.checkRangeDevicesCurrentPolar(90.0, 0.0, &rightAngle);

		/* Get Sensor 2 Angle and Distance */
		double leftAngle;						// Degrees
		double leftDistance = robot.checkRangeDevicesCurrentPolar(0.0, 90.0, &leftAngle);

		/* Sensor 1 Characteristics */ 
		double sensorRange = 800;
		double sensorX_R = robotX + robotRad;
		double sensorY_R = robotY + robotRad;
		double sensorTh_R = rightAngle;
		double Beta = 10;
		double Theta_R = degrees(atan(sensorY_R / sensorX_R));

		/* Sensor 2 Characteristics */
		double sensorX_L = robotX + robotRad;
		double sensorY_L = robotY + robotRad;
		double sensorTh_L = leftAngle;
		double Theta_L = degrees(atan(sensorY_L / sensorX_L));

		/* Sensor 1 - Rotate and Translate to Global Map */
		double Theta_S_right = radians(rightAngle);

		double X_S_right = cos(Theta_S_right)*(rightDistance);
		double Y_S_right = sin(Theta_S_right)*(rightDistance);

		Matrix<float, 2, 1> orig_Coordinate_right;
		orig_Coordinate_right << X_S_right, Y_S_right;

		Matrix<float, 2, 2> rotate_Matrix;
		rotate_Matrix << cos(robotTh), -sin(robotTh),
			sin(robotTh), cos(robotTh);

		MatrixXf rotated_Coordinates_right = rotate_Matrix * orig_Coordinate_right;

		double translate_X_right = rotated_Coordinates_right(0) + robotX;
		double translate_Y_right = rotated_Coordinates_right(1) + robotY;

		/* Sensor 2 - Rotate and Translate to Global Map */
		double Theta_S_left = radians(leftAngle);

		double X_S_left = cos(Theta_S_left)*(leftDistance);
		double Y_S_left = sin(Theta_S_left)*(leftDistance);

		Matrix<float, 2, 1> orig_Coordinate_left;
		orig_Coordinate_left << X_S_left, Y_S_left;

		MatrixXf rotated_Coordinates_left = rotate_Matrix * orig_Coordinate_left;

		double translate_X_left = rotated_Coordinates_left(0) + robotX;
		double translate_Y_left = rotated_Coordinates_left(1) + robotY;

		/* Sensor 1 Bounding Box - Only update within this box */ 
		double x_1_R = robotX + robotRad;
		double y_1_R = robotY + robotRad;

		double x_2_R = sensorRange * cos(sensorTh_R - Beta) + x_1_R;
		double y_2_R = sensorRange * sin(sensorTh_R - Beta) + y_1_R;

		double x_4_R = sensorRange * cos(sensorTh_R + Beta) + x_1_R;
		double y_4_R = sensorRange * sin(sensorTh_R + Beta) + y_1_R;

		double r2_R = sensorRange / cos(Beta);

		double x_3_R = r2_R * cos(sensorTh_R) + x_1_R;
		double y_3_R = r2_R * sin(sensorTh_R) + y_1_R;

		double x_lower_R = min(x_1_R, x_2_R, x_3_R, x_4_R);
		double y_lower_R = min(y_1_R, y_2_R, y_3_R, y_4_R);

		double x_upper_R = max(x_1_R, x_2_R, x_3_R, x_4_R);
		double y_upper_R = max(y_1_R, y_2_R, y_3_R, y_4_R);

		/* Sensor 2 Bounding Box - Only update within this box */
		double x_1_L = robotX + robotRad;
		double y_1_L = robotY + robotRad;

		double x_2_L = sensorRange * cos(sensorTh_L - Beta) + x_1_L;
		double y_2_L = sensorRange * sin(sensorTh_L - Beta) + y_1_L;

		double x_4_L = sensorRange * cos(sensorTh_L + Beta) + x_1_L;
		double y_4_L = sensorRange * sin(sensorTh_L + Beta) + y_1_L;

		double r2_L = sensorRange / cos(Beta);

		double x_3_L = r2_L * cos(sensorTh_L) + x_1_L;
		double y_3_L = r2_L * sin(sensorTh_L) + y_1_L;

		double x_lower_L = min(x_1_L, x_2_L, x_3_L, x_4_L);
		double y_lower_L = min(y_1_L, y_2_L, y_3_L, y_4_L);

		double x_upper_L = max(x_1_L, x_2_L, x_3_L, x_4_L);
		double y_upper_L = max(y_1_L, y_2_L, y_3_L, y_4_L);

		/* Set initial probabilities to 0.5 */
		if (iterationCount == 0)
		{
			int m, n;

			for (m = 0; m < 100; m++)
			{
				for (n = 0; n < 100; n++)
				{
					previousGrid[m][n] = 0.5f;
				}
			}

		}
		/* Set previous probabilities */
		else if (iterationCount == 1)
		{
			int o, p;

			for (o = 0; o < 100; o++)
			{
				for (p = 0; p < 100; p++)
				{
					previousGrid[o][p] = occupancyGrid[o][p];
				}
			}
		}

		/* Occupancy Grid - Update Loop */
		int i, j;

		for (i = 0; i < 100; i++)
		{
			for (j = 0; j < 100; j++)
			{
				int i_int = i * resolution -4000.0;
				int j_int = j * resolution -2500.0;

				/* Sensor 1 - Probability terms for Bayes Recursive Theorem */
				double P_occ_R = (probability(i_int, j_int, sensorX_R, sensorY_R, sensorTh_R, rightDistance, sensorRange));
				double P_1_occ_R = (1 - probability(i_int, j_int, sensorX_R, sensorY_R, sensorTh_R, rightDistance, sensorRange));

				/* Sensor 2 - Probability terms for Bayes Recursive Theorem */
				/*double P_occ_L = (probability(i_int, j_int, sensorX_L, sensorY_L, sensorTh_L, leftDistance, sensorRange));
				double P_1_occ_L = (1 - probability(i_int, j_int, sensorX_L, sensorY_L, sensorTh_L, leftDistance, sensorRange));*/

				double P_previous_occ = previousGrid[i][j];
				double P_1_previous_occ = (1 - previousGrid[i][j]);

				/* Check if current grid cell is within Sensor 1 Bounding Box */
				if (i_int > x_lower_R && i_int < x_upper_R && j_int > y_lower_R && j_int < y_upper_R)
				{
					/* Bayes Recursive Theorem */
					double Bayes_Recursive_Result = (P_occ_R * P_previous_occ) / ((P_occ_R *P_previous_occ) + (P_1_occ_R *P_1_previous_occ));
					
					occupancyGrid[i][j] = 0.5 + (0.5 * Bayes_Recursive_Result);

				}
				/* Check if current grid cell is within Sensor 2 Bounding Box */
				//if (i_int > x_lower_L && i_int < x_upper_L && j_int > y_lower_L && j_int < y_upper_L)
				//{
				//	/* Bayes Recursive Theorem */
				//	double Bayes_Recursive_Result = (P_occ_L * P_previous_occ) / ((P_occ_L *P_previous_occ) + (P_1_occ_L *P_1_previous_occ));

				//	occupancyGrid[i][j] = 0.5 + (0.5 * Bayes_Recursive_Result);

				//}
			}
		}
		/* Check to ensure that previousGrid is populated */
		iterationCount = 1;

		/* Following multiple executions the Occupancy Grid Map is generated */
		if (executedCount > 5000)
		{

			float scale = 0.025;
			float angle_x = 340.f;
			float angle_y = 55.f;
			// Request a 32-bits depth buffer when creating the window
			sf::ContextSettings contextSettings;
			contextSettings.depthBits = 32;

			// Create the main window
			sf::RenderWindow window(sf::VideoMode(800, 600), "Sonar Model Visualisation Program", sf::Style::Default, contextSettings);
			window.setVerticalSyncEnabled(true);
			window.setActive();

			// Enable Z-buffer read and write
			glEnable(GL_DEPTH_TEST);
			glDepthMask(GL_TRUE);
			glClearDepth(1.f);

			// Disable lighting
			glDisable(GL_LIGHTING);

			// Configure the viewport (the same size as the window)
			glViewport(0, 0, window.getSize().x, window.getSize().y);

			// Setup a perspective projection
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			GLfloat ratio = static_cast<float>(window.getSize().x) / window.getSize().y;
			glFrustum(-ratio, ratio, -1.f, 1.f, 1.f, 500.f);

			glClearColor(1.f, 1.f, 1.f, 1.f);
			// Start game loop
			while (window.isOpen())
			{
				// Process events
				sf::Event event;
				while (window.pollEvent(event))
				{
					// Close window: exit
					if (event.type == sf::Event::Closed)
						window.close();

					// Escape key: exit
					if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Escape))
						window.close();

					// Adjust the viewport when the window is resized
					if (event.type == sf::Event::Resized)
						glViewport(0, 0, event.size.width, event.size.height);

					// Process otehr keypresses
					if (event.type == sf::Event::KeyPressed)
					{

						// Movement of the model
						if (event.key.code == sf::Keyboard::Up)
						{
							angle_y += 5.0f;
							if (angle_y > 360.0f) angle_y -= 360.f;
						}
						if (event.key.code == sf::Keyboard::Down)
						{
							angle_y -= 5.0f;
							if (angle_y < 0.0f) angle_y += 360.f;
						}
						if (event.key.code == sf::Keyboard::Right)
						{
							angle_x += 5.0f;
							if (angle_x > 360.0f) angle_x -= 360.f;
						}
						if (event.key.code == sf::Keyboard::Left)
						{
							angle_x -= 5.0f;
							if (angle_x < 0.0f) angle_x += 360.f;
						}
						if (event.key.code == sf::Keyboard::Add || (event.key.code == sf::Keyboard::Equal && event.key.shift)) scale += 0.002;
						if (event.key.code == sf::Keyboard::Subtract || event.key.code == sf::Keyboard::Dash) scale -= 0.002;
					}
				}

				// Clear the depth buffer
				glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

				// We get the position of the mouse cursor, so that we can move the box accordingly
				float x = sf::Mouse::getPosition(window).x * 200.f / window.getSize().x - 100.f;
				float y = -sf::Mouse::getPosition(window).y * 200.f / window.getSize().y + 100.f;

				// Apply some transformations
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
				glTranslatef(0, 0, -100.f);

				// rotate and scale
				glRotatef(angle_x, 0.0, 1.0, 0.0);
				glRotatef(angle_y, 1.0, 1.0, 0.0);
				glScalef(scale, 0.1, scale);

				// draw Xaxis 
				glBegin(GL_LINES);
				// Red
				glColor3f(1.0, 0.0, 0.0);
				glVertex3f(0, 25, 0.0);
				glVertex3f(2500, 25, 0.0);
				// Green
				glColor3f(0.0, 1.0, 0.0);
				// z-axis (y-axis of the occ grid)
				glVertex3f(0.0, 25, 0.0);
				glVertex3f(0.0, 25, 2500.0);

				glEnd();

				// Black
				glColor3f(0.0, 0.0, 0.0);
				// Draw occ grid
				int i, j;
				for (i = 0; i < 100; i++)
				{
					glBegin(GL_LINE_STRIP);
					for (j = 0; j < 100; j++)
					{
						glVertex3f(resolution*i- 4000, occupancyGrid[i][j] * 50.0, resolution*j - 2500); //2500
					}
					glEnd();
				}

				for (i = 0; i < 100; i++)
				{
					glBegin(GL_LINE_STRIP);
					for (j = 0; j < 100; j++)
					{
						glVertex3f(resolution*j - 4000, occupancyGrid[j][i] * 50., resolution*i - 2500); // 2500
					}
					glEnd();
				}

				// Finally, display the rendered frame on screen
				window.display();

			}
		}

		executedCount = executedCount + 1;
	}

	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();
	Aria::exit(0);

}

double probability(double x, double y, double sX, double sY, double sTh, double sRead, double sRange)
{
	double prob_val;
	double rightDistance = sRead;
	double sensorRange = sRange;

	if (rightDistance <= sensorRange)
	{
		// Sonar Model // 
		double m = 0;
		double e = 0;
		double grid_X = x;
		double grid_Y = y;
		double sensorX = sX;
		double sensorY = sY;
		double sensorTh = sTh;

		double local_X = sensorX - grid_X;
		double local_Y = sensorY - grid_Y;
		double sensor_Th = radians(sensorTh);
		double sensor_Reading = rightDistance;
		double max_angle = 20;
		double max_angle_range = sensor_Th + max_angle;
		double max_occupied = 0.98;

		double theta = degrees(atan(local_Y / local_X));

		if (local_X < 0)
		{
			if (theta <= 0)
			{
				theta = theta + 180;
			}
			else if (theta > 0)
			{
				theta = theta - 180;
			}
		}

		if (sensorTh > 180 - max_angle && theta < 0)
		{
			theta = theta + 360;
		}
		else if (sensorTh < -180 + max_angle && theta > 0)
		{
			theta = theta - 360;
		}

		double tolerance = 100;

		double local_theta = sensor_Th - theta;

		local_theta = fabs(local_theta);

		/* Get distance r */
		double dist_to_cell = sqrt((pow(grid_X - sensorX, 2)) + pow(grid_Y - sensorY, 2));

		prob_val = 0.5;
		
		if (dist_to_cell < rightDistance + tolerance && local_theta < max_angle) // Region I
		{
			m = 0.5 + (dist_to_cell - (rightDistance - 50))* (0.5 / 50);
			e = 0.5 + (dist_to_cell - (rightDistance - 50))* (0.1 / 50); 
			prob_val = e + (local_theta - max_angle) *((m - e) / -max_angle);
		}

	}
	else
	{
		prob_val = 0.5;
	}

	return prob_val;
}

double radians(double angle)
{
	return (3.141 / 180.0) * angle;
}

double degrees(double angle)
{
	return (180.0 / 3.141) * angle;
}