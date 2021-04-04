#ifndef ROBOTARM_H
#define ROBOTARM_H

// Directions for eigen header file linking:
// > project properties > VC++ Directories > Include Directories > Eigen 3.2.8
// > project properties > C++ > General > Include Directories > Eigen folder
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <phidget21.h>
#include <iomanip>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include <cmath>
#include <vector>
#include <algorithm>
#include <time.h>
#include <stdio.h>
#include <dynamixel_sdk.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <termios.h>
#include <thread>

// Deconflict structures with pthread
#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>

using namespace Eigen;
using namespace std;

// Dynamixel pro control table
#define PRO_OPERATING_MODE 11
#define PRO_TORQUE_ENABLE 562
#define PRO_GOAL_POSITION 596
#define PRO_GOAL_VELOCITY 600
#define PRO_GOAL_ACCELERATION 606
#define PRO_PRES_POSITION 611
// Data byte length
#define PRO_POSITION_LENGTH 4
#define PRO_VELOCITY_LENGTH 4

// MX-28AR control table
#define MX_MOV_SPEED_LOW 32
#define MX_MOV_SPEED_HIGH 33
#define MX_TORQUE_LIM_LOW 34
#define MX_TORQUE_LIM_HIGH 35
#define MX_PRES_POS_LOW 36
#define MX_PRES_POS_HIGH 37
// Data byte length
#define MX_TORQUE_LENGTH 2
#define MX_POSITION_LENGTH 2
#define MX_VELOCITY_LENGTH 2

// Protocol version for actuator communication
#define PROTOCOL_VERSION1 1.0 // Dynamixel MX28
#define PROTOCOL_VERSION2 2.0 // Dynamixel Pro

#define DEVICENAME  "/dev/ttyUSB1"   //"COM6"   // COM port number under Windows for harvesting manipulator
#define DEVICENAME2 "/dev/ttyUSB0"  //"COM7"   // COM port number under Windows for catching manipulator
#define BAUD_NUM 57600       // COM speed
//#define pi 3.1415926535897

// Function prototypes for kinematics calculations
double azimuth(const Eigen::Vector3d &);
double newAzimuth(const Eigen::Vector3d &);
double pitch(const Eigen::Vector3d &);
Eigen::Matrix3d Rot(double []);
vector<double> eulerZYZ(const Eigen::Matrix<double, 3, 3> &);
vector<double> eulerZYZNeg(const Eigen::Matrix<double, 3, 3> &);
vector<double> ForKin(const Eigen::Matrix<double, 8, 1> &);
vector<double> ForKin_6DOF(const Eigen::Matrix<double, 6, 1> &);
Eigen::Matrix<double, 8, 1> IK(const Eigen::Vector3d &, vector<double>, const Eigen::Matrix<double, 8, 1> &, bool &);
Eigen::Matrix<double, 6, 8> Jacob(const Eigen::Matrix<double, 8, 1> &);
void checkCatchPos(double, double);


void write_csv_final( float, float, float, float, float, float, int, float, float, float, float, float, float, int, int, int, int, int, int, int, int, string);
void write_csv_step( float, float, float, float, float, float);
void write_csv_jointValidation( int, int, int, int, int, int, int, int);


// Function prototypes for manipulator control
void gotoPosition(const Eigen::Matrix<double, 8, 1> &);
void hideArm();
void returnArm();
void approachFruit(const Eigen::Matrix<double, 8, 1> &, const Eigen::Matrix<double, 8, 1> &, const Eigen::Vector3d &);
void moveBase(double, double);
void catchApple(double, double);

// Function prototypes for end effector control
void closeEE();
void openEE();

// Wrapper functions for parallel thread processing
void * gotoPositionA_wrapper(void * argument);
void * approachFruit_wrapper(void * argument);
void * gotoPositionD_wrapper(void * argument);
void * moveBaseA_wrapper(void * argument);
void * moveBaseF_wrapper(void * argument);
void * moveBaseD_wrapper(void * argument);


//extern int jointV[8];
//extern bool jointValid;
extern bool catchValid;
extern const double pi;

void OriginalMain(void);
int validateFruitPickability(Eigen::Vector3d);
void write_csv(vector<vector<double>>, std::string);
void threadWrapperForSevenJointSimul(int);
void testOneJVChoice(void);
void testGroupJVChoice(void);
bool validatePointReachability(Vector3d);

const double jLL[8] = {-.205, -.34, -1.5708, -4.7124, -1.5708, -3.4907, -4.7124, 0};   // Lower joint limits
const double jLT[8] = {-.18, -.30, -1.4708, -4.6124, -1.4708, -3.3907, -4.6124, .1};   // Lower joint thresholds
const double jLU[8] = {.205, .34, 1.5708, -2.3562, 1.5708, -1.2217, -1.5708, 3.1416};  // Upper joint limits
const double jUT[8] = {.18, .30, 1.4708, -2.4562, 1.4708, -1.3217, -1.6708, 3.0416};   // Upper joint thresholds
#endif
