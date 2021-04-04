#include "robotarm.h"

const double pi =  3.1415926535897;
// Harvesting manipulator joint limits
/*
const double jLL[8] = {-.205, -.34, -1.5708, -4.7124, -1.5708, -3.4907, -4.7124, 0};   // Lower joint limits
const double jLT[8] = {-.18, -.30, -1.4708, -4.6124, -1.4708, -3.3907, -4.6124, .1};   // Lower joint thresholds
const double jLU[8] = {.205, .34, 1.5708, -2.3562, 1.5708, -1.2217, -1.5708, 3.1416};  // Upper joint limits
const double jUT[8] = {.18, .30, 1.4708, -2.4562, 1.4708, -1.3217, -1.6708, 3.0416};   // Upper joint thresholds

const double jLL[8] = {0, 0, -1.5708, -4.7124, -1.5708, -3.4907, -4.7124, 0};   // Lower joint limits
const double jLT[8] = {0.025, 0.04, -1.4708, -4.6124, -1.4708, -3.3907, -4.6124, .1};   // Lower joint thresholds
const double jLU[8] = {.41, .68, 1.5708, -2.3562, 1.5708, -1.2217, -1.5708, 3.1416};  // Upper joint limits
const double jUT[8] = {.385, .64, 1.4708, -2.4562, 1.4708, -1.3217, -1.6708, 3.0416};   // Upper joint thresholds

const double jLL[8] = {-0.04, -.205, -1.5708, -4.7124, -1.5708, -3.4907, -4.7124, 0};   // Lower joint limits
const double jLT[8] = {0, -.18, -1.4708, -4.6124, -1.4708, -3.3907, -4.6124, .1};   // Lower joint thresholds
const double jLU[8] = {0.04, .205, 1.5708, -2.3562, 1.5708, -1.2217, -1.5708, 3.1416};  // Upper joint limits
const double jUT[8] = {0, .18, 1.4708, -2.4562, 1.4708, -1.3217, -1.6708, 3.0416};   // Upper joint thresholds

const double jLL[8] = {-.04, -.34, -1.5708, -4.7124, -1.5708, -3.4907, -4.7124, 0};   // Lower joint limits
const double jLT[8] = {-.0, -.30, -1.4708, -4.6124, -1.4708, -3.3907, -4.6124, .1};   // Lower joint thresholds
const double jLU[8] = {.04, .34, 1.5708, -2.3562, 1.5708, -1.2217, -1.5708, 3.1416};  // Upper joint limits
const double jUT[8] = {.0, .30, 1.4708, -2.4562, 1.4708, -1.3217, -1.6708, 3.0416};   // Upper joint thresholds



const double jLL[8] = {-.34, -.205, -1.5708, -4.7124, -1.5708, -3.4907, -4.7124, 0};   // Lower joint limits
const double jLT[8] = {-.30, -.18, -1.4708, -4.6124, -1.4708, -3.3907, -4.6124, .1};   // Lower joint thresholds
const double jLU[8] = {.34, .205, 1.5708, -2.3562, 1.5708, -1.2217, -1.5708, 3.1416};  // Upper joint limits
const double jUT[8] = {.30, .18, 1.4708, -2.4562, 1.4708, -1.3217, -1.6708, 3.0416};   // Upper joint thresholds
*/
// Flag for the manipulator's joint limit check (do not change this as it's used by the IK algorithm!). If
// joint limits are violated, the flag is set to false and the fruit is exluded from the harvesting cycle.

//bool jointValid = true;
//bool jointValid = true;

//int jointV[8] = {0, 0, 0, 0, 0, 0, 0, 0};

std::ofstream stepFile;

// Catching robot's kinematic parameters
const double x_offset = .10;	// Offset from the world reference frame origin along the x axis
const double y_offset = 0.0;		// Offset from the world reference frame origin along the y axis
const double link1_length = .38;
const double link2_length = .34;

// Catching robot's joint limits (radians)
const double catchLimit_L[2] = { -90, -145 };
const double catchLimit_U[2] = { 90, 145 };
// Create an exclusion zone where a dropped fruit would hit a mechanical part
const double x_limit = .35;
const double y_limit_L = -.2;
const double y_limit_U = .2;

// Manipulator variables used for real time communication
int ID_PRO[6] = {1, 2, 3, 4, 5, 6};	// Dynamixel Pro harvesting IDs
int ID_CATCH[2] = { 11, 12 };		// Dynamixel Pro catching IDs

// The manipulator's Denavit-Hartenberg (DH) parameters are
const int N = 8;	                                                        // degrees of freedom
const double a[8] = {0, 0, 0, 0, 0, 0, 0, .198};	                        // link length (meters)
const double alpha[8] = {-pi/2, pi/2, pi/2, pi/2, pi/2, pi/2, pi/2, pi/2};	// link twist
//double d[8] = {0, 0, .189, 0, .27, 0, .231, 0};                             // link offset (1&2 are variable)
//double theta[8] = {0, -pi/2, 0, 0, 0, 0, 0, 0};  	                        // joint angle (3-8 are variable)


// Global counter for turning on torque & setting acceleration
int moveCounter = 0;
int catchCounter = 0;

// Flag for the catching manipulator's joint limits. If the joint limits are violated, the drop point is
// moved to a predetermined staging location
bool catchValid = true;

// Set the azimuth angle (i.e. yaw) based on fruit's x, y coordinates
// azimuth angle from the origin
double azimuth(const Eigen::Vector3d &position)

{
    // Find the azimuth angle
    double a = atan(position(1,0)/position(0,0));
    return a;
}
// Set the azimuth angle (i.e. yaw) based on fruit's x, y coordinates
// azimuth angle from the ends of the y axis since only when it's outside y axis limits that azimuth angle is required, otherwise it should be 0.
double newAzimuth(const Eigen::Vector3d &position)

{
    double a;
    // Find the azimuth angle
    if(position(1,0) > jUT[1]){
        a = atan(position(1,0)/position(0,0));
    }
    else if(position(1,0) < jLT[1]){
        a = atan(position(1,0)/position(0,0));
    }
    else{
        a = 0;
    }
    return a;
}

// Set the pitch angle based on fruit height
double pitch(const Eigen::Vector3d &position)
{
    int p;

    if (position(2,0) < .5) // Height less than .5 meters
        p = -5;//-5;
    else if (position(2,0) >= .5 && position(2,0) < .6)
        p = -15;
    else if (position(2,0) >= .6 && position(2,0) < .7)
        p = -30;
    else
        p = -45;

    return (p*pi/180);
}

// Determine the 3 x 3 rotation matrix for the specified roll, pitch, and yaw angles
Eigen::Matrix3d Rot(double eulerRPY[])
{
    Matrix3d R, Rx, Ry, Rz, W;
    W << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    // Rotate the end-effector around the world x-axis by the roll
    Rx << 1, 0, 0,
          0, cos(eulerRPY[0]), -sin(eulerRPY[0]),
          0, sin(eulerRPY[0]), cos(eulerRPY[0]);

    // Rotate the end-effector around the world y-axis by the pitch
    Ry << cos(eulerRPY[1]), 0, sin(eulerRPY[1]),
          0, 1, 0,
        -sin(eulerRPY[1]), 0, cos(eulerRPY[1]);

    // Rotate the end-effector around the world z-axis by the yaw
    Rz << cos(eulerRPY[2]), -sin(eulerRPY[2]), 0,
          sin(eulerRPY[2]), cos(eulerRPY[2]), 0,
          0, 0, 1;

    // Calculate the rotation matrix
    R = Rz*Ry*Rx*W;
    return R;
}

// Determine the ZYZ euler angles (intrinsic rotations) for the 3 x 3 rotation matrix R
vector<double> eulerZYZ(const Eigen::Matrix<double, 3, 3> &R)
{
    double alpha, beta, gamma;
    // Find the ZYZ Euler angles (intrinsic rotations) from the 3 x 3 rotation matrix
    alpha = atan2(R(1,2),R(0,2)); //phi
    double a = pow(R(0,2),2)+pow(R(1,2),2);
    double b = pow(a,.5);
    beta = atan2(b,R(2,2)); //theta
    gamma = atan2(R(2,1),-R(2,0)); //psi

    // Return the ZYZ Euler angles as a vector
    vector<double> eulerZYZ(3);
    eulerZYZ[0] = alpha;
    eulerZYZ[1] = beta;
    eulerZYZ[2] = gamma;

    return eulerZYZ;
}

vector<double> eulerZYZNeg(const Eigen::Matrix<double, 3, 3> &R)
{
    double alpha, beta, gamma;
    // Find the ZYZ Euler angles (intrinsic rotations) from the 3 x 3 rotation matrix
    alpha = atan2(-R(1,2),-R(0,2)); //phi
    double a = pow(R(0,2),2)+pow(R(1,2),2);
    double b = -pow(a,.5);
    beta = atan2(b,R(2,2)); //theta
    gamma = atan2(-R(2,1),R(2,0)); //psi

    // Return the ZYZ Euler angles as a vector
    vector<double> eulerZYZ(3);
    eulerZYZ[0] = alpha;
    eulerZYZ[1] = beta;
    eulerZYZ[2] = gamma;

    return eulerZYZ;
}

/* IK algorithm determines if a joint solution exists at the specified position
   and ZYZ euler angles*/
Eigen::Matrix<double, 8, 1> IK(const Eigen::Vector3d &position, vector<double> euler, const Eigen::Matrix<double, 8, 1> &Qstart, bool &jointValid)
{
    double delt = .01;	// 100 Hz sampling rate
    int Time = 1;   	// duration of movement
    // Create a vector of time for numerical integration
    int length = (Time/delt)+1;
    vector<double> t(length);
    for (int i = 0; i < length; i++)
    {
        t[i] = i*delt;
    }

    // Starting joint vector
    Matrix<double, 8, 1> q;
    //q << Qstart(0, 0), Qstart(1, 0), Qstart(2, 0), Qstart(3, 0), Qstart(4, 0), Qstart(5, 0), Qstart(6, 0), Qstart(7, 0);
    q << Qstart(0, 0), 0.30, Qstart(2, 0), Qstart(3, 0), Qstart(4, 0), Qstart(5, 0), Qstart(6, 0), Qstart(7, 0);
    // Call the ForKin function for the home position. Start is a vector with [x,y,z,alpha,beta,gamma]
    vector<double> start = ForKin(q);

    // Determine end-effector translational velocities
    double Vx = (position(0,0)-start[0])/Time;
    double Vy = (position(1,0)-start[1])/Time;
    //double Vy = 0;
    double Vz = (position(2,0)-start[2])/Time;
    Vector3d Vtrans;
    Vtrans << Vx, Vy, Vz;

    // Determine euler angle velocities
    double Valpha = (euler[0]-start[3])/Time;
    double Vbeta = (euler[1]-start[4])/Time;
    double Vgamma = (euler[2]-start[5])/Time;
    Vector3d Veuler;
    Veuler << Valpha, Vbeta, Vgamma;

    // Positive definite gain matrices
    Matrix3d Kp, Ko;
    Kp << 100, 0, 0,
          0, 100, 0,
          0, 0, 100;
    Ko << 100, 0, 0,
          0, 100, 0,
          0, 0, 100;

    double xD, yD, zD, eul1, eul2, eul3;
    Vector3d Perror;
    Vector3d orientDx, orientDy, orientDz, orientEx, orientEy, orientEz, Oerror, omega;
    Vector3d P, O;
    Matrix3d T, Snd1, Snd2, Snd3, Sne1, Sne2, Sne3, L;
    Matrix<double, 6, 1> EEvel, M;
    vector<double> A(6);

    Matrix<double, 6, 6> Jt;
    Matrix<double, 8, 6> Jtrans, Jpsi;
    vector< vector<double> > JJ(6,vector<double>(8));
    Matrix<double, 8, 1> H1;

    // Weight matrix
    Matrix<double, 8, 8> w;
    w << .01, 0, 0, 0, 0, 0, 0, 0,
          0, .01, 0, 0, 0, 0, 0, 0,
          0, 0, .01, 0, 0, 0, 0, 0,
          0, 0, 0, .01, 0, 0, 0, 0,
          0, 0, 0, 0, .01, 0, 0, 0,
          0, 0, 0, 0, 0, .01, 0, 0,
          0, 0, 0, 0, 0, 0, .01, 0,
          0, 0, 0, 0, 0, 0, 0, .01;

    // Identity matrix
    Matrix<double, 8, 8> eye;
    eye << 1, 0, 0, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0, 0,
           0, 0, 0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 0, 0, 1;

    Matrix<double, 8, 1> qdot;


    vector<vector<double>> intermediateNumbers;
/*
    string fileName = "/home/abhi/Desktop/Heng/ZeroOriginNegativeYCorrected_2.csv";
    std::ofstream csvFile;
    csvFile.open (fileName, ios::out | ios::app);
    csvFile << "x,y,z,alpha,beta,gama,px,py,r1,r2,r3,r4,r5,r6\n";
    csvFile.close();
*/
    // Inverse Kinematics algorithm with closed loop control
    for (int i = 0; i < length; i++)
    {
        // Desired end-effector position (at each time step)
        xD = start[0]+t[i]*Vx;
        yD = start[1]+t[i]*Vy;
        zD = start[2]+t[i]*Vz;

        // Current end-effector position (at each time step)
        A = ForKin(q);
        //vector<double> targetAndJointParams = {A[0], A[1], A[2], A[3], A[4], A[5], q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7]};
        //intermediateNumbers.push_back(targetAndJointParams);

        // Find the position error
        Perror << xD-A[0], yD-A[1], zD-A[2];
        //Perror << xD-A[0], 0, zD-A[2];

        // Desired end-effector orientation (at each step)
        eul1 = start[3]+t[i]*Valpha;
        eul2 = start[4]+t[i]*Vbeta;
        eul3 = start[5]+t[i]*Vgamma;

        // Desired end-effector frame (see Siciliano eq. 2.18)
        orientDx(0) = cos(eul1)*cos(eul2)*cos(eul3)-sin(eul1)*sin(eul3);
        orientDx(1) = sin(eul1)*cos(eul2)*cos(eul3)+cos(eul1)*sin(eul3);
        orientDx(2) = -sin(eul2)*cos(eul3);
        orientDy(0) = -cos(eul1)*cos(eul2)*sin(eul3)-sin(eul1)*cos(eul3);
        orientDy(1) = -sin(eul1)*cos(eul2)*sin(eul3)+cos(eul1)*cos(eul3);
        orientDy(2) = sin(eul2)*sin(eul3);
        orientDz(0) = cos(eul1)*sin(eul2);
        orientDz(1) = sin(eul1)*sin(eul2);
        orientDz(2) = cos(eul2);

        // Current end-effector frame
        orientEx(0) = cos(A[3])*cos(A[4])*cos(A[5])-sin(A[3])*sin(A[5]);
        orientEx(1) = sin(A[3])*cos(A[4])*cos(A[5])+cos(A[3])*sin(A[5]);
        orientEx(2) = -sin(A[4])*cos(A[5]);
        orientEy(0) = -cos(A[3])*cos(A[4])*sin(A[5])-sin(A[3])*cos(A[5]);
        orientEy(1) = -sin(A[3])*cos(A[4])*sin(A[5])+cos(A[3])*cos(A[5]);
        orientEy(2) = sin(A[4])*sin(A[5]);
        orientEz(0) = cos(A[3])*sin(A[4]);
        orientEz(1) = sin(A[3])*sin(A[4]);
        orientEz(2) = cos(A[4]);

        // Find the orientation error (see Siciliano eq. 3.85, p139)
        Oerror = .5*(orientEx.cross(orientDx)+orientEy.cross(orientDy)+orientEz.cross(orientDz));

        // The relationship between the angular velocity and time derivative of the
        // euler angles is (Siciliano eq. 3.64, p130)
        T << 0, -sin(eul1), cos(eul1)*sin(eul2),
             0, cos(eul1), sin(eul1)*sin(eul2),
             1, 0, cos(eul2);

        // The desired angular velocity is
        omega = T*Veuler;
        // The desired end-effector velocity is
        EEvel << Vx, Vy, Vz, omega(0), omega(1), omega(2);

        // Find the skew symmetric matrices
        Snd1 << 0, -orientDx(2), orientDx(1),
                orientDx(2), 0, -orientDx(0),
                -orientDx(1), orientDx(0), 0;
        Snd2 << 0, -orientDy(2), orientDy(1),
                orientDy(2), 0, -orientDy(0),
                -orientDy(1), orientDy(0), 0;
        Snd3 << 0, -orientDz(2), orientDz(1),
                orientDz(2), 0, -orientDz(0),
                -orientDz(1), orientDz(0), 0;
        Sne1 << 0, -orientEx(2), orientEx(1),
                orientEx(2), 0, -orientEx(0),
                -orientEx(1), orientEx(0), 0;
        Sne2 << 0, -orientEy(2), orientEy(1),
                orientEy(2), 0, -orientEy(0),
                -orientEy(1), orientEy(0), 0;
        Sne3 << 0, -orientEz(2), orientEz(1),
                orientEz(2), 0, -orientEz(0),
                -orientEz(1), orientEz(0), 0;

        // See pg. 139 of Siciliano et al.
        L = -.5*(Snd1*Sne1+Snd2*Sne2+Snd3*Sne3);
        P = Vtrans+Kp*Perror;
        O = L.inverse()*(L.transpose()*omega+Ko*Oerror);
        M.topRows(3) = P;
        M.bottomRows(3) = O;

        // Find the current Jacobian
        Matrix<double, 6, 8> J = Jacob(q);

        // Find the right pseudo-inverse (Jpsi) of J
        Jtrans = J.transpose();
        Jt = J*Jtrans;

        // Use the inverse member function of a Partial Pivot LU matrix
        Eigen::PartialPivLU<Matrix<double, 6, 6>> Jinv(Jt);
        Jpsi = Jtrans*(Jinv.inverse());

        /**** Null space projection for joint limit avoidance *****/

        // Check joint positions
        for (int m = 0; m < 8; m++)
        {
            if (q(m,0) > jUT[m])
            {
                H1(m,0) = 2*(q(m,0)-jUT[m])/(pow(jLU[m]-jUT[m],2));
            }
            else if (q(m,0) < jLT[m])
            {
                H1(m,0) = 2*(q(m,0)-jLT[m])/(pow(jLL[m]-jLT[m],2));
            }
            else
            {
                H1(m,0) = 0;
            }
        }

        // The joint velocity is
        qdot = (Jpsi*M)+(eye-(Jpsi*J))*(w*H1);

        // Calculate the joint vector by numerical integration in discrete time
        q = q + (qdot*delt);
        //q[1] = 0.3;
    }
    //vector<double> fcords = ForKin(q);
    //vector<double> targetAndJointParams = {fcords[0], fcords[1], fcords[2], fcords[3], fcords[4], fcords[5], q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7]};
    //intermediateNumbers.push_back(targetAndJointParams);
    //write_csv(intermediateNumbers, fileName);

    //write_csv_step(q[0], q[1], q[2], q[3], q[4], q[5]);
    // Check for joint limit violations
    int jointV[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    /*for (int i = 0; i < 8; i++)
    {
        jointV[i] = 0;
    }*/
    for (int i = 0; i < 8; i++)
    {
        if (q(i,0) > jLU[i] || q(i,0) < jLL[i])
            jointV[i] = 1;
    }
    int total = 0;
    for (int i = 0; i < 8; i++)
        total = total+jointV[i];
    if (total == 0)
        jointValid = true;
    else
        jointValid = false;

    return q;
}

void write_csv_final( float a0, float a1, float a2, float f0, float f1, float f2, int t, float s0, float s1, float s2, float s3, float s4, float s5, int j1, int j2, int j3, int j4, int j5, int j6, int j7, int j8, string fileName)
{
    stepFile.open (fileName, ios::out | ios::app);
    //MyFile << " Time_Stamp: ,"<<ch << ",";
    stepFile << a0 << ",";
    stepFile << a1 << ",";
    stepFile << a2 << ",";
    stepFile << f0 << ",";
    stepFile << f1 << ",";
    stepFile << f2 << ",";
    stepFile << t << ",";
    stepFile << s0 << ",";
    stepFile << s1 << ",";
    stepFile << s2 << ",";
    stepFile << s3 << ",";
    stepFile << s4 << ",";
    stepFile << s5 << ",";
    stepFile << j1 << ",";
    stepFile << j2 << ",";
    stepFile << j3 << ",";
    stepFile << j4 << ",";
    stepFile << j5 << ",";
    stepFile << j6 << ",";
    stepFile << j7 << ",";
    stepFile << j8 << "\n";
    stepFile.close();
}

void write_csv_step( float s0, float s1, float s2, float s3, float s4, float s5)
{
    char ch[20];
    time_t now = time(NULL);
    std::strftime(ch, 20, "%Y-%m-%d_%H_%M_%S", localtime(&now));

    stepFile.open ("/home/abhi/Desktop/Heng/jointvaraiblesxy.csv", ios::out | ios::app);
    //MyFile << " Time_Stamp: ,"<<ch << ",";
    stepFile << s0 << ",";
    stepFile << s1 << ",";
    stepFile << s2 << ",";
    stepFile << s3 << ",";
    stepFile << s4 << ",";
    //MyFile << s5 << ",";
    stepFile << s5 << "\n";
    //MyFile << "\n";
    stepFile.close();
}

void write_csv_jointValidation( int j1, int j2, int j3, int j4, int j5, int j6, int j7, int j8)
{
    stepFile.open ("/home/abhi/Desktop/Heng/jointValidation.csv", ios::out | ios::app);
    stepFile << j1 << ",";
    stepFile << j2 << ",";
    stepFile << j3 << ",";
    stepFile << j4 << ",";
    stepFile << j5 << ",";
    stepFile << j6 << ",";
    stepFile << j7 << ",";
    stepFile << j8 << "\n";
    stepFile.close();
}

void write_csv(vector<vector<double>> lines, std::string fileName)
{
    std::ofstream csvFile;
    csvFile.open (fileName, ios::out | ios::app);
    for(const auto& line:lines){
        for(const auto& item:line){
            if (&item != &line[0]){
                csvFile << ",";

            }
            csvFile << item;
        }
        csvFile << "\n";
    }
    csvFile.close();
}

void approachFruit(const Eigen::Matrix<double, 8, 1> &Qstart, const Eigen::Matrix<double, 8, 1> &Qend, const Eigen::Vector3d &V)
{
    // Find the Jacobian at the approach position
    Matrix<double, 6, 8> J = Jacob(Qstart);

    Matrix<double, 6, 6> Jt;
    Matrix<double, 8, 6> Jtrans, Jpsi;

    // Find the right pseudo-inverse (Jpsi) of J
    Jtrans = J.transpose();
    Jt = J*Jtrans;
    // Use the inverse member function of a Partial Pivot LU matrix
    Eigen::PartialPivLU<Matrix<double, 6, 6>> Jinv(Jt);
    Jpsi = Jtrans*(Jinv.inverse());

    // Set the end-effector velocity (there is no angular velocity)
    Matrix<double, 6, 1> vel;
    vel << V(0, 0), V(1, 0), V(2, 0), 0, 0, 0;

    // Find the joint velocities (solution minimizes the norm of the joint velocities)
    Matrix<double, 8, 1> qdot = Jpsi*vel;

    // Convert from rad/sec to deg/sec
    for (int i = 2; i < 8; i++)
        qdot(i, 0) = qdot(i, 0) * 180 / pi;
    // Convert to motor velocity input (RPM*gear reduction ratio)
    qdot(2, 0) = (qdot(2, 0)/6) * 502;
    qdot(3, 0) = (qdot(3, 0)/6) * 502;
    qdot(4, 0) = (qdot(4, 0)/6) * 502;
    qdot(5, 0) = (qdot(5, 0)/6) * 502;
    qdot(6, 0) = (qdot(6, 0)/6) * 304;
    qdot(7, 0) = (qdot(7, 0)/6) * 304;

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
    portHandler->openPort();
    portHandler->setBaudRate(BAUD_NUM);

    // Initialize groupsynchwrite instance for goal velocity and goal position
    dynamixel::GroupSyncWrite groupSyncWriteVEL(portHandler, packetHandler2, PRO_GOAL_VELOCITY, PRO_VELOCITY_LENGTH);
    dynamixel::GroupSyncWrite groupSyncWritePOS(portHandler, packetHandler2, PRO_GOAL_POSITION, PRO_POSITION_LENGTH);
    // Initialize groupsynchread instance for current position
    dynamixel::GroupSyncRead groupSyncReadPOS(portHandler, packetHandler2, PRO_PRES_POSITION, PRO_POSITION_LENGTH);

    int32_t velocity[6];
    // Cast data types from double to int32_t
    for (int k = 0; k < 6; k++)
    {
        velocity[k] = (int32_t)qdot(k + 2, 0);
    }
    uint8_t param_goal_velocity[4];
    unsigned short index = 0;
    // Set the velocities
    for (int j = 0; j < 6; j++)
    {
        param_goal_velocity[index++] = DXL_LOBYTE(DXL_LOWORD(velocity[j]));
        param_goal_velocity[index++] = DXL_HIBYTE(DXL_LOWORD(velocity[j]));
        param_goal_velocity[index++] = DXL_LOBYTE(DXL_HIWORD(velocity[j]));
        param_goal_velocity[index++] = DXL_HIBYTE(DXL_HIWORD(velocity[j]));
        groupSyncWriteVEL.addParam(ID_PRO[j], param_goal_velocity);
        index = 0;
    }
    // Syncwrite goal velocity
    groupSyncWriteVEL.txPacket();

    // Convert joint vector from radians to degrees
    Matrix<double, 8, 1> q;
    q(0, 0) = Qend(0, 0);
    q(1, 0) = Qend(1, 0);
    q(2, 0) = Qend(2, 0)*(180.0 / pi);
    q(3, 0) = Qend(3, 0)*(180.0 / pi);
    q(4, 0) = Qend(4, 0)*(180.0 / pi);
    q(5, 0) = Qend(5, 0)*(180.0 / pi);
    q(6, 0) = Qend(6, 0)*(180.0 / pi);
    q(7, 0) = Qend(7, 0)*(180.0 / pi);

    // Cast data types from double to int32_t
    int32_t angle[6];
    for (int k = 0; k < 6; k++)
    {
        angle[k] = (int32_t)q(k + 2, 0);
        cout << endl << "Joint " << k + 1 << ": " << angle[k] << endl;
    }

    // Convert angles to joint commands
    angle[0] = angle[0] * (251000 / 180); // Set angle to position value of H54-200
    angle[1] = angle[1] * (251000 / 180);
    angle[2] = angle[2] * (251000 / 180); // Set angle to position value of H54-100
    angle[3] = angle[3] * (251000 / 180);
    angle[4] = angle[4] * (151875 / 180); // Set angle to position value of H42-20
    angle[5] = angle[5] * (151875 / 180);

    uint8_t param_goal_position[4];
    index = 0;
    // Set the goal positions
    for (int k = 0; k < 6; k++)
    {
        param_goal_position[index++] = DXL_LOBYTE(DXL_LOWORD(angle[k]));
        param_goal_position[index++] = DXL_HIBYTE(DXL_LOWORD(angle[k]));
        param_goal_position[index++] = DXL_LOBYTE(DXL_HIWORD(angle[k]));
        param_goal_position[index++] = DXL_HIBYTE(DXL_HIWORD(angle[k]));
        groupSyncWritePOS.addParam(ID_PRO[k], param_goal_position);
        index = 0;
    }
    // Syncwrite goal position
    groupSyncWritePOS.txPacket();

    // Quit function when goal position reached (function checkMoving)
    // Add parameter storage for present position values
    for (int i = 0; i < 6; i++)
    {
        groupSyncReadPOS.addParam(ID_PRO[i]);
    }
    int dxl_comm_result = COMM_TX_FAIL; // communication result
    uint8_t dxl_error = 0;
    bool is_moving = true;
    short motor[6] = { 1, 1, 1, 1, 1, 1 };
    int32_t pres_position = 0;
    do
    {
        //cout << "Arm is moving" << endl;
        // Syncread present position
        dxl_comm_result = groupSyncReadPOS.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS)
            packetHandler2->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0)
            packetHandler2->printRxPacketError(dxl_error);

        for (int i = 0; i < 6; i++)
        {
            pres_position = groupSyncReadPOS.getData(ID_PRO[i], PRO_PRES_POSITION, PRO_POSITION_LENGTH);
            if ((pres_position > angle[i] - 5000) && pres_position < (angle[i] + 5000))
                motor[i] = 0;
        }
        int total = 0;
        for (int count = 0; count < 6; count++)
            total += motor[count];
        if (total == 0)
            is_moving = false;
    } while (is_moving == true);

    portHandler->closePort();
}

/* Compute the forward kinematics of the joint vector Q. The function
   returns a vector in the format [x, y, z, alpha, beta, gamma] where
   x, y, z are the 3D coordinates & alpha, beta, gamma are the ZYZ euler
   angles.*/
vector<double> ForKin(const Eigen::Matrix<double, 8, 1> &Q)
{
    double _d[8] = {0, 0, .189, 0, .27, 0, .231, 0};                             // link offset (1&2 are variable)
    double _theta[8] = {0, -pi/2, 0, 0, 0, 0, 0, 0};
    // Set the link offset for the bottom prismatic joints
    _d[0] = Q(0,0);
    _d[1] = Q(1,0);
    // Set the joint angles for the revolute joints
    _theta[2] = Q(2,0);
    _theta[3] = Q(3,0);
    _theta[4] = Q(4,0);
    _theta[5] = Q(5,0);
    _theta[6] = Q(6,0);
    _theta[7] = Q(7,0);

    Matrix4d A0, A1, A2, A3, A4, A5, A6, A7, T;
    //
    A0(0,0) = cos(_theta[0]);
    A0(0,1) = -sin(_theta[0])*cos(alpha[0]);
    A0(0,2) = sin(_theta[0])*sin(alpha[0]);
    A0(0,3) = a[0]*cos(_theta[0]);
    A0(1,0) = sin(_theta[0]);
    A0(1,1) = cos(_theta[0])*cos(alpha[0]);
    A0(1,2) = -cos(_theta[0])*sin(alpha[0]);
    A0(1,3) = a[0]*sin(_theta[0]);
    A0(2,0) = 0;
    A0(2,1) = sin(alpha[0]);
    A0(2,2) = cos(alpha[0]);
    A0(2,3) = _d[0];
    A0(3,0) = 0;
    A0(3,1) = 0;
    A0(3,2) = 0;
    A0(3,3) = 1;
    //
    A1(0,0) = cos(_theta[1]);
    A1(0,1) = -sin(_theta[1])*cos(alpha[1]);
    A1(0,2) = sin(_theta[1])*sin(alpha[1]);
    A1(0,3) = a[1]*cos(_theta[1]);
    A1(1,0) = sin(_theta[1]);
    A1(1,1) = cos(_theta[1])*cos(alpha[1]);
    A1(1,2) = -cos(_theta[1])*sin(alpha[1]);
    A1(1,3) = a[1]*sin(_theta[1]);
    A1(2,0) = 0;
    A1(2,1) = sin(alpha[1]);
    A1(2,2) = cos(alpha[1]);
    A1(2,3) = _d[1];
    A1(3,0) = 0;
    A1(3,1) = 0;
    A1(3,2) = 0;
    A1(3,3) = 1;
    //
    A2(0,0) = cos(_theta[2]);
    A2(0,1) = -sin(_theta[2])*cos(alpha[2]);
    A2(0,2) = sin(_theta[2])*sin(alpha[2]);
    A2(0,3) = a[2]*cos(_theta[2]);
    A2(1,0) = sin(_theta[2]);
    A2(1,1) = cos(_theta[2])*cos(alpha[2]);
    A2(1,2) = -cos(_theta[2])*sin(alpha[2]);
    A2(1,3) = a[2]*sin(_theta[2]);
    A2(2,0) = 0;
    A2(2,1) = sin(alpha[2]);
    A2(2,2) = cos(alpha[2]);
    A2(2,3) = _d[2];
    A2(3,0) = 0;
    A2(3,1) = 0;
    A2(3,2) = 0;
    A2(3,3) = 1;
    //
    A3(0,0) = cos(_theta[3]);
    A3(0,1) = -sin(_theta[3])*cos(alpha[3]);
    A3(0,2) = sin(_theta[3])*sin(alpha[3]);
    A3(0,3) = a[3]*cos(_theta[3]);
    A3(1,0) = sin(_theta[3]);
    A3(1,1) = cos(_theta[3])*cos(alpha[3]);
    A3(1,2) = -cos(_theta[3])*sin(alpha[3]);
    A3(1,3) = a[3]*sin(_theta[3]);
    A3(2,0) = 0;
    A3(2,1) = sin(alpha[3]);
    A3(2,2) = cos(alpha[3]);
    A3(2,3) = _d[3];
    A3(3,0) = 0;
    A3(3,1) = 0;
    A3(3,2) = 0;
    A3(3,3) = 1;
    //
    A4(0,0) = cos(_theta[4]);
    A4(0,1) = -sin(_theta[4])*cos(alpha[4]);
    A4(0,2) = sin(_theta[4])*sin(alpha[4]);
    A4(0,3) = a[4]*cos(_theta[4]);
    A4(1,0) = sin(_theta[4]);
    A4(1,1) = cos(_theta[4])*cos(alpha[4]);
    A4(1,2) = -cos(_theta[4])*sin(alpha[4]);
    A4(1,3) = a[4]*sin(_theta[4]);
    A4(2,0) = 0;
    A4(2,1) = sin(alpha[4]);
    A4(2,2) = cos(alpha[4]);
    A4(2,3) = _d[4];
    A4(3,0) = 0;
    A4(3,1) = 0;
    A4(3,2) = 0;
    A4(3,3) = 1;
    //
    A5(0,0) = cos(_theta[5]);
    A5(0,1) = -sin(_theta[5])*cos(alpha[5]);
    A5(0,2) = sin(_theta[5])*sin(alpha[5]);
    A5(0,3) = a[5]*cos(_theta[5]);
    A5(1,0) = sin(_theta[5]);
    A5(1,1) = cos(_theta[5])*cos(alpha[5]);
    A5(1,2) = -cos(_theta[5])*sin(alpha[5]);
    A5(1,3) = a[5]*sin(_theta[5]);
    A5(2,0) = 0;
    A5(2,1) = sin(alpha[5]);
    A5(2,2) = cos(alpha[5]);
    A5(2,3) = _d[5];
    A5(3,0) = 0;
    A5(3,1) = 0;
    A5(3,2) = 0;
    A5(3,3) = 1;
    //
    A6(0,0) = cos(_theta[6]);
    A6(0,1) = -sin(_theta[6])*cos(alpha[6]);
    A6(0,2) = sin(_theta[6])*sin(alpha[6]);
    A6(0,3) = a[6]*cos(_theta[6]);
    A6(1,0) = sin(_theta[6]);
    A6(1,1) = cos(_theta[6])*cos(alpha[6]);
    A6(1,2) = -cos(_theta[6])*sin(alpha[6]);
    A6(1,3) = a[6]*sin(_theta[6]);
    A6(2,0) = 0;
    A6(2,1) = sin(alpha[6]);
    A6(2,2) = cos(alpha[6]);
    A6(2,3) = _d[6];
    A6(3,0) = 0;
    A6(3,1) = 0;
    A6(3,2) = 0;
    A6(3,3) = 1;
    //
    A7(0,0) = cos(_theta[7]);
    A7(0,1) = -sin(_theta[7])*cos(alpha[7]);
    A7(0,2) = sin(_theta[7])*sin(alpha[7]);
    A7(0,3) = a[7]*cos(_theta[7]);
    A7(1,0) = sin(_theta[7]);
    A7(1,1) = cos(_theta[7])*cos(alpha[7]);
    A7(1,2) = -cos(_theta[7])*sin(alpha[7]);
    A7(1,3) = a[7]*sin(_theta[7]);
    A7(2,0) = 0;
    A7(2,1) = sin(alpha[7]);
    A7(2,2) = cos(alpha[7]);
    A7(2,3) = _d[7];
    A7(3,0) = 0;
    A7(3,1) = 0;
    A7(3,2) = 0;
    A7(3,3) = 1;

    /* Use forward kinematics to find the coordinate transformation describing
       the position and orientation of each frame w.r.t. world coordinate frame.
       DH convention requires that prismatic joints cause translation in the
       local z-direction and the base-transform rotates that z-axis into the
       world x-axis direction */
    T << 0, 0, 1, 0,
         0, 1, 0, 0,
         -1, 0, 0, 0,
         0, 0, 0, 1;
    T = T*A0*A1*A2*A3*A4*A5*A6*A7;

    double x, y, z, al_pha, beta, gamma;
    x = T(0,3);
    y = T(1,3);
    z = T(2,3);

    // Find the ZYZ Euler angles
    al_pha = atan2(T(1,2),T(0,2));
    double a = pow(T(0,2),2)+pow(T(1,2),2);
    double b = pow(a,.5);
    beta = atan2(b,T(2,2));
    gamma = atan2(T(2,1),-T(2,0));

    // Return the position and ZYZ Euler angles as a vector
    vector<double> Transform(6);
    Transform[0] = x;
    Transform[1] = y;
    Transform[2] = z;
    Transform[3] = al_pha;
    Transform[4] = beta;
    Transform[5] = gamma;
    return Transform;
}

/* Compute the forward kinematics for the 6 DOF arm. The function
returns a vector in the format [x, y, z, alpha, beta, gamma] where
x, y, z are the 3D coordinates & alpha, beta, gamma are the ZYZ euler
angles.*/
/*vector<double> ForKin_6DOF(const Eigen::Matrix<double, 6, 1> &Q)
{
    // Set the joint angles for the revolute joints
    theta[2] = Q(0, 0);
    theta[3] = Q(1, 0);
    theta[4] = Q(2, 0);
    theta[5] = Q(3, 0);
    theta[6] = Q(4, 0);
    theta[7] = Q(5, 0);

    Matrix4d A2, A3, A4, A5, A6, A7, T;
    //
    A2(0, 0) = cos(theta[2]);
    A2(0, 1) = -sin(theta[2])*cos(alpha[2]);
    A2(0, 2) = sin(theta[2])*sin(alpha[2]);
    A2(0, 3) = a[2] * cos(theta[2]);
    A2(1, 0) = sin(theta[2]);
    A2(1, 1) = cos(theta[2])*cos(alpha[2]);
    A2(1, 2) = -cos(theta[2])*sin(alpha[2]);
    A2(1, 3) = a[2] * sin(theta[2]);
    A2(2, 0) = 0;
    A2(2, 1) = sin(alpha[2]);
    A2(2, 2) = cos(alpha[2]);
    A2(2, 3) = d[2];
    A2(3, 0) = 0;
    A2(3, 1) = 0;
    A2(3, 2) = 0;
    A2(3, 3) = 1;
    //
    A3(0, 0) = cos(theta[3]);
    A3(0, 1) = -sin(theta[3])*cos(alpha[3]);
    A3(0, 2) = sin(theta[3])*sin(alpha[3]);
    A3(0, 3) = a[3] * cos(theta[3]);
    A3(1, 0) = sin(theta[3]);
    A3(1, 1) = cos(theta[3])*cos(alpha[3]);
    A3(1, 2) = -cos(theta[3])*sin(alpha[3]);
    A3(1, 3) = a[3] * sin(theta[3]);
    A3(2, 0) = 0;
    A3(2, 1) = sin(alpha[3]);
    A3(2, 2) = cos(alpha[3]);
    A3(2, 3) = d[3];
    A3(3, 0) = 0;
    A3(3, 1) = 0;
    A3(3, 2) = 0;
    A3(3, 3) = 1;
    //
    A4(0, 0) = cos(theta[4]);
    A4(0, 1) = -sin(theta[4])*cos(alpha[4]);
    A4(0, 2) = sin(theta[4])*sin(alpha[4]);
    A4(0, 3) = a[4] * cos(theta[4]);
    A4(1, 0) = sin(theta[4]);
    A4(1, 1) = cos(theta[4])*cos(alpha[4]);
    A4(1, 2) = -cos(theta[4])*sin(alpha[4]);
    A4(1, 3) = a[4] * sin(theta[4]);
    A4(2, 0) = 0;
    A4(2, 1) = sin(alpha[4]);
    A4(2, 2) = cos(alpha[4]);
    A4(2, 3) = d[4];
    A4(3, 0) = 0;
    A4(3, 1) = 0;
    A4(3, 2) = 0;
    A4(3, 3) = 1;
    //
    A5(0, 0) = cos(theta[5]);
    A5(0, 1) = -sin(theta[5])*cos(alpha[5]);
    A5(0, 2) = sin(theta[5])*sin(alpha[5]);
    A5(0, 3) = a[5] * cos(theta[5]);
    A5(1, 0) = sin(theta[5]);
    A5(1, 1) = cos(theta[5])*cos(alpha[5]);
    A5(1, 2) = -cos(theta[5])*sin(alpha[5]);
    A5(1, 3) = a[5] * sin(theta[5]);
    A5(2, 0) = 0;
    A5(2, 1) = sin(alpha[5]);
    A5(2, 2) = cos(alpha[5]);
    A5(2, 3) = d[5];
    A5(3, 0) = 0;
    A5(3, 1) = 0;
    A5(3, 2) = 0;
    A5(3, 3) = 1;
    //
    A6(0, 0) = cos(theta[6]);
    A6(0, 1) = -sin(theta[6])*cos(alpha[6]);
    A6(0, 2) = sin(theta[6])*sin(alpha[6]);
    A6(0, 3) = a[6] * cos(theta[6]);
    A6(1, 0) = sin(theta[6]);
    A6(1, 1) = cos(theta[6])*cos(alpha[6]);
    A6(1, 2) = -cos(theta[6])*sin(alpha[6]);
    A6(1, 3) = a[6] * sin(theta[6]);
    A6(2, 0) = 0;
    A6(2, 1) = sin(alpha[6]);
    A6(2, 2) = cos(alpha[6]);
    A6(2, 3) = d[6];
    A6(3, 0) = 0;
    A6(3, 1) = 0;
    A6(3, 2) = 0;
    A6(3, 3) = 1;
    //
    A7(0, 0) = cos(theta[7]);
    A7(0, 1) = -sin(theta[7])*cos(alpha[7]);
    A7(0, 2) = sin(theta[7])*sin(alpha[7]);
    A7(0, 3) = a[7] * cos(theta[7]);
    A7(1, 0) = sin(theta[7]);
    A7(1, 1) = cos(theta[7])*cos(alpha[7]);
    A7(1, 2) = -cos(theta[7])*sin(alpha[7]);
    A7(1, 3) = a[7] * sin(theta[7]);
    A7(2, 0) = 0;
    A7(2, 1) = sin(alpha[7]);
    A7(2, 2) = cos(alpha[7]);
    A7(2, 3) = d[7];
    A7(3, 0) = 0;
    A7(3, 1) = 0;
    A7(3, 2) = 0;
    A7(3, 3) = 1;

    T = A2*A3*A4*A5*A6*A7;

    double x, y, z, alpha, beta, gamma;
    x = T(0, 3);
    y = T(1, 3);
    z = T(2, 3);

    // Find the ZYZ Euler angles
    alpha = atan2(T(1, 2), T(0, 2));
    double a = pow(T(0, 2), 2) + pow(T(1, 2), 2);
    double b = pow(a, .5);
    beta = atan2(b, T(2, 2));
    gamma = atan2(T(2, 1), -T(2, 0));

    // Return the position and ZYZ Euler angles as a vector
    vector<double> Transform(6);
    Transform[0] = x;
    Transform[1] = y;
    Transform[2] = z;
    Transform[3] = alpha;
    Transform[4] = beta;
    Transform[5] = gamma;
    return Transform;
}*/

/* Return a 6 x N multidimensional vector of the manipulator Jacobian at the
   joint angle vector Q*/
Eigen::Matrix<double, 6, 8> Jacob(const Eigen::Matrix<double, 8, 1> &Q)
{
    double _d[8] = {0, 0, .189, 0, .27, 0, .231, 0};                             // link offset (1&2 are variable)
    double _theta[8] = {0, -pi/2, 0, 0, 0, 0, 0, 0};
    // Set the link offset for the bottom prismatic joints
    _d[0] = Q(0,0);
    _d[1] = Q(1,0);
    // Set the joint angles for the revolute joints
    _theta[2] = Q(2,0);
    _theta[3] = Q(3,0);
    _theta[4] = Q(4,0);
    _theta[5] = Q(5,0);
    _theta[6] = Q(6,0);
    _theta[7] = Q(7,0);

    Matrix4d A0, A1, A2, A3, A4, A5, A6, A7, T;
    //
    A0(0,0) = cos(_theta[0]);
    A0(0,1) = -sin(_theta[0])*cos(alpha[0]);
    A0(0,2) = sin(_theta[0])*sin(alpha[0]);
    A0(0,3) = a[0]*cos(_theta[0]);
    A0(1,0) = sin(_theta[0]);
    A0(1,1) = cos(_theta[0])*cos(alpha[0]);
    A0(1,2) = -cos(_theta[0])*sin(alpha[0]);
    A0(1,3) = a[0]*sin(_theta[0]);
    A0(2,0) = 0;
    A0(2,1) = sin(alpha[0]);
    A0(2,2) = cos(alpha[0]);
    A0(2,3) = _d[0];
    A0(3,0) = 0;
    A0(3,1) = 0;
    A0(3,2) = 0;
    A0(3,3) = 1;
    //
    A1(0,0) = cos(_theta[1]);
    A1(0,1) = -sin(_theta[1])*cos(alpha[1]);
    A1(0,2) = sin(_theta[1])*sin(alpha[1]);
    A1(0,3) = a[1]*cos(_theta[1]);
    A1(1,0) = sin(_theta[1]);
    A1(1,1) = cos(_theta[1])*cos(alpha[1]);
    A1(1,2) = -cos(_theta[1])*sin(alpha[1]);
    A1(1,3) = a[1]*sin(_theta[1]);
    A1(2,0) = 0;
    A1(2,1) = sin(alpha[1]);
    A1(2,2) = cos(alpha[1]);
    A1(2,3) = _d[1];
    A1(3,0) = 0;
    A1(3,1) = 0;
    A1(3,2) = 0;
    A1(3,3) = 1;
    //
    A2(0,0) = cos(_theta[2]);
    A2(0,1) = -sin(_theta[2])*cos(alpha[2]);
    A2(0,2) = sin(_theta[2])*sin(alpha[2]);
    A2(0,3) = a[2]*cos(_theta[2]);
    A2(1,0) = sin(_theta[2]);
    A2(1,1) = cos(_theta[2])*cos(alpha[2]);
    A2(1,2) = -cos(_theta[2])*sin(alpha[2]);
    A2(1,3) = a[2]*sin(_theta[2]);
    A2(2,0) = 0;
    A2(2,1) = sin(alpha[2]);
    A2(2,2) = cos(alpha[2]);
    A2(2,3) = _d[2];
    A2(3,0) = 0;
    A2(3,1) = 0;
    A2(3,2) = 0;
    A2(3,3) = 1;
    //
    A3(0,0) = cos(_theta[3]);
    A3(0,1) = -sin(_theta[3])*cos(alpha[3]);
    A3(0,2) = sin(_theta[3])*sin(alpha[3]);
    A3(0,3) = a[3]*cos(_theta[3]);
    A3(1,0) = sin(_theta[3]);
    A3(1,1) = cos(_theta[3])*cos(alpha[3]);
    A3(1,2) = -cos(_theta[3])*sin(alpha[3]);
    A3(1,3) = a[3]*sin(_theta[3]);
    A3(2,0) = 0;
    A3(2,1) = sin(alpha[3]);
    A3(2,2) = cos(alpha[3]);
    A3(2,3) = _d[3];
    A3(3,0) = 0;
    A3(3,1) = 0;
    A3(3,2) = 0;
    A3(3,3) = 1;
    //
    A4(0,0) = cos(_theta[4]);
    A4(0,1) = -sin(_theta[4])*cos(alpha[4]);
    A4(0,2) = sin(_theta[4])*sin(alpha[4]);
    A4(0,3) = a[4]*cos(_theta[4]);
    A4(1,0) = sin(_theta[4]);
    A4(1,1) = cos(_theta[4])*cos(alpha[4]);
    A4(1,2) = -cos(_theta[4])*sin(alpha[4]);
    A4(1,3) = a[4]*sin(_theta[4]);
    A4(2,0) = 0;
    A4(2,1) = sin(alpha[4]);
    A4(2,2) = cos(alpha[4]);
    A4(2,3) = _d[4];
    A4(3,0) = 0;
    A4(3,1) = 0;
    A4(3,2) = 0;
    A4(3,3) = 1;
    //
    A5(0,0) = cos(_theta[5]);
    A5(0,1) = -sin(_theta[5])*cos(alpha[5]);
    A5(0,2) = sin(_theta[5])*sin(alpha[5]);
    A5(0,3) = a[5]*cos(_theta[5]);
    A5(1,0) = sin(_theta[5]);
    A5(1,1) = cos(_theta[5])*cos(alpha[5]);
    A5(1,2) = -cos(_theta[5])*sin(alpha[5]);
    A5(1,3) = a[5]*sin(_theta[5]);
    A5(2,0) = 0;
    A5(2,1) = sin(alpha[5]);
    A5(2,2) = cos(alpha[5]);
    A5(2,3) = _d[5];
    A5(3,0) = 0;
    A5(3,1) = 0;
    A5(3,2) = 0;
    A5(3,3) = 1;
    //
    A6(0,0) = cos(_theta[6]);
    A6(0,1) = -sin(_theta[6])*cos(alpha[6]);
    A6(0,2) = sin(_theta[6])*sin(alpha[6]);
    A6(0,3) = a[6]*cos(_theta[6]);
    A6(1,0) = sin(_theta[6]);
    A6(1,1) = cos(_theta[6])*cos(alpha[6]);
    A6(1,2) = -cos(_theta[6])*sin(alpha[6]);
    A6(1,3) = a[6]*sin(_theta[6]);
    A6(2,0) = 0;
    A6(2,1) = sin(alpha[6]);
    A6(2,2) = cos(alpha[6]);
    A6(2,3) = _d[6];
    A6(3,0) = 0;
    A6(3,1) = 0;
    A6(3,2) = 0;
    A6(3,3) = 1;
    //
    A7(0,0) = cos(_theta[7]);
    A7(0,1) = -sin(_theta[7])*cos(alpha[7]);
    A7(0,2) = sin(_theta[7])*sin(alpha[7]);
    A7(0,3) = a[7]*cos(_theta[7]);
    A7(1,0) = sin(_theta[7]);
    A7(1,1) = cos(_theta[7])*cos(alpha[7]);
    A7(1,2) = -cos(_theta[7])*sin(alpha[7]);
    A7(1,3) = a[7]*sin(_theta[7]);
    A7(2,0) = 0;
    A7(2,1) = sin(alpha[7]);
    A7(2,2) = cos(alpha[7]);
    A7(2,3) = _d[7];
    A7(3,0) = 0;
    A7(3,1) = 0;
    A7(3,2) = 0;
    A7(3,3) = 1;

    /* Use forward kinematics to find the coordinate transformation describing
       the position and orientation of each frame w.r.t. world coordinate frame.
       DH convention requires that prismatic joints cause translation in the
       local z-direction and the base-transform rotates that z-axis into the
       world x-axis direction */
    T << 0, 0, 1, 0,
         0, 1, 0, 0,
         -1, 0, 0, 0,
         0, 0, 0, 1;
    Matrix4d T0, T1, T2, T3, T4, T5, T6, T7;
    Vector3d t0z, t1z, t2z, t3z, t4z, t5z, t6z, t7z;
    Vector3d p0z, p1z, p2z, p3z, p4z, p5z, p6z, p7z;

    T0 = T*A0;
    t0z << T0(0,2), T0(1,2), T0(2,2);
    p0z << T0(0,3), T0(1,3), T0(2,3);

    T1 = T0*A1;
    t1z << T1(0,2), T1(1,2), T1(2,2);
    p1z << T1(0,3), T1(1,3), T1(2,3);

    T2 = T1*A2;
    t2z << T2(0,2), T2(1,2), T2(2,2);
    p2z << T2(0,3), T2(1,3), T2(2,3);

    T3 = T2*A3;
    t3z << T3(0,2), T3(1,2), T3(2,2);
    p3z << T3(0,3), T3(1,3), T3(2,3);

    T4 = T3*A4;
    t4z << T4(0,2), T4(1,2), T4(2,2);
    p4z << T4(0,3), T4(1,3), T4(2,3);

    T5 = T4*A5;
    t5z << T5(0,2), T5(1,2), T5(2,2);
    p5z << T5(0,3), T5(1,3), T5(2,3);

    T6 = T5*A6;
    t6z << T6(0,2), T6(1,2), T6(2,2);
    p6z << T6(0,3), T6(1,3), T6(2,3);

    T7 = T6*A7;
    t7z << T7(0,2), T7(1,2), T7(2,2);
    p7z << T7(0,3), T7(1,3), T7(2,3);

    // Calculate the geometric Jacobian w.r.t. world reference frame
    Matrix<double, 3, 8> JP, JO;
    Matrix<double, 6, 8> J;

    // For the two prismatic joints (see p. 112 of Siciliano)
    JP.col(0) = Vector3d(0, 0, 1);	// z axis of world reference frame
    JO.col(0) = Vector3d(0, 0, 0);
    JP.col(1) = Vector3d(T0(0,2), T0(1,2), T0(2,2));
    JO.col(1) = Vector3d(0, 0, 0);

    // For the six revolute joints
    JP.col(2) = Vector3d(t1z.cross(p7z-p1z));
    JO.col(2) = Vector3d(t1z);
    JP.col(3) = Vector3d(t2z.cross(p7z-p2z));
    JO.col(3) = Vector3d(t2z);
    JP.col(4) = Vector3d(t3z.cross(p7z-p3z));
    JO.col(4) = Vector3d(t3z);
    JP.col(5) = Vector3d(t4z.cross(p7z-p4z));
    JO.col(5) = Vector3d(t4z);
    JP.col(6) = Vector3d(t5z.cross(p7z-p5z));
    JO.col(6) = Vector3d(t5z);
    JP.col(7) = Vector3d(t6z.cross(p7z-p6z));
    JO.col(7) = Vector3d(t6z);

    // Assemble the Jacobian
    J.topRows(3) = JP;
    J.bottomRows(3) = JO;

    // Modify the first column of the Jacobian to translate velocity from z-axis
    // to the world x-axis
    J.col(0) << 1, 0, 0, 0, 0, 0;

    return J;
}

void gotoPosition(const Eigen::Matrix<double, 8, 1> &Q)
{
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
    portHandler->openPort();
    portHandler->setBaudRate(BAUD_NUM);

    // Initialize groupsynchwrite instance for goal velocity and goal position
    dynamixel::GroupSyncWrite groupSyncWriteVEL(portHandler, packetHandler2, PRO_GOAL_VELOCITY, PRO_VELOCITY_LENGTH);
    dynamixel::GroupSyncWrite groupSyncWritePOS(portHandler, packetHandler2, PRO_GOAL_POSITION, PRO_POSITION_LENGTH);
    // Initialize groupsynchread instance for current position
    dynamixel::GroupSyncRead groupSyncReadPOS(portHandler, packetHandler2, PRO_PRES_POSITION, PRO_POSITION_LENGTH);

    int dxl_comm_result = COMM_TX_FAIL; // communication result
    uint8_t dxl_error = 0;

    // Only turn on torque & set the acceleration the first time the arm moves
    if (moveCounter < 1)
    {
        // Set goal acceleration and enable actuator torque
        for (int i = 0; i < 6; i++)
        {
            dxl_comm_result = packetHandler2->write4ByteTxRx(portHandler, ID_PRO[i], PRO_GOAL_ACCELERATION, 0, &dxl_error);
            dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, ID_PRO[i], PRO_TORQUE_ENABLE, 1, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
                packetHandler2->printTxRxResult(dxl_comm_result);
            else if (dxl_error != 0)
                packetHandler2->printRxPacketError(dxl_error);
            else
                printf("\n Dynamixel Pro #%d has been successfully connected \n", ID_PRO[i]);
        }
        moveCounter++;
    }

    // Set velocity to a maximum of 8000 during field studies
    int32_t vel[6] = { 2000, 2000, 2000, 2000, 2000, 2000 };
    uint8_t param_goal_velocity[4];
    unsigned short index = 0;
    // Set the velocities at low speed
    for (int j = 0; j < 6; j++)
    {
        param_goal_velocity[index++] = DXL_LOBYTE(DXL_LOWORD(vel[j]));
        param_goal_velocity[index++] = DXL_HIBYTE(DXL_LOWORD(vel[j]));
        param_goal_velocity[index++] = DXL_LOBYTE(DXL_HIWORD(vel[j]));
        param_goal_velocity[index++] = DXL_HIBYTE(DXL_HIWORD(vel[j]));
        groupSyncWriteVEL.addParam(ID_PRO[j], param_goal_velocity);
        index = 0;
    }
    // Syncwrite goal velocity
    groupSyncWriteVEL.txPacket();

    // Convert joint vector from radians to degrees
    Matrix<double, 8, 1> q;
    q(0, 0) = Q(0, 0);
    q(1, 0) = Q(1, 0);
    q(2, 0) = Q(2, 0)*(180.0 / pi);
    q(3, 0) = Q(3, 0)*(180.0 / pi);
    q(4, 0) = Q(4, 0)*(180.0 / pi);
    q(5, 0) = Q(5, 0)*(180.0 / pi);
    q(6, 0) = Q(6, 0)*(180.0 / pi);
    q(7, 0) = Q(7, 0)*(180.0 / pi);

    // Cast data types from double to int32_t
    int32_t angle[6];
    for (int k = 0; k < 6; k++)
    {
        angle[k] = (int32_t)q(k+2, 0);
        cout << endl << "Joint " << k+1 << ": " << angle[k] << endl;
    }

    // Convert angles to joint commands
    angle[0] = angle[0]*(251000/180); // Set angle to position value of H54-200
    angle[1] = angle[1]*(251000/180);
    angle[2] = angle[2]*(251000/180); // Set angle to position value of H54-100
    angle[3] = angle[3]*(251000/180);
    angle[4] = angle[4]*(151875/180); // Set angle to position value of H42-20
    angle[5] = angle[5]*(151875/180);

    uint8_t param_goal_position[4];
    index = 0;
    // Set the goal positions
    for (int k = 0; k < 6; k++)
    {
        param_goal_position[index++] = DXL_LOBYTE(DXL_LOWORD(angle[k]));
        param_goal_position[index++] = DXL_HIBYTE(DXL_LOWORD(angle[k]));
        param_goal_position[index++] = DXL_LOBYTE(DXL_HIWORD(angle[k]));
        param_goal_position[index++] = DXL_HIBYTE(DXL_HIWORD(angle[k]));
        groupSyncWritePOS.addParam(ID_PRO[k], param_goal_position);
        index = 0;
    }
    // Syncwrite goal position
    groupSyncWritePOS.txPacket();

    // Quit function when goal position reached (function checkMoving)
    // Add parameter storage for present position values
    for (int i = 0; i < 6; i++)
    {
        groupSyncReadPOS.addParam(ID_PRO[i]);
    }

    int A = 0;
    bool is_moving = true;
    short motor[6] = { 1, 1, 1, 1, 1, 1 };
    int32_t pres_position = 0;

    do
    {
       // cout << "Arm is moving" << endl;
        // Syncread present position
        dxl_comm_result = groupSyncReadPOS.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS)
            packetHandler2->printTxRxResult(dxl_comm_result);
        else if (dxl_error != 0)
            packetHandler2->printRxPacketError(dxl_error);

        for (int i = 0; i < 6; i++)
        {
            pres_position = groupSyncReadPOS.getData(ID_PRO[i], PRO_PRES_POSITION, PRO_POSITION_LENGTH);
            if ((pres_position > angle[i] - 5000) && pres_position < (angle[i] + 5000))
                motor[i] = 0;
         //   cout << endl << A;
            A++;
        }
        int total = 0;
        for (int count = 0; count < 6; count++)
            total += motor[count];
        if (total == 0)
            is_moving = false;
    } while (is_moving == true);

    portHandler->closePort();
}


// Function that determines if the drop position is over a mechanical part or violates
// the catching robot's joint limits
void checkCatchPos(double x, double y)
{
    // Inverse kinematics for a two link planar arm
    x = x - x_offset;
    y = y - y_offset;
    double r = pow(x, 2) + pow(y, 2);
    r = pow(r, .5);

    double alpha = acos((pow(link1_length, 2) + pow(link2_length, 2) - pow(r, 2)) / (2 * link1_length*link2_length));
    double beta = acos((pow(r, 2) + pow(link1_length, 2) - pow(link2_length, 2)) / (2 * link1_length*r));

    // elbow up configuration
    double catch1 = atan2(y, x) + beta;
    double catch2 = pi + alpha;

    if (catch2 > pi)
        catch2 = catch2 - (2 * pi);

    // Convert to degrees
    catch1 = catch1 * 180 / pi;
    catch2 = catch2 * 180 / pi;

    //cout << catch1 << " " << catch2 << endl;
    //_getch();


    bool y_violated = false;
    // Check if the y coordinate of the drop position is in the exclusion zone
    if (y > y_limit_L && y < y_limit_U)
        y_violated = true;

    if (catch1 <= -90 || catch1 >= 90) // Catching robot joint 1 violated
    {
        catchValid = false;
        cout << endl << "Catching robot joint 1 violation" << endl;
    }
    else if (catch2 <= -145 || catch2 >= 145) // Catching robot joint 2 violated
    {
        catchValid = false;
        cout << endl << "Catching robot joint 2 violation" << endl;
    }
    else if (x < x_limit && y_violated == true) // Drop position is in the exclusion zone
    {
        catchValid = false;
        cout << endl << "Fruit drop over exclusion zone" << endl;
    }
    else
        catchValid = true;
}

// Function that sends the catching robot to the drop position
void catchApple(double x, double y)
{
    // Inverse kinematics for a two link planar arm
    x = x - x_offset;
    y = y - y_offset;
    double r = pow(x, 2) + pow(y, 2);
    r = pow(r, .5);

    double alpha = acos((pow(link1_length, 2) + pow(link2_length, 2) - pow(r, 2)) / (2 * link1_length*link2_length));
    double beta = acos((pow(r, 2) + pow(link1_length, 2) - pow(link2_length, 2)) / (2 * link1_length*r));

    // elbow up configuration
    double catch1 = atan2(y, x) + beta;
    double catch2 = pi + alpha;

    if (catch2 > pi)
        catch2 = catch2 - (2 * pi);

    catch1 = catch1 * 180 / pi;
    catch2 = catch2 * 180 / pi;
/*
    // Catching manipulator control starts here
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME2);
    dynamixel::PacketHandler *packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
    portHandler->openPort();
    portHandler->setBaudRate(BAUD_NUM);

    // Initialize groupsynchwrite instance for goal position
    dynamixel::GroupSyncWrite groupSyncWritePOS(portHandler, packetHandler2, PRO_GOAL_POSITION, PRO_POSITION_LENGTH);

    int dxl_comm_result = COMM_TX_FAIL; // communication result
    uint8_t dxl_error = 0;

    // Only turn on torque the first time the arm moves
    if (catchCounter < 1)
    {
        // Enable actuator torque
        for (int i = 0; i < 2; i++)
        {
            dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, ID_CATCH[i], PRO_TORQUE_ENABLE, 1, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
                packetHandler2->printTxRxResult(dxl_comm_result);
            else if (dxl_error != 0)
                packetHandler2->printRxPacketError(dxl_error);
            else
                printf("\n Dynamixel Pro #%d has been successfully connected \n", ID_CATCH[i]);
        }
        catchCounter++;
    }

    // Cast data types from double to int32_t
    int32_t angle[2];
    angle[0] = (int32_t)catch1;
    angle[1] = (int32_t)catch2;

    // Convert angles to joint commands
    angle[0] = -(angle[0] * (90342 / 90)); // Invert the sign for the inner actuator
    angle[1] = angle[1] * (90342 / 90);

    uint8_t param_goal_position[4];
    int index = 0;
    // Set the goal positions
    for (int k = 0; k < 2; k++)
    {
        param_goal_position[index++] = DXL_LOBYTE(DXL_LOWORD(angle[k]));
        param_goal_position[index++] = DXL_HIBYTE(DXL_LOWORD(angle[k]));
        param_goal_position[index++] = DXL_LOBYTE(DXL_HIWORD(angle[k]));
        param_goal_position[index++] = DXL_HIBYTE(DXL_HIWORD(angle[k]));
        groupSyncWritePOS.addParam(ID_CATCH[k], param_goal_position);
        index = 0;
    }
    // Syncwrite goal position
    groupSyncWritePOS.txPacket();

    portHandler->closePort();
*/
}

// Prototype code under development - Cartesian control for the 6 DOF manipulator's approach to the fruit
//void approachFruit(const Eigen::Matrix<double, 8, 1> &qstart, const Eigen::Matrix<double, 8, 1> &qfinal)
//{
//	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
//	dynamixel::PacketHandler *packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
//	portHandler->openPort();
//	portHandler->setBaudRate(BAUD_NUM);
//
//	// Initialize groupsynchwrite instance for goal velocity
//	dynamixel::GroupSyncWrite groupSyncWriteVEL(portHandler, packetHandler2, PRO_GOAL_VELOCITY, PRO_VELOCITY_LENGTH);
//	// Initialize groupsynchread instance for current position
//	dynamixel::GroupSyncRead groupSyncReadPOS(portHandler, packetHandler2, PRO_PRES_POSITION, PRO_POSITION_LENGTH);
//
//	int dxl_comm_result = COMM_TX_FAIL; // communication result
//	uint8_t dxl_error = 0;
//	int32_t vel = 0;
//	uint8_t param_goal_velocity[4];
//	unsigned short index = 0;
//	// Set the velocities to 0
//	for (int j = 0; j < 6; j++)
//	{
//		param_goal_velocity[index++] = DXL_LOBYTE(DXL_LOWORD(vel));
//		param_goal_velocity[index++] = DXL_HIBYTE(DXL_LOWORD(vel));
//		param_goal_velocity[index++] = DXL_LOBYTE(DXL_HIWORD(vel));
//		param_goal_velocity[index++] = DXL_HIBYTE(DXL_HIWORD(vel));
//		groupSyncWriteVEL.addParam(ID_PRO[j], param_goal_velocity);
//		index = 0;
//	}
//	// Syncwrite goal velocity
//	groupSyncWriteVEL.txPacket();
//	// Place actuators in velocity mode
//	for (int i = 0; i < 6; i++)
//	{
//		dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, ID_PRO[i], PRO_TORQUE_ENABLE, 0, &dxl_error);  // Turn off torque
//		dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, ID_PRO[i], PRO_OPERATING_MODE, 1, &dxl_error); // Set velocity mode
//		dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, ID_PRO[i], PRO_TORQUE_ENABLE, 1, &dxl_error);  // Turn on torque
//		if (dxl_comm_result != COMM_SUCCESS)
//			packetHandler2->printTxRxResult(dxl_comm_result);
//		else if (dxl_error != 0)
//			packetHandler2->printRxPacketError(dxl_error);
//		else
//			printf("\n Dynamixel Pro #%d has been successfully connected \n", ID_PRO[i]);
//	}
//
//	/////// Cartesian velocity control ///////
//
//	int Time = 2; // duration of approach (sec)
//
//	// Starting joint vector for the arm
//	Matrix<double, 6, 1> qI, qF;
//	qI << qstart(2, 0), qstart(3, 0), qstart(4, 0), qstart(5, 0), qstart(6, 0), qstart(7, 0);
//	qF << qfinal(2, 0), qfinal(3, 0), qfinal(4, 0), qfinal(5, 0), qfinal(6, 0), qfinal(7, 0);
//
//	// Get the approach and final positions in the 6 DOF manipulator reference frame
//	vector<double> position_A = ForKin_6DOF(qI);
//	vector<double> position_F = ForKin_6DOF(qF);
//
//	// Determine end-effector translational velocities
//	double Vx = (position_F[0] - position_A[0]) / Time;
//	double Vy = (position_F[1] - position_A[1]) / Time;
//	double Vz = (position_F[2] - position_A[2]) / Time;
//	Vector3d Vtrans;
//	Vtrans << Vx, Vy, Vz;
//
//	// Euler angle velocities from approach to final position are zero
//	Vector3d Veuler;
//	Veuler << 0, 0, 0;
//
//	// Positive definite gain matrices
//	Matrix3d Kp, Ko;
//	Kp << 100, 0, 0,
//		  0, 100, 0,
//		  0, 0, 100;
//	Ko << 100, 0, 0,
//		  0, 100, 0,
//		  0, 0, 100;
//
//	double xD, yD, zD, eul1, eul2, eul3;
//	Vector3d Perror;
//	Vector3d orientDx, orientDy, orientDz, orientEx, orientEy, orientEz, Oerror, omega;
//	Vector3d P, O;
//	Matrix3d T, Snd1, Snd2, Snd3, Sne1, Sne2, Sne3, L;
//	Matrix<double, 6, 1> EEvel, M;
//	vector<double> A(6);
//
//	Matrix<double, 6, 6> Jt;
//	Matrix<double, 6, 8> J;
//	Matrix<double, 8, 6> Jtrans, Jpsi;
//	vector< vector<double> > JJ(6, vector<double>(8));
//	Matrix<double, 8, 1> H1;
//
//	// Weight matrix (6 x 6)
//	Matrix<double, 6, 6> w;
//	w << .01, 0, 0, 0, 0, 0,
//		0, .01, 0, 0, 0, 0,
//		0, 0, .01, 0, 0, 0,
//		0, 0, 0, .01, 0, 0,
//		0, 0, 0, 0, .01, 0,
//		0, 0, 0, 0, 0, .01;
//
//	// Identity matrix (6 x 6)
//	Matrix<double, 6, 6> eye;
//	eye << 1, 0, 0, 0, 0, 0,
//		   0, 1, 0, 0, 0, 0,
//		   0, 0, 1, 0, 0, 0,
//		   0, 0, 0, 1, 0, 0,
//		   0, 0, 0, 0, 1, 0,
//		   0, 0, 0, 0, 0, 1;
//
//	// Start the clock
//	clock_t start, time;
//	start = clock();
//	float t = 0;
//
//	Matrix<double, 6, 1> qdot;
//	// Inverse Kinematics algorithm with closed loop control
//	while (t < 2)
//	{
//		// Desired end-effector position (at time t)
//		xD = position_A(0, 0) + t * Vx;
//		yD = position_A(1, 0) + t * Vy;
//		zD = position_A(2, 0) + t * Vz;
//
//		// The current end-effector position is.......
//		for (int i = 0; i < 6; i++)
//		{
//			groupSyncReadPOS.addParam(ID_PRO[i]);
//		}
//		dxl_comm_result = groupSyncReadPOS.txRxPacket();
//		if (dxl_comm_result != COMM_SUCCESS)
//			packetHandler2->printTxRxResult(dxl_comm_result);
//		else if (dxl_error != 0)
//			packetHandler2->printRxPacketError(dxl_error);
//		int32_t pres_position[6] = { 0, 0, 0, 0, 0, 0 };
//		for (int i = 0; i < 6; i++)
//		{
//			pres_position[i] = groupSyncReadPOS.getData(ID_PRO[i], PRO_PRES_POSITION, PRO_POSITION_LENGTH);
//		}
//		// Convert encoder readings to angles
//		angle[0] = angle[0] * (251000 / 180);
//		angle[1] = angle[1] * (251000 / 180);
//		angle[2] = angle[2] * (251000 / 180);
//		angle[3] = angle[3] * (251000 / 180);
//		angle[4] = angle[4] * (151875 / 180);
//		angle[5] = angle[5] * (151875 / 180);
//
//		A = ForKin(q);
//
//		// Find the position error
//		Perror << xD - A[0], yD - A[1], zD - A[2];
//
//		// Desired end-effector orientation (at each step)
//		eul1 = start[3] + t * Valpha;
//		eul2 = start[4] + t * Vbeta;
//		eul3 = start[5] + t * Vgamma;
//
//		// Desired end-effector frame (see Siciliano eq. 2.18)
//		orientDx(0) = cos(eul1)*cos(eul2)*cos(eul3) - sin(eul1)*sin(eul3);
//		orientDx(1) = sin(eul1)*cos(eul2)*cos(eul3) + cos(eul1)*sin(eul3);
//		orientDx(2) = -sin(eul2)*cos(eul3);
//		orientDy(0) = -cos(eul1)*cos(eul2)*sin(eul3) - sin(eul1)*cos(eul3);
//		orientDy(1) = -sin(eul1)*cos(eul2)*sin(eul3) + cos(eul1)*cos(eul3);
//		orientDy(2) = sin(eul2)*sin(eul3);
//		orientDz(0) = cos(eul1)*sin(eul2);
//		orientDz(1) = sin(eul1)*sin(eul2);
//		orientDz(2) = cos(eul2);
//
//		// Current end-effector frame
//		orientEx(0) = cos(A[3])*cos(A[4])*cos(A[5]) - sin(A[3])*sin(A[5]);
//		orientEx(1) = sin(A[3])*cos(A[4])*cos(A[5]) + cos(A[3])*sin(A[5]);
//		orientEx(2) = -sin(A[4])*cos(A[5]);
//		orientEy(0) = -cos(A[3])*cos(A[4])*sin(A[5]) - sin(A[3])*cos(A[5]);
//		orientEy(1) = -sin(A[3])*cos(A[4])*sin(A[5]) + cos(A[3])*cos(A[5]);
//		orientEy(2) = sin(A[4])*sin(A[5]);
//		orientEz(0) = cos(A[3])*sin(A[4]);
//		orientEz(1) = sin(A[3])*sin(A[4]);
//		orientEz(2) = cos(A[4]);
//
//		// Find the orientation error
//		Oerror = .5*(orientEx.cross(orientDx) + orientEy.cross(orientDy) + orientEz.cross(orientDz));
//
//		// The relationship between the angular velocity and time derivative of the
//		// euler angles is (Siciliano eq. 3.64)
//		T << 0, -sin(eul1), cos(eul1)*sin(eul2),
//			 0, cos(eul1), sin(eul1)*sin(eul2),
//			 1, 0, cos(eul2);
//
//		// The desired angular velocity is
//		omega = T*Veuler;
//
//		// The desired end-effector velocity is
//		EEvel << Vx, Vy, Vz, omega(0), omega(1), omega(2);
//
//		// Find the skew symmetric matrices
//		Snd1 << 0, -orientDx(2), orientDx(1),
//			orientDx(2), 0, -orientDx(0),
//			-orientDx(1), orientDx(0), 0;
//		Snd2 << 0, -orientDy(2), orientDy(1),
//			orientDy(2), 0, -orientDy(0),
//			-orientDy(1), orientDy(0), 0;
//		Snd3 << 0, -orientDz(2), orientDz(1),
//			orientDz(2), 0, -orientDz(0),
//			-orientDz(1), orientDz(0), 0;
//		Sne1 << 0, -orientEx(2), orientEx(1),
//			orientEx(2), 0, -orientEx(0),
//			-orientEx(1), orientEx(0), 0;
//		Sne2 << 0, -orientEy(2), orientEy(1),
//			orientEy(2), 0, -orientEy(0),
//			-orientEy(1), orientEy(0), 0;
//		Sne3 << 0, -orientEz(2), orientEz(1),
//			orientEz(2), 0, -orientEz(0),
//			-orientEz(1), orientEz(0), 0;
//
//		// See pg. 139 of Siciliano et al.
//		L = -.5*(Snd1*Sne1 + Snd2*Sne2 + Snd3*Sne3);
//		P = Vtrans + Kp*Perror;
//		O = L.inverse()*(L.transpose()*omega + Ko*Oerror);
//		M.topRows(3) = P;
//		M.bottomRows(3) = O;
//
//		// Find the current Jacobian
//		Matrix<double, 6, 8> J = Jacob(q);
//
//		// Find the right pseudo-inverse (Jpsi) of J
//		Jtrans = J.transpose();
//		Jt = J*Jtrans;
//
//		// Use the inverse member function of a Partial Pivot LU matrix
//		Eigen::PartialPivLU<Matrix<double, 6, 6>> Jinv(Jt);
//		Jpsi = Jtrans*(Jinv.inverse());
//
//		// The joint velocity is
//		qdot = (Jpsi*M) + (eye - (Jpsi*J))*(w*H1);
//
//		// Calculate the joint vector by numerical integration in discrete time
//		q = q + (qdot*delt);
//	}
//
//	// Convert joint vector from radians to degrees
//	Matrix<double, 8, 1> q;
//	q(0, 0) = qstart(0, 0);
//	q(1, 0) = qstart(1, 0);
//	q(2, 0) = qstart(2, 0)*(180.0 / pi);
//	q(3, 0) = qstart(3, 0)*(180.0 / pi);
//	q(4, 0) = qstart(4, 0)*(180.0 / pi);
//	q(5, 0) = qstart(5, 0)*(180.0 / pi);
//	q(6, 0) = qstart(6, 0)*(180.0 / pi);
//	q(7, 0) = qstart(7, 0)*(180.0 / pi);
//
//	// Cast data types from double to int32_t
//	int32_t angle[6];
//	for (int k = 0; k < 6; k++)
//	{
//		angle[k] = (int32_t)q(k + 2, 0);
//		cout << endl << "Joint " << k + 1 << ": " << angle[k] << endl;
//	}
//	// Convert angles to joint commands
//	angle[0] = angle[0] * (251000 / 180); // Set angle to position value of H54-200
//	angle[1] = angle[1] * (251000 / 180);
//	angle[2] = angle[2] * (251000 / 180); // Set angle to position value of H54-100
//	angle[3] = angle[3] * (251000 / 180);
//	angle[4] = angle[4] * (151875 / 180); // Set angle to position value of H42-20
//	angle[5] = angle[5] * (151875 / 180);
//
//	// Quit function when goal position reached (function checkMoving)
//	// Add parameter storage for present position values
//	for (int i = 0; i < 6; i++)
//	{
//		groupSyncReadPOS.addParam(ID_PRO[i]);
//	}
//
//	bool is_moving = true;
//	short motor[6] = { 1, 1, 1, 1, 1, 1 };
//	int32_t pres_position = 0;
//	do
//	{
//		//// Try increasing velocity
//		//vel = vel - 100;
//		//index = 0;
//		//for (int j = 0; j < 6; j++)
//		//{
//		//	param_goal_velocity[index++] = DXL_LOBYTE(DXL_LOWORD(vel));
//		//	param_goal_velocity[index++] = DXL_HIBYTE(DXL_LOWORD(vel));
//		//	param_goal_velocity[index++] = DXL_LOBYTE(DXL_HIWORD(vel));
//		//	param_goal_velocity[index++] = DXL_HIBYTE(DXL_HIWORD(vel));
//		//	groupSyncWriteVEL.addParam(ID_PRO[j], param_goal_velocity);
//		//	index = 0;
//		//}
//		//// Syncwrite goal velocity
//		//groupSyncWriteVEL.txPacket();
//
//		cout << "Arm is moving" << endl;
//
//		// Syncread present position
//		dxl_comm_result = groupSyncReadPOS.txRxPacket();
//		if (dxl_comm_result != COMM_SUCCESS)
//			packetHandler2->printTxRxResult(dxl_comm_result);
//		else if (dxl_error != 0)
//			packetHandler2->printRxPacketError(dxl_error);
//		for (int i = 0; i < 6; i++)
//		{
//			pres_position = groupSyncReadPOS.getData(ID_PRO[i], PRO_PRES_POSITION, PRO_POSITION_LENGTH);
//			if ((pres_position > angle[i] - 2500) && pres_position < (angle[i] + 2500))
//				motor[i] = 0;
//		}
//
//		int total = 0;
//		for (int count = 0; count < 6; count++)
//			total += motor[count];
//		if (total == 0)
//			is_moving = false;
//	} while (is_moving == true);
//
//	portHandler->closePort();
//}

