#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/preprocessor/arithmetic/inc.hpp>  
#include <boost/preprocessor/cat.hpp>
using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
ros::Publisher odom_ref_pub;
ros::Publisher cov_matrix_pub;
MatrixXd Q = MatrixXd::Identity(12, 12); 
MatrixXd Rt = MatrixXd::Identity(6,6);
// X vector position (x, y, z, roll, pitch, yaw, vx, vy, vz, bias_gyro_x, bias_gyro_y, bias_gyro_z, bias_acc_x, bias_acc_y, bias_acc_z)
VectorXd X = VectorXd::Zero(15); 
MatrixXd Cov = MatrixXd::Identity(15, 15); // covariance matrix
VectorXd N = VectorXd::Zero(12); // process noise
Vector3d g = Vector3d(0, 0, 9.81); // gravity vector

// preprocess the arguments
#define Concat(a,b) a##b
// covariance matrix
double normalizeAngle(double angle) {
    // if (angle > M_PI) {
    //   angle -= 2 * M_PI;
    // } else if (angle < -M_PI) {
    //   angle += 2 * M_PI;
    // }
    // return angle;
    return atan2(sin(angle),cos(angle));
  }


// Quaterniond eulerToQuat()
double last_t = 0;
const double DT_MAX =1.0;
#define ASSIGN_X(z, i, _) BOOST_PP_CAT(x, BOOST_PP_INC(i)) = X(i);
#define ASSIGN_N(z, i, _) BOOST_PP_CAT(n, BOOST_PP_INC(i)) = N(i);
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double t = msg->header.stamp.toSec();
    double dt = t - last_t;
    last_t = t;
    if (dt > DT_MAX) {
        return;
    }
    // observation of the IMU
    Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    //your code for propagation

    VectorXd f = VectorXd::Zero(15);
    Matrix3d G; // same as cal_jacobian.py
    Matrix3d Rot;
    double phi, theta, psi;
    Vector3d b_gyro = X.block<3,1>(9,0);
    Vector3d n_gyro = N.block<3,1>(0,0);
    Vector3d b_acc = X.block<3,1>(12,0);
    Vector3d n_acc = N.block<3,1>(3,0);

    Eigen::MatrixXd A_j = Eigen::MatrixXd::Zero(15, 15);
    Eigen::MatrixXd U_j = Eigen::MatrixXd::Zero(15, 12);
    Eigen::Matrix3d block_G_inv_d, block_R_d;
    //x1-x15
    double x1, x2, x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15;
    //n1-n12
    double n1, n2, n3, n4, n5, n6,n7,n8,n9,n10,n11,n12;
    // u1-u6
    double u1, u2, u3, u4, u5, u6;
    //assign values use concatenation

    BOOST_PP_REPEAT(15, ASSIGN_X, _)
    BOOST_PP_REPEAT(12, ASSIGN_N, _)

    u1 = gyro(0); u2 = gyro(1); u3 = gyro(2);
    u4 = acc(0); u5 = acc(1); u6 = acc(2);

    /*step 1: mean propagation for imu*/
    //mu_t = mu_t-1 + dt * f(mu_t-1, u_t-1,0)
    phi = X(3);
    theta = X(4);
    psi = X(5);
    G <<cos(theta), 0, -cos(phi)*sin(theta), 
        0,          1,             sin(phi),
        sin(theta), 0,  cos(phi)*cos(theta);
    
    Rot <<  cos(psi)*cos(theta)-sin(psi)*sin(phi)*sin(theta),   -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(psi)*sin(phi),
            cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta),    cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),
                                        -cos(phi)*sin(theta),             sin(phi),                              cos(phi)*cos(theta);

    Matrix3d G_inv = G.inverse();
    f.block<3,1>(0,0) = X.block<3,1>(6,0);
    f.block<3,1>(3,0) = G_inv * (gyro -b_gyro - n_gyro);
    f.block<3,1>(6,0) = Rot * (acc -b_acc - n_acc)+ g;
    f.block<3,1>(9,0) = Vector3d::Zero();
    f.block<3,1>(12,0) = Vector3d::Zero();
    X = X + dt * f;
    X(3) = normalizeAngle(X(3));
    X(4) = normalizeAngle(X(4));
    X(5) = normalizeAngle(X(5));
    /* Step 2: Coverience propagation*/
    // A_j is the jacobian in jacobian.txt
    // block_G_inv_d << 0, -(-n1 + u1 - x10)*sin(x5)/(sin(x5)**2 + cos(x5)**2) + (-n3 + u3 - x12)*cos(x5)/(sin(x5)**2 + cos(x5)**2), 0,
    //                  0, 
    // Eigen::MatrixXd block_A_3_9 <<0, 0, 0, 0, -(-n1 + u1 - x10)*sin(x5)/(sin(x5)*sin(x5) + cos(x5)*cos(x5)) + (-n3 + u3 - x12)*cos(x5)/(sin(x5)*sin(x5) + cos(x5)*cos(x5)), 0, 0, 0, 0, -cos(x5)/(sin(x5)*sin(x5) + cos(x5)*cos(x5)), 0, -sin(x5)/(sin(x5)*sin(x5) + cos(x5)*cos(x5)), 0, 0, 0, 
    // 0, 0, 0, (sin(x4)*sin(x5)*sin(x5) + sin(x4)*cos(x5)*cos(x5))*(-n1 + u1 - x10)*sin(x4)*sin(x5)/pow(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5),2) - (sin(x4)*sin(x5)*sin(x5) + sin(x4)*cos(x5)*cos(x5))*(-n3 + u3 - x12)*sin(x4)*cos(x5)/pow(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5),2) + (-n1 + u1 - x10)*sin(x5)*cos(x4)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)) - (-n3 + u3 - x12)*cos(x4)*cos(x5)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)), (-n1 + u1 - x10)*sin(x4)*cos(x5)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)) + (-n3 + u3 - x12)*sin(x4)*sin(x5)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)), 0, 0, 0, 0, -sin(x4)*sin(x5)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)), -1, sin(x4)*cos(x5)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)), 0, 0, 0,
    // 0, 0, 0, -(sin(x4)*sin(x5)*sin(x5) + sin(x4)*cos(x5)*cos(x5))*(-n1 + u1 - x10)*sin(x5)/pow(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5),2) + (sin(x4)*sin(x5)*sin(x5) + sin(x4)*cos(x5)*cos(x5))*(-n3 + u3 - x12)*cos(x5)/pow(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5),2), -(-n1 + u1 - x10)*cos(x5)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)) - (-n3 + u3 - x12)*sin(x5)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)), 0, 0, 0, 0, sin(x5)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)), 0, -cos(x5)/(sin(x5)*sin(x5)*cos(x4) + cos(x4)*cos(x5)*cos(x5)), 0, 0, 0,
    // 0, 0, 0, -(-n4 + u4 - x13)*sin(x5)*sin(x6)*cos(x4) + (-n5 + u5 - x14)*sin(x4)*sin(x6) + (-n6 + u6 - x15)*sin(x6)*cos(x4)*cos(x5), (-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6))*(-n6 + u6 - x15) + (-sin(x4)*sin(x6)*cos(x5) - sin(x5)*cos(x6))*(-n4 + u4 - x13), (-sin(x4)*sin(x5)*cos(x6) - sin(x6)*cos(x5))*(-n4 + u4 - x13) + (sin(x4)*cos(x5)*cos(x6) - sin(x5)*sin(x6))*(-n6 + u6 - x15) - (-n5 + u5 - x14)*cos(x4)*cos(x6), 0, 0, 0, 0, 0, 0, sin(x4)*sin(x5)*sin(x6) - cos(x5)*cos(x6), sin(x6)*cos(x4), -sin(x4)*sin(x6)*cos(x5) - sin(x5)*cos(x6),
    // 0, 0, 0, (-n4 + u4 - x13)*sin(x5)*cos(x4)*cos(x6) - (-n5 + u5 - x14)*sin(x4)*cos(x6) - (-n6 + u6 - x15)*cos(x4)*cos(x5)*cos(x6), (sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5))*(-n6 + u6 - x15) + (sin(x4)*cos(x5)*cos(x6) - sin(x5)*sin(x6))*(-n4 + u4 - x13), (-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6))*(-n4 + u4 - x13) + (sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6))*(-n6 + u6 - x15) - (-n5 + u5 - x14)*sin(x6)*cos(x4), 0, 0, 0, 0, 0, 0, -sin(x4)*sin(x5)*cos(x6) - sin(x6)*cos(x5), -cos(x4)*cos(x6), sin(x4)*cos(x5)*cos(x6) - sin(x5)*sin(x6),
    // 0, 0, 0, (-n4 + u4 - x13)*sin(x4)*sin(x5) + (-n5 + u5 - x14)*cos(x4) - (-n6 + u6 - x15)*sin(x4)*cos(x5), -(-n4 + u4 - x13)*cos(x4)*cos(x5) - (-n6 + u6 - x15)*sin(x5)*cos(x4), 0, 0, 0, 0, 0, 0, 0, sin(x5)*cos(x4), -sin(x4), -cos(x4)*cos(x5),
    // A_j.block<3,3>(0,6) = Matrix3d::Identity();
    // A_j.block<15,6>(3,0) = block_A_3_9;
    A_j <<  0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, -(-n1 + u1 - x10)*sin(x5)/(pow(sin(x5), 2) + pow(cos(x5), 2)) + (-n3 + u3 - x12)*cos(x5)/(pow(sin(x5), 2) + pow(cos(x5), 2)), 0, 0, 0, 0, -cos(x5)/(pow(sin(x5), 2) + pow(cos(x5), 2)), 0, -sin(x5)/(pow(sin(x5), 2) + pow(cos(x5), 2)), 0, 0, 0,
            0, 0, 0, (sin(x4)*pow(sin(x5), 2) + sin(x4)*pow(cos(x5), 2))*(-n1 + u1 - x10)*sin(x4)*sin(x5)/pow(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2), 2) - (sin(x4)*pow(sin(x5), 2) + sin(x4)*pow(cos(x5), 2))*(-n3 + u3 - x12)*sin(x4)*cos(x5)/pow(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2), 2) + (-n1 + u1 - x10)*sin(x5)*cos(x4)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)) - (-n3 + u3 - x12)*cos(x4)*cos(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), (-n1 + u1 - x10)*sin(x4)*cos(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)) + (-n3 + u3 - x12)*sin(x4)*sin(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), 0, 0, 0, 0, -sin(x4)*sin(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), -1, sin(x4)*cos(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), 0, 0, 0,
            0, 0, 0, -(sin(x4)*pow(sin(x5), 2) + sin(x4)*pow(cos(x5), 2))*(-n1 + u1 - x10)*sin(x5)/pow(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2), 2) + (sin(x4)*pow(sin(x5), 2) + sin(x4)*pow(cos(x5), 2))*(-n3 + u3 - x12)*cos(x5)/pow(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2), 2), -(-n1 + u1 - x10)*cos(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)) - (-n3 + u3 - x12)*sin(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), 0, 0, 0, 0, sin(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), 0, -cos(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), 0, 0, 0,
            0, 0, 0, -(-n4 + u4 - x13)*sin(x5)*sin(x6)*cos(x4) + (-n5 + u5 - x14)*sin(x4)*sin(x6) + (-n6 + u6 - x15)*sin(x6)*cos(x4)*cos(x5), (-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6))*(-n6 + u6 - x15) + (-sin(x4)*sin(x6)*cos(x5) - sin(x5)*cos(x6))*(-n4 + u4 - x13), (-sin(x4)*sin(x5)*cos(x6) - sin(x6)*cos(x5))*(-n4 + u4 - x13) + (sin(x4)*cos(x5)*cos(x6) - sin(x5)*sin(x6))*(-n6 + u6 - x15) - (-n5 + u5 - x14)*cos(x4)*cos(x6), 0, 0, 0, 0, 0, 0, sin(x4)*sin(x5)*sin(x6) - cos(x5)*cos(x6), sin(x6)*cos(x4), -sin(x4)*sin(x6)*cos(x5) - sin(x5)*cos(x6),
            0, 0, 0, (-n4 + u4 - x13)*sin(x5)*cos(x4)*cos(x6) - (-n5 + u5 - x14)*sin(x4)*cos(x6) - (-n6 + u6 - x15)*cos(x4)*cos(x5)*cos(x6), (sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5))*(-n6 + u6 - x15) + (sin(x4)*cos(x5)*cos(x6) - sin(x5)*sin(x6))*(-n4 + u4 - x13), (-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6))*(-n4 + u4 - x13) + (sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6))*(-n6 + u6 - x15) - (-n5 + u5 - x14)*sin(x6)*cos(x4), 0, 0, 0, 0, 0, 0, -sin(x4)*sin(x5)*cos(x6) - sin(x6)*cos(x5), -cos(x4)*cos(x6), sin(x4)*cos(x5)*cos(x6) - sin(x5)*sin(x6),
            0, 0, 0, (-n4 + u4 - x13)*sin(x4)*sin(x5) + (-n5 + u5 - x14)*cos(x4) - (-n6 + u6 - x15)*sin(x4)*cos(x5), -(-n4 + u4 - x13)*cos(x4)*cos(x5) - (-n6 + u6 - x15)*sin(x5)*cos(x4), 0, 0, 0, 0, 0, 0, 0, sin(x5)*cos(x4), -sin(x4), -cos(x4)*cos(x5),
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    // U_j is the jacobian in jacobian.txt
    U_j <<  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            -cos(x5)/(pow(sin(x5), 2) + pow(cos(x5), 2)), 0, -sin(x5)/(pow(sin(x5), 2) + pow(cos(x5), 2)), 0, 0, 0, 0, 0, 0, 0, 0, 0,
            -sin(x4)*sin(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), -1, sin(x4)*cos(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), 0, 0, 0, 0, 0, 0, 0, 0, 0,
            sin(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), 0, -cos(x5)/(pow(sin(x5), 2)*cos(x4) + cos(x4)*pow(cos(x5), 2)), 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, sin(x4)*sin(x5)*sin(x6) - cos(x5)*cos(x6), sin(x6)*cos(x4), -sin(x4)*sin(x6)*cos(x5) - sin(x5)*cos(x6), 0, 0, 0, 0, 0, 0,
            0, 0, 0, -sin(x4)*sin(x5)*cos(x6) - sin(x6)*cos(x5), -cos(x4)*cos(x6), sin(x4)*cos(x5)*cos(x6) - sin(x5)*sin(x6), 0, 0, 0, 0, 0, 0,
            0, 0, 0, sin(x5)*cos(x4), -sin(x4), -cos(x4)*cos(x5), 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;


    MatrixXd Ft = MatrixXd::Identity(15,15) + dt * A_j;
    MatrixXd Vt = dt * U_j;
    Cov = Ft * Cov * Ft.transpose() + Vt * Q * Vt.transpose();
 
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;


    Eigen::Vector3d t_cw, t_ic, t_wi;
    Eigen::Matrix3d R_cw, R_ic, R_wi;
    Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    t_cw << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z; 
    R_cw << q.toRotationMatrix();
    t_ic << 0.05, 0.05, 0;
    R_ic << 1,  0,  0,
            0, -1,  0,
            0,  0, -1;
    printf("pose_ori_x: %f\n", msg->pose.pose.orientation.x);
    R_wi = R_cw.transpose() * R_ic;
    // t_wi = -R_cw.transpose()*R_ic.transpose()*t_ic - R_cw.transpose()*t_cw;
    t_wi = R_cw.transpose() * (t_ic - t_cw);
    // rotation matrix to euler angles
    // double phi, theta, psi;
    // phi = atan2(R_wi(2,1), R_wi(2,2));
    // theta = atan2(-R_wi(2,0), sqrt(pow(R_wi(2,1), 2) + pow(R_wi(2,2), 2)));
    // psi = atan2(R_wi(1,0), R_wi(0,0));
    double phi = asin(R_wi(2, 1));
    // printf("phi: %f\n", phi);
    double theta = atan2(-R_wi(2, 0) / cos(phi), R_wi(2, 2) / cos(phi));
    double psi = atan2(-R_wi(0, 1) / cos(phi), R_wi(1, 1) / cos(phi));

    Eigen::VectorXd zt = VectorXd::Zero(6);     // observation vector
    Eigen::MatrixXd Kt = MatrixXd::Zero(6, 15); // Kalman gain
    Eigen::MatrixXd Kt_1 = MatrixXd::Zero(6, 15); // Kalman gain
    Eigen::MatrixXd Ht = MatrixXd::Zero(6, 15);//Ct for linearization
    Ht.block<6,6>(0,0) = MatrixXd::Identity(6,6);  // Position and orientation

    // zt = [x, y, z, roll, pitch, yaw]
    zt << t_wi(0), t_wi(1), t_wi(2), phi, theta, psi;

    Kt_1 = Cov*Ht.transpose()*(Ht*Cov*Ht.transpose() + Rt).inverse();
    // solve the linear equation Ax = b to get Kt
    Eigen::MatrixXd S = Ht * Cov * Ht.transpose() + Rt;
    Eigen::MatrixXd PHt = Cov * Ht.transpose();          // Cross-covariance
    Kt = S.ldlt().solve(PHt.transpose()).transpose(); // Kalman gain
    
    //normalize the angles
    // zt(3) = normalizeAngle(zt(3));
    // zt(4) = normalizeAngle(zt(4));
    // zt(5) = normalizeAngle(zt(5));

    // innovation terms
    Eigen::VectorXd yt = zt - Ht * X; // innovation
    yt(3) = normalizeAngle(yt(3));
    yt(4) = normalizeAngle(yt(4));
    yt(5) = normalizeAngle(yt(5));
    printf("yt3: %f\n",yt(3));
    // update the X vector
    X = X + Kt_1 * yt;

    printf("X3: %f\n",X(3));
    // update the covariance matrix
    Cov = Cov - Kt_1 * Ht * Cov;
    //normalize the angles
    // X(3) = normalizeAngle(X(3));
    // X(4) = normalizeAngle(X(4));
    // X(5) = normalizeAngle(X(5));

    // visualize Cov matrix 
    Eigen::MatrixXd abs_cov = Cov.cwiseAbs();

    // Step 2: Normalize to [0, 255]
    double min_val = abs_cov.minCoeff();
    double max_val = abs_cov.maxCoeff();
    if (max_val == min_val) max_val += 1e-9; // Avoid division by zero
    Eigen::MatrixXd norm_cov = ((abs_cov.array() - min_val) / (max_val - min_val)) * 255;

    // Step 3: Convert to OpenCV matrix (grayscale)
    cv::Mat cv_image(abs_cov.rows(), abs_cov.cols(), CV_8UC1);
    for (int i = 0; i < norm_cov.rows(); ++i) {
        for (int j = 0; j < norm_cov.cols(); ++j) {
            cv_image.at<uchar>(i, j) = static_cast<uchar>(norm_cov(i, j));
        }
    }

    // Step 4: Convert to ROS Image message and publish
    cv_bridge::CvImage img_msg;
    img_msg.encoding = "mono8"; // Grayscale
    img_msg.image = cv_image;
    cov_matrix_pub.publish(img_msg.toImageMsg());

    // publish the odometry
    // euler
    
    Quaterniond q_ekf;
    Matrix3d Rot_ekf;
    double phi_ekf = X(3);
    double theta_ekf = X(4);
    double psi_ekf = X(5);
    Rot_ekf <<  cos(psi_ekf)*cos(theta_ekf)-sin(psi_ekf)*sin(phi_ekf)*sin(theta_ekf),   -cos(phi_ekf)*sin(psi_ekf), cos(psi_ekf)*sin(theta_ekf)+cos(theta_ekf)*sin(psi_ekf)*sin(phi_ekf),
                cos(theta_ekf)*sin(psi_ekf)+cos(psi_ekf)*sin(phi_ekf)*sin(theta_ekf),    cos(phi_ekf)*cos(psi_ekf), sin(psi_ekf)*sin(theta_ekf)-cos(psi_ekf)*cos(theta_ekf)*sin(phi_ekf),
                                                     -cos(phi_ekf)*sin(theta_ekf),             sin(phi_ekf),                              cos(phi_ekf)*cos(theta_ekf);

    // q_ekf = Quaterniond(AngleAxisd(X(3), Vector3d::UnitX()) * AngleAxisd(X(4), Vector3d::UnitY()) * AngleAxisd(X(5), Vector3d::UnitZ()));
    // q_ekf.normalize();
    q_ekf = Quaterniond(Rot_ekf);

    
    nav_msgs::Odometry odom;
    odom.header.stamp =  msg->header.stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = X(0);
    odom.pose.pose.position.y = X(1);
    odom.pose.pose.position.z = X(2);
    odom.pose.pose.orientation.x = q_ekf.x();
    odom.pose.pose.orientation.y = q_ekf.y();
    odom.pose.pose.orientation.z = q_ekf.z();
    odom.pose.pose.orientation.w = q_ekf.w();
    odom.twist.twist.linear.x = X(6);
    odom.twist.twist.linear.y = X(7);
    odom.twist.twist.linear.z = X(8);

    odom_pub.publish(odom);

    // publish the reference odometry
    Quaterniond q_ref;
    q_ref = Quaterniond(R_wi);
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp =  msg->header.stamp;
    odom_ref.header.frame_id = "world";
    odom_ref.child_frame_id = "base_link";
    odom_ref.pose.pose.position.x = t_wi(0);
    odom_ref.pose.pose.position.y = t_wi(1);
    odom_ref.pose.pose.position.z = t_wi(2);
    odom_ref.pose.pose.orientation.x = q_ref.x();
    odom_ref.pose.pose.orientation.y = q_ref.y();
    odom_ref.pose.pose.orientation.z = q_ref.z();
    odom_ref.pose.pose.orientation.w = q_ref.w();
    odom_ref_pub.publish(odom_ref);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    odom_ref_pub = n.advertise<nav_msgs::Odometry>("ekf_odom_ref", 100);
    cov_matrix_pub = n.advertise<sensor_msgs::Image>("cov_matrix", 100);
    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // You should also tune these parameters
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.1 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}


