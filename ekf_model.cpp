#include <ekf_model.h>
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/preprocessor/arithmetic/inc.hpp>  
#include <boost/preprocessor/cat.hpp>

namespace ekf_imu_vision {
#define Concat(a,b) a##b

#define ASSIGN_X(z, i, _) BOOST_PP_CAT(x, BOOST_PP_INC(i)) = x(i);
#define ASSIGN_N(z, i, _) BOOST_PP_CAT(n, BOOST_PP_INC(i)) = n(i);
#define ASSIGN_U(z, i, _) BOOST_PP_CAT(u, BOOST_PP_INC(i)) = u(i);

double normalizeAngle(double angle) 
{
    // if (angle > M_PI) {
    //   angle -= 2 * M_PI;
    // } else if (angle < -M_PI) {
    //   angle += 2 * M_PI;
    // }
    // return angle;
    return atan2(sin(angle),cos(angle));
}

// rotation matrix to euler angles

Vec15 modelF(const Vec15& x, const Vec6& u, const Vec12& n) {
  
  // TODO
  // return the model xdot = f(x,u,n)

  	Vec15 xdot;

	//  1: x0:2 ~ x, y, z """
	//  2: x3:5 ~ phi theta psi """
	//  3: x6:8 ~ vx vy vz """
	//  4: x9:11 ~ bgx bgy bgz """
	//  5: x12:14 ~  bax bay baz """

	// u0:2 wmx, wmy, wmz
	// u3:5 amx, amy, amz

	// n0:2 ngx, ngy, ngz
	// n3:5 nax, nay, naz
	// n6:8 nbgx, nbgy, nbgz
	// n9:11 nbax, nbay, nbaz

	Vec3 b_gyro = x.block<3,1>(9,0);
    Vec3 n_gyro = n.block<3,1>(0,0);
    Vec3 b_acc = x.block<3,1>(12,0);
    Vec3 n_acc = n.block<3,1>(3,0);
	Vec3 gyro = Vec3(u(0), u(1), u(2));
	Vec3 acc = Vec3(u(3), u(4), u(5));
	Vec3 g = Vec3(0, 0, 9.81); // gravity vector
	Mat3x3 G; // same as cal_jacobian.py
	Mat3x3 Rot;

	double phi, theta, psi;

	phi = x(3);
	theta = x(4);
	psi = x(5);

	G <<cos(theta), 0, -cos(phi)*sin(theta), 
        0,          1,             sin(phi),
        sin(theta), 0,  cos(phi)*cos(theta);

	Rot <<  cos(psi)*cos(theta)-sin(psi)*sin(phi)*sin(theta),   -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(psi)*sin(phi),
		cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta),    cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),
									-cos(phi)*sin(theta),             sin(phi),                              cos(phi)*cos(theta);

	Mat3x3 G_inv = G.inverse();	
	
	xdot.block<3,1>(0,0) = x.block<3,1>(6,0);
    xdot.block<3,1>(3,0) = G_inv * (gyro -b_gyro - n_gyro);
    xdot.block<3,1>(6,0) = Rot * (acc -b_acc - n_acc)+ g;
    xdot.block<3,1>(9,0) = n.segment(6,3);
    xdot.block<3,1>(12,0) = n.segment(9,3);

	xdot(3) = normalizeAngle(xdot(3));
	xdot(4) = normalizeAngle(xdot(4));
	xdot(5) = normalizeAngle(xdot(5));

	return xdot;
}

Mat15x15 jacobiFx(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the derivative wrt original state df/dx

	Mat15x15 At;
	double x1, x2, x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15;
    //n1-n12
    double n1, n2, n3, n4, n5, n6,n7,n8,n9,n10,n11,n12;
    // u1-u6
    double u1, u2, u3, u4, u5, u6;

	BOOST_PP_REPEAT(15, ASSIGN_X, _)
    BOOST_PP_REPEAT(12, ASSIGN_N, _)
	BOOST_PP_REPEAT(6, ASSIGN_U, _)
	
	At <<  0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
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

	return At;
}

Mat15x12 jacobiFn(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the derivative wrt noise df/dn

	Mat15x12 Ut;
	double x4,x5,x6;

	x4 = x(3);
	x5 = x(4);
	x6 = x(5);
	Ut <<  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
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
		
	return Ut;
}

/* ============================== model of PnP ============================== */

Vec6 modelG1(const Vec15& x, const Vec6& v) {
  
	// TODO
	// return the model g(x,v), where x = x_origin the down looking camera
	// observation model of PnP

	Vec6 zt;

	zt=x.segment(0,6);

	return zt;
}

Mat6x15 jacobiG1x(const Vec15& x, const Vec6& v) {

	// TODO
	// return the derivative wrt original state dz/dx, where x = x_origin

	Mat6x15 Ct;
	Ct = Mat6x15::Zero();
	Ct.block<6,6>(0,0) = Mat6x6::Identity();

	return Ct;
}

Mat6x6 jacobiG1v(const Vec15& x, const Vec6& v) {

	// TODO;
	// return the derivative wrt noise dz/dv

	Mat6x6 I6;
		
	I6.setIdentity();
	return I6;
}

/* ============================== model of stereo VO relative pose ============================== */

Vec6 modelG2(const Vec21& x, const Vec6& v) {

	// TODO
		// return the model g(x,v), where x = (x_origin, x_augmented)

	// rotation matrix R_wb world to body
	// rotation matrix R_wk world to keyframe

	// euler angles to rotation matrix
	Eigen::Quaterniond q_wb, q_wk;
	// x(3:5) = phi, theta, psi
	// rotate in zxy order
	q_wb = Eigen::AngleAxisd(x(5), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(x(3), Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxisd(x(4), Eigen::Vector3d::UnitY());
	q_wk = Eigen::AngleAxisd(x(20), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(x(18), Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxisd(x(19), Eigen::Vector3d::UnitY());
		
	
	q_wb.normalize();
	q_wk.normalize();
	Eigen::Matrix3d R_wb = q_wb.toRotationMatrix();
	Eigen::Matrix3d R_wk = q_wk.toRotationMatrix();

	Eigen::Vector3d p_b = x.block<3,1>(0,0);
	Eigen::Vector3d p_k = x.block<3,1>(15,0);

	// relative pose
	Vec6 zt;
	zt.block<3,1>(0,0) = R_wk.transpose() * (p_b - p_k);
	// rotation in euler angles
	Eigen::Matrix3d R_kb = R_wk.transpose() * R_wb;
	// zxy order
	double phi, theta, psi; // corresponding to x(3:5) roll pitch yaw
	// yaw angle
	psi = atan2(-R_kb(0, 1), R_kb(1, 1));
	// pitch angle
	theta = atan2(-R_kb(2, 0), R_kb(2, 2));
	// roll angle
	phi = asin(R_kb(2, 1));

	zt(3) = phi;
	zt(4) = theta;
	zt(5) = psi;
	return zt;
}

Mat6x21 jacobiG2x(const Vec21& x, const Vec6& v) {

  // TODO
  // return the derivative wrt original state dz/dx, where x = (x_origin, x_augmented)

	Mat6x21 Ct;
	double x1, x2, x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21;
	//n1-n12

	BOOST_PP_REPEAT(21, ASSIGN_X, _)

	Ct << -sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21), sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20), -sin(x20)*cos(x19), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sin(x19)*sin(x20)*sin(x21) - cos(x20)*cos(x21), -sin(x19)*sin(x20)*cos(x21) - sin(x21)*cos(x20), sin(x20)*cos(x19), -(x1 - x16)*sin(x20)*sin(x21)*cos(x19) + (-x17 + x2)*sin(x20)*cos(x19)*cos(x21) + (-x18 + x3)*sin(x19)*sin(x20), (x1 - x16)*(-sin(x19)*sin(x21)*cos(x20) - sin(x20)*cos(x21)) + (-x17 + x2)*(sin(x19)*cos(x20)*cos(x21) - sin(x20)*sin(x21)) - (-x18 + x3)*cos(x19)*cos(x20), (x1 - x16)*(-sin(x19)*sin(x20)*cos(x21) - sin(x21)*cos(x20)) + (-x17 + x2)*(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21)), 
		-sin(x21)*cos(x19), cos(x19)*cos(x21), sin(x19), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sin(x21)*cos(x19), -cos(x19)*cos(x21), -sin(x19), (x1 - x16)*sin(x19)*sin(x21) - (-x17 + x2)*sin(x19)*cos(x21) + (-x18 + x3)*cos(x19), 0, -(x1 - x16)*cos(x19)*cos(x21) - (-x17 + x2)*sin(x21)*cos(x19), 
		sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21), -sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21), cos(x19)*cos(x20), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -sin(x19)*sin(x21)*cos(x20) - sin(x20)*cos(x21), sin(x19)*cos(x20)*cos(x21) - sin(x20)*sin(x21), -cos(x19)*cos(x20), (x1 - x16)*sin(x21)*cos(x19)*cos(x20) - (-x17 + x2)*cos(x19)*cos(x20)*cos(x21) - (-x18 + x3)*sin(x19)*cos(x20), (x1 - x16)*(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21)) + (-x17 + x2)*(sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20)) - (-x18 + x3)*sin(x20)*cos(x19), (x1 - x16)*(sin(x19)*cos(x20)*cos(x21) - sin(x20)*sin(x21)) + (-x17 + x2)*(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21)), 
		0, 0, 0, ((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*sin(x4)*sin(x6) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*sin(x4)*cos(x6) + cos(x19)*cos(x20)*cos(x4))/sqrt(1 - pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*sin(x6)*cos(x4) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*cos(x4)*cos(x6) + sin(x4)*cos(x19)*cos(x20), 2)), 0, (-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*cos(x4)*cos(x6) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*sin(x6)*cos(x4))/sqrt(1 - pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*sin(x6)*cos(x4) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*cos(x4)*cos(x6) + sin(x4)*cos(x19)*cos(x20), 2)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (-sin(x19)*sin(x4)*cos(x20) - sin(x21)*sin(x6)*cos(x19)*cos(x20)*cos(x4) - cos(x19)*cos(x20)*cos(x21)*cos(x4)*cos(x6))/sqrt(1 - pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*sin(x6)*cos(x4) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*cos(x4)*cos(x6) + sin(x4)*cos(x19)*cos(x20), 2)), (-(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) + (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) - sin(x20)*sin(x4)*cos(x19))/sqrt(1 - pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*sin(x6)*cos(x4) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*cos(x4)*cos(x6) + sin(x4)*cos(x19)*cos(x20), 2)), ((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*cos(x4)*cos(x6) - (sin(x19)*cos(x20)*cos(x21) - sin(x20)*sin(x21))*sin(x6)*cos(x4))/sqrt(1 - pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*sin(x6)*cos(x4) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*cos(x4)*cos(x6) + sin(x4)*cos(x19)*cos(x20), 2)), 
		0, 0, 0, ((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) - sin(x5)*cos(x19)*cos(x20)*cos(x4))*((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*sin(x6)*cos(x4)*cos(x5) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*cos(x4)*cos(x5)*cos(x6) - sin(x4)*cos(x19)*cos(x20)*cos(x5))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)) + ((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5))*((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*sin(x5)*sin(x6)*cos(x4) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*sin(x5)*cos(x4)*cos(x6) - sin(x4)*sin(x5)*cos(x19)*cos(x20))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)), pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) - sin(x5)*cos(x19)*cos(x20)*cos(x4), 2)/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)) + (-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x6)*cos(x5) - sin(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*cos(x5)*cos(x6) - sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5))*((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)), (-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*cos(x6) - sin(x6)*cos(x5)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)))*((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)) + ((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*cos(x5)*cos(x6) - sin(x5)*sin(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)))*((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) - sin(x5)*cos(x19)*cos(x20)*cos(x4))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) - sin(x5)*cos(x19)*cos(x20)*cos(x4))*((sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6))*sin(x21)*cos(x19)*cos(x20) - (-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6))*cos(x19)*cos(x20)*cos(x21) - sin(x19)*cos(x20)*cos(x4)*cos(x5))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)) + ((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5))*(-(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6))*sin(x21)*cos(x19)*cos(x20) + (sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5))*cos(x19)*cos(x20)*cos(x21) - sin(x19)*sin(x5)*cos(x20)*cos(x4))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)), (-(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) - sin(x20)*sin(x5)*cos(x19)*cos(x4))*((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)) + ((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) - sin(x20)*cos(x19)*cos(x4)*cos(x5))*((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) - sin(x5)*cos(x19)*cos(x20)*cos(x4))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)), (-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) - (sin(x19)*cos(x20)*cos(x21) - sin(x20)*sin(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)))*((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)) + ((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + (sin(x19)*cos(x20)*cos(x21) - sin(x20)*sin(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)))*((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) - sin(x5)*cos(x19)*cos(x20)*cos(x4))/(pow(-(sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(-sin(x4)*sin(x5)*sin(x6) + cos(x5)*cos(x6)) - (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(sin(x4)*sin(x5)*cos(x6) + sin(x6)*cos(x5)) + sin(x5)*cos(x19)*cos(x20)*cos(x4), 2) + pow((sin(x19)*sin(x21)*cos(x20) + sin(x20)*cos(x21))*(sin(x4)*sin(x6)*cos(x5) + sin(x5)*cos(x6)) + (-sin(x19)*cos(x20)*cos(x21) + sin(x20)*sin(x21))*(-sin(x4)*cos(x5)*cos(x6) + sin(x5)*sin(x6)) + cos(x19)*cos(x20)*cos(x4)*cos(x5), 2)), 
		0, 0, 0, (sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6))*(-(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x4)*sin(x6) + (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*sin(x4)*cos(x6) + sin(x20)*cos(x19)*cos(x4))/(pow(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6), 2) + pow((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) + sin(x20)*sin(x4)*cos(x19), 2)) + (sin(x19)*cos(x4) - sin(x21)*sin(x4)*sin(x6)*cos(x19) - sin(x4)*cos(x19)*cos(x21)*cos(x6))*(-(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) + (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) - sin(x20)*sin(x4)*cos(x19))/(pow(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6), 2) + pow((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) + sin(x20)*sin(x4)*cos(x19), 2)), 0, ((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*cos(x4)*cos(x6) + (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*sin(x6)*cos(x4))*(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6))/(pow(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6), 2) + pow((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) + sin(x20)*sin(x4)*cos(x19), 2)) + (sin(x21)*cos(x19)*cos(x4)*cos(x6) - sin(x6)*cos(x19)*cos(x21)*cos(x4))*(-(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) + (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) - sin(x20)*sin(x4)*cos(x19))/(pow(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6), 2) + pow((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) + sin(x20)*sin(x4)*cos(x19), 2)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6))*(-sin(x19)*sin(x20)*sin(x4) - sin(x20)*sin(x21)*sin(x6)*cos(x19)*cos(x4) - sin(x20)*cos(x19)*cos(x21)*cos(x4)*cos(x6))/(pow(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6), 2) + pow((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) + sin(x20)*sin(x4)*cos(x19), 2)) + (-(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) + (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) - sin(x20)*sin(x4)*cos(x19))*(-sin(x19)*sin(x21)*sin(x6)*cos(x4) - sin(x19)*cos(x21)*cos(x4)*cos(x6) + sin(x4)*cos(x19))/(pow(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6), 2) + pow((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) + sin(x20)*sin(x4)*cos(x19), 2)), (sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6))*((-sin(x19)*sin(x21)*cos(x20) - sin(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*cos(x20)*cos(x21) - sin(x20)*sin(x21))*cos(x4)*cos(x6) + sin(x4)*cos(x19)*cos(x20))/(pow(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6), 2) + pow((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) + sin(x20)*sin(x4)*cos(x19), 2)), (-(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*cos(x4)*cos(x6) + (-sin(x19)*sin(x20)*cos(x21) - sin(x21)*cos(x20))*sin(x6)*cos(x4))*(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6))/(pow(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6), 2) + pow((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) + sin(x20)*sin(x4)*cos(x19), 2)) + (-sin(x21)*cos(x19)*cos(x4)*cos(x6) + sin(x6)*cos(x19)*cos(x21)*cos(x4))*(-(-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) + (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) - sin(x20)*sin(x4)*cos(x19))/(pow(sin(x19)*sin(x4) + sin(x21)*sin(x6)*cos(x19)*cos(x4) + cos(x19)*cos(x21)*cos(x4)*cos(x6), 2) + pow((-sin(x19)*sin(x20)*sin(x21) + cos(x20)*cos(x21))*sin(x6)*cos(x4) - (sin(x19)*sin(x20)*cos(x21) + sin(x21)*cos(x20))*cos(x4)*cos(x6) + sin(x20)*sin(x4)*cos(x19), 2))
	;
	return Ct;
	}

Mat6x6 jacobiG2v(const Vec21& x, const Vec6& v) {
  
  // TODO
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;
  I6.setIdentity();
  return I6;
}

}  // namespace ekf_imu_vision