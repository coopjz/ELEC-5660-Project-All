#define BACKWARD_HAS_DW 1
#include "backward.hpp"

#include <ekf_filter.h>
namespace backward {

  	backward::SignalHandling sh;
  
} // namespace backward

namespace ekf_imu_vision {
EKFImuVision::EKFImuVision(/* args */) {}
double clamp_euler_angle(double theta)
{
    double result = atan2 (sin(theta),cos(theta)); 
    return result;
}

static inline double normalize_angle(double angle) {
	const double result = fmod(angle + M_PI, 2.0 * M_PI);
	if (result <= 0.0)
	  return result + M_PI;
	return result - M_PI;
  }
static inline void normalize_state(Vec21 &state) {
	state(3) = normalize_angle(state(3));
	state(4) = normalize_angle(state(4));
	state(5) = normalize_angle(state(5));
  
	state(18) = normalize_angle(state(18));
	state(19) = normalize_angle(state(19));
	state(20) = normalize_angle(state(20));
  }

EKFImuVision::~EKFImuVision() {}
void EKFImuVision::init(ros::NodeHandle& nh) {
	node_ = nh;

	/* ---------- parameter ---------- */
	Qt_.setZero();
	Rt1_.setZero();
	Rt2_.setZero();

	// addition and removel of augmented state
	
	// TODO
	// set M_a_ and M_r_
	M_a_.setZero(); //add
	M_r_.setZero(); //remove

	M_r_.block<15,15>(0,0) = Mat(15, 15)::Identity();
	M_a_.block<15,15>(0,0) = Mat(15, 15)::Identity();
	M_a_.block<6,6>(15,0) = Mat(6, 6)::Identity();



	for (int i = 0; i < 3; i++) {
		/* process noise */
		node_.param("aug_ekf/ng", Qt_(i, i), -1.0);
		node_.param("aug_ekf/na", Qt_(i + 3, i + 3), -1.0);
		node_.param("aug_ekf/nbg", Qt_(i + 6, i + 6), -1.0);
		node_.param("aug_ekf/nba", Qt_(i + 9, i + 9), -1.0);
		node_.param("aug_ekf/pnp_p", Rt1_(i, i), -1.0);
		node_.param("aug_ekf/pnp_q", Rt1_(i + 3, i + 3), -1.0);
		node_.param("aug_ekf/vo_pos", Rt2_(i, i), -1.0);
		node_.param("aug_ekf/vo_rot", Rt2_(i + 3, i + 3), -1.0);
	}

	init_        = false;
	node_.param("aug_ekf/simple_bag", simple_bag_, true);

	for(int i = 0; i < 4; i++)
		latest_idx[i] = 0;

	/* ---------- subscribe and publish ---------- */
	imu_sub_ =
		node_.subscribe<sensor_msgs::Imu>("imu", 100, &EKFImuVision::imuCallback, this);
	pnp_sub_     = node_.subscribe<nav_msgs::Odometry>("tag_odom", 20, &EKFImuVision::PnPCallback, this);
	// opti_tf_sub_ = node_.subscribe<geometry_msgs::PointStamped>("opti_tf_odom", 10,
																// &EKFImuVision::opticalCallback, this);
	stereo_sub_  = node_.subscribe<stereo_vo::relative_pose>("/vo/Relative_pose", 20,
															&EKFImuVision::stereoVOCallback, this);
	fuse_odom_pub_ = node_.advertise<nav_msgs::Odometry>("ekf_fused_odom", 10);
	path_pub_         = node_.advertise<nav_msgs::Path>("/aug_ekf/Path", 100);
	
	tag_path_pub_ = node_.advertise<nav_msgs::Path>("/aug_ekf/tag_path", 100);
	ros::Duration(0.5).sleep();

	ROS_INFO("Start ekf.");
	}

void EKFImuVision::PnPCallback(const nav_msgs::OdometryConstPtr& msg) {

	// TODO
	// construct a new state using the absolute measurement from marker PnP and process the new state

	bool pnp_lost = fabs(msg->pose.pose.position.x) < 1e-4 && fabs(msg->pose.pose.position.y) < 1e-4 &&
		fabs(msg->pose.pose.position.z) < 1e-4;
	if (pnp_lost) return;

	Mat3x3 R_w_b, R_c_w, R_b_c,R_w_t;
	Vec3 t_w_b, t_c_w, t_b_c;

	if (this->simple_bag_)
	{
		ROS_INFO("simple bag");
		R_w_b = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
			.toRotationMatrix();
			t_w_b[0] = msg->pose.pose.position.x;
			t_w_b[1] = msg->pose.pose.position.y;
			t_w_b[2] = msg->pose.pose.position.z;
			
	}
	else
	{
		R_c_w = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
			.toRotationMatrix();
		t_c_w[0] = msg->pose.pose.position.x;
		t_c_w[1] = msg->pose.pose.position.y;
		t_c_w[2] = msg->pose.pose.position.z;

		R_b_c << 1, 0, 0,
			    0, -1, 0,
				0, 0, -1;
		t_b_c << 0.07, -0.02, 0.01;
		R_w_t <<  0,  1,  0,
             1,  0,  0,
             0,  0, -1;
		R_w_b = R_w_t*R_c_w.transpose() * R_b_c.transpose();
		t_w_b = R_w_t*(-R_c_w.transpose()* R_b_c.transpose() * t_b_c - R_c_w.transpose() * t_c_w);

	}
	AugState         new_state;

	new_state.mean = Vec21::Zero();
	new_state.covariance = Mat21x21::Zero();
	new_state.time_stamp = msg->header.stamp;
	new_state.type = pnp;
	new_state.ut.head(3)  =     t_w_b;
	new_state.ut.segment(3, 3) = rotation2Euler(R_w_b);


		geometry_msgs::PoseStamped tag_pose;
	tag_pose.header.stamp = new_state.time_stamp;
	tag_pose.header.frame_id = "world";
	tag_pose.pose.position.x = t_w_b[0];
	tag_pose.pose.position.y = t_w_b[1];
	tag_pose.pose.position.z = t_w_b[2];
	tag_path_.header.stamp = new_state.time_stamp;
	tag_path_.header.frame_id = "world";
	tag_path_.poses.push_back(tag_pose);
	tag_path_pub_.publish(tag_path_);
	if (!processNewState(new_state, false)) {
		return;
	}

}

// void EKFImuVision::PnPCallback(const nav_msgs::OdometryConstPtr &msg) {

// 	// DONE:
// 	// construct a new state using the absolute measurement from marker PnP and process the new state
  
// 	// ROS_ERROR("----------------PnPCallback--------------------");
  
// 	bool pnp_lost = fabs(msg->pose.pose.position.x) < 1e-4 && fabs(msg->pose.pose.position.y) < 1e-4 &&
// 		fabs(msg->pose.pose.position.z) < 1e-4;
// 	if (pnp_lost) return;
  
// 	Mat3x3 R_w_b = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
// 									  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
// 					   .toRotationMatrix();
// 	Vec3 t_w_b;
// 	t_w_b[0] = msg->pose.pose.position.x;
// 	t_w_b[1] = msg->pose.pose.position.y;
// 	t_w_b[2] = msg->pose.pose.position.z;
  
// 	AugState new_state;
  
// 	new_state.mean = Vec21::Zero();
// 	new_state.covariance = Mat21x21::Zero();
// 	new_state.time_stamp = msg->header.stamp;
// 	new_state.type = pnp;
// 	new_state.ut.head(3)  =     t_w_b;
// 	new_state.ut.segment(3, 3) = rotation2Euler(R_w_b);
	

// 	geometry_msgs::PoseStamped tag_pose;
// 	tag_pose.header.stamp = new_state.time_stamp;
// 	tag_pose.header.frame_id = "world";
// 	tag_pose.pose.position.x = t_w_b[0];
// 	tag_pose.pose.position.y = t_w_b[1];
// 	tag_pose.pose.position.z = t_w_b[2];
// 	tag_path_.header.stamp = new_state.time_stamp;
// 	tag_path_.header.frame_id = "world";
// 	tag_path_.poses.push_back(tag_pose);
// 	tag_path_pub_.publish(tag_path_);

// 	if (!processNewState(new_state, false)) {
// 	  return;
// 	}

  
//   }

void EKFImuVision::stereoVOCallback(const stereo_vo::relative_poseConstPtr& msg) {
	if (latest_idx[imu]==0) {
		ROS_WARN("imu not received");
		return;
	}

	// TODO
	// label the previous keyframe
	// construct a new state using the relative measurement from VO and process the new state
	Mat3x3 R_k_c;
	Vec3 t_k_c;
	R_k_c = Eigen::Quaterniond(msg->relative_pose.orientation.w, msg->relative_pose.orientation.x,
		msg->relative_pose.orientation.y, msg->relative_pose.orientation.z)
		.toRotationMatrix();
	t_k_c[0] = msg->relative_pose.position.x;
	t_k_c[1] = msg->relative_pose.position.y;
	t_k_c[2] = msg->relative_pose.position.z;

	AugState new_state;
	new_state.time_stamp = msg->header.stamp;
	new_state.type = vo;
	new_state.key_frame_time_stamp = msg->key_stamp;
	new_state.ut.head(3)  =     t_k_c;
	new_state.ut.segment(3, 3) = rotation2Euler(R_k_c);
	new_state.mean.setZero();
	new_state.covariance.setZero();
	bool change_keyframe = false;
	change_keyframe = latest_idx[keyframe] >= 0 &&
	(aug_state_hist_.at(latest_idx[keyframe]).time_stamp != new_state.key_frame_time_stamp) && latest_idx[keyframe] < aug_state_hist_.size();
	
	if( change_keyframe)
	{
		ROS_WARN("========Change frame========");
	}
	if (!processNewState(new_state, change_keyframe)) {
		return;
	}
}

void EKFImuVision::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

	// TODO
	// construct a new state using the IMU input and process the new state
	Vec3 acc, gyro;

	gyro[0] = imu_msg->angular_velocity.x;
	gyro[1] = imu_msg->angular_velocity.y;
	gyro[2] = imu_msg->angular_velocity.z;
	acc[0] = imu_msg->linear_acceleration.x;
	acc[1] = imu_msg->linear_acceleration.y;
	acc[2] = imu_msg->linear_acceleration.z;


	// gyro[0] =  imu_msg->angular_velocity.z;   // Z-forward → X (roll)
	// gyro[1] = -imu_msg->angular_velocity.x;   // X-right → -Y (pitch)
	// gyro[2] = -imu_msg->angular_velocity.y;   // Y-down → -Z (yaw)
	// acc[0] =  imu_msg->linear_acceleration.z;   // Z-forward → X
	// acc[1] = -imu_msg->linear_acceleration.x;   // X-right → -Y
	// acc[2] = -imu_msg->linear_acceleration.y;   // Y-down → -Z



	AugState new_state;
	new_state.time_stamp = imu_msg->header.stamp;
	new_state.type = imu;
	new_state.ut.head(3)  =     gyro;
	new_state.ut.segment(3, 3) = acc;
	new_state.mean.setZero();
	new_state.covariance.setZero();
	if (!processNewState(new_state, false)) {
		return;
	}
}


const double DT_MAX =1.0;
void EKFImuVision::predictIMU(AugState& cur_state, AugState& prev_state, Vec6 ut) {

	// TODO
	// predict by IMU inputs

	// 1. get time interval
	double dt = (cur_state.time_stamp - prev_state.time_stamp).toSec();
	// if (dt > DT_MAX) {
    //     return;
    // }

	Vec15 mean_prev, mean_pred;
	Vec12 noise;  // set the noise to zero for test, real world should be set to the noise
	Mat15x15 At, Ft;
	Mat15x12 Ut, Vt;

	noise.setZero();
	mean_prev = prev_state.mean.head(15);
	mean_pred = mean_prev + dt *modelF(mean_prev, cur_state.ut, noise);

	At = jacobiFx(mean_prev, cur_state.ut, noise);
	Ft = At * dt + Mat15x15::Identity();
	Ut = jacobiFn(mean_prev, cur_state.ut, noise);
	Vt = dt * Ut;

	cur_state.mean = prev_state.mean;
	cur_state.mean.head(15) = mean_pred;

	Eigen::MatrixXd Cov_prev = prev_state.covariance.block<15, 15>(0, 0);
	cur_state.covariance.block<15, 15>(0, 0) =
		Ft * Cov_prev * Ft.transpose() + Vt * Qt_ * Vt.transpose();
	cur_state.covariance.block<15, 6>(0, 15) = 
		Ft * prev_state.covariance.block<15, 6>(0, 15);
	cur_state.covariance.block<6, 15>(15, 0) =
		prev_state.covariance.block<6, 15>(15, 0) * Ft.transpose();
	cur_state.covariance.block<6, 6>(15, 15) =
		prev_state.covariance.block<6, 6>(15, 15);

}

void EKFImuVision::updatePnP(AugState& cur_state, AugState& prev_state) {

	// TODO
	// update by marker PnP measurements
	Vec6 zt = Vec6::Zero(), vt= Vec6::Zero();
	Mat21x6 Kt = Mat21x6::Zero();
	// Mat6x21 Kt_1 = Mat6x21::Zero();
	Mat6x21 Ht = Mat6x21::Zero();
	// prev_mean
	Vec15 prev_mean = prev_state.mean.head(15);
	Ht.block<6, 15>(0, 0) =jacobiG1x(prev_mean, vt);
	Mat6x6 Wt = jacobiG1v(prev_mean, vt);
	Kt = prev_state.covariance * Ht.transpose() * (Ht * prev_state.covariance * Ht.transpose() + Wt * Rt1_ * Wt.transpose()).inverse();
	zt = cur_state.ut;
	Vec6 yt = zt - modelG1(prev_mean, vt);
	//normalize
	normalizeAngle(yt(3));
	normalizeAngle(yt(4));
	normalizeAngle(yt(5));
	// update
	cur_state.mean = prev_state.mean + Kt * yt;
	cur_state.covariance = prev_state.covariance - Kt * Ht * prev_state.covariance;

	// visualize
	// ROS_INFO("remember to visualize");
}


void EKFImuVision::updateVO(AugState& cur_state, AugState& prev_state) {

	// TODO

	// update by relative pose measurements
	Vec6 zt = Vec6::Zero(), vt= Vec6::Zero();
	Mat21x6 Kt = Mat21x6::Zero();
	// Mat6x21 Kt_1 = Mat6x21::Zero();
	Mat6x21 Ht = Mat6x21::Zero();
	// prev_mean prev_state.mean;
	Ht =jacobiG2x(prev_state.mean, vt);
	Mat6x6 Wt = jacobiG2v(prev_state.mean, vt);
	Kt = prev_state.covariance * Ht.transpose() * (Ht * prev_state.covariance * Ht.transpose() + Wt * Rt2_ * Wt.transpose()).inverse();
	zt = cur_state.ut;
	Vec6 yt = zt - modelG2(prev_state.mean, vt);
	// Vec6 yt = zt - Ht * prev_state.mean;
	//normalize
	normalizeAngle(yt(3));
	normalizeAngle(yt(4));
	normalizeAngle(yt(5));
	// update
	cur_state.mean = prev_state.mean + Kt * yt;
	cur_state.covariance = prev_state.covariance - Kt * Ht * prev_state.covariance;
		
	normalize_state(cur_state.mean);
	// // visualize
	// ROS_INFO("remember to visualize");
	// if (cur_state.type == keyframe) {
	// 	changeAugmentedState(cur_state);
	//   }

}

void EKFImuVision::changeAugmentedState(AugState& state) {
//   ROS_ERROR("----------------change keyframe------------------------");

	// TODO
	// change augmented state
	
	Vec(21) mean = state.mean;
	Mat(21, 21) covariance = state.covariance;
	// change the augmented state to the new state

	state.mean =M_a_* M_r_* mean;
	state.covariance = M_a_* M_r_* covariance * M_r_.transpose() * M_a_.transpose();
	// change the augmented state keyframe
	// state.key_frame_time_stamp = state.time_stamp;

}

bool EKFImuVision::processNewState(AugState& new_state, bool change_keyframe) {
	ROS_INFO_STREAM("latest idx:"<<latest_idx[imu]<<","<<latest_idx[pnp]<<','<<latest_idx[vo]<<','<<latest_idx[keyframe]);

	// TODO
	// process the new state
	// step 1: insert the new state into the queue and get the iterator to start to propagate (be careful about the change of key frame).
	// step 2: try to initialize the filter if it is not initialized.
	// step 3: repropagate from the iterator you extracted.
	// step 4: remove the old states.
	// step 5: publish the latest fused odom
	auto start_it = insertNewState(new_state);
	if(change_keyframe)
	{
		auto keyframe_it = start_it;
		while (keyframe_it != aug_state_hist_.begin() && keyframe_it->time_stamp != new_state.key_frame_time_stamp)
		{
			keyframe_it--;
		}
		if (keyframe_it->time_stamp.toSec() !=0){
			// changeAugmentedState(*keyframe_it);
			latest_idx[keyframe] = std::distance(aug_state_hist_.begin(),keyframe_it);
			aug_state_hist_[latest_idx[keyframe]].type = keyframe;
			// keyframe_it;
			// changeAugmentedState(*keyframe_it);
			start_it = keyframe_it;
			ROS_INFO("Keyframe changed.");
			// if(init_)
			// 	updateVO(*start_it, *(start_it-1));
		}
	}
	// init_ = true;
	if (!init_)
	{
		init_ = initFilter();
		if (init_) {
			ROS_INFO("EKF initialized.");
		}
		else{
			ROS_ERROR("EKF waiting for initialization.");
			return false;
		}
	}
	else
	{
		repropagate(start_it, init_);
		removeOldState();
		publishFusedOdom();
	}
	return true;


}




deque<AugState>::iterator EKFImuVision::insertNewState(AugState& new_state){
  
	ros::Time time = new_state.time_stamp;
	deque<AugState>::iterator state_it;

	// TODO
	// insert the new state to the queue
	// update the latest_idx of the type of the new state
	// return the iterator point to the new state in the queue 
	// auto it = std::upper_bound(
    //     aug_state_hist_.begin(),
    //     aug_state_hist_.end(),
    //     new_state.time_stamp,
    //     [](const ros::Time& t, const AugState& s) {
    //         return t < s.time_stamp;
    //     }
    // );
	state_it = aug_state_hist_.end();
	while (state_it != aug_state_hist_.begin() && time<=(*(state_it-1)).time_stamp){
		state_it--;
	  }

	state_it = aug_state_hist_.insert(state_it, new_state);
	size_t pos = std::distance(aug_state_hist_.begin(), state_it);
	latest_idx[new_state.type] = pos;


	for (int i = 0; i < 4; i++) {
		if (latest_idx[i] > pos) {
			latest_idx[i]++;
		}
	}

	return state_it;

}

void EKFImuVision::repropagate(deque<AugState>::iterator& new_input_it, bool& init) {


	// TODO
	// repropagate along the queue from the new input according to the type of the inputs / measurements
	// remember to consider the initialization case  
	auto it = new_input_it;
	while(it != aug_state_hist_.end()) {
		AugState& cur_state = *it;
		AugState& prev_state = *(it - 1);
		if (it == aug_state_hist_.begin())
		{
			it++; 
			continue;
		}

		if (cur_state.type == imu) {
			predictIMU(cur_state, prev_state, cur_state.ut);
		} 
		else if (cur_state.type == pnp) {
			updatePnP(cur_state, prev_state);

		} 
		else if (cur_state.type == vo) {
			updateVO(cur_state, prev_state);
		} 
		else if (cur_state.type == keyframe) {
			updateVO(cur_state, prev_state);
			changeAugmentedState(cur_state);
		}

		it++;
	}
}


void EKFImuVision::removeOldState() {

	// TODO
	// remove the unnecessary old states to prevent the queue from becoming too long

	unsigned int remove_idx = min(min(latest_idx[imu], latest_idx[pnp]), latest_idx[keyframe]);

	aug_state_hist_.erase(aug_state_hist_.begin(), aug_state_hist_.begin() + remove_idx);

	for(int i = 0; i < 4; i++){
		latest_idx[i] -= remove_idx;
	}

  
}

void EKFImuVision::publishFusedOdom() {
	AugState last_state = aug_state_hist_.back();

	double phi, theta, psi;
	phi   = last_state.mean(3);
	theta = last_state.mean(4);
	psi   = last_state.mean(5);

	if (last_state.mean.head(3).norm() > 20) {
		ROS_ERROR_STREAM("error state: " << last_state.mean.head(3).transpose());
		return;
	}

	// using the zxy euler angle
	Eigen::Quaterniond q = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());
	nav_msgs::Odometry odom;
	odom.header.frame_id = "world";
	odom.header.stamp    = last_state.time_stamp;

	odom.pose.pose.position.x = last_state.mean(0);
	odom.pose.pose.position.y = last_state.mean(1);
	odom.pose.pose.position.z = last_state.mean(2);

	odom.pose.pose.orientation.w = q.w();
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();

	odom.twist.twist.linear.x = last_state.mean(6);
	odom.twist.twist.linear.y = last_state.mean(7);
	odom.twist.twist.linear.z = last_state.mean(8);


	fuse_odom_pub_.publish(odom);

	geometry_msgs::PoseStamped path_pose;
	path_pose.header.frame_id = path_.header.frame_id = "world";
	path_pose.pose.position.x                         = last_state.mean(0);
	path_pose.pose.position.y                         = last_state.mean(1);
	path_pose.pose.position.z                         = last_state.mean(2);
	path_.poses.push_back(path_pose);
	path_pub_.publish(path_);
	// publish the tag path
	geometry_msgs::PoseStamped tag_path_pose;
	tag_path_pose.header.frame_id = "world";
}


bool EKFImuVision::initFilter() {
	ROS_INFO_STREAM("latest idx:"<<latest_idx[imu]<<","<<latest_idx[pnp]<<','<<latest_idx[vo]<<','<<latest_idx[keyframe]);
	// TODO
	// Initial the filter when a keyframe after marker PnP measurements is available
	// if (latest_idx[keyframe] > latest_idx[pnp]) {
	// 	ROS_INFO("find keyframe");
	// 	// find the keyframe
	// 	auto it = aug_state_hist_.begin() + latest_idx[keyframe];
	// 	if (it->type == keyframe) {
	// 		// find the PnP
	// 		auto it_pnp = aug_state_hist_.begin() + latest_idx[pnp];
	// 		if (it_pnp->type == pnp) {
	// 			if (initUsingPnP(it_pnp))
	// 				changeAugmentedState(*it_pnp);
	// 			repropagate(it, init_);
	// 			return true;
	// 		}
	// 	}
	// }
	if(latest_idx[imu]==0 && latest_idx[vo]==0)
		return false;
	else
	{

		// use current initial position and orientation (set as zero pose )
		AugState start_state;
		start_state.mean = Vec21::Zero(21);
		start_state.covariance = Mat21x21::Identity(21, 21);
		start_state.time_stamp = aug_state_hist_[latest_idx[imu]].time_stamp;
		start_state.type = pnp;
		start_state.ut.head(3) = Vec3::Zero();
		start_state.ut.segment(3, 3) = Vec3::Zero();
		auto it = insertNewState(start_state);
		
		initUsingPnP(it);
	}
	

	
	
	return true;
}


bool EKFImuVision::initUsingPnP(deque<AugState>::iterator start_it) {

	// TODO
	// Initialize the absolute pose of the state in the queue using marker PnP measurement.
	// This is only step 1 of the initialization.
	if (start_it->type!=pnp){
		ROS_ERROR("it is not PnP state");
		return false;
	}
	AugState start_state;
	start_state.mean = Vec21::Zero(21);
	start_state.covariance =Mat21x21::Identity(21,21);
	
	updatePnP(*start_it, start_state);
	ROS_INFO("init using PnP");
	return true;
}


Vec3 EKFImuVision::rotation2Euler(const Mat3x3& R) {
	double phi   = asin(R(2, 1));
	double theta = atan2(-R(2, 0), R(2, 2));
	double psi   = atan2(-R(0, 1), R(1, 1));
	return Vec3(phi, theta, psi);
}

}  // namespace ekf_imu_vision


