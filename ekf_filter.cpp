#include <ekf_filter.h>

// #include "backward.hpp"
// namespace backward {

//   	backward::SignalHandling sh;
  
// } // namespace backward

namespace ekf_imu_vision {
EKFImuVision::EKFImuVision(/* args */) {}

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
		node_.subscribe<sensor_msgs::Imu>("/dji_sdk_1/dji_sdk/imu", 100, &EKFImuVision::imuCallback, this);
	pnp_sub_     = node_.subscribe<nav_msgs::Odometry>("tag_odom", 10, &EKFImuVision::PnPCallback, this);
	// opti_tf_sub_ = node_.subscribe<geometry_msgs::PointStamped>("opti_tf_odom", 10,
																// &EKFImuVision::opticalCallback, this);
	stereo_sub_  = node_.subscribe<stereo_vo::relative_pose>("/vo/Relative_pose", 10,
															&EKFImuVision::stereoVOCallback, this);
	fuse_odom_pub_ = node_.advertise<nav_msgs::Odometry>("ekf_fused_odom", 10);
	path_pub_         = node_.advertise<nav_msgs::Path>("/aug_ekf/Path", 100);

	ros::Duration(0.5).sleep();

	ROS_INFO("Start ekf.");
	}

void EKFImuVision::PnPCallback(const nav_msgs::OdometryConstPtr& msg) {

	// TODO
	// construct a new state using the absolute measurement from marker PnP and process the new state

	bool pnp_lost = fabs(msg->pose.pose.position.x) < 1e-4 && fabs(msg->pose.pose.position.y) < 1e-4 &&
		fabs(msg->pose.pose.position.z) < 1e-4;
	if (pnp_lost) return;

	Mat3x3 R_w_b, R_c_w, R_b_c;
	Vec3 t_w_b, t_c_w, t_b_c;

	if (this->simple_bag_)
	{

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
		R_w_b = R_c_w.transpose() * R_b_c;
		t_w_b = R_c_w.transpose() * t_b_c - R_c_w.transpose() * t_c_w;

	}
	AugState         new_state;

	new_state.time_stamp = msg->header.stamp;
	new_state.type = pnp;
	new_state.ut.head(3)  =     t_w_b;
	new_state.ut.segment(3, 3) = rotation2Euler(R_w_b);

	new_state.mean.setZero();
	new_state.covariance.setZero();

	if (!processNewState(new_state, false)) {
		return;
	}

}

void EKFImuVision::stereoVOCallback(const stereo_vo::relative_poseConstPtr& msg) {

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
		aug_state_hist_[latest_idx[keyframe]].time_stamp != msg->key_stamp;
	
	if( change_keyframe)
	{
		ROS_WARN("========keyframe is the same as VO frame========");
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
	ROS_INFO("remember to visualize");
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
	//normalize
	normalizeAngle(yt(3));
	normalizeAngle(yt(4));
	normalizeAngle(yt(5));
	// update
	cur_state.mean = prev_state.mean + Kt * yt;
	cur_state.covariance = prev_state.covariance - Kt * Ht * prev_state.covariance;

	// visualize
	ROS_INFO("remember to visualize");

}

void EKFImuVision::changeAugmentedState(AugState& state) {
  ROS_ERROR("----------------change keyframe------------------------");

	// TODO
	// change augmented state
	
	Vec(21) mean = state.mean;
	Mat(21, 21) covariance = state.covariance;
	// change the augmented state to the new state

	state.mean =M_a_* M_r_* mean;
	state.covariance = M_a_* M_r_* covariance * M_r_.transpose() * M_a_.transpose();
	// change the augmented state keyframe
	state.key_frame_time_stamp = state.time_stamp;

}

bool EKFImuVision::processNewState(AugState& new_state, bool change_keyframe) {

	// TODO
	// process the new state
	// step 1: insert the new state into the queue and get the iterator to start to propagate (be careful about the change of key frame).
	// step 2: try to initialize the filter if it is not initialized.
	// step 3: repropagate from the iterator you extracted.
	// step 4: remove the old states.
	// step 5: publish the latest fused odom
	auto start_it = insertNewState(new_state);

	if (change_keyframe) {
		latest_idx[keyframe] = latest_idx[vo];
		aug_state_hist_[latest_idx[keyframe]].type = keyframe;
	}
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
	auto it = std::upper_bound(
        aug_state_hist_.begin(),
        aug_state_hist_.end(),
        new_state.time_stamp,
        [](const ros::Time& t, const AugState& s) {
            return t < s.time_stamp;
        }
    );
	state_it = aug_state_hist_.insert(it, new_state);
	size_t pos = std::distance(aug_state_hist_.begin(), state_it);
	latest_idx[new_state.type] = pos;
	// update the latest_idx of the type of the new state

	// if (state_it->type==vo && latest_idx[vo]!=0)
	// {
	// 	ROS_INFO("vo exist");
	// 	ROS_INFO("keyframe_time:%f", state_it->key_frame_time_stamp.toSec());
	// 	ROS_INFO("latest_vo_time:%f", aug_state_hist_[latest_idx[vo]].time_stamp.toSec());
	// 	if (state_it->key_frame_time_stamp==aug_state_hist_[latest_idx[vo]].time_stamp)
	// 	{
	// 		latest_idx[keyframe] = latest_idx[vo];
	// 		state_it = aug_state_hist_.begin()+latest_idx[keyframe];
	// 		state_it->type=keyframe;
	// 	}
	// }

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
			if (!init) {
				init = initUsingPnP(it);
			}
		} 
		// else if (cur_state.type == vo) {
		// 	updateVO(cur_state, prev_state);
		// } 
		// else if (cur_state.type == keyframe) {
		// 	updateVO(cur_state, prev_state);
		// 	changeAugmentedState(cur_state);
		// }

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
}


bool EKFImuVision::initFilter() {
	ROS_INFO_STREAM("latest idx:"<<latest_idx[imu]<<","<<latest_idx[pnp]<<','<<latest_idx[vo]<<','<<latest_idx[keyframe]);
	// TODO
	// Initial the filter when a keyframe after marker PnP measurements is available
	if (latest_idx[keyframe] > latest_idx[pnp]) {
		ROS_INFO("find keyframe");
		// find the keyframe
		auto it = aug_state_hist_.begin() + latest_idx[keyframe];
		if (it->type == keyframe) {
			// find the PnP
			auto it_pnp = aug_state_hist_.begin() + latest_idx[pnp];
			if (it_pnp->type == pnp) {
				if (initUsingPnP(it_pnp))
					changeAugmentedState(*it);
				return true;
			}
		}
	}
	
	return false;
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
	start_state.covariance = 0.6 * Mat21x21::Identity(21,21);
	
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