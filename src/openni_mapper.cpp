// openni_tracker.cpp
//Mapper

#include <ros/ros.h>
#include <ros/package.h>
//#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <tf/transform_datatypes.h>
#include <XnOpenNI.h>
#include <vector> 
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <numeric>

//For controlling the robot
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
using namespace std;
//TOPIC: /r_arm_controller/joint_trajectory_action/goalrosto
//double pi = 3.14;
int avg_len = 20;
int rate_hz = 30;
int freq = 2;
//1,2,3,5,6,10,15,30
int mod_freq = rate_hz / freq;
//Real robot rate is 30; 
//double goal_t = 1/(rate_hz);
double goal_t =  1.0 / freq;

//Moving average length
//Duration from start
//How often to send goal 
double pi = 3.14159265359;
xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;
XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

tf::Transform getTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    //static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    			   m[3], m[4], m[5],
    			   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);
    //No longer a void function 
    transform = change_frame * transform;
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
    return transform; 
}

std::vector<double> get_angles(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);
    
    std::vector<double> all_angles (14);  // init with right arm(7), left arm(7), head(2), and base(2) [in that order] 
    
    //std::vector<double> r_angles (7);  //To test one arm
    
    //std::vector<double> l_angles (7);
    /*
    std::vector<double> h_angles (2);
    std::vector<double> base_angles (2);
    */
    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;



        tf::Transform T_cam_head = getTransform(user, XN_SKEL_HEAD,frame_id, "head");
        tf::Transform T_cam_neck = getTransform(user, XN_SKEL_NECK,frame_id, "neck");
        tf::Transform T_cam_torso = getTransform(user, XN_SKEL_TORSO, frame_id, "torso");

	// compute stuff
	// The right-side in Openni is the left side for the user and vice-versa 

        tf::Transform T_cam_rshoulder = getTransform(user, XN_SKEL_LEFT_SHOULDER, frame_id, "left_shoulder");
        tf::Transform T_cam_relbow = getTransform(user, XN_SKEL_LEFT_ELBOW, frame_id, "left_elbow");
        tf::Transform T_cam_rhand = getTransform(user, XN_SKEL_LEFT_HAND,  frame_id, "left_hand");

        tf::Transform T_cam_lshoulder =getTransform(user, XN_SKEL_RIGHT_SHOULDER,frame_id, "right_shoulder");
        tf::Transform T_cam_lelbow = getTransform(user, XN_SKEL_RIGHT_ELBOW,frame_id, "right_elbow");
        tf::Transform T_cam_lhand = getTransform(user, XN_SKEL_RIGHT_HAND, frame_id, "right_hand");

        //tf::Transform T_cam_rhip = getTransform(user, XN_SKEL_LEFT_HIP, frame_id, "left_hip");
        //tf::Transform T_cam_rknee = getTransform(user, XN_SKEL_LEFT_KNEE, frame_id, "left_knee");
        //tf::Transform T_cam_rfoot = getTransform(user, XN_SKEL_LEFT_FOOT, frame_id, "left_foot");

        //tf::Transform T_cam_lhip = getTransform(user, XN_SKEL_RIGHT_HIP, frame_id, "right_hip");
        //tf::Transform T_cam_lknee = getTransform(user, XN_SKEL_RIGHT_KNEE, frame_id, "right_knee");
        //tf::Transform T_cam_lfoot = getTransform(user, XN_SKEL_RIGHT_FOOT, frame_id, "right_foot");



	

	
	tf::Transform T_torso_rshoulder = T_cam_torso.inverse() * T_cam_rshoulder;
	//tf::Transform T_torso_rshoulder = T_cam_rhand.inverse() * T_cam_rshoulder; //The hand frame is fixed while the torso frame is not. If we want to include base-movement we can adjust for this. 
	tf::Transform T_shoulder_relbow = T_cam_rshoulder.inverse() * T_cam_relbow;
	tf::Quaternion q1 = T_torso_rshoulder.getRotation();
	tf::Matrix3x3 m1(q1);
	double rroll, rpitch, ryaw;
	m1.getRPY(rroll, rpitch, ryaw);
	tf::Quaternion q2 = T_shoulder_relbow.getRotation(); 

	tf::Matrix3x3 m2(q2);
	double x, r_elbow_angle, z;
	m2.getRPY(x, r_elbow_angle, z);
	double r_roll_angle = rroll - pi/2;
	double r_pan_angle = rpitch - pi/2;
	double r_lift_angle = -1 * ryaw;

	tf::Transform T_torso_lshoulder = T_cam_torso.inverse() * T_cam_lshoulder;
	tf::Transform T_shoulder_lelbow = T_cam_lshoulder.inverse() * T_cam_lelbow;
	tf::Quaternion q3 = T_torso_lshoulder.getRotation();
	tf::Matrix3x3 m3(q3);
	double lroll, lpitch, lyaw;
	m3.getRPY(lroll, lpitch, lyaw);
	tf::Quaternion q4 = T_shoulder_lelbow.getRotation();
	tf::Matrix3x3 m4(q4);
	double x2, l_elbow_angle, z2;
	m4.getRPY(x2, l_elbow_angle, z2);
	double l_roll_angle = pi/2 - lroll;
	double l_pan_angle = pi/2 + lpitch;
	double l_lift_angle = lyaw;


	//std::cout << "Right-Arm Roll: " << r_roll_angle << ", Right-Arm Pan: " << r_pan_angle << ", Right-Arm Lift: " << r_lift_angle << std::endl;
	
   	if (q2[1] < 0.7)
		r_elbow_angle = pi - r_elbow_angle;


	if (q4[3] < 0.7)
		l_elbow_angle = -1 * (pi + l_elbow_angle);
	    

	    
   	if (q1[1] > 0.53) 
		r_pan_angle = -1 * r_pan_angle;
		//r_roll_angle = r_roll_angle + pi;
    
    
    	if (q3[1] < -0.53)
        	l_pan_angle = -1 * l_pan_angle; 
    
    
		
	//if (q1[0] < 0.03)
	//	r_roll_angle = r_roll_angle + pi;
	
	
		//r_roll_angle = r_roll_angle + pi;
	//std::cout << "Elbow Quaternion: " << q2[1] << std::endl;
    //std::cout << "L-Elbow Quaternion: " << q4[0] << std::endl;
	//std::cout << "L-Elbow Quaternion: " << q4[3] << std::endl;
	//std::cout << "L-Elbow Quaternion: " << q4[2] << std::endl;
	//std::cout << "Elbow Angle: " << r_elbow_angle/(pi) * 180 << std::endl;
	//std::cout << "L-Elbow Angle: " << l_elbow_angle/(pi) * 180  << std::endl;
        
	all_angles[0] = min(max(r_pan_angle, -2.28), 0.68);

    	all_angles[1] = min(max(r_lift_angle, -0.53), 1.3);

	all_angles[2] = r_roll_angle;	

    	all_angles[3] = max((r_elbow_angle - pi), -2.0);
    
	
	all_angles[7] = l_pan_angle;
	all_angles[8] = min(max(l_lift_angle, -0.53), 1.3);
	all_angles[9] = l_roll_angle;
	all_angles[10] = l_elbow_angle;

	
	//std::cout << "Right Roll Quaternion: " << q1[0] << std::endl;
	//std::cout << "Right Lift Quaternion: " << q1[2] << std::endl;
	//std::cout << "Right Pan Quaternion: " << q1[1] << std::endl;
	//std::cout << "Right Pan Angle: " << all_angles[0] << std::endl;
	//std::cout << "Right-Arm Roll: " << r_roll_angle << std::endl;
    //std::cout << "Left Pan Angle: " << all_angles[7]/(pi) * 180 << std::endl;
    //std::cout << "Left Pan Quaternion 1: " << q3[1] << std::endl;
    
	return all_angles;
        //return {r_angles, l_angles, h_angles, base_angles};


	
    }
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}


int main(int argc, char **argv) {
    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh;
    ROS_INFO("Michael's Motion Mapping (M3) ");
    string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");
    ROS_INFO("finished init!");
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");
    ROS_INFO("I doubt we got here");
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
	    if (nRetVal != XN_STATUS_OK) {
		    ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
            return nRetVal;
	    }
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);


	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(rate_hz);


        ros::NodeHandle pnh("~");
        string frame_id("camera_depth_frame");
        pnh.getParam("camera_frame_id", frame_id);
	
	TrajClient* r_traj_client_;
	r_traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
	TrajClient* l_traj_client_;
	l_traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);	

    	pr2_controllers_msgs::JointTrajectoryGoal r_goal;
    	r_goal.trajectory.header.stamp = ros::Time::now();
   	r_goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    	r_goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    	r_goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    	r_goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    	r_goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    	r_goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    	r_goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    	pr2_controllers_msgs::JointTrajectoryGoal l_goal;
    	l_goal.trajectory.header.stamp = ros::Time::now();
   	l_goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
    	l_goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
    	l_goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    	l_goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
    	l_goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
    	l_goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
    	l_goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

	std::cout << "Please go to PSI Pose" << std::endl;
        trajectory_msgs::JointTrajectoryPoint sr_point;
        trajectory_msgs::JointTrajectoryPoint sl_point;
	std::vector<double> sr_angles(7);
	std::vector<double> sl_angles(7);
	sr_point.positions = sr_angles;
	sl_point.positions = sl_angles;
	sr_point.positions[0] = -pi/2;
	sr_point.positions[1] = 0.0;
	sr_point.positions[2] = 0.0;
	sr_point.positions[3] = -pi/2;
	sr_point.positions[4] = 0.0;
	sr_point.positions[5] = 0.0;
	sr_point.positions[6] = 0.0;
	sl_point.positions[0] = pi/2;
	sl_point.positions[1] = 0.0;
	sl_point.positions[2] = 0.0;
	sl_point.positions[3] = -pi/2;
	sl_point.positions[4] = 0.0;
	sl_point.positions[5] = 0.0;
	sl_point.positions[6] = 0.0;
	sr_point.time_from_start = ros::Duration(5.0);
	r_goal.trajectory.points.push_back(sr_point);
    	sl_point.time_from_start = ros::Duration(5.0);
	l_goal.trajectory.points.push_back(sl_point);
	r_traj_client_->sendGoal(r_goal); 
	l_traj_client_->sendGoal(l_goal);
    //usleep(9000000);
	//double rprev_val_1 = -pi/2, rprev_val_2 = 0.0, rprev_val_3 = 0.0, rprev_val_4 = -pi/2; 
	//double lprev_val_1 = pi/2, lprev_val_2 = 0.0, lprev_val_3 = 0.0, lprev_val_4 = -pi/2; 
	//double rprev_val_1;
	//double val = 5.0; 
	int i = 0;

	std::vector<double> r0_avg_vals (avg_len, -pi/2);
	std::vector<double> r1_avg_vals (avg_len, 0);  
	std::vector<double> r2_avg_vals (avg_len, 0);  
	std::vector<double> r3_avg_vals (avg_len, -pi/2);
	std::vector<double> l0_avg_vals (avg_len, pi/2);
	std::vector<double> l1_avg_vals (avg_len, 0);  
	std::vector<double> l2_avg_vals (avg_len, 0);  
	std::vector<double> l3_avg_vals (avg_len, -pi/2);
    double r0_avg = -pi/2;
    double r1_avg = 0;
    double r2_avg = 0;
    double r3_avg = -pi/2;
    double l0_avg = pi/2;
    double l1_avg = 0;
    double l2_avg = 0;
    double l3_avg = -pi/2;
    double r0_pavg = -pi/2;
    double r1_pavg = 0;
    double r2_pavg = 0;
    double r3_pavg = -pi/2;
    double l0_pavg = pi/2;
    double l1_pavg = 0;
    double l2_pavg = 0;
    double l3_pavg = -pi/2;
    double val = 0.01;
    int j = 0;
    bool sendG = true; 
    
	while (ros::ok()) 
	{
		g_Context.WaitAndUpdateAll();
		std::vector<double> angles = get_angles(frame_id);		
		trajectory_msgs::JointTrajectoryPoint r_point;
    		trajectory_msgs::JointTrajectoryPoint l_point;
		/*
		if (i > 0)
		{
			val = 0.05; 

		}
		if (abs(angles.at(1) - rprev_val_1) > val)
		{
		*/
	    std::vector<double> r_angles(7);
	    std::vector<double> l_angles(7);
	    r_point.positions = r_angles;
	    l_point.positions = l_angles;
	    //r1_avg_vals[i_r1] = angles.at(0);
	    
	    r0_avg_vals[i] = angles.at(0);
	    r1_avg_vals[i] = angles.at(1);
	    r2_avg_vals[i] = angles.at(2);
	    r3_avg_vals[i] = angles.at(3);
	    l0_avg_vals[i] = angles.at(7);
	    l1_avg_vals[i] = angles.at(8);
	    l2_avg_vals[i] = angles.at(9);
	    l3_avg_vals[i] = angles.at(10);

	    //std::cout << "Current Angle: " << r1_avg_vals[i_r1] << std::endl;
	    
	    r0_avg = (std::accumulate(r0_avg_vals.begin(), r0_avg_vals.end(), 0.0)) / avg_len;
	    r1_avg = (std::accumulate(r1_avg_vals.begin(), r1_avg_vals.end(), 0.0)) / avg_len;
	    r2_avg = (std::accumulate(r2_avg_vals.begin(), r2_avg_vals.end(), 0.0)) / avg_len;
	    r3_avg = (std::accumulate(r3_avg_vals.begin(), r3_avg_vals.end(), 0.0)) / avg_len;
            l0_avg = (std::accumulate(l0_avg_vals.begin(), l0_avg_vals.end(), 0.0)) / avg_len;
	    l1_avg = (std::accumulate(l1_avg_vals.begin(), l1_avg_vals.end(), 0.0)) / avg_len;
	    l2_avg = (std::accumulate(l2_avg_vals.begin(), l2_avg_vals.end(), 0.0)) / avg_len;
            l3_avg = (std::accumulate(l3_avg_vals.begin(), l3_avg_vals.end(), 0.0)) / avg_len;
            //std::cout << "Average Pan Value: " << r1_avg << std::endl;
	    //r_point.positions[0] = min(angles.at(0), max(abs(angles.at(0)), 0.0));
	    //r_point.positions[0] = angles.at(0);
	    r_point.positions[0] = r0_avg;
	    r_point.positions[1] = r1_avg;
	    r_point.positions[2] = r2_avg;
	    r_point.positions[3] = r3_avg;
	    l_point.positions[0] = l0_avg;
	    l_point.positions[1] = l1_avg;
	    l_point.positions[2] = l2_avg;
	    l_point.positions[3] = l3_avg;
	    
	    /*
		if (abs(r0_avg - r0_pavg) > val && abs(r1_avg - r1_pavg) > val && abs(r2_avg - r2_pavg) > val && abs(r3_avg - r3_pavg) > val )
		{
		    r0_pavg = r0_avg;
		    r1_pavg = r1_avg;
		    r2_pavg = r2_avg;
		    r3_pavg = r3_avg;    
		    sendG = true; 
		}
		else
		{
		    sendG = false; 
		}
		*/
	    /*
	    std::cout << "Pan Angle Command: " << r0_avg << std::endl;
	    std::cout << "Lift Angle Command: " << r0_avg << std::endl;
	    std::cout << "Roll Angle Command: " << r0_avg << std::endl;
	    std::cout << "Elbow Angle Command: " << r0_avg << std::endl;
	    */
	    
	    //std::cout << l0_avg << std::endl;		
	    //r_point.positions[0] = r1_avg;		
            //r_point.positions[1] = angles.at(1);
            //r_point.positions[2] = angles.at(2);		
            //r_point.positions[3] = angles.at(3);
	    //r_point.positions[1] = min(angles.at(1), max(abs(angles.at(1)), 0.0));
	    //r_point.positions[2] = min(angles.at(2), max(abs(angles.at(2)), 0.0));
	    //r_point.positions[3] = min(angles.at(3), max(abs(angles.at(3)), 0.0));
	    /*
	    rprev_val_1 = r_point.positions[0];
	    rprev_val_2 = r_point.positions[1];
	    rprev_val_3 = r_point.positions[2];
	    rprev_val_4 = r_point.positions[3];
	    */
	    /*
	    l_point.positions[0] = angles.at(7);
	    l_point.positions[1] = angles.at(8);
	    l_point.positions[2] = angles.at(9);
	    l_point.positions[3] = angles.at(10);
    	*/
    	    r_point.time_from_start = ros::Duration(goal_t);
	    r_goal.trajectory.points.push_back(r_point);
            l_point.time_from_start = ros::Duration(goal_t);
	    l_goal.trajectory.points.push_back(l_point);

	    //std::cout << r_point.positions[2] << std::endl;
	    

		
		i += 1;
		j += 1;
		if (j % mod_freq == 0 && sendG) 
		{
            		r_traj_client_->sendGoal(r_goal); 
	        	l_traj_client_->sendGoal(l_goal);
		}
		
		if (i == (avg_len - 1))
		{
			i = 0;
		}
		if (j == rate_hz)
		{
		    j = 0;
		}

        //std::cout << "Index value: " << i_r1 << std::endl;
		//r.sleep();100
				
		

	
	}
	
	/*
	int i = 0;

	while (ros::ok()) {
		i++; 
		g_Context.WaitAndUpdateAll();
    		trajectory_msgs::JointTrajectoryPoint r_point;
    		trajectory_msgs::JointTrajectoryPoint l_point;
		r_point.positions = get_angles(frame_id);
		//r_point.positions = get_angles(frame_id)[0];
	    	r_point.time_from_start = ros::Duration(1.0);
		r_goal.trajectory.points.push_back(r_point);
		r_traj_client_->sendGoal(r_goal); 
		//std::cout << "Moving" << std::endl;
		r.sleep();
	}
	*/
	delete r_traj_client_; 
	delete l_traj_client_; 
	g_Context.Shutdown();
	return 0;
}
