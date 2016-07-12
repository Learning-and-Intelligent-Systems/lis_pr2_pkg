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
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
using namespace std;

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

std::vector<double> publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

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

        tf::Transform T_cam_rhip = getTransform(user, XN_SKEL_LEFT_HIP, frame_id, "left_hip");
        tf::Transform T_cam_rknee = getTransform(user, XN_SKEL_LEFT_KNEE, frame_id, "left_knee");
        tf::Transform T_cam_rfoot = getTransform(user, XN_SKEL_LEFT_FOOT, frame_id, "left_foot");

        tf::Transform T_cam_lhip = getTransform(user, XN_SKEL_RIGHT_HIP, frame_id, "right_hip");
        tf::Transform T_cam_lknee = getTransform(user, XN_SKEL_RIGHT_KNEE, frame_id, "right_knee");
        tf::Transform T_cam_lfoot = getTransform(user, XN_SKEL_RIGHT_FOOT, frame_id, "right_foot");

	tf::Transform T_torso_shoulder = T_cam_torso.inverse() * T_cam_rshoulder;
	tf::Transform T_shoulder_elbow = T_cam_rshoulder.inverse() * T_cam_relbow;
	tf::Quaternion q1 = T_torso_shoulder.getRotation();
	tf::Matrix3x3 m1(q1);
	double roll, pitch, yaw;
	m1.getRPY(roll, pitch, yaw);
	tf::Quaternion q2 = T_shoulder_elbow.getRotation();
	tf::Matrix3x3 m2(q2);
	double x, r_elbow_angle, z;
	m2.getRPY(x, r_elbow_angle, z);
	double r_roll_angle = roll - pi/2;
	double r_pan_angle = pitch - pi/2;
	double r_lift_angle = -1 * yaw;

	//std::cout << "Right-Arm Roll: " << r_roll_angle << ", Right-Arm Pan: " << r_pan_angle << ", Right-Arm Lift: " << r_lift_angle << std::endl;
   	if (q2[1] < 0.7)
		r_elbow_angle = pi - r_elbow_angle;
	//std::cout << "Elbow Angle: " << r_elbow_angle/(pi) * 180 << std::endl;
       
	std::vector<double> angles;
	angles.push_back(r_pan_angle);
	angles.push_back(max(r_lift_angle, -0.53));
	angles.push_back(r_roll_angle);
	angles.push_back(max((r_elbow_angle - pi), -2.0));
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(0);
	return angles; 

	
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

	ros::Rate r(30);

        
        ros::NodeHandle pnh("~");
        string frame_id("camera_depth_frame");
        pnh.getParam("camera_frame_id", frame_id);
	
	TrajClient* traj_client_;
	traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    	pr2_controllers_msgs::JointTrajectoryGoal r_goal;
   	r_goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    	r_goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    	r_goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    	r_goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    	r_goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    	r_goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    	r_goal.trajectory.joint_names.push_back("r_wrist_roll_joint");


	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
    		trajectory_msgs::JointTrajectoryPoint point;
		point.positions = publishTransforms(frame_id);
	    	point.time_from_start = ros::Duration(0.5);
		r_goal.trajectory.points.push_back(point);
		//traj_client_->sendGoal(r_goal); 
		r.sleep();
	}
	delete traj_client_; 
	g_Context.Shutdown();
	return 0;
}
