#include "QuatRotEuler.h"


geometry_msgs::Vector3 SetVector3(float x, float y, float z){
	geometry_msgs::Vector3 Vec;
	Vec.x = x;
	Vec.y = y;
	Vec.z = z;
	return Vec;
}

geometry_msgs::Quaternion setQuat(float qx, float qy, float qz, float qw){
	geometry_msgs::Quaternion q;
	q.x = qx;
	q.y = qy;
	q.z = qz;
	q.w = qw;
	return q;
}


//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
geometry_msgs::Quaternion rot2quat(Eigen::Matrix3d M){
	double trace = M.trace();

	Eigen::Matrix<float, 4, 1> q;
	if (trace > 0) {// M_EPSILON = 0
		double s = 0.5 / sqrt(trace + 1.0);
		q << 0.25 / s,
			(M(2,1) - M(1,2)) * s,
			(M(0,2) - M(2,0)) * s,
			(M(1,0) - M(0,1)) * s;
	}
	else {
		if (M(0,0) > M(1,1) && M(0,0) > M(2,2)) {
			double s = 2.0 * sqrt(1.0 + M(0,0) - M(1,1) - M(2,2));
			q << (M(2,1) - M(1,2)) / s,
				 0.25 * s,
				 (M(0,1) + M(1,0)) / s,
				 (M(0,2) + M(2,0)) / s;
		}
		else if (M(1,1) > M(2,2)) {
			double s = 2.0 * sqrt(1.0 + M(1,1) - M(0,0) - M(2,2));
			q << (M(0,2) - M(2,0)) / s,
			     (M(0,1) + M(1,0)) / s,
			     0.25 * s,
			     (M(1,2) + M(2,1)) / s;
		}
		else {
			double s = 2.0 * sqrt(1.0 + M(2,2) - M(0,0) - M(1,1));
			q << (M(1,0) - M(0,1))/s,
			     (M(0,2) + M(2,0))/s,
			     (M(1,2) + M(2,1))/s,
			     0.25 * s;
		}
	}

	geometry_msgs::Quaternion quat;
	quat = setQuat(q[1], q[2], q[3], q[0]);
    return quat;	
}

geometry_msgs::Quaternion quatProd(geometry_msgs::Quaternion q1,
	                               geometry_msgs::Quaternion q2){
	geometry_msgs::Quaternion qout;
	Eigen::Vector3d q1_v, q2_v, qout_v;
	double q1_s, q2_s, qout_s;

	//Vector parts of the quaternions
	q1_v << q1.x, q1.y, q1.z;
	q2_v << q2.x, q2.y, q2.z;

	//Scalar parts of the quaternions
	q1_s = q1.w;
	q2_s = q2.w;

	//Quaternion multiplication formula
	qout_v = q1_s*q2_v + q2_s*q1_v - q1_v.cross(q2_v);
	qout_s = q1_s*q2_s - q1_v.dot(q2_v);

	//Assign to quaternion structure
	qout = setQuat(qout_v(0), qout_v(1), qout_v(2), qout_s);
	return qout;
}

geometry_msgs::Quaternion quatInv(geometry_msgs::Quaternion quat){
	geometry_msgs::Quaternion q;
	q.x = -quat.x;
	q.y = -quat.y;
	q.z = -quat.z;
	q.w = quat.w;
	return q;
}

double getHeadingFromQuat(geometry_msgs::Quaternion quat){
	geometry_msgs::Vector3 RPY = quat2rpy(quat);
	return RPY.z;
}

geometry_msgs::Vector3 quat2rpy(geometry_msgs::Quaternion quat){
	double qx, qy, qz, qw, roll, pitch, yaw;
	qx = quat.x;
	qy = quat.y;
	qz = quat.z;
	qw = quat.w;

	//Formulas for roll, pitch, yaw
	roll = atan2(2*(qw*qx + qy*qz) , 1 - 2*(qx*qx + qy*qy) );
	pitch = asin(2*(qw*qy - qz*qx));
	yaw = atan2(2*(qw*qz + qx*qy),1 - 2*(qy*qy + qz*qz) );

	geometry_msgs::Vector3 rpy = SetVector3(roll, pitch, yaw);
	return rpy;
}

geometry_msgs::Quaternion rpy2quat(geometry_msgs::Vector3 rpy){
	double roll, pitch, yaw;
	geometry_msgs::Quaternion q1, q2, q3, qout;

	roll = rpy.x;
	pitch = rpy.y;
	yaw = rpy.z;

	q1 = setQuat(sin(roll/2), 0, 0, cos(roll/2));
	q2 = setQuat(0, sin(pitch/2), 0, cos(pitch/2));
	q3 = setQuat(0, 0, sin(yaw/2), cos(yaw/2));

	qout = quatProd(q1, quatProd(q2, q3));

	return qout;
}

Eigen::Matrix3d rotx(double theta){
	Eigen::Matrix3d R;

	R << 1,          0,           0,
	     0, cos(theta), -sin(theta),
	     0, sin(theta),  cos(theta);

	return R;
}

Eigen::Matrix3d roty(double theta){
	Eigen::Matrix3d R;

	R <<  cos(theta), 0, sin(theta),
	               0, 1,          0,
	     -sin(theta), 0, cos(theta);

	return R;
}

Eigen::Matrix3d rotz(double theta){
	Eigen::Matrix3d R;

	R <<  cos(theta), -sin(theta), 0,
	      sin(theta),  cos(theta), 0,
	               0,           0, 1;

	return R;
}