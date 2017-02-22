
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <tf/transform_listener.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace enc = sensor_msgs::image_encodings;
//using namespace tf;
using namespace cv;

class PoseEstimation{


private:
ros::NodeHandle nh_ ;
ros::Subscriber image_sub;
ros::Subscriber kinect_sub;
ros::Publisher pose_pub;
tf::TransformListener listener;

nav_msgs::Odometry cam_odom;

public:

PoseEstimation() : nh_("~") {

pose_pub = nh_.advertise<nav_msgs::Odometry>("/pose", 100, true);
image_sub = nh_.subscribe<sensor_msgs::ImageConstPtr>("/usb_cam/image_raw",1, &PoseEstimation::Image_callback, this);
kinect_sub = nh_.subscribe<sensor_msgs::ImageConstPtr>("camera/rgb/image_raw", 1, &PoseEstimation::Kinect_callback, this);
}
void Kinect_callback(sensor_msgs::ImageConstPtr msg){

ros::Time now = ros::Time::now();

cv_bridge::CvImagePtr cv_ptr;
	  	 try
	  	 {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	  	 }
	  	 catch (cv_bridge::Exception& e)
	 	 {
	     	  ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
	  	 }

	std::vector<cv::Point3d> pointsModel;
	pointsModel.push_back(cv::Point3d(-3.7, -2.275, 0));
	pointsModel.push_back(cv::Point3d(0, -2.275, 0));
	pointsModel.push_back(cv::Point3d(3.7, -2.275, 0));
	pointsModel.push_back(cv::Point3d(-3.7, 2.275, 0));
	pointsModel.push_back(cv::Point3d(3.7, 2.275, 0));
	
	// camera intrinsic matrix
	double K_[3][3] =
	{ { 675, 0, 320 },
	{ 0, 675, 240 },
	{ 0, 0, 1 } };
	cv::Mat K = cv::Mat(3, 3, CV_64F, K_).clone();
	cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F); // distortion coeffs
	
		cv::Mat imageInputGray;
		cvtColor(cv_ptr->image, imageInputGray, cv::COLOR_BGR2GRAY);
		
		// find all CCC targets
		std::vector<cv::Point2d> allTargets;
		allTargets = findTargets(imageInputGray);
		
		// draw dots on all targets found
		for (unsigned int i = 0; i < allTargets.size(); i++) {
			// input image, circle center, circle radius, color, thickness (negative = filled)
			cv::circle(cv_ptr->image, allTargets[i], 3, cv::Scalar(0, 0, 255), -1);
		}

		// find the 5‐CCC target pattern, and put the targets in the order
		//	1	2	3
		//	4		5
		std::vector<cv::Point2d> orderedTargets;
		orderedTargets = orderTargets(allTargets);

		// optionally label targets in the image
		for (unsigned int i = 0; i < orderedTargets.size(); i++) {
			
			char szLabel[50];
			//sprintf_s(szLabel, "%d", i);
			
			// input image, char array, array of Point2d, font face, font scale, font color, thickness
			putText(cv_ptr->image, szLabel, cv::Point2d(orderedTargets[i]), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 0), 2);
		}
		Mat rotVec, transVec;
		if (orderedTargets.size() == 5) {
			std::cout << "ordered targets = 5" << std::endl;
			// Calculate the pose.
			

			// model points, image points, intrinsic camera parameter matrix, distortion coefficients, output rotation and translation
			bool foundPose = cv::solvePnP(pointsModel, allTargets, K, dist, rotVec, transVec);
			if (foundPose) {
				std::vector<cv::Point3d> pointsAxes;
				std::vector<cv::Point2d> pointsImage;
				
				// draw the xyz coordinate axes on the image.
				pointsAxes.push_back(cv::Point3d(0, 0, 0)); // origin
				pointsAxes.push_back(cv::Point3d(1, 0, 0)); // x axis
				pointsAxes.push_back(cv::Point3d(0, 1, 0)); // y axis
				pointsAxes.push_back(cv::Point3d(0, 0, 1)); // z axis
				cv::projectPoints(pointsAxes, rotVec, transVec, K, dist, pointsImage);
				line(cv_ptr->image, pointsImage[0], pointsImage[1], cv::Scalar(0, 0, 255), 2);
				line(cv_ptr->image, pointsImage[0], pointsImage[2], cv::Scalar(0, 255, 0), 2);
				line(cv_ptr->image, pointsImage[0], pointsImage[3], cv::Scalar(255, 0, 0), 2);

			}

		imshow("Image", cv_ptr->image);
		cvWaitKey(10000);
		tf::StampedTransform KinInOdom;
		listener.waitForTransform("/camera_link", "/camera_link", now, ros::Duration(5.0));
		listener.lookupTransform("/camera_link","/camera_link", now, KinInOdom);
		GetPose(rotVec,transVec,KinInOdom, now);
	}
}
void Image_callback(sensor_msgs::ImageConstPtr msg){

ros::Time now = ros::Time::now();

cv_bridge::CvImagePtr cv_ptr;
	  	 try
	  	 {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	  	 }
	  	 catch (cv_bridge::Exception& e)
	 	 {
	     	  ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
	  	 }

	std::vector<cv::Point3d> pointsModel;
	pointsModel.push_back(cv::Point3d(-3.7, -2.275, 0));
	pointsModel.push_back(cv::Point3d(0, -2.275, 0));
	pointsModel.push_back(cv::Point3d(3.7, -2.275, 0));
	pointsModel.push_back(cv::Point3d(-3.7, 2.275, 0));
	pointsModel.push_back(cv::Point3d(3.7, 2.275, 0));
	
	// camera intrinsic matrix
	double K_[3][3] =
	{ { 675, 0, 320 },
	{ 0, 675, 240 },
	{ 0, 0, 1 } };
	cv::Mat K = cv::Mat(3, 3, CV_64F, K_).clone();
	cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F); // distortion coeffs
	
		cv::Mat imageInputGray;
		cvtColor(cv_ptr->image, imageInputGray, cv::COLOR_BGR2GRAY);
		
		// find all CCC targets
		std::vector<cv::Point2d> allTargets;
		allTargets = findTargets(imageInputGray);
		
		// draw dots on all targets found
		for (unsigned int i = 0; i < allTargets.size(); i++) {
			// input image, circle center, circle radius, color, thickness (negative = filled)
			cv::circle(cv_ptr->image, allTargets[i], 3, cv::Scalar(0, 0, 255), -1);
		}

		// find the 5‐CCC target pattern, and put the targets in the order
		//	1	2	3
		//	4		5
		std::vector<cv::Point2d> orderedTargets;
		orderedTargets = orderTargets(allTargets);

		// optionally label targets in the image
		for (unsigned int i = 0; i < orderedTargets.size(); i++) {
			
			char szLabel[50];
			//sprintf_s(szLabel, "%d", i);
			
			// input image, char array, array of Point2d, font face, font scale, font color, thickness
			putText(cv_ptr->image, szLabel, cv::Point2d(orderedTargets[i]), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 0), 2);
		}
		Mat rotVec, transVec;
		if (orderedTargets.size() == 5) {
			//std::cout << "ordered targets = 5" << std::endl;
			// Calculate the pose.
			

			// model points, image points, intrinsic camera parameter matrix, distortion coefficients, output rotation and translation
			bool foundPose = cv::solvePnP(pointsModel, allTargets, K, dist, rotVec, transVec);
			if (foundPose) {
				std::vector<cv::Point3d> pointsAxes;
				std::vector<cv::Point2d> pointsImage;
				
				// draw the xyz coordinate axes on the image.
				pointsAxes.push_back(cv::Point3d(0, 0, 0)); // origin
				pointsAxes.push_back(cv::Point3d(1, 0, 0)); // x axis
				pointsAxes.push_back(cv::Point3d(0, 1, 0)); // y axis
				pointsAxes.push_back(cv::Point3d(0, 0, 1)); // z axis
				cv::projectPoints(pointsAxes, rotVec, transVec, K, dist, pointsImage);
				line(cv_ptr->image, pointsImage[0], pointsImage[1], cv::Scalar(0, 0, 255), 2);
				line(cv_ptr->image, pointsImage[0], pointsImage[2], cv::Scalar(0, 255, 0), 2);
				line(cv_ptr->image, pointsImage[0], pointsImage[3], cv::Scalar(255, 0, 0), 2);

			}
		

		imshow("Image", cv_ptr->image);
		cvWaitKey(10000);
		tf::StampedTransform USBInOdom;
		listener.waitForTransform("/camera_link", "/camera_link", now, ros::Duration(5.0));
		listener.lookupTransform("/camera_link","/camera_link", now, USBInOdom);
		GetPose(rotVec,transVec,USBInOdom, now);

	}
}
void GetPose(cv::Mat RotationVector, cv::Mat TranslationVector, tf::StampedTransform CameraInOdomFrame, ros::Time now){
	
		Mat RotationMatrix(3,3,CV_64F);
		Rodrigues(RotationVector,RotationMatrix);
		TranslationVector = 0.0254*TranslationVector;
		
		Mat TarToCam(3,4, CV_64F);
		Mat TargetInCamera(4,4, CV_64F);
		Mat CameraInTarget(4,4,CV_64F);
		Mat CamInOdom_Transform(4,4,CV_64F);
		Mat CamInOdom_T(4,4,CV_64F);

		double LastRowData[] ={ 0, 0, 0, 1};
		Mat LastRow = Mat(1,4, CV_64F, LastRowData).clone();

		hconcat(RotationMatrix,TranslationVector,TarToCam);
		vconcat(TarToCam,LastRow,TargetInCamera);

		tf::Matrix3x3 Camera_In_Odom_rotation = CameraInOdomFrame.getBasis();
		tf::Vector3 Camera_In_Odom_translation = CameraInOdomFrame.getOrigin();
		tf::Vector3 Row1 = Camera_In_Odom_rotation.getRow(0);
		tf::Vector3 Row2 = Camera_In_Odom_rotation.getRow(1);
		tf::Vector3 Row3 = Camera_In_Odom_rotation.getRow(2);
		double CIORdata[] = {Row1.getX(),Row1.getY(),Row1.getZ(),
								Row2.getX(),Row2.getY(),Row2.getZ(),
								Row3.getX(),Row3.getY(),Row3.getZ()};

		double CIOTdata[] =  {Camera_In_Odom_translation.getX(), Camera_In_Odom_translation.getY(),  Camera_In_Odom_translation.getZ()};
								  
		Mat CIOT = Mat(3,1,CV_64F, CIOTdata).clone();
		Mat CIOR = Mat(3,3,CV_64F, CIORdata).clone();
		hconcat(CIOR,CIOT,CamInOdom_T);
		vconcat(CamInOdom_T,LastRow,CamInOdom_Transform);

		CameraInTarget = TargetInCamera.inv(1);
		Mat OdomToWorld = (CameraInTarget)*(CamInOdom_Transform.inv(1));
		
		geometry_msgs::PoseStamped Cam_Pose;
		Cam_Pose.header.frame_id = "/camera_link";
		Cam_Pose.header.stamp = now;
		

		Cam_Pose.pose.position.x = OdomToWorld.at<double>(1,4);
		Cam_Pose.pose.position.y = OdomToWorld.at<double>(2,4);
		Cam_Pose.pose.position.z = OdomToWorld.at<double>(3,4);

 		Cam_Pose.pose.orientation.w = 0.5*sqrt(1.0 + OdomToWorld.at<double>(1,1)+ OdomToWorld.at<double>(2,2) + OdomToWorld.at<double>(3,3));
 		Cam_Pose.pose.orientation.x = (OdomToWorld.at<double>(3,2)-OdomToWorld.at<double>(2,3))/(4*Cam_Pose.pose.orientation.w);
		Cam_Pose.pose.orientation.y = (OdomToWorld.at<double>(1,3)-OdomToWorld.at<double>(3,1))/(4*Cam_Pose.pose.orientation.w);
		Cam_Pose.pose.orientation.z = (OdomToWorld.at<double>(2,1)-OdomToWorld.at<double>(1,2))/(4*Cam_Pose.pose.orientation.w);
		


		cam_odom.header.frame_id = "/camera_link";
		cam_odom.header.stamp = now;
		cam_odom.pose.pose = Cam_Pose.pose; 


		pose_pub.publish(cam_odom);
	}


std::vector<cv::Point2d> findTargets(cv::Mat Image) {
	std::vector<cv::Point2d> targets;
	cv::Mat imageThresh;

	// use Otsu global thresholding
	// image, output thresholded image, threshold value, output value
	cv::threshold(Image, imageThresh, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY_INV);

	// threshold_type ‐ invert
	//// Do adaptive threshold ... this compares each pixel to a local
	//// mean of the neighborhood.  The result is a binary image, where
	//// dark areas of the original image are now white (1's).

	// imputimage, ouput threshold image, output value where condition met, local neighborhood, threshold type, ...
	// block size (any large number), a constant to subtract from the mean
	// adaptiveThreshold(Image, imageThresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 91, 0); 

	// apply morphological operations to get rid of small (noise) regions.
	cv::Mat structuringElmt = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	cv::Mat imageOpen;
	morphologyEx(imageThresh, imageOpen, cv::MORPH_OPEN, structuringElmt);
	cv::Mat imageClose;
	morphologyEx(imageOpen, imageClose, cv::MORPH_CLOSE, structuringElmt);

	// now find contours.
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	// input image is destroyed, output vector of contours, hierarchical representation, retrieve all contours all pixels of each contour
	cv::findContours(imageClose, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

	// analyze components and find CCCs
	for (unsigned int i1 = 0; i1 < (int)contours.size(); i1++) {
		int i2 = hierarchy[i1][2];
		if (i2 < 0) continue;

		// compute centroids of inner and outer regions
		cv::Moments mu1 = cv::moments(contours[i1]);
		cv::Point2d x1(mu1.m10 / mu1.m00, mu1.m01 / mu1.m00);
		cv::Moments mu2 = cv::moments(contours[i2]);
		cv::Point2d x2(mu2.m10 / mu2.m00, mu2.m01 / mu2.m00);

		// check if centroids coincide
		float DPIXEL = 3.0;
		if (norm(x1 - x2) > DPIXEL) continue;

		// check the "circularity" ratio of the outer region, which is
		// the ratio of area to perimeter squared:  R = 4*pi*A/P^2.
		// R is 1 for a circle, and pi/4 for a square.
		double P1 = arcLength(contours[i1], true);
		double A1 = contourArea(contours[i1]);
		if (4 * 3.1415*A1 / (P1*P1) < 3.1415 / 4)

			// let's say that we want our region to be at least as round as a square.
			continue;

		// this must be a valid target; add it to the output list.
		targets.push_back(x1);
	}

	return targets;
}
// This function tries to find the 5‐target pattern that looks like this
//  1  2  3
//  4     5
std::vector<cv::Point2d> orderTargets(std::vector<cv::Point2d> allTargets) {
		
		std::vector<cv::Point2d> orderedTargets;
		unsigned int i1, i2, i3, i4, i5;
		unsigned int nCCC = allTargets.size();
		
		// find 3 CCCs that are in a line.
		// distance from a CCC to the midpt between points 1,3
		double dMin = 1e9;

		// the distance between points 1,3
		double d13 = 1;

		for (unsigned int i = 0; i<nCCC; i++) {
			for (unsigned int j = i + 1; j<nCCC; j++) {
				
				// get the mid point between i,j.
				cv::Point2d midPt = (allTargets[i] + allTargets[j]) * 0.5;
				
				// find the CCC that is closest to this midpoint.
				for (unsigned int k = 0; k<nCCC; k++) {
					if (k == i || k == j) continue;
					double d = norm(allTargets[k] - midPt); // distance from midpoint
					if (d < dMin) {
						
						// this is the minimum found so far; save it.
						dMin = d;
						i1 = i;
						i2 = k;
						i3 = j;
						d13 = norm(allTargets[i] - allTargets[j]);
					}
				}
			}
		}
		
		// if the best distance from the midpoint is < 30% of the distance between
		// the two other points, then we probably have our colinear set.
		if (dMin / d13 > 0.3) return orderedTargets; // return an empty list
		
		/*
		We have found 3 colinear targets:  p1 ‐‐ p2 ‐‐ p3.
		Now find the one closest to p1; call it p4.
		*/
		
		dMin = 1e9;
		for (unsigned int i = 0; i<nCCC; i++) {
			if (i != i1 && i != i2 && i != i3) {
				double d = norm(allTargets[i] - allTargets[i1]);
				if (d < dMin) {
					dMin = d;
					i4 = i;
				}
			}
		}
		// return an empty list
		if (dMin > 1e7) return orderedTargets;
		
		// now find the one closest to p3; call it p5
		dMin = 1e9;
		for (unsigned int i = 0; i<nCCC; i++) {
			if (i != i1 && i != i2 && i != i3 && i != i4) {
				double d = norm(allTargets[i] - allTargets[i3]);
				if (d < dMin) {
					dMin = d;
					i5 = i;
				}
			}
		}

		if (dMin > 1e7) return orderedTargets; 
		cv::Vec2d p41 = allTargets[i4] - allTargets[i1];
		cv::Vec2d p31 = allTargets[i3] - allTargets[i1];
		double m[2][2] = { { p41[0], p31[0] },{ p41[1], p31[1] } };
		double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
		// Put the targets into the output list.
		if (det < 0) {
			orderedTargets.push_back(allTargets[i1]);
			orderedTargets.push_back(allTargets[i2]);
			orderedTargets.push_back(allTargets[i3]);
			orderedTargets.push_back(allTargets[i4]);
			orderedTargets.push_back(allTargets[i5]);
		}
		else {
			orderedTargets.push_back(allTargets[i3]);
			orderedTargets.push_back(allTargets[i2]);
			orderedTargets.push_back(allTargets[i1]);
			orderedTargets.push_back(allTargets[i5]);
			orderedTargets.push_back(allTargets[i4]);
		}
		return orderedTargets;
	}
};

int main(int argc, char** argv) {

	ROS_INFO("Starting pose_estimation_node");
	ros::init(argc, argv, "pose_estimation_node");

	PoseEstimation pe;

	ros::spin();

	return(0);

}
