 /* 
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
*/
#ifndef APRILTAG2_ROS_H
#define APRILTAG2_ROS_H


//Basic includes
#include <iostream>
#include <sstream>
#include <string>

// Opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

// Include for the apriltags
#include <apriltag.h>
#include <tag36h11.h>
#include <tag36h10.h>
#include <tag36artoolkit.h>
#include <tag25h9.h>
#include <tag25h7.h>
#include <tag16h5.h>
#include <apriltag_msgs/AprilTagDetection.h>
#include <apriltag_msgs/AprilTagDetectionArray.h>
#include "getopt.h" 

// ROS includes
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

#include "boost/thread/mutex.hpp"

namespace apriltag2_detector_ros {
    class Node {
        public:
            Node();
            void spin(int argc, char** argv);

        private:
            boost::mutex lock_;
            ros::NodeHandle nh_;
            unsigned long queue_size_;
            cv_bridge::CvImagePtr cv_ptr;
            std_msgs::Header image_header_;
            bool got_image_;
            int freq_;
            std::queue<sensor_msgs::ImageConstPtr> image_queue;
            
            boost::mutex image_queue_lock;

            double tag_size;
            double hamming;
            std::string tag_family;
            std::string image_topic;
            std::string detection_topic;
            std::string detection_array_topic;
            std::string blur_percent;

            bool check_num_tag;
            int num_tag;

            
            void AddObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d >& objectPoints);
            void AddImagePoints (apriltag_detection_t *detection, std::vector<cv::Point2d >& imagePoints);
            void AddImageOuterPoints (apriltag_detection_t *detection,std::vector<cv::Point2d >& imageOuterPoints);
            Eigen::Matrix4d GetRelativeTransform(std::vector<cv::Point3d> objectPoints,
                                                 std::vector<cv::Point2d> imagePoints,
                                                 double &fx, double &fy, double &cx, double &cy, 
                                                 cv::Mat_<float> &DistCoeff);
            geometry_msgs::PoseStamped MakeTagPose(
                                        const Eigen::Matrix4d& transform,
                                        const Eigen::Quaternion<double> &rot_quaternion,
                                        const std_msgs::Header& header);
            static int IdComparison(const void* first, const void* second);
            void RemoveDuplicates (zarray_t &detections);
            void WaitForImage();
            void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
            void FrameCallback(const sensor_msgs::ImageConstPtr& image);
            // void FrameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
            void ComputeCog(double p[4][2], double (&returnArray)[2]);
            void RotationMatrixZ(double alpha, cv::Mat_<float>& R_z_90);
            cv::Mat getOrderBuff();
    };
}

#endif 
