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

#include "apriltag2_ros.h"
#include "homography.h"
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <time.h>
// #include "utils.cpp"


// Namespaces
using namespace std;
using namespace cv;

#define FONTSIZE 1.0

// only publish one specific ID
#define LIMIT_PUBLISH 0
#define PUBLISH_ID 0

int IdComparison_ (const void* first, const void* second) {
    int id1 = ((apriltag_detection_t*) first)->id;
    int id2 = ((apriltag_detection_t*) second)->id;
    //return (id1 > id2) - (id1 < id2);
    //return (id1 -id2);
    if (id1 > id2) {    
        return 1;
    }    
    if (id1 < id2) {    
        return -1;
    }    
    return 0;
    //return ( *(int*)id1 - *(int*)id2 );
    //return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
}


namespace apriltag2_detector_ros {

	int Node::IdComparison (const void* first, const void* second) {
		int id1 = ((apriltag_detection_t*) first)->id;
		int id2 = ((apriltag_detection_t*) second)->id;
        /*
        return (id1 > id2) - (id1 < id2);
        return (id1 -id2);
        if (id1 > id2) {    
            return 1;
        }    
        if (id1 < id2) {    
            return -1;
        }    
        return 0;
        return ( *(int*)id1 - *(int*)id2 );
        */
		return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
	}


	void Node::RemoveDuplicates (zarray_t &Detections) {
        // remove an apriltag that appears on an image more than one time
        
        // The comparison function will be passed a pointer to two elements to be compared
        // and should return a measure of the difference between them (see strcmp()).
        // I.e. it should return a negative number if the first element is 'less than'
        // the second, zero if they are equivalent, and a positive number if the first
        // element is 'greater than' the second. The function should have the following format:
        // 
        // int comparison_function(const element_type *first, const element_type *second)
        
        // Detections are a array of detected apriltags
        //qsort(Detections.data, Detections.size, Detections.el_sz, IdComparison_); //zarray_sort(&Detections, &IdComparison);
		//zarray_sort(&Detections, &IdComparison);
        //size_t arraySize = sizeof(Detections.data)/sizeof(*Detections.data);
        //sort((Detections.data), (Detections.data)+arraySize);

        /*
        for (int i=0; i<zarray_size(&Detections); i++){
			apriltag_detection_t *Detection1; // An apriltag
            zarray_get(&Detections, i, &Detection1);
            cout << "Detections: " << endl 
                 << Detection1->id
                 << endl;
        }
        cout << "zarray_size(&Detections): " << zarray_size(&Detections) << endl;
        exit(0);
        */
		int Counts = 0;
		bool DuplicateDetected = false;

            // cout << "zarray_size(&Detections)-1: " << zarray_size(&Detections)-1 << endl;
		while (true) {

			if (Counts > zarray_size(&Detections)-1) {
				// The entire detection set has been parsed
				return;
			}

			apriltag_detection_t *Detection; // An apriltag
			zarray_get(&Detections, Counts, &Detection);
			int id_current = Detection->id;

			// Default id_next value of -1 ensures that if the last detection
			// is a duplicated tag ID, it will get removed
			int id_next = -1;

			if (Counts < zarray_size(&Detections)-1) {
				zarray_get(&Detections, Counts+1, &Detection);
				id_next = Detection->id;
			}


			if (id_current == id_next || (id_current != id_next && DuplicateDetected)) {
                //cout << "DuplicateDetected!" << endl;
				DuplicateDetected = true;

				// Remove the current tag detection from detections array
                // If shuffle is true, the last element in the array will be placed in
                // the newly-open space; if false, the zarray is compacted.
				int shuffle = 0;
				zarray_remove_index(&Detections, Counts, shuffle);

				if (id_current != id_next) {
					ROS_WARN_STREAM("Remove ID: " << id_current << " because "
									"appearing more than once in this image.");
					DuplicateDetected = false; // Reset
				}
				continue;
			}
			else {
				Counts++;
			}
		}
	}


	geometry_msgs::PoseStamped Node::MakeTagPose(
					const Eigen::Matrix4d& Transform,
					const Eigen::Quaternion<double> &RotQuaternion,
					const std_msgs::Header& header) {
		geometry_msgs::PoseStamped pose;
		pose.header = header;

		/*===== Position and orientation */
		pose.pose.position.x    = Transform(0, 3);
		pose.pose.position.y    = Transform(1, 3);
		pose.pose.position.z    = Transform(2, 3);
		pose.pose.orientation.x = RotQuaternion.x();
		pose.pose.orientation.y = RotQuaternion.y();
		pose.pose.orientation.z = RotQuaternion.z();
		pose.pose.orientation.w = RotQuaternion.w();

		return pose;
	}


	Eigen::Matrix4d Node::GetRelativeTransform( std::vector<cv::Point3d> objectPoints,
												std::vector<cv::Point2d> imagePoints,
												double &fx, double &fy, double &cx, double &cy, 
                                                Mat_<float> &DistCoeff) {

	  // perform Perspective-n-Point camera pose estimation using the
	  // above 3D-2D point correspondences
	  cv::Mat rvec, tvec;
	  cv::Matx33d CameraMatrix(fx,  0, cx,
							   0,  fy, cy,
							   0,   0,  1);
	  cv::Vec4f DistCoeffs(DistCoeff(0), DistCoeff(1), DistCoeff(2), DistCoeff(3)); // distortion coefficients

	  // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
	  // need to first check WHAT is a bottleneck in this code, and only
	  // do this if PnP solution is the bottleneck.
	  cv::solvePnP(objectPoints, imagePoints, CameraMatrix, DistCoeffs, rvec, tvec);
	  cv::Matx33d R;
	  cv::Rodrigues(rvec, R);
	  Eigen::Matrix3d wRo;
	  wRo << R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2);

	  Eigen::Matrix4d T; // homogeneous transformation matrix
	  T.topLeftCorner(3, 3) = wRo;
	  T.col(3).head(3) <<
		  tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
	  T.row(3) << 0,0,0,1;
	  return T;
	}


	void Node::AddObjectPoints (
		double s, cv::Matx44d T_oi, std::vector<cv::Point3d >& objectPoints) {

	  // Add to object point vector the tag corner coordinates in the bundle frame
	  // Going counterclockwise starting from the bottom left corner
	  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s,-s, 0, 1));
	  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s,-s, 0, 1));
	  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s, s, 0, 1));
	  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s, s, 0, 1));
	}

	void Node::AddImagePoints (
		apriltag_detection_t *Detection,
		std::vector<cv::Point2d >& imagePoints) {

	  // Add to image point vector the tag corners in the image frame
	  // Going counterclockwise starting from the bottom left corner
	  double tag_x[4] = {-1,1,1,-1};
	  double tag_y[4] = {1,1,-1,-1}; // Negated because AprilTag tag local
									 // frame has y-axis pointing DOWN
									 // while we use the tag local frame
									 // with y-axis pointing UP
	  for (int i=0; i<4; i++) {
		// Homography projection taking tag local frame coordinates to image pixels
		double im_x, im_y;
		homography_project(Detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
		imagePoints.push_back(cv::Point2d(im_x, im_y));
	  }
	}


    void Node::AddImageOuterPoints (
        apriltag_detection_t *Detection,
        std::vector<cv::Point2d >& imageOuterPoints) {

      // Add to image point vector the tag corners in the image frame
      // Going counterclockwise starting from the bottom left corner
      double tag_inner_x[4] = {-1,1,1,-1};
      double tag_inner_y[4] = {1,1,-1,-1}; // Negated because AprilTag tag local
                                     // frame has y-axis pointing DOWN
                                     // while we use the tag local frame
                                     // with y-axis pointing UP
      double white_border = 1.0;
      // double black_border = 1.0;
      double bit_size = 2.0 / (2*Detection->family->black_border + Detection->family->d);
      // double bit_size = 2.0 / (Detection->family->width_at_border);
      double wsz = white_border*bit_size;

      double tag_outer_x[4] = {-1-wsz,1+wsz,1+wsz,-1-wsz};
      double tag_outer_y[4] = {1+wsz,1+wsz,-1-wsz,-1-wsz};

      for (int i=0; i<4; i++) {
        // Homography projection taking tag local frame coordinates to image pixels
        double im_x, im_y;
        homography_project(Detection->H, tag_outer_x[i], tag_outer_y[i], &im_x, &im_y);
        imageOuterPoints.push_back(cv::Point2d(im_x, im_y));
      }
    }


    void Node::RotationMatrixZ(double alpha, cv::Mat_<float> (&R_z_90)) {
        double alpha_rad = alpha*M_PI/180;
        R_z_90 << cos(alpha_rad), -sin(alpha_rad), 0.0, sin(alpha_rad), cos(alpha_rad), 0.0, 0.0, 0.0, 1.0;
    }

    void Node::ComputeCog(double p[4][2], double (&returnArray)[2]) {

        // find the center of the apriltag
        int cog_x, cog_y, temp1, temp2;

        for (int var = 0; var < 4; ++var) {
            temp1 = temp1 + p[var][0];
            temp2 = temp2 + p[var][1];
        }
        cog_x = temp1/4;
        cog_y = temp2/4;

        returnArray[0] = cog_x;
        returnArray[1] = cog_y;
    }


    void Node::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {

        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    // records last received image
    void Node::FrameCallback(const sensor_msgs::ImageConstPtr& image) {
        got_image_ = 1; // flag to make sure it receives a image at the very begining of the program
        image_header_ = image->header;
        boost::mutex::scoped_lock(image_queue_lock);
        image_queue.push(image);
        // ROS_INFO_STREAM("Queue size: " << image_queue.size());

    }
    // void Node::FrameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info) {
    //     boost::mutex::scoped_lock(lock_);
    //     image_header_ = image->header;
    //     try {
    //         // copy realsense images to here and also decode into CV format
    //         cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    //     }
    //     catch (cv_bridge::Exception& e) {
    //         ROS_ERROR("cv_bridge exception: %s", e.what());
    //         return;
    //     }
    //     got_image_ = true;
    // }

    cv::Mat Node::getOrderBuff(){
        image_queue_lock.lock();
        if (image_queue.size()==0) {
            image_queue_lock.unlock();
            //ros::spinOnce();
            //cout << "Pointcloud queue is empty" << endl;
            //cout << "size: " << Empty.size() << endl;
            cv::Mat Empty;
            return Empty;
        }
        // ROS_INFO_STREAM("Queue size: " << image_queue.size());
        sensor_msgs::ImageConstPtr image = image_queue.front();
        // Convert to sensor_msg to pcl type
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        image_queue.pop();
        image_queue_lock.unlock();

        return cv_ptr->image;
    }

    void Node::WaitForImage() {
        while ( ros::ok ()){
            if(got_image_) return;
            ros::spinOnce();
        }
    }

    Node::Node():
        nh_("~"),
        queue_size_(1),
        cv_ptr(),
        image_header_(),
        got_image_(false) {
        nh_.param("tag_size", tag_size, tag_size);
        nh_.param("tag_family",tag_family, tag_family);
        nh_.param("image_topic", image_topic, image_topic);
        nh_.param("detection_topic", detection_topic, detection_topic);
        nh_.param("detection_array_topic", detection_array_topic, detection_array_topic);
        nh_.param("blur_percent",blur_percent,blur_percent);
        nh_.param("hamming", hamming, hamming);

        nh_.param("check_num_tag", check_num_tag, check_num_tag);
        nh_.param("num_tag", num_tag, num_tag);
        cout<<"-->In Node() constructor!"<<endl;

    }

    void Node::spin(int argc, char** argv) {

        // message_filters::Subscriber<sensor_msgs::Image> raw_image_subscriber(nh_, 
        //                                                                      image_topic, 
        //                                                                      queue_size_);
        // message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber(nh_, 
        //                                                                             camera_info_topic, 
        //                                                                             queue_size_);

        cout<< "camera_image_topic: " <<  image_topic << endl;
        // cout<< "camera_info_topic: "  <<  camera_info_topic  << endl;

        // message_filters::TimeSynchronizer<sensor_msgs::Image, 
        //                                   sensor_msgs::CameraInfo> image_info_sync(raw_image_subscriber, 
        //                                                                            camera_info_subscriber, 
        //                                                                            queue_size_);

        // image_info_sync.registerCallback(boost::bind(&Node::FrameCallback, this, _1, _2));
        ros::Subscriber image_subscriber = nh_.subscribe(image_topic, queue_size_, &Node::FrameCallback, this);
        ros::Publisher detections_pub_array_ = nh_.advertise<apriltag_msgs::AprilTagDetectionArray>(detection_array_topic, 1000);
        ros::Publisher detections_pub_ = nh_.advertise<apriltag_msgs::AprilTagDetection>(detection_topic, 1000);
        ros::Rate loop_rate(100);
        cv::namedWindow("view");
        cv::startWindowThread();

        image_transport::ImageTransport it(nh_); // subscribe to and publish images
        image_transport::Subscriber sub = it.subscribe(image_topic, 1, &Node::ImageCallback, this);
        cout << "OpenCV Version used: " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << endl;

        //wait for an image to be ready
        ROS_INFO("waiting for IMAGE...");
        WaitForImage(); {
            ROS_INFO("...Finished waiting for IMAGE!!");

            //when an image is ready ....
            boost::mutex::scoped_lock(lock_);
        }

        // parse apriltags' options
        getopt_t *getopt = getopt_create();
        getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
        getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
        getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
        getopt_add_string(getopt, 'f', "family", tag_family.c_str(), "Tag family to use");
        getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
        getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
        getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
        getopt_add_double(getopt, 'b', "blur", blur_percent.c_str(), "Apply low-pass blur to input");
        getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
        // getopt_add_bool(getopt, '1', "refine-decode", 1, "Spend more time trying to decode tags");
        // getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

        if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) 
        {
            printf("Usage: %s [options]\n", argv[0]);
            getopt_do_usage(getopt);
            exit(0);
        }


        // Initialize tag detector with options
        apriltag_family_t *tf = new apriltag_family_t;
        const char *famname = getopt_get_string(getopt, "family");
        cout << "Creating apriltag_detector..." << endl;
        apriltag_detector_t *td = apriltag_detector_create();
        cout<<"Starting the APRILTAG LOOP"<<endl;
// Apriltag
        if (!strcmp(famname, "tag36h11")){
            tf = tag36h11_create();
        }
        else if (!strcmp(famname, "tag36h10"))
            tf = tag36h10_create();
        else if (!strcmp(famname, "tag36artoolkit"))
            tf = tag36artoolkit_create();
        else if (!strcmp(famname, "tag25h9"))
            tf = tag25h9_create();
        else if (!strcmp(famname, "tag25h7"))
            tf = tag25h7_create();
        else if (!strcmp(famname, "tag16h5")) {
            cout << "-->creating tag16h5 family" << endl;
            tf = tag16h5_create();
            cout << "-->created tag16h5 family" << endl;
        }
        else {
            printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
            exit(-1);
        }
        // tf->black_border = getopt_get_int(getopt, "border");

        // apriltag_detector_t *td = apriltag_detector_create();
        apriltag_detector_add_family_bits(td, tf, hamming);
        td->quad_decimate = getopt_get_double(getopt, "decimate");
        td->quad_sigma = getopt_get_double(getopt, "blur");
        td->nthreads = getopt_get_int(getopt, "threads");
        td->debug = getopt_get_bool(getopt, "debug");
        td->refine_edges = getopt_get_bool(getopt, "refine-edges");
        // td->refine_decode = getopt_get_bool(getopt, "refine-decode");
        // td->refine_pose = getopt_get_bool(getopt, "refine-pose");

        // int image_counter = 0;
        while(ros::ok()) {
            // In the following variable it will be put the coordinates of the center of the tag
            double cog [2];
            Mat frame, gray;
            cv::Mat image = getOrderBuff();
            if(!image.empty()) {

                // boost::mutex::scoped_lock(lock_);
                // cv::Mat image = cv_ptr->image;

                // image_counter = image_counter + 1;
                // std::cout << "image_counter: " << image_counter << std::endl;
                frame = image;
                cv::cvtColor(frame, gray, 6); // apriltags would be better if it is gray
                
                // Make an image_u8_t header for the Mat data
                image_u8_t im = { .width = gray.cols,
                                  .height = gray.rows,
                                  .stride = gray.cols,
                                  .buf = gray.data};

                // Run AprilTag2 algorithm on the image
                zarray_t *Detections = apriltag_detector_detect(td, &im);

                /*
				 * Restriction: any tag ID can appear at most once in the scene. Thus, get all
				 * the tags visible in the scene and remove any tags with IDs of which there
				 * are multiple in the scene
                 */
                
                // cout << "<<<< before remove" << endl;
                // cout << "TAGSIZE: " << zarray_size(Detections) << endl;
				// Node::RemoveDuplicates(*Detections);
                // cout << "<<<< after remove" << endl;
                // cout << "TAGSIZE: " << zarray_size(Detections) << endl;

                apriltag_msgs::AprilTagDetectionArray TagDetectedArray;
                for (int i = 0; i < zarray_size(Detections); i++) {

					// Get the i-th detected tag
                    apriltag_detection_t *det;
                    zarray_get(Detections, i, &det);
                    // Draw detection outlines
                    line(frame, Point(det->p[0][0], det->p[0][1]),
                            Point(det->p[1][0], det->p[1][1]),
                            Scalar(0, 0xff, 0), 2);
                    line(frame, Point(det->p[0][0], det->p[0][1]),
                            Point(det->p[3][0], det->p[3][1]),
                            Scalar(0, 0, 0xff), 2);
                    line(frame, Point(det->p[1][0], det->p[1][1]),
                            Point(det->p[2][0], det->p[2][1]),
                            Scalar(0xff, 0, 0), 2);
                    line(frame, Point(det->p[2][0], det->p[2][1]),
                            Point(det->p[3][0], det->p[3][1]),
                            Scalar(0xff, 0, 0), 2);
                    
                    /** Calibration from ROS package

                     * ('D = ', [0.10113390451526362, -0.2484389017231441, 0.0008114847894993265, 
                     * 	      0.0021082808374029375, 0.0])
                     * ('K = ', [604.4744033162482, 0.0, 324.48119524409725, 
                     * 		  0.0, 603.6064555379237, 245.52629168364535, 0.0, 0.0, 1.0])
                     * ('R = ', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
                     * ('P = ', [609.8197631835938, 0.0, 325.6439227767478, 
                     * 		  0.0, 0.0, 610.3912353515625, 
                     * 		  245.79869719891576, 0.0, 0.0, 
                     * 		  0.0, 1.0, 0.0])
                     * None
                     * # oST version 5.0 parameters

                     * [image]

                     * width
                     * 640

                     * height
                     * 480

                     * [narrow_stereo]

                     * camera matrix
                     * 604.474403 0.000000 324.481195
                     * 0.000000 603.606456 245.526292
                     * 0.000000 0.000000 1.000000

                     * distortion
                     * 0.101134 -0.248439 0.000811 0.002108 0.000000

                     * rectification
                     * 1.000000 0.000000 0.000000
                     * 0.000000 1.000000 0.000000
                     * 0.000000 0.000000 1.000000

                     * projection
                     * 609.819763 0.000000 325.643923 0.000000
                     * 	0.000000 610.391235 245.798697 0.000000
                     * 	0.000000 0.000000 1.000000 0.000000
                     * */


                    Node::ComputeCog(det->p, cog); // find the center of the apriltag
                    Mat_<float> CameraMatrix(3,3);
                    Mat_<float> DistCoeffs(5,1);

                    CameraMatrix << 616.3681640625, 0.0, 319.93463134765625,
                                    0.0, 616.7451171875, 243.6385955810547,
                                     0.00000000, 0.00000000, 1.000000;
                    DistCoeffs << 0.101134, -0.248439, 0.000811, 0.002108, 0.000000;

                    // Get camera intrinsic properties
                    double fx,fy,cx,cy;
                    fx = CameraMatrix(0);
                    fy = CameraMatrix(4);
                    cx = CameraMatrix(2);
                    cy = CameraMatrix(5);

                    // Camera 15357017 parameters
                    // Points initialization
                    // Only 2 ponts in this example, in real code they are read from file.
                    Mat_<Point2f> points(1,1);
                    points(0) = Point2f(cog[0],cog[1]);
                    Mat_<float> rectificationMatrix(3,3);
                    rectificationMatrix << 1.000000, 0.000000, 0.000000, 
                                           0.000000, 1.000000, 0.000000, 
                                           0.000000, 0.000000, 1.000000;

                    // Undistorsion of the pixels we are interested in
                    Mat dst; // leave empty, opencv will fill it.
                    cv::undistortPoints(points, dst, CameraMatrix, DistCoeffs, rectificationMatrix);

                    //===================================================================
                    // Get estimated tag pose in the camera frame.
                    //
                    // Note on frames:
                    // The raw AprilTags 2 uses the following frames:
                    //   - camera frame: looking from behind the camera (like a
                    //     photographer), x is right, y is up and z is towards you
                    //     (i.e. the back of camera)
                    //   - tag frame: looking straight at the tag (oriented correctly),
                    //     x is right, y is down and z is away from you (into the tag).
                    // But we want:
                    //   - camera frame: looking from behind the camera (like a
                    //     photographer), x is right, y is down and z is straight
                    //     ahead
                    //   - tag frame: looking straight at the tag (oriented correctly),
                    //     x is right, y is up and z is towards you (out of the tag).
                    // Using these frames together with cv::solvePnP directly avoids
                    // AprilTag 2's frames altogether.


                    // using apriltag method
                    matd_t *M = homography_to_pose(det->H, fx, fy, cx, cy);
                    Eigen::Matrix3d Rot;
                    Rot << MATD_EL(M, 0, 0), MATD_EL(M, 0, 1), MATD_EL(M, 0, 2), 
                           MATD_EL(M, 1, 0), MATD_EL(M, 1, 1), MATD_EL(M, 1, 2), 
                           MATD_EL(M, 2, 0), MATD_EL(M, 2, 1), MATD_EL(M, 2, 2);
                    // cout << "\n\n\n----------------------------------------" << endl;
                    // cout << "R: \n" << Rot << endl;
                    // cout << "T: \n" << MATD_EL(M, 0, 3) << ", " << 
                    //                    MATD_EL(M, 1, 3) << ", " << 
                    //                    MATD_EL(M, 2, 3) << endl;
                    // cout << "R2: \n" << Rot2 << endl;
                    // cout << "T2: \n" << Transform.topRightCorner(1, 3) << endl;
                    // cout << "det(Rot): " << Rot.determinant() << endl;
                    // cout << "det(Rot2): " << Rot2.determinant() << endl;

                    // using other method
                    std::vector<cv::Point3d> standaloneTagObjectPoints;
                    std::vector<cv::Point2d> standaloneTagImagePoints;
                    // std::vector<cv::Point2d> standaloneTagImageOuterPoints;
                    Node::AddObjectPoints(tag_size/2, cv::Matx44d::eye(), standaloneTagObjectPoints);
                    Node::AddImagePoints(det, standaloneTagImagePoints);
                    // Node::AddImageOuterPoints(det, standaloneTagImageOuterPoints);
                    Eigen::Matrix4d Transform = GetRelativeTransform(standaloneTagObjectPoints,
                                                                     standaloneTagImagePoints,
                                                                     fx, fy, cx, cy, 
                                                                     DistCoeffs);
                    Eigen::Matrix3d Rot2 = Transform.block(0, 0, 3, 3);
                    Eigen::Quaternion<double> RotQuaternion(Rot2);

                    static tf::TransformBroadcaster Broadcaster;
                    tf::Transform Transform_pub;
                    Transform_pub.setOrigin(tf::Vector3(Transform(0, 3), Transform(1, 3), Transform(2, 3)));
                    double norm = std::sqrt(std::pow(RotQuaternion.x(), 2) + 
                                            std::pow(RotQuaternion.y(), 2) + 
                                            std::pow(RotQuaternion.z(), 2) + 
                                            std::pow(RotQuaternion.w(), 2));
                    tf::Quaternion q(RotQuaternion.x(), RotQuaternion.y(), RotQuaternion.z(), RotQuaternion.w());
                    Transform_pub.setRotation(q);
                    Broadcaster.sendTransform(tf::StampedTransform(Transform_pub, cv_ptr->header.stamp, 
                                                                   "camera_color_optical_frame", to_string(det->id)));

                    // cout << "q: " << RotQuaternion.x() << ", " 
                    //               << RotQuaternion.y() << ", "
                    //               << RotQuaternion.z() << ", " 
                    //               << RotQuaternion.w() << endl;

                    geometry_msgs::PoseStamped pose =
                                Node::MakeTagPose(Transform, RotQuaternion, cv_ptr->header);

                    // Try to get the position of the tag w.r.t. the camera
                    geometry_msgs::PoseArray TagArray;

                    /* The following is what will be written on the image (in the tag)*/ 

                    // it is very convenient to use stringstream 
                    // to convert between strings and other numerical types
                    stringstream ID;
                    ID << setprecision(4) << 
                            "ID: "<< det->id << 
                            " Bearing = [ " << pose.pose.position.x << ","
                                    << pose.pose.position.y << ", "
                                    << pose.pose.position.z << " ]";

                    putText(frame, ID.str(), Point2f(25, 70+50*(i-1)), 
                            FONT_HERSHEY_PLAIN, FONTSIZE,  Scalar(255,255,255));

                    stringstream Distance;
                    Distance << setprecision(4) << 
                            "Range = [ " 
                                    << MATD_EL(M, 0, 3) << ", "
                                    << MATD_EL(M, 1, 3) << ", "
                                    << MATD_EL(M, 2, 3) << " ]";
                    putText(frame, Distance.str(), Point2f(90, 90+50*(i-1)), 
                            FONT_HERSHEY_PLAIN, FONTSIZE,  Scalar(255,255,255));

                    // put ID on the apriltag
                    int baseline = 0;
                    Size textsize = getTextSize(to_string(det->id), FONT_HERSHEY_SCRIPT_SIMPLEX, 
                                                FONTSIZE, 2, &baseline);
                    putText(frame, to_string(det->id), Point(det->c[0] - textsize.width/2,
                                                             det->c[1] + textsize.height/2),
                            FONT_HERSHEY_SCRIPT_SIMPLEX, FONTSIZE, Scalar(0xff, 0x99, 0), 2);
                    cv::circle(frame, cv::Point(cog[0], cog[1]), 20, CV_RGB(255,0,0));
                    cv::circle(frame, cv::Point(0, 0), 20, CV_RGB(0,255,0));
                    


                    // Add the detection to the back of the tag detection array

                    if (LIMIT_PUBLISH){
                        if (det->id == PUBLISH_ID){
                            apriltag_msgs::AprilTagDetection TagDetected;
                            TagDetected.header = image_header_;
                            TagDetected.frame_index = image_header_.seq;
                            TagDetected.pose.header = pose.header;
                            TagDetected.pose.pose.position = pose.pose.position;
                            TagDetected.pose.pose.orientation = pose.pose.orientation;
                            TagDetected.id = det->id;
                            TagDetected.size = tag_size;
                            for(int i = 0; i < 4; i++){
                                geometry_msgs::Point inner_p;
                                geometry_msgs::Point outer_p;
                                inner_p.x = det->p[i][0];
                                inner_p.y = det->p[i][1];
                                cv::circle(frame, cv::Point(inner_p.x, inner_p.y), 20, CV_RGB(255,0,0));
                                inner_p.z = 0;
                                outer_p.x = det->outer_p[i][0];
                                outer_p.y = det->outer_p[i][1];
                                cv::circle(frame, cv::Point(outer_p.x, outer_p.y), 20, CV_RGB(0,255,0));
                                outer_p.z = 0;
                                TagDetected.inner_corners.push_back(inner_p);
                                TagDetected.outer_corners.push_back(outer_p);
                            }
                            TagDetectedArray.detections.push_back(TagDetected);
                            TagArray.poses.push_back(pose.pose);
                            detections_pub_.publish(TagDetected);
                        }
                    }
                    else {
                        apriltag_msgs::AprilTagDetection TagDetected;
                        TagDetected.header = image_header_;
                        TagDetected.frame_index = image_header_.seq;
                        TagDetected.pose.header = pose.header;
                        TagDetected.pose.pose.position = pose.pose.position;
                        TagDetected.pose.pose.orientation = pose.pose.orientation;
                        TagDetected.id = det->id;
                        // std::cout << "apriltag center : " << cog[0] << ", " << cog[1] <<std::endl;
                        // Just for testing, manually assign the id here
                        // if (cog[0] < 300) TagDetected.id = 1;
                        // else TagDetected.id =2;
                        TagDetected.size = tag_size;
                        for(int i = 0; i < 4; i++){
                            geometry_msgs::Point inner_p;
                            geometry_msgs::Point outer_p;
                            inner_p.x = det->p[i][0];
                            inner_p.y = det->p[i][1];
                            cv::circle(frame, cv::Point(inner_p.x, inner_p.y), 5, CV_RGB(255,0,0));
                            inner_p.z = 0;
                            outer_p.x = det->outer_p[i][0];
                            outer_p.y = det->outer_p[i][1];
                            cv::circle(frame, cv::Point(outer_p.x, outer_p.y), 5, CV_RGB(0,255,0));
                            outer_p.z = 0;
                            TagDetected.inner_corners.push_back(inner_p);
                            TagDetected.outer_corners.push_back(outer_p);
                        }
                        TagDetectedArray.detections.push_back(TagDetected);
                        TagArray.poses.push_back(pose.pose);
                        detections_pub_.publish(TagDetected);
                    }

                }
                TagDetectedArray.header = image_header_;
                TagDetectedArray.frame_index = image_header_.seq;
                // srand(time(0));
                // if (rand()%10 < 7)
                if (TagDetectedArray.detections.size()> num_tag && check_num_tag){
                    std::cout << "first:  " << TagDetectedArray.detections[0].id <<"  second:  " << TagDetectedArray.detections[1].id <<"  Third: "<< TagDetectedArray.detections[2].id <<std::endl;
                    ROS_INFO_STREAM("Apriltag got wrong tags");

                } 
                detections_pub_array_.publish(TagDetectedArray);
                // std::cout << "published array" << std::endl;
                zarray_destroy(Detections);
                cv::destroyWindow("view");
                imshow("Tag Detections", frame);
                if (waitKey(30) >= 0)   break;
            } 
            else{
                // std::cout << "empty" << std::endl;
            }
            ros::spinOnce();
                // loop_rate.sleep();
        }

        //	Destruction of the apriltag_detector
        apriltag_detector_destroy(td);
        if (!strcmp(famname, "tag36h11"))
            tag36h11_destroy(tf);
        else if (!strcmp(famname, "tag36h10"))
            tag36h10_destroy(tf);
        else if (!strcmp(famname, "tag36artoolkit"))
            tag36artoolkit_destroy(tf);
        else if (!strcmp(famname, "tag25h9"))
            tag25h9_destroy(tf);
        else if (!strcmp(famname, "tag25h7"))
            tag25h7_destroy(tf);
        getopt_destroy(getopt);
    }
}
