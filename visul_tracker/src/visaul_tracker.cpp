//
// Created by skywoodsz on 2021/11/12.
//
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>                                                               
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui_c.h>
#include "visul_tracker/serial_pc2mcu.h"
//#include "PID_controller.h"
//#include "chessboard.h"
#include <opencv2/aruco.hpp>

#define SER

using namespace std;
using namespace cv;

void GenerateAruco()
{
    Mat markerImage;
    // Load the predefined dictionary
    Ptr<cv::aruco::Dictionary>dictionary=aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // Generate the marker
    aruco::drawMarker(dictionary, 33, 200, markerImage, 1);

    imwrite("./aruco33.png", markerImage);
}

bool get_msg = false;
geometry_msgs::Pose target_pose;
void angle_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	get_msg = true;
    geometry_msgs::PoseStamped pose = *msg;
    target_pose = pose.pose;

    // cout<<"get!!"<<endl;
}

int main(int argc, char *argv[])
{
 	ros::init(argc, argv, "aruco_tracker");
	ros::NodeHandle node_handle;
	ros::Subscriber sub = node_handle.subscribe("/tracker/angle", 10, angle_callback);
  	ros::Publisher pub = node_handle.advertise<geometry_msgs::PoseStamped>("/tracker/aruco",10);
	ros::Publisher origin_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/tracker/origin",10);
	geometry_msgs::PoseStamped aruco_pose, origin_pose, send_pose;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;

	#ifdef SER
		Serial_PC2MCU serial;
		string port = "/dev/ttyTHS0"; //COM need to fix
		if(!serial.init(port, 15200))
		{
		    exit(0);
		}
    #endif
	

    // 0. 相机校正
    const cv::Mat K = ( cv::Mat_<double> ( 3,3 )
            << 1259.157807, 0, 626.587968, 0, 1259.207534, 323.224541, 0, 0, 1);
    const cv::Mat D = ( cv::Mat_<double> ( 5,1 )
            << -0.396304, 0.324814, -0.003378, -0.002047, 0.000000);

    Ptr<cv::aruco::Dictionary>dictionary=aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    Mat frame;
    VideoCapture capture(1);
    double origin[3], traker[3];
    bool origin_init = false;
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        capture >> frame;
        if(frame.empty())
        {
            std::cout<<"Can't capture the video!"<<std::endl;
            exit(0);
        }
        // 0. 校正
        cv::Mat UndistortImage;
        cv::undistort(frame, UndistortImage, K, D, K);

        // aruco detect
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;

        cv::aruco::detectMarkers(UndistortImage, dictionary, corners, ids);// if at least one marker detected

        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(UndistortImage, corners, ids);

            std::vector<cv::Vec3d> rvecs;
            std::vector<cv::Vec3d> tvecs;

            cv::aruco::estimatePoseSingleMarkers(corners, 0.02, K, D, rvecs, tvecs); // draw axis for each marker

            for(int i = 0; i<ids.size(); i++)
            {
                if (!origin_init)
                {
                    putText(UndistortImage, "Wait for the origin init!", Point(50, 50),
                            cv::FONT_HERSHEY_COMPLEX,2, cv::Scalar(0, 255, 255), 2, 8, 0);
                    if(ids[i] == 55)
                    {
                        origin_init = true;

                        origin[0] = tvecs[i][0];
                        origin[1] = tvecs[i][1]; // - tvecs[i][1];
                        origin[2] = tvecs[i][2];

						origin_pose.header.stamp = ros::Time::now();
                        origin_pose.pose.position.x = origin[0];
						origin_pose.pose.position.y = origin[1];
						origin_pose.pose.position.z = origin[2];
						origin_pub.publish(origin_pose);
                    }
                }
                else
                {
                    if(ids[i] == 33)
                    {
                        traker[0] = tvecs[i][0];
                        traker[1] = tvecs[i][1];
                        traker[2] = tvecs[i][2];

						tf::Quaternion q;/*定义四元数*/
                        q.setRPY(rvecs[i][0],rvecs[i][1],rvecs[i][2]);

                        broadcaster.sendTransform( //tf创建的对象发布消息
                        tf::StampedTransform(		  
                            tf::Transform(q, tf::Vector3(traker[1], traker[0], traker[2])),
                            ros::Time::now(),"aruco_link", "camera_link")); 
                        
                        tf::StampedTransform transform;
                        try{
                            listener.lookupTransform("/camera_link", "/aruco_link",
                                                ros::Time(0), transform);
                        }
                            catch (tf::TransformException &ex) {
                            ROS_ERROR("%s",ex.what());
                            ros::Duration(1.0).sleep();
                            continue;
                        }

						// aruco_pose.header.stamp = ros::Time::now();
						// aruco_pose.pose.position.x = traker[0]; // G
						// aruco_pose.pose.position.y = - traker[1]; // R
						// aruco_pose.pose.position.z = traker[2];

                        aruco_pose.header.stamp = ros::Time::now();
                        aruco_pose.pose.position.x = traker[0];
						aruco_pose.pose.position.y = traker[1];
						aruco_pose.pose.position.z = traker[2];

						send_pose.header.stamp = ros::Time::now();
                        send_pose.pose.position.x = aruco_pose.pose.position.x - origin_pose.pose.position.x;
						send_pose.pose.position.y = aruco_pose.pose.position.y - origin_pose.pose.position.y;
						send_pose.pose.position.z = aruco_pose.pose.position.z - origin_pose.pose.position.z;
                        
                        pub.publish(send_pose);

						origin_pose.header.stamp = ros::Time::now();
						origin_pub.publish(origin_pose);
                    }
                }
                putText(UndistortImage, "Tracking...", Point(50, 50),
                        cv::FONT_HERSHEY_COMPLEX,2, cv::Scalar(0, 255, 255), 2, 8, 0);

                cv::aruco::drawAxis(UndistortImage, K, D, rvecs[i], tvecs[i], 0.1);
            }
        }

        int bx, by, y0;
		#if (defined SER)
        // cout<<"get_msg: "<<get_msg<<endl;
				if(get_msg)
				{
					// serial.send_data (bx, by, lx, ly);
                    bx = target_pose.position.y * 130;
                    by = target_pose.position.z * 0.006 + y0;
					serial.send_data (bx, by);
					//serial.send_data (0, 0, 0, 0);
					std::cout<<"send the data!"<<std::endl;
				}
		#endif


        // std::cout<<"the origin is: "<<origin[0]<<", "<<origin[1]<<", "<<origin[2]<<std::endl;
        // std::cout<<"the tracking is: "<< traker[0]<<", "<< traker[1]<<", "<< traker[2]<<std::endl;
        std::cout<<"the tracking is: "<<aruco_pose.pose.position.x - origin_pose.pose.position.x
        <<", "<<aruco_pose.pose.position.y - origin_pose.pose.position.y
        <<", "<<aruco_pose.pose.position.z - origin_pose.pose.position.z
        <<std::endl;

        // std::cout<<"the tracking is: "<<aruco_pose.pose.position.x
        // <<", "<<aruco_pose.pose.position.y 
        // <<", "<<aruco_pose.pose.position.z 
        // <<std::endl;
        float temp_x, temp_y;
        temp_x = send_pose.pose.position.x; 
        temp_y = send_pose.pose.position.y;
        putText(UndistortImage, "space: ", Point(10, 200),
        cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0, 0, 255), 1, 8, 0);
        putText(UndistortImage, to_string(temp_x)
        + ", " + to_string(temp_y)
        , Point(10, 230),
        cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0, 0, 255), 1, 8, 0);

        putText(UndistortImage, "angle: ", Point(10, 260),
        cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0, 0, 255), 1, 8, 0);
        putText(UndistortImage, to_string(target_pose.position.y)
        + ", " + to_string(target_pose.position.z)
        , Point(10, 290),
        cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0, 0, 255), 1, 8, 0);

        putText(UndistortImage, "send: ", Point(10, 320),
        cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0, 0, 255), 1, 8, 0);
        putText(UndistortImage, to_string(bx)
        + ", " + to_string(by)
        , Point(10, 350),
        cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0, 0, 255), 1, 8, 0);

        cv::imshow("out", UndistortImage);

        if(waitKey(20) == 'q')
            break;
        ros::spinOnce();
        // loop_rate.sleep();
    }

    capture.release();
    destroyAllWindows();
    return 0;
}



