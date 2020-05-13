#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <iomanip>


static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}
std_msgs::String output; 

void centre(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr raw;
    raw = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(raw->image, gray, CV_BGR2GRAY);

    // Use Canny instead of threshold to catch squares with gradient shading
	cv::Mat bw;
	cv::Canny(gray, bw, 0, 50, 5);

    std::vector<std::vector<cv::Point> > contours;
	cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<cv::Moments> sc(contours.size());
    for( size_t i=0;i<contours.size();i++){
        sc[i] = moments( contours[i] );
    }

    std::vector<cv::Point> approx;
	cv::Mat dst = raw->image.clone();

	for (int i = 0; i < contours.size(); i++)
	{
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

		// Skip small or non-convex objects 
		if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx)){
			continue;}

		if (approx.size() == 3){
			setLabel(dst, "TRI", contours[i]);    // Triangles
		}
		else if (approx.size() >= 4 && approx.size() <= 6)
		{
			// Number of vertices of polygonal curve
			int vtc = approx.size();

			// Get the cosines of all corners
			std::vector<double> cos;
			for (int j = 2; j < vtc+1; j++){
				cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));
            }
			// Sort ascending the cosine values
			std::sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();

			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3){
				setLabel(dst, "RECT", contours[i]);
				for( size_t i = 0; i < contours.size(); i++ ){
        			output.data= std::to_string(sc[i].m10/(sc[i].m00 + 1e-5)) + "," + std::to_string(sc[i].m01/(sc[i].m00 + 1e-5));
        			std::cout<<sc[i].m10/(sc[i].m00 + 1e-5)<<" , "<<sc[i].m01/(sc[i].m00 + 1e-5)<<std::endl;
    			}
			}
			else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27){
				setLabel(dst, "PENTA", contours[i]);}
			else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45){
        		setLabel(dst, "HEXA", contours[i]);}
		}
	}
    return;   

}
int main(int argc, char **argv){
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("image",10, centre);
  ros::Publisher pub = nh.advertise<std_msgs::String>("image_topic",10);
  ros::Rate loopRate(60);
  while(ros::ok()){
    ros::spinOnce();
    pub.publish(output);
    loopRate.sleep();
  }
  return 0;
}

