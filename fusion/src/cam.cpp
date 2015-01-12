#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>

#include <pal_detection_msgs/FaceDetection.h>
#include <pal_detection_msgs/FaceDetections.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <ctype.h>

using namespace std;
using namespace ros;
using namespace cv;

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;


bool track;

void get_frame(const sensor_msgs::Image& img)
{
	cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(img);
    cvImgPtr->image.copyTo(image);
    flip(image,image,1);
}


void get_faces(const pal_detection_msgs::FaceDetections msg)
{
   if(msg.faces.empty())
   		return;
    selection.x=msg.faces[0].x;
    selection.y=msg.faces[0].y;
    selection.width=msg.faces[0].width;
    selection.height=msg.faces[0].height;
    track=true;
    trackObject=-1;
}




int main(int argc, char **argv)
{
	init(argc, argv, "cam_node");
    NodeHandle n_sub;
    Subscriber frame_sub=n_sub.subscribe("nao_camera/image_raw", 1000, get_frame);
    Subscriber face_sub=n_sub.subscribe("pal_face/faces", 1000, get_faces);
    namedWindow("frame",cv::WINDOW_AUTOSIZE);
    Rate rate(15);
    Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
    track=false;
while (ros::ok())
    {
    	spinOnce();
        rate.sleep();      
       	if (image.empty())
       		continue;
       	if(trackObject)
       	{
			cvtColor(image, hsv, COLOR_BGR2HSV);	
       		int _vmin = vmin, _vmax = vmax;
			inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                        Scalar(180, 256, MAX(_vmin, _vmax)), mask);
             int ch[] = {0, 0};
             hue.create(hsv.size(), hsv.depth());
             mixChannels(&hsv, 1, &hue, 1, ch, 1);
             if(trackObject<1)
            {
            	Mat roi(hue, selection), maskroi(mask, selection);
            	calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
            	normalize(hist, hist, 0, 255, CV_MINMAX);
				trackWindow = selection;
                trackObject = 1;
                
                histimg = Scalar::all(0);
                int binW = histimg.cols / hsize;
                Mat buf(1, hsize, CV_8UC3);
                for( int i = 0; i < hsize; i++ )
                     buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                cvtColor(buf, buf, CV_HSV2BGR);

                for( int i = 0; i < hsize; i++ )
                {
                    int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
                    rectangle( histimg, Point(i*binW,histimg.rows),
                               Point((i+1)*binW,histimg.rows - val),
                               Scalar(buf.at<Vec3b>(i)), -1, 8 );
                }
          	}
       		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
            backproj &= mask;
            RotatedRect trackBox = CamShift(backproj, trackWindow,
                                    TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
            if( trackWindow.area() <= 1 )
               {
                    int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
                    trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                       trackWindow.x + r, trackWindow.y + r) &
                                  Rect(0, 0, cols, rows);
                }

                if( backprojMode )
                    cvtColor( backproj, image, COLOR_GRAY2BGR );
                ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );
            }
        }
       		
       		
        imshow( "frame", image );
		waitKey(30);
    }

    return 0;
}
