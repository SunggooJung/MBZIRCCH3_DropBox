#include "Header.h"

//#define COLOR_FIND
#define MODE_DEBUG

#define FOCAL_LENGTH    (float)    (  310.86 )
#define OBJECT_WIDTH    (float)    (     1.0 )
#define WIDTH_OFFSET    (  int)    (      10 )
#define HEIGHT_OFFSET   (  int)    (      10 )

struct RectBoundInfo    BndInfo;
struct Color_Value      Color;
int centerX_pre = 640/2;
int centerY_pre = 480/2;

bool flag_bin = 0;

void OdroidCallback(const std_msgs::UInt8 &msg)
{
    flag_bin 		 = msg.data;
}


class ImageConverter
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher rgb_pub_;
    image_transport::Publisher box_pub_;
    image_transport::Subscriber image_sub_;
    ros::Publisher Obs_info_pub;
    std_msgs::Float32MultiArray msg_obs_pos;
    ros::Subscriber odroid_sub_;


    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/firefly_mv/image_raw", 1, &ImageConverter::imageRgb, this);
        rgb_pub_ = it_.advertise("/rgb/image_raw", 1);
        box_pub_ = it_.advertise("/rgb/detect_box", 1);
        Obs_info_pub = nh_.advertise<std_msgs::Float32MultiArray>("/obstacle/center_info",1);
        odroid_sub_ = nh_.subscribe("/flag_BIN", 1, &OdroidCallback);

    }

    ~ImageConverter()
    {

    }

    void imageRgb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        point_gray_frame = cv_ptr->image;
        // Update GUI Window        
        cv::waitKey(3);
    }
};


void on_trackbar(int, void*)
{
    //This function gets called whenever a
    // trackbar position is changed
}

void createTrackbars(){

    cv::createTrackbar("H_MIN", trackbarWindowName, &Color.H_MIN, Color.H_MAX, on_trackbar);
    cv::createTrackbar("H_MAX", trackbarWindowName, &Color.H_MAX, Color.H_MAX, on_trackbar);
    cv::createTrackbar("S_MIN", trackbarWindowName, &Color.S_MIN, Color.S_MAX, on_trackbar);
    cv::createTrackbar("S_MAX", trackbarWindowName, &Color.S_MAX, Color.S_MAX, on_trackbar);
    cv::createTrackbar("V_MIN", trackbarWindowName, &Color.V_MIN, Color.V_MAX, on_trackbar);
    cv::createTrackbar("V_MAX", trackbarWindowName, &Color.V_MAX, Color.V_MAX, on_trackbar);
}

int main(int argc, char** argv)
{       
    ros::init(argc, argv, "dropbox_detector");
    ImageConverter ic;
    ros::Rate loop_rate(20);

#ifdef COLOR_FIND
    cv::namedWindow(trackbarWindowName, cv::WINDOW_AUTOSIZE);
    createTrackbars();
#endif

    cv::Mat rgb_frame;
    cv::Mat hsv_frame;
    cv::Mat box_hsv_range;
    cv::Mat drawing;
    cv::Mat edges;

    while (ros::ok()){
        if (flag_bin){

            cout << flag_bin << "\n";

            resize(point_gray_frame,rgb_frame,cv::Point(640,480));
            width =  rgb_frame.cols;
            height = rgb_frame.rows;

            cv::vector<cv::Mat> channels;
            split(rgb_frame, channels);

            cv::Mat red_frame;
            cv::Mat green_frame;
            cv::Mat blue_frame;
            cv::Mat res;
            cv::Mat andOP;

            cv::threshold(channels[2], red_frame, 230, 255, CV_THRESH_BINARY);
            cv::threshold(channels[1], green_frame, 230, 255, CV_THRESH_BINARY);
            cv::threshold(channels[0], blue_frame, 230, 255, CV_THRESH_BINARY);

            cv::bitwise_and(red_frame, green_frame, res);
            cv::bitwise_and(res, blue_frame, andOP);

            cv::imshow("red", red_frame);
            cv::imshow("green", green_frame);
            cv::imshow("blue", blue_frame);


            dilate(andOP, andOP, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(andOP, andOP, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(andOP, andOP, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(andOP, andOP, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(andOP, andOP, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(andOP, andOP, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));

            cv::imshow("res", andOP);

            drawing = rgb_frame.clone();

            BndInfo.maxarea = 0.0;

/*
            drawing = rgb_frame.clone();
            cv::cvtColor(rgb_frame, hsv_frame, CV_BGR2HSV);

            BndInfo.maxarea = 0.0;

            cv::inRange(hsv_frame, cv::Scalar(Color.H_MIN, Color.S_MIN, Color.V_MIN), cv::Scalar(Color.H_MAX, Color.S_MAX, Color.V_MAX), box_hsv_range);

            cv::Canny(box_hsv_range, edges, 50,150, 3);
            dilate(edges, edges, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(edges, edges, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(edges, edges, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(edges, edges, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(edges, edges, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            dilate(edges, edges, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));

    #ifdef MODE_DEBUG
            cv::imshow("edges",edges);
    #endif
*/
            //--------------------------bound rect ------------------------------------------------------------------------------
            vector<vector<cv::Point> > contours;
            vector<cv::Vec4i> hierarchy;

            cv::findContours(andOP,contours,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
            vector<cv::Rect> boundRect(contours.size());

            cv::Scalar box_color = cv::Scalar(0,255,0);

            if(contours.size()!=0)
            {
                for(int i = 0; i < contours.size(); i++)
                {
                    boundRect[i] = cv::boundingRect(cv::Mat(contours[i]));
                    BndInfo.area = abs(boundRect[i].tl().x - boundRect[i].br().x) * abs(boundRect[i].tl().y - boundRect[i].br().y);
                    if (BndInfo.area > BndInfo.maxarea && BndInfo.area > 900)
                    {
                        BndInfo.maxboxind = i;
                        BndInfo.maxarea = BndInfo.area;
                    }
                }


                BndInfo.TopLeft[0] = boundRect[BndInfo.maxboxind].tl().x;
                BndInfo.TopLeft[1] = boundRect[BndInfo.maxboxind].tl().y;
                BndInfo.BottomRight[0] = boundRect[BndInfo.maxboxind].br().x;
                BndInfo.BottomRight[1] = boundRect[BndInfo.maxboxind].br().y;

                BndInfo.BoxWidth = abs(BndInfo.TopLeft[0] - BndInfo.BottomRight[0]);
                BndInfo.BoxHeight = abs(BndInfo.TopLeft[1] - BndInfo.BottomRight[1]);


                //if(BndInfo.BoxHeight<20 || BndInfo.BoxWidth < 20 || BndInfo.BoxWidth > width || BndInfo.BoxHeight > height) continue;

                cv::rectangle(drawing, boundRect[BndInfo.maxboxind].tl(), boundRect[BndInfo.maxboxind].br(),cv::Scalar(0,0,255),2,8,0);

                BndInfo.center[0] = BndInfo.BoxWidth/2.0  + BndInfo.TopLeft[0] ;
                BndInfo.center[1] = BndInfo.BoxHeight/2.0 + BndInfo.TopLeft[1] ;

                if (  ( abs(BndInfo.center[0] - centerX_pre) > 80.0 || (BndInfo.BoxWidth/BndInfo.BoxHeight) < 0.9 || (BndInfo.BoxWidth/BndInfo.BoxHeight) > 3.0 ) )// && fcc_psi_info == 1 ) // I put PSI_NAV to make traking on when it turns due to the gate over
                {
                    BndInfo.center[0] = centerX_pre;
                    BndInfo.center[1] = centerY_pre;

                    centerX_pre = 640/2 + WIDTH_OFFSET;
                    centerY_pre = 480/2 + HEIGHT_OFFSET+20;
                }

                else
                {
                    centerX_pre = BndInfo.center[0];
                    centerY_pre = BndInfo.center[1];
                }

                line(drawing, cv::Point(width/2+WIDTH_OFFSET,height/2+HEIGHT_OFFSET), cv::Point(BndInfo.center[0],BndInfo.center[1]), cv::Scalar(0, 0, 0), 2, 8, 0);
                circle(drawing, cv::Point(BndInfo.center[0],BndInfo.center[1]), 3, cv::Scalar(0, 0, 255), -1, 8, 0);
                circle(drawing, cv::Point(width/2+WIDTH_OFFSET, height/2+HEIGHT_OFFSET), 3, cv::Scalar(255, 0, 0), -1, 8, 0);

                float distance_to_obs = (FOCAL_LENGTH * OBJECT_WIDTH) / (abs(boundRect[BndInfo.maxboxind].tl().x - boundRect[BndInfo.maxboxind].br().x));


            //------------------------bound rect end----------------------------------------------------------------------------------------
                ic.msg_obs_pos.data.clear();
                ic.msg_obs_pos.data.resize(3);
                ic.msg_obs_pos.data[0] = (BndInfo.center[0] - (width/2+WIDTH_OFFSET));
                ic.msg_obs_pos.data[1] = (BndInfo.center[1] - (height/2+HEIGHT_OFFSET));
                ic.msg_obs_pos.data[2] = distance_to_obs;

                sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", rgb_frame).toImageMsg();
                sensor_msgs::ImagePtr box_msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", drawing).toImageMsg();
                ic.Obs_info_pub.publish(ic.msg_obs_pos);
                ic.rgb_pub_.publish(rgb_msg);
                ic.box_pub_.publish(box_msg);
            }
#ifdef MODE_DEBUG
        cv::imshow("rgb",drawing);
#endif

        }
        ros::spinOnce();

        cv::waitKey(20);
        cout << flag_bin << "\n";

     }
     return 0;
}




