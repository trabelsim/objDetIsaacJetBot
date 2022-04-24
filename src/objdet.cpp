#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/dnn.hpp>
#include <opencv4/opencv2/dnn/all_layers.hpp>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/JointState.h"

using namespace std;
using namespace cv;
using namespace dnn;

static const std::string OPENCV_WINDOW = "Image window";

static const std::string IMAGE_TOPIC = "/camera_info";
static const std::string PUBLISH_TOPIC = "/image_converter/output_video";

static cv_bridge::CvImagePtr cv_ptr;

ros::Publisher pub;

/* PATHS for OPEN CV */
// Load classes names from the coco.names file
std::vector<std::string> class_names;
/*
    classesFile path should be changed when passing to jetbot
*/
static const string classesFile = "/home/maroine/catkin_ws/src/if_isaac/classes/classes.names";
static ifstream ifs(classesFile);


/*
    readNet config and weights paths should be changed when passing to jetbot
*/
auto model = readNet("/home/maroine/catkin_ws/src/if_isaac/graphs/frozen_inference_graph.pb",
                     "/home/maroine/catkin_ws/src/if_isaac/graphs/graph.pbtxt",
                     "TensorFlow");

Mat input_temp, input;


static float velocity = 1;
static float stop = 0;
sensor_msgs::JointState msg;

void resetVelValue(){
    msg.name.push_back("left_wheel_joint");
    msg.velocity.push_back(stop);
    pub.publish(msg);
    msg.name.push_back("right_wheel_joint");
    msg.velocity.push_back(stop);
    pub.publish(msg);
}


void moveToObject(bool objOnTheRight)
{
    if (objOnTheRight) {
        msg.name.push_back("left_wheel_joint");
        msg.velocity.push_back(velocity);
        pub.publish(msg);
        ROS_INFO("Left wheel activated! %d", velocity);
    }else{
        msg.name.push_back("right_wheel_joint");
        msg.velocity.push_back(velocity);
        pub.publish(msg);
        ROS_INFO("Right wheel activated! %d", velocity);
    }
}

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
//    ROS_INFO_STREAM("New image from " << frame_id);

    // cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        input_temp = cv_ptr->image;
        if (input_temp.empty())
        {
            cout << "No input image" << endl;
            return;
        }
        int input_width = input_temp.size().width;
        int input_height = input_temp.size().height;
        resize(input_temp, input, Size(input_width, input_height), INTER_AREA);

        // Creating blob from the single frame
        // scalar factor = 1.0 (2nd argument) [multiplier for frame values]
        // bool swapRB = true [flag which indicates that swap first and last channels in 3-channel image is necessary]
        // bool crop = false [flag which indicates wheter image will be cropped after resize or not]
        // last argument ddepth (here not provided) [depth of output blob. Choose CV_32F or CV_8U]
        // Mat blob = blobFromImage(input, 1.0, Size(300, 300), Scalar(127.5, 127.5, 127.5), true, false);
        Mat blob = blobFromImage(input, 1.0, Size(300, 300), Scalar(), true, false, CV_32F);
        model.setInput(blob);

        // forward pass through the model to cagraph.pbtxtrry out detection
        Mat output = model.forward();
        //std::cout << "Output size[2] : " << output.size[2] << " , output size[3] : " << output.size[3] << std::endl;
        Mat detectionMat(output.size[2], output.size[3], CV_32F, output.ptr<float>());
        RNG rng(12345);
        for (int i = 0; i < detectionMat.rows; i++)
        {

            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            int class_id = detectionMat.at<float>(i, 1);
            float confidence = detectionMat.at<float>(i, 2);

            // Check if the detection is of good quality
            if (confidence > 0.6)
            {
                std::cout << "Considered class: " << class_id << std::endl;
                std::cout << "Confidence: " << confidence << std::endl;
                 int box_x = static_cast<int>(detectionMat.at<float>(i, 3) * input.cols);
                int box_y = static_cast<int>(detectionMat.at<float>(i, 4) * input.rows);
                int box_width = static_cast<int>(detectionMat.at<float>(i, 5) * input.cols - box_x);
                int box_height = static_cast<int>(detectionMat.at<float>(i, 6) * input.rows - box_y);
                rectangle(input, Point(box_x, box_y), Point(box_x + box_width, box_y + box_height), color, 3);
                string conf = to_string(floorf(confidence * 100) / 100);
                conf = " " + conf;
                string text = class_names[class_id - 1].c_str() + conf + "%";
                putText(input, text, Point(box_x, box_y - 5), FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                std::cout << "X width: " << box_width << ", Y height: " << box_height << std::endl;
                std::cout << "X pos: " << box_x << ", Y pos: " << box_y << std::endl;
                bool objOnTheRight = (box_x >= 600) ? true : false;
                moveToObject(objOnTheRight);
                resetVelValue();
            }
        }

        imshow("image", input);
        waitKey(1); // Best option for now for real-time

        //destroyAllWindows();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: ", e.what());
        return;
    }

    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);

    // pub.publish(cv_ptr->toImageMsg());
}



int main(int argc, char** argv){

    string line;
    while (getline(ifs, line))
        class_names.push_back(line);

    /* ROS INITIALIZER */

    ros::init(argc, argv, "listener_image");
    ros::NodeHandle n;
    pub = n.advertise<sensor_msgs::JointState>("/joint_command", 1);
    ros::Subscriber sub = n.subscribe("/rgb",1,image_cb);
    ros::Rate loop_rate(10);
    ros::spin();

    return 0;
}
