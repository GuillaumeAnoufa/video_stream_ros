#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <csignal>
#include <iostream>
#include <thread>

inline void SignalHandler(int signum) {
  printf("ros video driver will exit\r\n");
  ros::shutdown();
  exit(signum);
}


int main(int argc, char **argv)
{
    /** Ros related */
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                        ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle video_node;
    signal(SIGINT, SignalHandler);

    std::string source_type = "cam";
    std::string video_source = "/dev/video0";
    int width = 0;
    int height = 0;
    int fps = 0;

    video_node.getParam("source_type", source_type);
    video_node.getParam("video_source", video_source);
    video_node.getParam("width", width);
    video_node.getParam("height", height);
    video_node.getParam("fps", fps);

    image_transport::ImageTransport it(video_node);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

    cv::VideoCapture cap_;
    int openingAttempts = 0;
    
    std::cout << "config : " << width << ", " << height << ", " << fps << "\n";

    while(!cap_.isOpened()) {
        try {
            cap_.open(video_source);
        }
        catch (std::exception& e) {
            std::cout << "Unable to open camera at" << video_source << ", exception caught : " << std::string(e.what()) << std::endl;
        }
        openingAttempts++;
        if(cap_.isOpened()) {
            std::cout << "Successfully opened camera at " << video_source << std::endl;
            break;
        }
        if(openingAttempts < 10) {
            std::cout << "Error: Couldn't open camera at " << video_source << " attempt[" << std::to_string(openingAttempts) << "]  retrying...." << std::endl;
        } else {
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(500));
    }

    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width);
    cap_.set(cv::CAP_PROP_FPS,          fps);

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    
    ros::Rate camera_fps_rate(fps);
    while (video_node.ok()) {
        cap_ >> frame;
        // Check if grabbed frame is actually full with some content
        if(!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        if (source_type == "videofile")
        {
            camera_fps_rate.sleep();
        }
    }

    return 0;
}
