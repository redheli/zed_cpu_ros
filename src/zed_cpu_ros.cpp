#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include <zed-open-capture/calibration.hpp>
#include <zed-open-capture/ocv_display.hpp>
#include <zed-open-capture/videocapture.hpp>
#include <zed-open-capture/videocapture.hpp>


#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5

namespace arti
{

/*!
 * \brief Rescale the OpenCV images [cv::Mat] according to the selected resolution to better display them on screen and show
 * \param name Name of the display window
 * \param img Image to be displayed
 * \param res Image resolution
 * \param change_name Add rescaling information in window name if true
 * \param info optional info string
 */
void showImage( std::string name, cv::Mat& img, sl_oc::video::RESOLUTION res, bool change_name=true, std::string info="" )
{
    cv::Mat resized;
    switch(res)
    {
    default:
    case sl_oc::video::RESOLUTION::VGA:
        resized = img;
        break;
    case sl_oc::video::RESOLUTION::HD720:
        if(change_name) name += " [Resize factor 0.6]";
        cv::resize( img, resized, cv::Size(), 0.6, 0.6 );
        break;
    case sl_oc::video::RESOLUTION::HD1080:
    case sl_oc::video::RESOLUTION::HD2K:
        if(change_name) name += " [Resize factor 0.4]";
        cv::resize( img, resized, cv::Size(), 0.4, 0.4 );
        break;
    }

    if(!info.empty())
    {
        cv::putText( resized, info, cv::Point(20,40),cv::FONT_HERSHEY_SIMPLEX, 0.75,
                     cv::Scalar(100,100,100), 2);
    }

    cv::imshow( name, resized );
}

/**
 * @brief       the camera ros warpper class
 */
class ZedCameraROS
{
public:
  /**
   * @brief      { function_description }
   *
   * @param[in]  resolution  The resolution
   * @param[in]  frame_rate  The frame rate
   */
  ZedCameraROS()
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    // get ros param
    private_nh.param("resolution", resolution_, 1);
    // private_nh.param("frame_rate", frame_rate_, 30.0);
    private_nh.param("config_file_location", config_file_location_, std::string(""));
    private_nh.param("left_frame_id", left_frame_id_, std::string("left_camera"));
    private_nh.param("right_frame_id", right_frame_id_, std::string("right_camera"));
    private_nh.param("show_image", show_image_, false);
    private_nh.param("use_zed_config", use_zed_config_, true);
    private_nh.param("device_name", device_name_, std::string("/dev/video0"));
    private_nh.param("encoding", encoding_, std::string("brg8"));

    // setup publisher stuff
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
    image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

    image_transport::Publisher left_image_rect_pub = it.advertise("left/image_rect", 1);
    image_transport::Publisher right_image_rect_pub = it.advertise("right/image_rect", 1);

    // ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
    // ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

    sensor_msgs::CameraInfo left_info, right_info;


    // TODO use resolution from param
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD1080;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = 1;

    // ----> Create Video Capture
    sl_oc::video::VideoCapture cap_0(params);

    frame_rate_ = int(params.fps);

    if( !cap_0.initializeVideo() )
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        throw std::runtime_error("initializeVideo fail");
    }

    std::cout << "Connected to camera sn: " << cap_0.getSerialNumber() << "[" << cap_0.getDeviceName() << "]" << std::endl;

    // set brightness value
    const int brightness_expect = 8;
    auto curr_brightness = cap_0.getBrightness();
    std::cout<<"current brightness "<<curr_brightness<<std::endl;
    cap_0.setBrightness( brightness_expect );
    curr_brightness = cap_0.getBrightness();
    std::cout<<"new brightness value "<<curr_brightness<<std::endl;

    // correctFramerate(resolution_, frame_rate_);

    // ROS_INFO("Try to initialize the camera , resolution %d",resolution_);
    // StereoCamera zed(device_name_, resolution_, frame_rate_);
    // ROS_INFO("Initialized the camera");

    // ----> Frame size
    int w,h;
    cap_0.getFrameSize(w,h);
    // <---- Frame size

    ROS_INFO("Try load camera calibration files");
    // if (use_zed_config_)
    // {
      ROS_INFO("Loading from zed calibration files");
      // get camera info from zed
      // TODO check file exist
      if(sl_oc::tools::checkFile(config_file_location_) == false)
      {
         throw std::runtime_error("zed calibration file not exist <"+config_file_location_+">");
      }


      // ----> Initialize calibration
      cv::Mat map_left_x, map_left_y;
      cv::Mat map_right_x, map_right_y;
      cv::Mat cameraMatrix_left, cameraMatrix_right;
      sl_oc::tools::initCalibration(config_file_location_, cv::Size(w/2,h), map_left_x, map_left_y, map_right_x, map_right_y,
                      cameraMatrix_left, cameraMatrix_right);

      std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
      std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;
      // ----> Initialize calibration

      cv::Mat frameBGR, left_raw, left_rect, right_raw, right_rect;

      uint64_t last_ts=0;

     

    // ROS_INFO("Got camera calibration files");
    // // loop to publish images;
    // cv::Mat left_image, right_image;
    ros::Rate r(frame_rate_);

    while (nh.ok())
    {
      ros::Time now = ros::Time::now();
      // Get last available frame
      const sl_oc::video::Frame frame = cap_0.getLastFrame();

      // ----> If the frame is valid we can convert, rectify and display it
      if(frame.data==nullptr) 
      {
        ROS_ERROR("Frame Data Empty!");
        continue;
      }
      if(frame.timestamp == last_ts){
        ROS_ERROR("Old Frame!");
        continue;
      }
      
#ifdef TEST_FPS
            if(lastFrameTs!=0)
            {
                // ----> System time
                double now = static_cast<double>(getSteadyTimestamp())/1e9;
                double elapsed_sec = now - lastTime;
                lastTime = now;
                std::cout << "[System] Frame period: " << elapsed_sec << "sec - Freq: " << 1./elapsed_sec << " Hz" << std::endl;
                // <---- System time

                // ----> Frame time
                double frame_dT = static_cast<double>(frame.timestamp-lastFrameTs)/1e9;
                std::cout << "[Camera] Frame period: " << frame_dT << "sec - Freq: " << 1./frame_dT << " Hz" << std::endl;
                // <---- Frame time
            }
            lastFrameTs = frame.timestamp;
#endif

      last_ts = frame.timestamp;
      // TODO compare ROS now and frame.timestamp

      // ----> Conversion from YUV 4:2:2 to BGR for visualization
      cv::Mat frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
      cv::Mat frameBGR;
      cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
      // <---- Conversion from YUV 4:2:2 to BGR for visualization
      // ----> Extract left and right images from side-by-side
      left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
      right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

      // Display images
      if (show_image_){
        // only use left
        showImage("left RAW", left_raw, params.res);
        // showImage("right RAW", right_raw, params.res);
      }
      // <---- Extract left and right images from side-by-side

      // ----> Apply rectification
      cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_LINEAR );
      cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_LINEAR );

      if (show_image_){
        // only use left
        // showImage("right RECT", right_rect, params.res);
        showImage("left RECT", left_rect, params.res);
        // ----> Keyboard handling
        int key = cv::waitKey( 5 );
        if(key=='q' || key=='Q') // Quit
            break;
        // <---- Keyboard handling
      }

      if (left_image_pub.getNumSubscribers() > 0)
      {
        publishImage(left_raw, left_image_pub, "left_frame", now);
      }
      if (right_image_pub.getNumSubscribers() > 0)
      {
        publishImage(right_raw, right_image_pub, "right_frame", now);
      }
      if (left_image_rect_pub.getNumSubscribers() > 0)
      {
        publishImage(left_rect, left_image_rect_pub, "left_frame", now);
      }
      if (right_image_rect_pub.getNumSubscribers() > 0)
      {
        publishImage(right_rect, right_image_rect_pub, "right_frame", now);
      }
      
      r.sleep();
      // since the frame rate was set inside the camera, no need to do a ros sleep
    }
  }

  /**
   * @brief      { publish image }
   *
   * @param[in]  img           The image
   * @param      img_pub       The image pub
   * @param[in]  img_frame_id  The image frame identifier
   * @param[in]  t             { parameter_description }
   * @param[in]  encoding      image_transport encoding
   */
  void publishImage(const cv::Mat& img, image_transport::Publisher& img_pub, const std::string& img_frame_id,
                    ros::Time t)
  {
    cv_bridge::CvImage cv_image;
    // TODO(dizeng) maybe we can save a copy here?
    // or it seems like CV mat is passing by reference?
    cv_image.image = img;
    // TODO(dizeng)
    // by default the cv::mat from zed is bgr8, here just chaing encoding seems
    // doesn't work, need to implement conversion function specificly
    cv_image.encoding = encoding_;
    cv_image.header.frame_id = img_frame_id;
    cv_image.header.stamp = t;
    auto msg = cv_image.toImageMsg();
    ROS_INFO("publish image rows %d cols %d data %ld encoding %s",img.rows,
      img.cols, msg->data.size(),encoding_.c_str());
    img_pub.publish(msg);
  }
  /**
   * @brief      Correct frame rate according to resolution
   *
   * @param[in]  resolution          The resolution
   * @param      frame_rate   			 The camera frame rate
   */
  void correctFramerate(const int resolution, double& frame_rate)
  {
    double max_frame_rate;
    std::string reso_str = "";
    switch (resolution)
    {
      case 0:
        max_frame_rate = 15;
        reso_str = "2K";
        break;
      case 1:
        max_frame_rate = 30;
        reso_str = "FHD";
        break;
      case 2:
        max_frame_rate = 60;
        reso_str = "HD";
        break;
      case 3:
        max_frame_rate = 100;
        reso_str = "VGA";
        break;
      default:
        ROS_FATAL("Unknow resolution passed");
        return;
    }
    if (frame_rate > max_frame_rate)
      ROS_WARN("frame_rate(%fHz) too high for resolution(%s), downgraded to %fHz", frame_rate, reso_str.c_str(),
               max_frame_rate);
    frame_rate = max_frame_rate;
  }

private:
  int resolution_;
  std::string device_name_;
  double frame_rate_;
  bool show_image_, use_zed_config_;
  double width_, height_;
  std::string left_frame_id_, right_frame_id_;
  std::string config_file_location_;
  std::string encoding_;
};
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "zed_camera");
    arti::ZedCameraROS zed_ros;
    return EXIT_SUCCESS;
  }
  catch (std::runtime_error& e)
  {
    ros::shutdown();
    return EXIT_FAILURE;
  }
}
