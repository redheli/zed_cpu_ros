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
// class StereoCamera
// {
// public:
//   /**
//    * @brief      { stereo camera driver }
//    *
//    * @param[in]  resolution  The resolution
//    * @param[in]  frame_rate  The frame rate
//    */
//   StereoCamera(std::string device_name, int resolution, double frame_rate) : frame_rate_(30.0)
//   {
//     camera_ = new cv::VideoCapture(device_name);
//     cv::Mat raw;
//     cv::Mat left_image;
//     cv::Mat right_image;
//     setResolution(resolution);
//     // // this function doesn't work very well in current Opencv 2.4, so, just use ROS to control frame rate.
//     // setFrameRate(frame_rate);

//     ROS_INFO("Stereo Camera Set Resolution %d, width %f, height %f", resolution, camera_->get(WIDTH_ID),
//              camera_->get(HEIGHT_ID));
//   }

//   ~StereoCamera()
//   {
//     // std::cout << "Destroy the pointer" << std::endl;
//     delete camera_;
//   }

//   /**
//    * @brief      Sets the resolution.
//    *
//    * @param[in]  type  The type
//    */
//   void setResolution(int type)
//   {
//     ROS_INFO("tttttttttttttttttttttttttttttt %d",type);
//     switch (type)
//     {
//       case 0:
//         width_ = 4416;
//         height_ = 1242;
//         break;
//       case 1:
//         width_ = 3840;
//         height_ = 1080;
//         break;
//       case 2:
//         width_ = 2560;
//         height_ = 720;
//         break;
//       case 3:
//         width_ = 1344;
//         height_ = 376;
//         break;
//       default:
//         ROS_FATAL("Unknow resolution passed to camera: %d", type);
//     }
//     ROS_INFO("set resolutionleft aaa width %d height %d",width_,height_);
//     camera_->set(WIDTH_ID, width_);
//     camera_->set(HEIGHT_ID, height_);
//     // make sure that the number set are right from the hardware
//     width_ = camera_->get(WIDTH_ID);
//     height_ = camera_->get(HEIGHT_ID);
//     ROS_INFO("set resolutionleft width %d height %d",width_,height_);
//   }

//   /**
//    * @brief      Sets the frame rate.
//    *
//    * @param[in]  frame_rate  The frame rate
//    */
//   void setFrameRate(double frame_rate)
//   {
//     camera_->set(FPS_ID, frame_rate);
//     frame_rate_ = camera_->get(FPS_ID);
//   }

//   /**
//    * @brief      Gets the images.
//    *
//    * @param      left_image   The left image
//    * @param      right_image  The right image
//    *
//    * @return     The images.
//    */
//   bool getImages(cv::Mat& left_image, cv::Mat& right_image)
//   {
//     cv::Mat raw;
//     if (camera_->grab())
//     {
//       camera_->retrieve(raw);
//       cv::Rect left_rect(0, 0, width_ / 2, height_);
//       cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
//       left_image = raw(left_rect);
//       right_image = raw(right_rect);
//       cv::waitKey(10);
//       return true;
//     }
//     else
//     {
//       return false;
//     }
//   }

// private:
//   cv::VideoCapture* camera_;
//   int width_;
//   int height_;
//   double frame_rate_;
//   bool cv_three_;
// };


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
    params.res = sl_oc::video::RESOLUTION::HD2K;
    params.fps = sl_oc::video::FPS::FPS_15;
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

      // if (!config_file_location_.empty())
      // {

      // }
      // else
      // {
      //   ROS_FATAL("Please input zed config file path");
      // }
    // }
    // else
    // {
    //   ROS_INFO("Loading from ROS calibration files");
    //   // here we just use camera infor manager to load info
    //   // get config from the left, right.yaml in config
    //   ros::NodeHandle left_nh("left");
    //   ros::NodeHandle right_nh("right");
    //   camera_info_manager::CameraInfoManager left_info_manager(left_nh, "camera/left",
    //                                                            "package://zed_cpu_ros/config/left.yaml");
    //   left_info = left_info_manager.getCameraInfo();

    //   camera_info_manager::CameraInfoManager right_info_manager(right_nh, "camera/right",
    //                                                             "package://zed_cpu_ros/config/right.yaml");
    //   right_info = right_info_manager.getCameraInfo();

    //   left_info.header.frame_id = left_frame_id_;
    //   right_info.header.frame_id = right_frame_id_;
    // }

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

      // // TODO cut to left,right image
      // cv::Rect left_rect(0, 0, width_ / 2, height_);
      // cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
      // left_image = raw(left_rect);
      // right_image = raw(right_rect);
      // // TODO resize image if show

      // if (!zed.getImages(left_image, right_image))
      // {
      //   ROS_INFO_ONCE("Can't find camera");
      // }
      // else
      // {
      //   ROS_INFO_ONCE("Success, found camera");
      // }
      // if (show_image_)
      // {
      //   cv::imshow("left", left_image);
      //   cv::imshow("right", right_image);
      // }
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
        publishImage(right_rect, right_image_pub, "right_frame", now);
      }
      
      // if (left_cam_info_pub.getNumSubscribers() > 0)
      // {
      //   publishCamInfo(left_cam_info_pub, left_info, now);
      // }
      // if (right_cam_info_pub.getNumSubscribers() > 0)
      // {
      //   publishCamInfo(right_cam_info_pub, right_info, now);
      // }
      r.sleep();
      // since the frame rate was set inside the camera, no need to do a ros sleep
    }
  }

  /**
   * @brief      Gets the camera information From Zed config.
   *
   * @param[in]  config_file         The configuration file
   * @param[in]  resolution          The resolution
   * @param[in]  left_cam_info_msg   The left camera information message
   * @param[in]  right_cam_info_msg  The right camera information message
   */
  void getZedCameraInfo(std::string config_file, int resolution, sensor_msgs::CameraInfo& left_info,
                        sensor_msgs::CameraInfo& right_info)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);
    std::string left_str = "LEFT_CAM_";
    std::string right_str = "RIGHT_CAM_";
    std::string reso_str = "";

    switch (resolution)
    {
      case 0:
        reso_str = "2K";
        break;
      case 1:
        reso_str = "FHD";
        break;
      case 2:
        reso_str = "HD";
        break;
      case 3:
        reso_str = "VGA";
        break;
    }
    // left value
    double l_cx = pt.get<double>(left_str + reso_str + ".cx");
    double l_cy = pt.get<double>(left_str + reso_str + ".cy");
    double l_fx = pt.get<double>(left_str + reso_str + ".fx");
    double l_fy = pt.get<double>(left_str + reso_str + ".fy");
    double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
    double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
    ROS_INFO("left camera cx %f cy %f",l_cx,l_cy);
    // right value
    double r_cx = pt.get<double>(right_str + reso_str + ".cx");
    double r_cy = pt.get<double>(right_str + reso_str + ".cy");
    double r_fx = pt.get<double>(right_str + reso_str + ".fx");
    double r_fy = pt.get<double>(right_str + reso_str + ".fy");
    double r_k1 = pt.get<double>(right_str + reso_str + ".k1");
    double r_k2 = pt.get<double>(right_str + reso_str + ".k2");

    // get baseline and convert mm to m
    boost::optional<double> baselineCheck;
    double baseline = 0.0;
    // some config files have "Baseline" instead of "BaseLine", check accordingly...
    if (baselineCheck = pt.get_optional<double>("STEREO.BaseLine"))
    {
      baseline = pt.get<double>("STEREO.BaseLine") * 0.001;
    }
    else if (baselineCheck = pt.get_optional<double>("STEREO.Baseline"))
    {
      baseline = pt.get<double>("STEREO.Baseline") * 0.001;
    }
    else
    {
      throw std::runtime_error("baseline parameter not found");
    }

    // get Rx and Rz
    double rx = pt.get<double>("STEREO.RX_" + reso_str);
    double rz = pt.get<double>("STEREO.RZ_" + reso_str);
    double ry = pt.get<double>("STEREO.CV_" + reso_str);

    // assume zeros, maybe not right
    double p1 = 0, p2 = 0, k3 = 0;

    left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // TODO(dizeng) verify loading default zed config is still working

    // distortion parameters
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    left_info.D.resize(5);
    left_info.D[0] = l_k1;
    left_info.D[1] = l_k2;
    left_info.D[2] = k3;
    left_info.D[3] = p1;
    left_info.D[4] = p2;

    right_info.D.resize(5);
    right_info.D[0] = r_k1;
    right_info.D[1] = r_k2;
    right_info.D[2] = k3;
    right_info.D[3] = p1;
    right_info.D[4] = p2;

    // Intrinsic camera matrix
    // 	[fx  0 cx]
    // K =  [ 0 fy cy]
    //	[ 0  0  1]
    left_info.K.fill(0.0);
    left_info.K[0] = l_fx;
    left_info.K[2] = l_cx;
    left_info.K[4] = l_fy;
    left_info.K[5] = l_cy;
    left_info.K[8] = 1.0;

    right_info.K.fill(0.0);
    right_info.K[0] = r_fx;
    right_info.K[2] = r_cx;
    right_info.K[4] = r_fy;
    right_info.K[5] = r_cy;
    right_info.K[8] = 1.0;

    // rectification matrix
    // Rl = R_rect, R_r = R * R_rect
    // since R is identity, Rl = Rr;
    left_info.R.fill(0.0);
    right_info.R.fill(0.0);
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
    cv::Mat rmat(3, 3, CV_64F);
    cv::Rodrigues(rvec, rmat);
    int id = 0;
    cv::MatIterator_<double> it, end;
    for (it = rmat.begin<double>(); it != rmat.end<double>(); ++it, id++)
    {
      left_info.R[id] = *it;
      right_info.R[id] = *it;
    }

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    left_info.P.fill(0.0);
    left_info.P[0] = l_fx;
    left_info.P[2] = l_cx;
    left_info.P[5] = l_fy;
    left_info.P[6] = l_cy;
    left_info.P[10] = 1.0;

    right_info.P.fill(0.0);
    right_info.P[0] = r_fx;
    right_info.P[2] = r_cx;
    right_info.P[3] = (-1 * l_fx * baseline);
    right_info.P[5] = r_fy;
    right_info.P[6] = r_cy;
    right_info.P[10] = 1.0;

    left_info.width = right_info.width = width_;
    left_info.height = right_info.height = height_;
    ROS_INFO("left width %d height %d",left_info.width,left_info.height);

    left_info.header.frame_id = left_frame_id_;
    right_info.header.frame_id = right_frame_id_;
  }

  /**
   * @brief      { publish camera info }
   *
   * @param[in]  pub_cam_info  The pub camera information
   * @param[in]  cam_info_msg  The camera information message
   * @param[in]  now           The now
   */
  void publishCamInfo(const ros::Publisher& pub_cam_info, sensor_msgs::CameraInfo& cam_info_msg, ros::Time now)
  {
    cam_info_msg.header.stamp = now;
    pub_cam_info.publish(cam_info_msg);
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
