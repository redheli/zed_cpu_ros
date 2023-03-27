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

#include <zed-open-capture/videocapture_def.hpp>

#pragma once

/**
 * @brief      Gets the camera information From Zed config.
 *
 * @param[in]  config_file         The configuration file
 * @param[in]  resolution          The resolution
 * @param[in]  left_cam_info_msg   The left camera information message
 * @param[in]  right_cam_info_msg  The right camera information message
 */

// clang-format off
/**
 * @brief      Gets the camera information From Zed config.
 *
 * @param[in]  config_file         The configuration file
 * @param[in]  resolution          The resolution
 * @param[in]  left_cam_info_msg   The left camera information message
 * @param[in]  right_cam_info_msg  The right camera information message
 * @param[in]  distortion_all_zero  Set distortion params all zero, as the image is rectified
 */
void getZedCameraInfo(std::string config_file, 
                      sl_oc::video::RESOLUTION resolution, 
                      sensor_msgs::CameraInfo& left_info,
                      sensor_msgs::CameraInfo& right_info,
                      const bool distortion_all_zero = true)
// clang-format on
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(config_file, pt);
  std::string left_str = "LEFT_CAM_";
  std::string right_str = "RIGHT_CAM_";
  std::string reso_str = "";
  std::string left_frame_id = "left_camera";
  std::string right_frame_id = "right_camera";

  // std::string resolution_str;
  //   switch ((int) image_size.width) {
  //       case 2208:
  //           resolution_str = "2k";
  //           break;
  //       case 1920:
  //           resolution_str = "fhd";
  //           break;
  //       case 1280:
  //           resolution_str = "hd";
  //           break;
  //       case 672:
  //           resolution_str = "vga";
  //           break;
  //       default:
  //           resolution_str = "hd";
  //           break;
  //   }
  size_t image_width = 0;
  size_t image_height = 0;
  switch (resolution)
  {
    case sl_oc::video::RESOLUTION::HD2K:
      reso_str = "2K";
      image_width = 2208;
      image_height = 1242;
      break;
    case sl_oc::video::RESOLUTION::HD1080:
      reso_str = "FHD";
      image_width = 1920;
      image_height = 1080;
      break;
    case sl_oc::video::RESOLUTION::HD720:
      reso_str = "HD";
      image_width = 1280;
      image_height = 720;
      break;
    case sl_oc::video::RESOLUTION::VGA:
      reso_str = "VGA";
      image_width = 672;
      image_height = 376;
      break;
    default:
      throw std::runtime_error("Invalid resolution");
      break;
  }
  // left value
  double l_cx = pt.get<double>(left_str + reso_str + ".cx");
  double l_cy = pt.get<double>(left_str + reso_str + ".cy");
  double l_fx = pt.get<double>(left_str + reso_str + ".fx");
  double l_fy = pt.get<double>(left_str + reso_str + ".fy");
  double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
  double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
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
  if (distortion_all_zero == false)
  {
    left_info.D[0] = l_k1;
    left_info.D[1] = l_k2;
    left_info.D[2] = k3;
    left_info.D[3] = p1;
    left_info.D[4] = p2;
  }

  right_info.D.resize(5);
  if (distortion_all_zero == false)
  {
    right_info.D[0] = r_k1;
    right_info.D[1] = r_k2;
    right_info.D[2] = k3;
    right_info.D[3] = p1;
    right_info.D[4] = p2;
  }

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

  left_info.width = right_info.width = image_width;
  left_info.height = right_info.height = image_height;

  left_info.header.frame_id = left_frame_id;
  right_info.header.frame_id = right_frame_id;
}
