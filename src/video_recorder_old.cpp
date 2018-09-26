/****************************************************************************
* Software License Agreement (Apache License)
*
*     Copyright (C) 2012-2013 Open Source Robotics Foundation
*
*     Licensed under the Apache License, Version 2.0 (the "License");
*     you may not use this file except in compliance with the License.
*     You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
*     Unless required by applicable law or agreed to in writing, software
*     distributed under the License is distributed on an "AS IS" BASIS,
*     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*     See the License for the specific language governing permissions and
*     limitations under the License.
*
*****************************************************************************/

// this file has been slightly altered and is not the same as the
// https://github.com/ros-perception/image_pipeline/edit/indigo/image_view/src/nodes/video_recorder.cpp

// this is the version with the timer that didn't really work. i don't know what I am doing wrong, but i will give up because this seemed much slower anyway

#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif

cv::VideoWriter outputVideo;

int g_count = 0;
ros::Time g_last_wrote_time = ros::Time(0);
std::string encoding;
std::string codec;
int fps;
std::string filename;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;
double duration;
bool is_recording = false;
bool created_timer = false;

void shutdowntimerCallback(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered. Reached maximum duration of %f seconds. Shutting down.", duration);
  ros::shutdown();
}

void callback(const sensor_msgs::ImageConstPtr& image_msg)
{

  if (!outputVideo.isOpened())
    {
        cv::Size size(image_msg->width, image_msg->height);

        outputVideo.open(filename,
#if CV_MAJOR_VERSION == 3
                cv::VideoWriter::fourcc(codec.c_str()[0],
#else
                CV_FOURCC(codec.c_str()[0],
#endif
                          codec.c_str()[1],
                          codec.c_str()[2],
                          codec.c_str()[3]),
                fps,
                size,
                true);

        if (!outputVideo.isOpened())
        {
            ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
            exit(-1);
        }

        ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording.");
    }

    if ((image_msg->header.stamp - g_last_wrote_time) < ros::Duration(1 / fps))
    {
      // Skip to get video with correct fps
      return;
    }

    try
    {
      cv_bridge::CvtColorForDisplayOptions options;
      options.do_dynamic_scaling = use_dynamic_range;
      options.min_image_value = min_depth_range;
      options.max_image_value = max_depth_range;
      options.colormap = colormap;
      const cv::Mat image = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding, options)->image;
      if (!image.empty())
      {
        outputVideo << image;
        ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
        g_count++;
        is_recording = true;
        g_last_wrote_time = image_msg->header.stamp;
      }
      else
      {
          ROS_WARN("Frame skipped, no data!");
      }
    }
    catch(cv_bridge::Exception)
    {
        ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
        return;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    local_nh.param("filename", filename, std::string("output.avi"));
    bool stamped_filename;
    local_nh.param("stamped_filename", stamped_filename, false);
    local_nh.param("fps", fps, 15);
    local_nh.param("codec", codec, std::string("MJPG"));
    local_nh.param("encoding", encoding, std::string("bgr8"));
    // cv_bridge::CvtColorForDisplayOptions
    local_nh.param("min_depth_range", min_depth_range, 0.0);
    local_nh.param("max_depth_range", max_depth_range, 0.0);
    local_nh.param("use_dynamic_depth_range", use_dynamic_range, false);
    local_nh.param("colormap", colormap, -1);
    local_nh.param("duration", duration, 10.0);

    if (stamped_filename)
    {
      std::size_t found = filename.find_last_of("/\\");
      std::string path = filename.substr(0, found + 1);
      std::string basename = filename.substr(found + 1);
      std::stringstream ss;
      ss << ros::Time::now().toNSec() << basename;
      filename = path + ss.str();
      ROS_INFO("Video recording to %s", filename.c_str());
    }

    if (codec.size() != 4)
    {
        ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

    image_transport::ImageTransport it(nh);
    std::string topic = nh.resolveName("image");
    image_transport::Subscriber sub_image = it.subscribe(topic, 1, callback);

    ROS_INFO_STREAM("Waiting for topic " << topic << "...");
    ros::Rate r(10); // 1 hz#

    while(ros::ok())
    {
      ROS_INFO_STREAM("is_recording" << is_recording << "\x1b[1F");
      if (is_recording && !created_timer)
      {
        ros::Timer timer = nh.createTimer(ros::Duration(duration), shutdowntimerCallback);
        created_timer = true;
        ROS_INFO("Created shutdown timer");
      }
      ros::spinOnce();
      r.sleep();
    }
    std::cout << "\nVideo saved as " << filename << std::endl;
}
