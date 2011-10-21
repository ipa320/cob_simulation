/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_point_cloud_publisher
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: August 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// ROS includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

using namespace message_filters;

typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy;


//####################
//#### PcPublisher class ####
class CreateColoredPointCloud : public nodelet::Nodelet
{
private:
  ros::NodeHandle n_; ///< ROS node handle

  // topics to subscribe, callback is called for new messages arriving
  image_transport::ImageTransport image_transport_;       ///< Image transport instance
  image_transport::SubscriberFilter color_image_sub_;       ///< Subscribes to gray image data
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;

  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > pc_sync_;

  //sensor_msgs::CvBridge cv_bridge_0_; ///< Converts ROS image messages to openCV IplImages
  cv::Mat c_color_image_8U3_;   ///< Received color values from sensor fusion

  // topics to publish
  ros::Publisher pc_pub_;

public:
  // Constructor
  CreateColoredPointCloud()
  : image_transport_(n_)
  {}

  // Destructor
  ~CreateColoredPointCloud()
  {
  }

  void initNode()
  {
    pc_sync_ = boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(3)));
    pc_pub_ = n_.advertise<sensor_msgs::PointCloud2>("colored_point_cloud2", 1);
    color_image_sub_.subscribe(image_transport_,"image_color", 1);
    pc_sub_.subscribe(n_, "point_cloud2", 1);

    pc_sync_->connectInput(color_image_sub_, pc_sub_);
    pc_sync_->registerCallback(boost::bind(&CreateColoredPointCloud::syncCallback, this, _1, _2));
  }

  virtual void onInit()
  {
    n_ = getNodeHandle();
    image_transport_ = image_transport::ImageTransport(n_);
    initNode();
  }


  // topic callback functions
  // function will be called when a new message arrives on a topic
  void syncCallback(const sensor_msgs::Image::ConstPtr& img_rgb, const sensor_msgs::PointCloud2::ConstPtr& pc)
  {
    sensor_msgs::PointCloud2 cpc_msg;
    // create point_cloud message
    cpc_msg.header = pc->header;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img_rgb,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //c_color_image_8U3_ = cv_ptr->image;//cv_bridge_0_.imgMsgToCv(img_rgb, "passthrough");
    //cv::Mat cpp_color_image_8U3 = c_color_image_8U3_;

    cpc_msg.width = pc->width;
    cpc_msg.height = pc->height;
    cpc_msg.fields.resize(4);
    cpc_msg.fields[0].name = "x";
    cpc_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cpc_msg.fields[1].name = "y";
    cpc_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cpc_msg.fields[2].name = "z";
    cpc_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cpc_msg.fields[3].name = "rgb";
    cpc_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    int offset = 0;
    for (size_t d = 0; d < cpc_msg.fields.size(); d++, offset += 4)
    {
      cpc_msg.fields[d].offset = offset;
    }
    cpc_msg.point_step = offset;
    cpc_msg.row_step = cpc_msg.point_step * cpc_msg.width;
    cpc_msg.data.resize (cpc_msg.width*cpc_msg.height*cpc_msg.point_step);
    cpc_msg.is_dense = true;
    cpc_msg.is_bigendian = false;

    unsigned char* c_ptr = 0;
    int cpc_msg_idx=0;
    for (int row = 0; row < cv_ptr->image.rows; row++)
    {
      c_ptr = cv_ptr->image.ptr<unsigned char>(row);
      //TODO: remove filtered points
      for (int col = 0; col < cv_ptr->image.cols; col++, cpc_msg_idx++)
      {
        memcpy(&cpc_msg.data[cpc_msg_idx * cpc_msg.point_step], &pc->data[cpc_msg_idx * pc->point_step], 3*sizeof(float));
        memcpy(&cpc_msg.data[cpc_msg_idx * cpc_msg.point_step + cpc_msg.fields[3].offset], &c_ptr[3*col], 3*sizeof(unsigned char));
      }
    }
    pc_pub_.publish(cpc_msg);
  }

};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, specify name of node
  ros::init(argc, argv, "cpc_publisher");

  CreateColoredPointCloud cpcPublisher;
  cpcPublisher.initNode();

  ros::spin();

  return 0;
}


//PLUGINLIB_DECLARE_CLASS(cob_env_model, CreateColoredPointCloud, CreateColoredPointCloud, nodelet::Nodelet);



