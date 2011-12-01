/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: converter.cpp 33249 2010-03-11 02:09:24Z rusu $
 *
 */

/**
@mainpage

\author Radu Bogdan Rusu

@b point_cloud_converter converts between the old (sensor_msgs::PointCloud) and the new (sensor_msgs::PointCloud2) format.
**/

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;

class PointCloudConverter 
{
  private:
    boost::mutex m_mutex_;
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_points_, sub_points2_;
    ros::Publisher pub_points_, pub_points2_;

    int queue_size_;
    string points_in_, points2_in_, points_out_, points2_out_;
    int num_subs;

  public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PointCloudConverter () : nh_ ("~"), queue_size_ (100), 
                             points_in_ ("/points_in"), points2_in_ ("/points2_in"), 
                             points_out_ ("/points_out"), points2_out_ ("/points2_out"), num_subs(0)
    {
      // Subscribe to the cloud topic using both the old message format and the new
      sub_points_ = nh_.subscribe (points_in_, queue_size_, &PointCloudConverter::cloud_cb_points, this);
      //sub_points2_ = nh_.subscribe (points2_in_, queue_size_, &PointCloudConverter::cloud_cb_points2, this);
      pub_points_ = nh_.advertise<sensor_msgs::PointCloud> (points_out_, queue_size_, boost::bind(&PointCloudConverter::connectCb, this), boost::bind(&PointCloudConverter::disconnectCb, this));
      pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2> (points2_out_, queue_size_);
      ROS_INFO ("PointCloudConverter initialized to transform from PointCloud (%s) to PointCloud2 (%s).", nh_.resolveName (points_in_).c_str (), nh_.resolveName (points2_out_).c_str ());
      ROS_INFO ("PointCloudConverter initialized to transform from PointCloud2 (%s) to PointCloud (%s).", nh_.resolveName (points2_in_).c_str (), nh_.resolveName (points_out_).c_str ());
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the available point cloud fields as a space separated string
      * \param points a pointer to the PointCloud message
      */
    inline std::string
      getFieldsList (const sensor_msgs::PointCloud2 &cloud)
    {
      std::string result;
      for (size_t i = 0; i < cloud.fields.size () - 1; ++i)
        result += cloud.fields[i].name + " ";
      result += cloud.fields[cloud.fields.size () - 1].name;
      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the available point cloud channels as a space separated string
      * \param points a pointer to the PointCloud message
      */
    inline std::string
      getFieldsList (const sensor_msgs::PointCloud &points)
    {
      std::string result = "x y z";
      for (size_t i = 0; i < points.channels.size (); i++)
        result = result + " " + points.channels[i].name;
      return (result);
    }

    void connectCb()
    {
      num_subs++;
      if(num_subs > 0)
      {
        sub_points2_ = nh_.subscribe (points2_in_, queue_size_, &PointCloudConverter::cloud_cb_points2, this);
      }
    }

    void disconnectCb()
    {
      num_subs--;
      if(num_subs <= 0)
      {
        sub_points2_.shutdown();
      }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief New format callback */
    void
      cloud_cb_points2 (const sensor_msgs::PointCloud2ConstPtr &msg)
    {
      if (pub_points_.getNumSubscribers () <= 0)
      {
        //ROS_DEBUG ("[point_cloud_converter] Got a PointCloud2 with %d points on %s, but no subscribers.", msg->width * msg->height, nh_.resolveName (points2_in_).c_str ());
        return;
      }

      ROS_DEBUG ("[point_cloud_converter] Got a PointCloud2 with %d points (%s) on %s.", msg->width * msg->height, getFieldsList (*msg).c_str (), nh_.resolveName (points2_in_).c_str ());
      
      sensor_msgs::PointCloud output;
      // Convert to the new PointCloud format
      if (!sensor_msgs::convertPointCloud2ToPointCloud (*msg, output))
      {
        ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud2 to sensor_msgs::PointCloud failed!");
        return;
      }
      ROS_DEBUG ("[point_cloud_converter] Publishing a PointCloud with %d points on %s.", (int)output.points.size (), nh_.resolveName (points_out_).c_str ());
      pub_points_.publish (output);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Old format callback */
    void
      cloud_cb_points (const sensor_msgs::PointCloudConstPtr &msg)
    {
      if (pub_points2_.getNumSubscribers () <= 0)
      {
        //ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points on %s, but no subscribers.", (int)msg->points.size (), nh_.resolveName (points_in_).c_str ());
        return;
      }

      ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points (%s) on %s.", (int)msg->points.size (), getFieldsList (*msg).c_str (), nh_.resolveName (points_in_).c_str ());

      sensor_msgs::PointCloud2 output;
      // Convert to the old point cloud format
      if (!sensor_msgs::convertPointCloudToPointCloud2 (*msg, output))
      {
        ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
        return;
      }
      ROS_DEBUG ("[point_cloud_converter] Publishing a PointCloud2 with %d points on %s.", output.height * output.width, nh_.resolveName (points2_out_).c_str ());
      pub_points2_.publish (output);
    }

};

/* ---[ */
int
  main (int argc, char** argv)
{
  // ROS init
  ros::init (argc, argv, "point_cloud_converter", ros::init_options::AnonymousName);

  PointCloudConverter p;
  ros::spin ();

  return (0);
}

