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
 * ROS stack name: cob_vision
 * ROS package name: cob_env_model
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 05/2011
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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>


class FixGazeboPointcloud
{
public:
	typedef sensor_msgs::PointCloud PointCloud;

	FixGazeboPointcloud()
	{
		sub_ = nh_.subscribe("full_cloud", 1, &FixGazeboPointcloud::input_callback, this);
		pub_ = nh_.advertise<sensor_msgs::PointCloud> ("full_cloud_fixed", 1);
	}

	void input_callback(const PointCloud::Ptr &cloud)
	{
		//ROS_INFO("[ConcatNormals] SyncCallback");
		//ROS_INFO("channels: %d", cloud->channels.size());
		cloud->channels.clear();
		//ROS_INFO("channels: %d", cloud->channels.size());
		//cloud->channels[0].name = std::string("unknown");

		pub_.publish (cloud);

	}

	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fix_gazebo_pointcloud_node");
	FixGazeboPointcloud fix;

	ros::spin();

	return 0;
}
