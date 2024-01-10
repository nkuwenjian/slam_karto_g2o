/******************************************************************************
 * Copyright (c) 2023, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jian Wen (nkuwenjian@gmail.com)
 *****************************************************************************/

#include <map>
#include <memory>
#include <mutex>  // NOLINT
#include <string>
#include <thread>  // NOLINT
#include <vector>

#include "glog/logging.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/MapMetaData.h"
#include "open_karto/Mapper.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/message_filter.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "slam_karto_g2o/g2o_solver.h"

#pragma once

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#define UNUSED(var) (void)(var)

namespace slam_karto_g2o {

class SlamKartoG2o {
 public:
  SlamKartoG2o() = default;
  virtual ~SlamKartoG2o();
  void Initialize();

 private:
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool MapCallback(nav_msgs::GetMap::Request& req,    // NOLINT
                   nav_msgs::GetMap::Response& res);  // NOLINT
  void LoadRosParamFromNodeHandle(const ros::NodeHandle& nh,
                                  double* transform_publish_period);
  bool GetOdomPose(karto::Pose2* karto_pose, const ros::Time& t);
  const karto::LaserRangeFinder* GetLaser(
      const sensor_msgs::LaserScan::ConstPtr& scan);
  bool AddScan(const karto::LaserRangeFinder* laser,
               const sensor_msgs::LaserScan::ConstPtr& scan,
               karto::Pose2* karto_pose);
  bool UpdateMap();
  void PublishTransform();
  void PublishLoop(double transform_publish_period);

  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ = nullptr;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>
      scan_filter_sub_ = nullptr;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> scan_filter_ =
      nullptr;
  ros::Publisher sst_;
  ros::Publisher sstm_;
  ros::ServiceServer ss_;
  nav_msgs::GetMap::Response map_;

  // Storage for ROS parameters.
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double map_update_interval_;

  // Range threshold of laser range finder.
  double range_threshold_;

  // Time for tolerance on the published transform, basically defines how long a
  // map->odom transform is good for.
  double transform_tolerance_;

  double resolution_;
  std::mutex map_mutex_;
  std::mutex map_to_odom_mutex_;

  // Karto bookkeeping.
  std::unique_ptr<karto::Mapper> mapper_ = nullptr;
  std::unique_ptr<karto::Dataset> dataset_ = nullptr;
  std::unique_ptr<G2oSolver> solver_ = nullptr;
  std::map<std::string, const karto::LaserRangeFinder*> lasers_;

  // Internal state.
  bool got_map_ = false;
  std::unique_ptr<std::thread> transform_thread_ = nullptr;
  tf::Transform map_to_odom_;
  std::chrono::system_clock::time_point last_map_update_timestamp_;
};

}  // namespace slam_karto_g2o
