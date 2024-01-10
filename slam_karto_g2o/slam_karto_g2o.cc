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

#include "slam_karto_g2o/slam_karto_g2o.h"

namespace slam_karto_g2o {

void SlamKartoG2o::Initialize() {
  map_to_odom_.setIdentity();

  // Set up advertisements and subscriptions
  tf_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();
  sst_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = nh_.advertiseService("dynamic_map", &SlamKartoG2o::MapCallback, this);
  scan_filter_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(
          nh_, "base_scan", 5);
  scan_filter_ = std::make_unique<tf::MessageFilter<sensor_msgs::LaserScan>>(
      *scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback([this](auto&& PH1) { LaserCallback(PH1); });

  // Initialize Karto structures.
  mapper_ = std::make_unique<karto::Mapper>();
  dataset_ = std::make_unique<karto::Dataset>();

  // Set solver to be used in loop closure.
  solver_ = std::make_unique<G2oSolver>();
  mapper_->SetScanSolver(solver_.get());

  // Retrieve parameters
  ros::NodeHandle private_nh("~");
  double transform_publish_period;
  LoadRosParamFromNodeHandle(private_nh, &transform_publish_period);

  // Create a thread to periodically publish the latest map->odom transform; it
  // needs to go out regularly, uninterrupted by potentially long periods of
  // computation in our main loop.
  transform_thread_ =
      std::make_unique<std::thread>([this, transform_publish_period] {
        PublishLoop(transform_publish_period);
      });
}

SlamKartoG2o::~SlamKartoG2o() {
  if (transform_thread_ != nullptr) {
    transform_thread_->join();
  }
}

void SlamKartoG2o::LoadRosParamFromNodeHandle(
    const ros::NodeHandle& nh, double* transform_publish_period) {
  // Sanity checks.
  CHECK_NOTNULL(mapper_);

  nh.param("map_frame", map_frame_, std::string("map"));
  nh.param("odom_frame", odom_frame_, std::string("odom"));
  nh.param("base_frame", base_frame_, std::string("base_link"));
  nh.param("map_update_interval", map_update_interval_, 5.0);
  nh.param("range_threshold", range_threshold_, 12.0);
  nh.param("transform_tolerance", transform_tolerance_, 0.0);
  nh.param("transform_publish_period", *transform_publish_period, 0.05);

  if (!nh.getParam("resolution", resolution_)) {
    // Compatibility with slam_gmapping, which uses "delta" to mean resolution.
    if (!nh.getParam("delta", resolution_)) {
      resolution_ = 0.05;
    }
  }

  // Setting general parameters from the parameter server.
  bool use_scan_matching;
  if (nh.getParam("use_scan_matching", use_scan_matching)) {
    mapper_->setParamUseScanMatching(use_scan_matching);
  }

  bool use_scan_barycenter;
  if (nh.getParam("use_scan_barycenter", use_scan_barycenter)) {
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);
  }

  double minimum_time_interval;
  if (nh.getParam("minimum_time_interval", minimum_time_interval)) {
    mapper_->setParamMinimumTimeInterval(minimum_time_interval);
  }

  double minimum_travel_distance;
  if (nh.getParam("minimum_travel_distance", minimum_travel_distance)) {
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance);
  }

  double minimum_travel_heading;
  if (nh.getParam("minimum_travel_heading", minimum_travel_heading)) {
    mapper_->setParamMinimumTravelHeading(minimum_travel_heading);
  }

  int scan_buffer_size;
  if (nh.getParam("scan_buffer_size", scan_buffer_size)) {
    mapper_->setParamScanBufferSize(scan_buffer_size);
  }

  double scan_buffer_maximum_scan_distance;
  if (nh.getParam("scan_buffer_maximum_scan_distance",
                  scan_buffer_maximum_scan_distance)) {
    mapper_->setParamScanBufferMaximumScanDistance(
        scan_buffer_maximum_scan_distance);
  }

  double link_match_minimum_response_fine;
  if (nh.getParam("link_match_minimum_response_fine",
                  link_match_minimum_response_fine)) {
    mapper_->setParamLinkMatchMinimumResponseFine(
        link_match_minimum_response_fine);
  }

  double link_scan_maximum_distance;
  if (nh.getParam("link_scan_maximum_distance", link_scan_maximum_distance)) {
    mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);
  }

  double loop_search_maximum_distance;
  if (nh.getParam("loop_search_maximum_distance",
                  loop_search_maximum_distance)) {
    mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);
  }

  bool do_loop_closing;
  if (nh.getParam("do_loop_closing", do_loop_closing)) {
    mapper_->setParamDoLoopClosing(do_loop_closing);
  }

  int loop_match_minimum_chain_size;
  if (nh.getParam("loop_match_minimum_chain_size",
                  loop_match_minimum_chain_size)) {
    mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);
  }

  double loop_match_maximum_variance_coarse;
  if (nh.getParam("loop_match_maximum_variance_coarse",
                  loop_match_maximum_variance_coarse)) {
    mapper_->setParamLoopMatchMaximumVarianceCoarse(
        loop_match_maximum_variance_coarse);
  }

  double loop_match_minimum_response_coarse;
  if (nh.getParam("loop_match_minimum_response_coarse",
                  loop_match_minimum_response_coarse)) {
    mapper_->setParamLoopMatchMinimumResponseCoarse(
        loop_match_minimum_response_coarse);
  }

  double loop_match_minimum_response_fine;
  if (nh.getParam("loop_match_minimum_response_fine",
                  loop_match_minimum_response_fine)) {
    mapper_->setParamLoopMatchMinimumResponseFine(
        loop_match_minimum_response_fine);
  }

  // Setting correlation parameters from the parameter server.
  double correlation_search_space_dimension;
  if (nh.getParam("correlation_search_space_dimension",
                  correlation_search_space_dimension)) {
    mapper_->setParamCorrelationSearchSpaceDimension(
        correlation_search_space_dimension);
  }

  double correlation_search_space_resolution;
  if (nh.getParam("correlation_search_space_resolution",
                  correlation_search_space_resolution)) {
    mapper_->setParamCorrelationSearchSpaceResolution(
        correlation_search_space_resolution);
  }

  double correlation_search_space_smear_deviation;
  if (nh.getParam("correlation_search_space_smear_deviation",
                  correlation_search_space_smear_deviation)) {
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(
        correlation_search_space_smear_deviation);
  }

  // Setting correlation parameters, loop closure parameters from the parameter
  // server.
  double loop_search_space_dimension;
  if (nh.getParam("loop_search_space_dimension", loop_search_space_dimension)) {
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);
  }

  double loop_search_space_resolution;
  if (nh.getParam("loop_search_space_resolution",
                  loop_search_space_resolution)) {
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);
  }

  double loop_search_space_smear_deviation;
  if (nh.getParam("loop_search_space_smear_deviation",
                  loop_search_space_smear_deviation)) {
    mapper_->setParamLoopSearchSpaceSmearDeviation(
        loop_search_space_smear_deviation);
  }

  // Setting scan matcher parameters from the parameter server.
  double distance_variance_penalty;
  if (nh.getParam("distance_variance_penalty", distance_variance_penalty)) {
    mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);
  }

  double angle_variance_penalty;
  if (nh.getParam("angle_variance_penalty", angle_variance_penalty)) {
    mapper_->setParamAngleVariancePenalty(angle_variance_penalty);
  }

  double fine_search_angle_offset;
  if (nh.getParam("fine_search_angle_offset", fine_search_angle_offset)) {
    mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);
  }

  double coarse_search_angle_offset;
  if (nh.getParam("coarse_search_angle_offset", coarse_search_angle_offset)) {
    mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);
  }

  double coarse_angle_resolution;
  if (nh.getParam("coarse_angle_resolution", coarse_angle_resolution)) {
    mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);
  }

  double minimum_angle_penalty;
  if (nh.getParam("minimum_angle_penalty", minimum_angle_penalty)) {
    mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);
  }

  double minimum_distance_penalty;
  if (nh.getParam("minimum_distance_penalty", minimum_distance_penalty)) {
    mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);
  }

  bool use_response_expansion;
  if (nh.getParam("use_response_expansion", use_response_expansion)) {
    mapper_->setParamUseResponseExpansion(use_response_expansion);
  }
}

void SlamKartoG2o::PublishLoop(double transform_publish_period) {
  if (transform_publish_period <= 0.0) {
    return;
  }

  ros::Rate r(1.0 / transform_publish_period);
  while (ros::ok()) {
    PublishTransform();
    r.sleep();
  }
}

void SlamKartoG2o::PublishTransform() {
  // Sanity checks.
  CHECK_NOTNULL(tf_broadcaster_);

  std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
  ros::Time tf_expiration =
      ros::Time::now() + ros::Duration(transform_tolerance_);
  tf_broadcaster_->sendTransform(tf::StampedTransform(
      map_to_odom_, tf_expiration, map_frame_, odom_frame_));
}

const karto::LaserRangeFinder* SlamKartoG2o::GetLaser(
    const sensor_msgs::LaserScan::ConstPtr& scan) {
  // Sanity checks.
  CHECK_NOTNULL(scan);

  // Check whether we know about this laser yet.
  if (lasers_.find(scan->header.frame_id) == lasers_.end()) {
    return lasers_[scan->header.frame_id];
  }

  // New laser, need to create a Karto device for it.
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = scan->header.frame_id;
  ident.stamp_ = scan->header.stamp;
  try {
    tf_.transformPose(base_frame_, ident, laser_pose);
  } catch (tf::TransformException& ex) {
    LOG(ERROR) << "Failed to compute laser pose, aborting initialization: "
               << ex.what();
    return nullptr;
  }

  double yaw = tf::getYaw(laser_pose.getRotation());
  LOG(INFO) << std::fixed << "Laser " << scan->header.frame_id.c_str()
            << "'s pose w.r.t. base: (" << laser_pose.getOrigin().x() << ","
            << laser_pose.getOrigin().y() << "," << yaw << ").";

  // Create a laser range finder device and copy in data from the first scan.
  std::string name = scan->header.frame_id;
  karto::LaserRangeFinder* laser =
      karto::LaserRangeFinder::CreateLaserRangeFinder(
          karto::LaserRangeFinder_Custom, karto::Name(name));
  laser->SetOffsetPose(
      {laser_pose.getOrigin().x(), laser_pose.getOrigin().y(), yaw});
  laser->SetMinimumRange(scan->range_min);
  laser->SetMaximumRange(scan->range_max);
  laser->SetMinimumAngle(scan->angle_min);
  laser->SetMaximumAngle(scan->angle_max);
  laser->SetAngularResolution(scan->angle_increment);
  laser->SetRangeThreshold(range_threshold_);

  // Store this laser device for later.
  lasers_[scan->header.frame_id] = laser;

  // Add it to the dataset.
  dataset_->Add(laser);
  return laser;
}

bool SlamKartoG2o::GetOdomPose(karto::Pose2* karto_pose, const ros::Time& t) {
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident(
      tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0)),
      t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  } catch (tf::TransformException& ex) {
    LOG(ERROR) << "Failed to compute odom pose, skipping scan: " << ex.what();
    return false;
  }

  double yaw = tf::getYaw(odom_pose.getRotation());
  karto_pose->SetPosition(
      {odom_pose.getOrigin().x(), odom_pose.getOrigin().y()});
  karto_pose->SetHeading(yaw);
  return true;
}

void SlamKartoG2o::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  // Sanity checks.
  if (scan == nullptr) {
    return;
  }

  // Check whether we know about this laser yet
  const karto::LaserRangeFinder* laser = GetLaser(scan);
  if (laser == nullptr) {
    LOG(ERROR) << "Failed to create laser device for "
               << scan->header.frame_id.c_str() << ", discarding scan.";
    return;
  }

  karto::Pose2 odom_pose;
  if (!AddScan(laser, scan, &odom_pose)) {
    return;
  }

  VLOG(4) << std::fixed << "Add scan at pose: (" << odom_pose.GetX() << ","
          << odom_pose.GetY() << "," << odom_pose.GetHeading() << ").";

  auto curr_timestamp = std::chrono::system_clock::now();
  double time_diff = 0.0;
  if (got_map_) {
    time_diff = (curr_timestamp - last_map_update_timestamp_).count();
  }

  if (!got_map_ || time_diff > map_update_interval_) {
    if (UpdateMap()) {
      last_map_update_timestamp_ = curr_timestamp;
      got_map_ = true;
    }
  }
}

bool SlamKartoG2o::UpdateMap() {
  std::lock_guard<std::mutex> lock(map_mutex_);

  std::unique_ptr<karto::OccupancyGrid> occ_grid(
      karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(),
                                            resolution_));

  if (occ_grid == nullptr) {
    LOG(ERROR) << "Failed to create a occupancy grid from scans.";
    return false;
  }

  if (!got_map_) {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  // Translate to ROS format.
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  const karto::Vector2<kt_double>& offset =
      occ_grid->GetCoordinateConverter()->GetOffset();

  if (map_.map.info.width != static_cast<uint32_t>(width) ||
      map_.map.info.height != static_cast<uint32_t>(height) ||
      map_.map.info.origin.position.x != offset.GetX() ||
      map_.map.info.origin.position.y != offset.GetY()) {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y = 0; y < height; ++y) {
    for (kt_int32s x = 0; x < width; ++x) {
      // Getting the value at position (x,y).
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value) {
        case karto::GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;
        default:
          LOG(ERROR) << "Encountered unknown cell value at: (" << x << "," << y
                     << ").";
          break;
      }
    }
  }

  // Set the header information on the map.
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

  return true;
}

bool SlamKartoG2o::AddScan(const karto::LaserRangeFinder* laser,
                           const sensor_msgs::LaserScan::ConstPtr& scan,
                           karto::Pose2* karto_pose) {
  // Sanity checks.
  CHECK_NOTNULL(laser);
  CHECK_NOTNULL(scan);
  CHECK_NOTNULL(karto_pose);

  if (!GetOdomPose(karto_pose, scan->header.stamp)) {
    LOG(ERROR) << "Failed to get odom pose.";
    return false;
  }

  // Create a vector of doubles for karto.
  std::vector<kt_double> readings;
  for (const float& range : scan->ranges) {
    readings.push_back(range);
  }

  // Create localized range scan.
  auto* range_scan = new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetOdometricPose(*karto_pose);
  range_scan->SetCorrectedPose(*karto_pose);

  // Add the localized range scan to the mapper.
  if (!mapper_->Process(range_scan)) {
    delete range_scan;
    return false;
  }

  // Compute the map->odom transform.
  const karto::Pose2& corrected_pose = range_scan->GetCorrectedPose();
  tf::Stamped<tf::Pose> odom_to_map;
  try {
    tf::Transform corrected_tf = tf::Transform(
        tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
        tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0));
    tf_.transformPose(odom_frame_,
                      tf::Stamped<tf::Pose>(corrected_tf.inverse(),
                                            scan->header.stamp, base_frame_),
                      odom_to_map);
  } catch (tf::TransformException& ex) {
    LOG(ERROR) << "Transform from base_link to odom failed: " << ex.what();
    odom_to_map.setIdentity();
    return false;
  }

  map_to_odom_mutex_.lock();
  map_to_odom_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()))
                     .inverse();
  map_to_odom_mutex_.unlock();

  // Add the localized range scan to the dataset (for memory management).
  dataset_->Add(range_scan);

  return true;
}

bool SlamKartoG2o::MapCallback(nav_msgs::GetMap::Request& req,
                               nav_msgs::GetMap::Response& res) {
  UNUSED(req);
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (got_map_ && map_.map.info.width != 0U && map_.map.info.height != 0U) {
    res = map_;
    return true;
  }
  return false;
}

}  // namespace slam_karto_g2o
