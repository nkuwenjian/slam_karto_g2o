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

#include <memory>

#include "glog/logging.h"
#include "open_karto/Mapper.h"

#include "slam_karto_g2o/g2o_solver.h"

#define UNUSED(var) (void)(var)

namespace slam_karto_g2o {

/**
 * Sample code to demonstrate karto map creation
 * Create a laser range finder device and three "dummy" range scans.
 * Add the device and range scans to a karto Mapper.
 */
std::unique_ptr<karto::Dataset> CreateMap(karto::Mapper* mapper) {
  // Sanity checks.
  CHECK_NOTNULL(mapper);

  std::unique_ptr<karto::Dataset> dataset = std::make_unique<karto::Dataset>();

  // Create a laser range finder device - use SmartPointer to let karto
  // subsystem manage memory.
  karto::Name name("laser0");
  karto::LaserRangeFinder* laser =
      karto::LaserRangeFinder::CreateLaserRangeFinder(
          karto::LaserRangeFinder_Custom, name);
  laser->SetOffsetPose({1.0, 0.0, 0.0});
  laser->SetAngularResolution(karto::math::DegreesToRadians(0.5));
  laser->SetRangeThreshold(12.0);

  dataset->Add(laser);

  // Create three localized range scans, all using the same range readings, but
  // with different poses.

  // Create a vector of range readings. Simple example where all the
  // measurements are the same value.
  std::vector<kt_double> readings(361, 3.0);

  {
    // Create localized range scan.
    auto* range_scan = new karto::LocalizedRangeScan(name, readings);
    range_scan->SetOdometricPose({0.0, 0.0, 0.0});
    range_scan->SetCorrectedPose({0.0, 0.0, 0.0});

    // Add the localized range scan to the mapper.
    mapper->Process(range_scan);
    LOG(INFO) << std::fixed
              << "Odometric pose: " << range_scan->GetOdometricPose()
              << ", corrected pose: " << range_scan->GetCorrectedPose();

    // Add the localized range scan to the dataset.
    dataset->Add(range_scan);
  }

  {
    // Create localized range scan.
    auto* range_scan = new karto::LocalizedRangeScan(name, readings);
    range_scan->SetOdometricPose({1.0, 0.0, 1.57});
    range_scan->SetCorrectedPose({1.0, 0.0, 1.57});

    // Add the localized range scan to the mapper.
    mapper->Process(range_scan);
    LOG(INFO) << std::fixed
              << "Odometric pose: " << range_scan->GetOdometricPose()
              << ", corrected pose: " << range_scan->GetCorrectedPose();

    // Add the localized range scan to the dataset.
    dataset->Add(range_scan);
  }

  {
    // Create localized range scan.
    auto* range_scan = new karto::LocalizedRangeScan(name, readings);
    range_scan->SetOdometricPose({1.0, -1.0, 2.35619449});
    range_scan->SetCorrectedPose({1.0, -1.0, 2.35619449});

    // Add the localized range scan to the mapper.
    mapper->Process(range_scan);
    LOG(INFO) << std::fixed
              << "Odometric pose: " << range_scan->GetOdometricPose()
              << ", corrected pose: " << range_scan->GetCorrectedPose();

    // Add the localized range scan to the dataset.
    dataset->Add(range_scan);
  }

  return dataset;
}

/**
 * Sample code to demonstrate basic occupancy grid creation and print occupancy
 * grid.
 */
std::unique_ptr<karto::OccupancyGrid> CreateOccupancyGrid(
    karto::Mapper* mapper, kt_double resolution) {
  // Sanity checks.
  CHECK_NOTNULL(mapper);
  LOG(INFO) << "Generating map...";

  // Create a map (occupancy grid) - time it.
  std::unique_ptr<karto::OccupancyGrid> occ_grid(
      karto::OccupancyGrid::CreateFromScans(mapper->GetAllProcessedScans(),
                                            resolution));
  return occ_grid;
}

/**
 * Sample code to print a basic occupancy grid.
 */
void PrintOccupancyGrid(const karto::OccupancyGrid* occ_grid) {
  if (occ_grid == nullptr) {
    return;
  }

  // Output ASCII representation of map.
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset =
      occ_grid->GetCoordinateConverter()->GetOffset();

  LOG(INFO) << std::fixed << "width: " << width << ", height: " << height
            << ", scale: " << occ_grid->GetCoordinateConverter()->GetScale()
            << ", offset: (" << offset.GetX() << "," << offset.GetY() << ").";

  for (kt_int32s y = height - 1; y >= 0; y--) {
    for (kt_int32s x = 0; x < width; x++) {
      // Getting the value at position (x,y).
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value) {
        case karto::GridStates_Unknown:
          std::cout << "*";
          break;
        case karto::GridStates_Occupied:
          std::cout << "X";
          break;
        case karto::GridStates_Free:
          std::cout << " ";
          break;
        default:
          std::cout << "?";
      }
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

}  // namespace slam_karto_g2o

int main(int argc, char* argv[]) {
  UNUSED(argc);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  // Create karto default mapper.
  std::unique_ptr<karto::Mapper> mapper = std::make_unique<karto::Mapper>();

  // Create and set solver.
  std::unique_ptr<slam_karto_g2o::G2oSolver> solver =
      std::make_unique<slam_karto_g2o::G2oSolver>();
  mapper->SetScanSolver(solver.get());

  // Sample code that creates a map from sample device and sample localized
  // range scans.

  // Clear mapper.
  mapper->Reset();

  // Create map from created dataset.
  std::unique_ptr<karto::Dataset> dataset =
      slam_karto_g2o::CreateMap(mapper.get());

  // Create occupancy grid at 0.1 meters resolution and print grid.
  std::unique_ptr<karto::OccupancyGrid> occ_grid =
      slam_karto_g2o::CreateOccupancyGrid(mapper.get(), 0.1);

  slam_karto_g2o::PrintOccupancyGrid(occ_grid.get());

  return 0;
}
