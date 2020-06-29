/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/drivers/gnss/parser/data_parser.h"

#include <proj_api.h>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>

#include "Eigen/Geometry"
#include "boost/array.hpp"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/drivers/gnss/gnss_gflags.h"
#include "modules/drivers/gnss/parser/newtonm2_parser.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"
#include "modules/drivers/gnss/util/utils.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "ros/include/ros/ros.h"
#include "std_msgs/String.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::common::adapter::AdapterManager;
using ::apollo::drivers::gnss::EpochObservation;
using ::apollo::drivers::gnss::GnssBestPose;
using ::apollo::drivers::gnss::GnssEphemeris;
using ::apollo::drivers::gnss::Heading;
using ::apollo::drivers::gnss::Imu;
using ::apollo::drivers::gnss::Ins;
using ::apollo::drivers::gnss::InsStat;
using ::apollo::localization::CorrectedImu;
using ::apollo::localization::Gps;

namespace {

constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
const char *WGS84_TEXT = "+proj=latlong +ellps=WGS84";

// covariance data for pose if can not get from novatel inscov topic
static const boost::array<double, 36> POSE_COVAR = {
    2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01};

template <class T>
void PublishMessageRaw(const ros::Publisher &pub, const T *pb) {
  std_msgs::String msg_pub;

  if (pb->SerializeToString(&msg_pub.data)) {
    pub.publish(msg_pub);
    return;
  }
  AERROR << "Failed to serialize message.";
}

Parser *CreateParser(config::Config config, bool is_base_station = false) {
  switch (config.data().format()) {
    case config::Stream::NOVATEL_BINARY:
      return Parser::CreateNovatel(config);

    case config::Stream::NEWTONM2_BINARY:
      return Parser::CreateNewtonM2(config);

    default:
      return nullptr;
  }
}

}  // namespace

DataParser::DataParser(const config::Config &config) : config_(config) {
  std::string utm_target_param;

  wgs84pj_source_ = pj_init_plus(WGS84_TEXT);
  utm_target_ = pj_init_plus(config_.proj4_text().c_str());
  gnss_status_.set_solution_status(0);
  gnss_status_.set_num_sats(0);
  gnss_status_.set_position_type(0);
  gnss_status_.set_solution_completed(false);
  ins_status_.set_type(InsStatus::INVALID);
  pubgps = nh.advertise<readgps::gps>("/apollo/ros/sensor/gnss/best_pose",10);
  pubheading = nh.advertise<readgps::Heading>("/apollo/ros/sensor/gnss/heading",10);
  pubimu = nh.advertise<readgps::imu>("/apollo/ros/sensor/gnss/imu",10);
}

bool DataParser::Init() {
  ins_status_.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
  gnss_status_.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());

  AdapterManager::FillInsStatusHeader(FLAGS_sensor_node_name, &ins_status_);
  AdapterManager::PublishInsStatus(ins_status_);
  AdapterManager::FillGnssStatusHeader(FLAGS_sensor_node_name, &gnss_status_);
  AdapterManager::PublishGnssStatus(gnss_status_);

  AINFO << "Creating data parser of format: " << config_.data().format();
  data_parser_.reset(CreateParser(config_, false));
  if (!data_parser_) {
    AFATAL << "Failed to create data parser.";
    return false;
  }

  inited_flag_ = true;
  return true;
}

void DataParser::ParseRawData(const std_msgs::String::ConstPtr &msg) {
  if (!inited_flag_) {
    AERROR << "Data parser not init.";
    return;
  }

  data_parser_->Update(msg->data);
  Parser::MessageType type;
  MessagePtr msg_ptr;

  while (ros::ok()) {
    type = data_parser_->GetMessage(&msg_ptr);
    if (type == Parser::MessageType::NONE) break;
    DispatchMessage(type, msg_ptr);

    // std::ofstream outfile;
    // outfile.open("/apollo/modules/drivers/gnss/parser/prove.txt");
    // outfile << "data_parser_->latitude4ros: " << data_parser_->getlatitude4ros() << std::endl;
    // outfile.close();
  }
}

void DataParser::CheckInsStatus(::apollo::drivers::gnss::Ins *ins) {
  if (ins_status_record_ != static_cast<uint32_t>(ins->type())) {
    ins_status_record_ = static_cast<uint32_t>(ins->type());
    switch (ins->type()) {
      case apollo::drivers::gnss::Ins::GOOD:
        ins_status_.set_type(apollo::drivers::gnss_status::InsStatus::GOOD);
        break;

      case apollo::drivers::gnss::Ins::CONVERGING:
        ins_status_.set_type(
            apollo::drivers::gnss_status::InsStatus::CONVERGING);
        break;

      case apollo::drivers::gnss::Ins::INVALID:
      default:
        ins_status_.set_type(apollo::drivers::gnss_status::InsStatus::INVALID);
        break;
    }
    AdapterManager::FillInsStatusHeader(FLAGS_sensor_node_name, &ins_status_);
    AdapterManager::PublishInsStatus(ins_status_);
  }
}

void DataParser::CheckGnssStatus(::apollo::drivers::gnss::Gnss *gnss) {
  gnss_status_.set_solution_status(
      static_cast<uint32_t>(gnss->solution_status()));
  gnss_status_.set_num_sats(static_cast<uint32_t>(gnss->num_sats()));
  gnss_status_.set_position_type(static_cast<uint32_t>(gnss->position_type()));

  if (static_cast<uint32_t>(gnss->solution_status()) == 0) {
    gnss_status_.set_solution_completed(true);
  } else {
    gnss_status_.set_solution_completed(false);
  }
  AdapterManager::FillGnssStatusHeader(FLAGS_sensor_node_name, &gnss_status_);
  AdapterManager::PublishGnssStatus(gnss_status_);
}

void DataParser::DispatchMessage(Parser::MessageType type, MessagePtr message) {
  std_msgs::String msg_pub;

  switch (type) {
    case Parser::MessageType::GNSS:
      CheckGnssStatus(As<::apollo::drivers::gnss::Gnss>(message));
      break;

    case Parser::MessageType::BEST_GNSS_POS:
      PublishBestpos(message);
      break;

    case Parser::MessageType::IMU:
      PublishImu(message);
      break;

    case Parser::MessageType::INS:
      CheckInsStatus(As<::apollo::drivers::gnss::Ins>(message));
      PublishCorrimu(message);
      PublishOdometry(message);
      break;

    case Parser::MessageType::INS_STAT:
      PublishInsStat(message);
      break;

    case Parser::MessageType::BDSEPHEMERIDES:
    case Parser::MessageType::GPSEPHEMERIDES:
    case Parser::MessageType::GLOEPHEMERIDES:
      PublishEphemeris(message);
      break;

    case Parser::MessageType::OBSERVATION:
      PublishObservation(message);
      break;

    case Parser::MessageType::HEADING:
      PublishHeading(message);
      break;

    default:
      break;
  }
}

void DataParser::PublishInsStat(const MessagePtr message) {
  InsStat ins_stat = InsStat(*As<InsStat>(message));
  AdapterManager::FillInsStatHeader(FLAGS_sensor_node_name, &ins_stat);
  AdapterManager::PublishInsStat(ins_stat);
}

void DataParser::PublishBestpos(const MessagePtr message) {
  GnssBestPose bestpos = GnssBestPose(*As<GnssBestPose>(message));
  AdapterManager::FillGnssBestPoseHeader(FLAGS_sensor_node_name, &bestpos);
  AdapterManager::PublishGnssBestPose(bestpos);
  bestpos4ros_highlevel.header.timestamp_sec = bestpos.header().timestamp_sec();
  bestpos4ros_highlevel.header.module_name = bestpos.header().module_name();
  bestpos4ros_highlevel.header.sequence_num = bestpos.header().sequence_num();
  bestpos4ros_highlevel.measurement_time = bestpos.measurement_time();
  bestpos4ros_highlevel.sol_status = bestpos.sol_status();
  bestpos4ros_highlevel.sol_type = bestpos.sol_type();
  bestpos4ros_highlevel.latitude = bestpos.latitude();
  bestpos4ros_highlevel.longitude = bestpos.longitude();
  bestpos4ros_highlevel.height_msl = bestpos.height_msl();
  bestpos4ros_highlevel.undulation = bestpos.undulation();
  bestpos4ros_highlevel.datum_id = bestpos.datum_id();
  bestpos4ros_highlevel.latitude_std_dev = bestpos.latitude_std_dev();
  bestpos4ros_highlevel.longitude_std_dev = bestpos.longitude_std_dev();
  bestpos4ros_highlevel.height_std_dev = bestpos.height_std_dev();
  bestpos4ros_highlevel.base_station_id = bestpos.base_station_id();
  bestpos4ros_highlevel.differential_age = bestpos.differential_age();
  bestpos4ros_highlevel.solution_age = bestpos.solution_age();
  bestpos4ros_highlevel.num_sats_tracked = bestpos.num_sats_tracked();
  bestpos4ros_highlevel.num_sats_in_solution = bestpos.num_sats_in_solution();
  bestpos4ros_highlevel.num_sats_l1 = bestpos.num_sats_l1();
  bestpos4ros_highlevel.num_sats_multi = bestpos.num_sats_multi();
  bestpos4ros_highlevel.extended_solution_status = bestpos.extended_solution_status();
  bestpos4ros_highlevel.galileo_beidou_used_mask = bestpos.galileo_beidou_used_mask();
  bestpos4ros_highlevel.gps_glonass_used_mask = bestpos.gps_glonass_used_mask();
  pubgps.publish(bestpos4ros_highlevel);
}

void DataParser::PublishImu(const MessagePtr message) {
  Imu raw_imu = Imu(*As<Imu>(message));
  Imu *imu = As<Imu>(message);

  raw_imu.mutable_linear_acceleration()->set_x(-imu->linear_acceleration().y());
  raw_imu.mutable_linear_acceleration()->set_y(imu->linear_acceleration().x());
  raw_imu.mutable_linear_acceleration()->set_z(imu->linear_acceleration().z());

  raw_imu.mutable_angular_velocity()->set_x(-imu->angular_velocity().y());
  raw_imu.mutable_angular_velocity()->set_y(imu->angular_velocity().x());
  raw_imu.mutable_angular_velocity()->set_z(imu->angular_velocity().z());

  AdapterManager::FillRawImuHeader(FLAGS_sensor_node_name, &raw_imu);
  AdapterManager::PublishRawImu(raw_imu);
  imu4ros.header.timestamp_sec = raw_imu.header().timestamp_sec();
  imu4ros.header.module_name = raw_imu.header().module_name();
  imu4ros.header.sequence_num = raw_imu.header().sequence_num();
  imu4ros.measurement_time = raw_imu.measurement_time();
  imu4ros.linear_acceleration.x = raw_imu.mutable_linear_acceleration()->x();
  imu4ros.linear_acceleration.y = raw_imu.mutable_linear_acceleration()->y();
  imu4ros.linear_acceleration.z = raw_imu.mutable_linear_acceleration()->z();
  imu4ros.angular_velocity.x = raw_imu.mutable_angular_velocity()->x();
  imu4ros.angular_velocity.y = raw_imu.mutable_angular_velocity()->y();
  imu4ros.angular_velocity.z = raw_imu.mutable_angular_velocity()->z();
  pubimu.publish(imu4ros);
}

void DataParser::PublishOdometry(const MessagePtr message) {
  Ins *ins = As<Ins>(message);
  Gps gps;

  double unix_sec = common::time::TimeUtil::Gps2unix(ins->measurement_time());
  gps.mutable_header()->set_timestamp_sec(unix_sec);
  auto *gps_msg = gps.mutable_localization();

  // 1. pose xyz
  double x = ins->position().lon();
  double y = ins->position().lat();
  x *= DEG_TO_RAD_LOCAL;
  y *= DEG_TO_RAD_LOCAL;

  pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

  gps_msg->mutable_position()->set_x(x);
  gps_msg->mutable_position()->set_y(y);
  gps_msg->mutable_position()->set_z(ins->position().height());

  // 2. orientation
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(ins->euler_angles().z() - 90 * DEG_TO_RAD_LOCAL,
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());

  gps_msg->mutable_orientation()->set_qx(q.x());
  gps_msg->mutable_orientation()->set_qy(q.y());
  gps_msg->mutable_orientation()->set_qz(q.z());
  gps_msg->mutable_orientation()->set_qw(q.w());

  gps_msg->mutable_linear_velocity()->set_x(ins->linear_velocity().x());
  gps_msg->mutable_linear_velocity()->set_y(ins->linear_velocity().y());
  gps_msg->mutable_linear_velocity()->set_z(ins->linear_velocity().z());

  AdapterManager::PublishGps(gps);
  geometry_msgs::TransformStamped transform;
  GpsToTransformStamped(gps, &transform);
  tf_broadcaster_.sendTransform(transform);
}

void DataParser::PublishCorrimu(const MessagePtr message) {
  Ins *ins = As<Ins>(message);
  CorrectedImu imu;
  double unix_sec = common::time::TimeUtil::Gps2unix(ins->measurement_time());
  imu.mutable_header()->set_timestamp_sec(unix_sec);

  auto *imu_msg = imu.mutable_imu();
  imu_msg->mutable_linear_acceleration()->set_x(
      -ins->linear_acceleration().y());
  imu_msg->mutable_linear_acceleration()->set_y(ins->linear_acceleration().x());
  imu_msg->mutable_linear_acceleration()->set_z(ins->linear_acceleration().z());

  imu_msg->mutable_angular_velocity()->set_x(-ins->angular_velocity().y());
  imu_msg->mutable_angular_velocity()->set_y(ins->angular_velocity().x());
  imu_msg->mutable_angular_velocity()->set_z(ins->angular_velocity().z());

  imu_msg->mutable_euler_angles()->set_x(ins->euler_angles().x());
  imu_msg->mutable_euler_angles()->set_y(-ins->euler_angles().y());
  imu_msg->mutable_euler_angles()->set_z(ins->euler_angles().z() -
                                         90 * DEG_TO_RAD_LOCAL);

  AdapterManager::PublishImu(imu);
}

void DataParser::PublishEphemeris(const MessagePtr message) {
  GnssEphemeris eph = GnssEphemeris(*As<GnssEphemeris>(message));
  AdapterManager::PublishGnssRtkEph(eph);
}

void DataParser::PublishObservation(const MessagePtr message) {
  EpochObservation observation =
      EpochObservation(*As<EpochObservation>(message));
  AdapterManager::PublishGnssRtkObs(observation);
}

void DataParser::PublishHeading(const MessagePtr message) {
  Heading heading = Heading(*As<Heading>(message));
  AdapterManager::FillGnssHeadingHeader(FLAGS_sensor_node_name, &heading);
  AdapterManager::PublishGnssHeading(heading);
  heading4ros.header.timestamp_sec = heading.header().timestamp_sec();
  heading4ros.header.module_name = heading.header().module_name();
  heading4ros.header.sequence_num = heading.header().sequence_num();
  heading4ros.measurement_time = heading.measurement_time();
  heading4ros.solution_status = heading.solution_status();
  heading4ros.position_type = heading.position_type();
  heading4ros.baseline_length = heading.baseline_length();
  heading4ros.heading = heading.heading();
  heading4ros.pitch = heading.pitch();
  heading4ros.reserved = heading.reserved();
  heading4ros.heading_std_dev = heading.heading_std_dev();
  heading4ros.pitch_std_dev = heading.pitch_std_dev();
  heading4ros.station_id = heading.station_id();
  heading4ros.satellite_tracked_number = heading.satellite_tracked_number();
  heading4ros.satellite_soulution_number = heading.satellite_soulution_number();
  heading4ros.satellite_number_obs = heading.satellite_number_obs();
  heading4ros.satellite_number_multi = heading.satellite_number_multi();
  heading4ros.solution_source = heading.solution_source();
  heading4ros.extended_solution_status = heading.extended_solution_status();
  heading4ros.galileo_beidou_sig_mask = heading.galileo_beidou_sig_mask();
  heading4ros.gps_glonass_sig_mask = heading.gps_glonass_sig_mask();
  pubheading.publish(heading4ros);
}

void DataParser::GpsToTransformStamped(
    const ::apollo::localization::Gps &gps,
    geometry_msgs::TransformStamped *transform) {
  ros::Time time;
  transform->header.stamp = time.fromSec(gps.header().timestamp_sec());
  transform->header.frame_id = config_.tf().frame_id();
  transform->child_frame_id = config_.tf().child_frame_id();
  transform->transform.translation.x = gps.localization().position().x();
  transform->transform.translation.y = gps.localization().position().y();
  transform->transform.translation.z = gps.localization().position().z();
  transform->transform.rotation.x = gps.localization().orientation().qx();
  transform->transform.rotation.y = gps.localization().orientation().qy();
  transform->transform.rotation.z = gps.localization().orientation().qz();
  transform->transform.rotation.w = gps.localization().orientation().qw();
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
