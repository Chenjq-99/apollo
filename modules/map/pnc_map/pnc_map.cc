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

#include "modules/map/pnc_map/pnc_map.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"
#include "google/protobuf/text_format.h"

#include "modules/common_msgs/map_msgs/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"

DEFINE_double(
    look_backward_distance, 50,
    "look backward this distance when creating reference line from routing");

DEFINE_double(look_forward_short_distance, 180,
              "short look forward this distance when creating reference line "
              "from routing when ADC is slow");
DEFINE_double(
    look_forward_long_distance, 250,
    "look forward this distance when creating reference line from routing");

namespace apollo {
namespace hdmap {

using apollo::common::PointENU;
using apollo::common::VehicleState;
using apollo::common::util::PointFactory;
using apollo::routing::RoutingResponse;

namespace {

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

}  // namespace

PncMap::PncMap(const HDMap *hdmap) : hdmap_(hdmap) {}

const hdmap::HDMap *PncMap::hdmap() const { return hdmap_; }

LaneWaypoint PncMap::ToLaneWaypoint(
    const routing::LaneWaypoint &waypoint) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()));
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return LaneWaypoint(lane, waypoint.s());
}

double PncMap::LookForwardDistance(double velocity) {
  auto forward_distance = velocity * FLAGS_look_forward_time_sec;

  return forward_distance > FLAGS_look_forward_short_distance
             ? FLAGS_look_forward_long_distance
             : FLAGS_look_forward_short_distance;
}

LaneSegment PncMap::ToLaneSegment(const routing::LaneSegment &segment) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return LaneSegment(lane, segment.start_s(), segment.end_s());
}

void PncMap::UpdateNextRoutingWaypointIndex(int cur_index) {
    /*
    cur_index是车辆投影点所在LaneSegment在route_indices_中的索引
    */
  if (cur_index < 0) {
    next_routing_waypoint_index_ = 0;
    return;
  }
  if (cur_index >= static_cast<int>(route_indices_.size())) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
    return;
  }
  // Search backwards when the car is driven backward on the route.
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index >
             cur_index) {
        /*
        倒车，next_routing_waypoint所在的LaneSegment在route_indices_中的索引 > cur_index
        --next_routing_waypoint_index_，直到两者在同一个LaneSegment上，这个是LaneSegment层面的搜索
        */
    --next_routing_waypoint_index_; 
  }
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index ==
             cur_index &&
         adc_waypoint_.s <
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
        /*
        --next_routing_waypoint_index_，直到next_routing_waypoint.s < adc_waypoint_.s，这个是WayPoint层面的搜索
        */
    --next_routing_waypoint_index_;
  }
  // Search forwards
  /*
    向前搜索类似
  */
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index <
             cur_index) {
    ++next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index ==
             routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >=
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }
  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}

std::vector<routing::LaneWaypoint> PncMap::FutureRouteWaypoints() const {
  const auto &waypoints = routing_.routing_request().waypoint();
  return std::vector<routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

void PncMap::UpdateRoutingRange(int adc_index) {
  // Track routing range.
  range_lane_ids_.clear();
  range_start_ = std::max(0, adc_index - 1);
  range_end_ = range_start_;
  while (range_end_ < static_cast<int>(route_indices_.size())) {
    const auto &lane_id = route_indices_[range_end_].segment.lane->id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      break;
    }
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}

bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state) {
    /*
    这里的无人车状态不是无人车自身坐标，速度等信息，
    而是在上述更新路由信息过程中得到的route_indices_中，无人车在哪个LaneSegment中，距离无人车最近的下一个查询点waypoint的信息。
    */
   /*
   总体任务：
   找到当前车辆在对应车道上的投影点adc_waypoint_，车辆投影点所在LaneSegment在route_indices_中的索引adc_route_index_，
   下一个最近查询点在routing_waypoint_index_中的索引next_routing_waypoint_index_
   */
  if (!ValidateRouting(routing_)) {
    AERROR << "The routing is invalid when updating vehicle state.";
    return false;
  }
  if (!adc_state_.has_x() ||
      (common::util::DistanceXY(adc_state_, vehicle_state) >
       FLAGS_replan_lateral_distance_threshold +
           FLAGS_replan_longitudinal_distance_threshold)) {
    // Position is reset, but not replan.
    next_routing_waypoint_index_ = 0;
    adc_route_index_ = -1;
    stop_for_destination_ = false;
  }

  adc_state_ = vehicle_state;
  // 根据车辆状态找到在对应车道上的投影点adc_waypoint_
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    AERROR << "Failed to get waypoint from routing with point: "
           << "(" << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
    return false;
  }
  // 根据adc_waypoint_得到投影点在route_indices_中的索引adc_route_index_
  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    return false;
  }

  // Track how many routing request waypoints the adc have passed.
  // 找到下一个waypoint在routing_waypoint_index_数组中的index
  UpdateNextRoutingWaypointIndex(route_index);
  adc_route_index_ = route_index;
  UpdateRoutingRange(adc_route_index_);

  if (routing_waypoint_index_.empty()) {
    AERROR << "No routing waypoint index.";
    return false;
  }

  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1) {
    stop_for_destination_ = true;
  }
  return true;
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &routing) const {
  return IsNewRouting(routing_, routing);
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &prev,
                          const routing::RoutingResponse &routing) {
  if (!ValidateRouting(routing)) {
    ADEBUG << "The provided routing is invalid.";
    return false;
  }
  return !common::util::IsProtoEqual(prev, routing);
}

bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
    /*
        主要做了两件事：
        1. 把routing剥开，拿到最里层的LaneSegment,存一下{road_index, passage_index, lane_index}
        2. 获取routing的LaneWayPoints，并且给每个LaneWayPoint匹配到他所在的LaneSegment的idx
    */ 
  range_lane_ids_.clear();
  route_indices_.clear();
  all_lane_ids_.clear();
  // 逐级遍历routing，把最里层的LaneSegment拿出来存到route_indices_
  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        all_lane_ids_.insert(passage.segment(lane_index).id());
        route_indices_.emplace_back();
        route_indices_.back().segment =
            ToLaneSegment(passage.segment(lane_index));
        if (route_indices_.back().segment.lane == nullptr) {
          AERROR << "Failed to get lane segment from passage.";
          return false;
        }
        route_indices_.back().index = {road_index, passage_index, lane_index};
      }
    }
  }

  range_start_ = 0;
  range_end_ = 0;
  adc_route_index_ = -1;
  next_routing_waypoint_index_ = 0;
  UpdateRoutingRange(adc_route_index_);

  routing_waypoint_index_.clear();
  const auto &request_waypoints = routing.routing_request().waypoint();
  if (request_waypoints.empty()) {
    AERROR << "Invalid routing: no request waypoints.";
    return false;
  }
  // 给每个routing中的点匹配到它所在的LaneSegment在route_indices_中的索引，一同存到routing_waypoint_index_
  int i = 0;
  for (size_t j = 0; j < route_indices_.size(); ++j) {
    while (i < request_waypoints.size() &&
           RouteSegments::WithinLaneSegment(route_indices_[j].segment,
                                            request_waypoints.Get(i))) {
      routing_waypoint_index_.emplace_back(
          LaneWaypoint(route_indices_[j].segment.lane,
                       request_waypoints.Get(i).s()),
          j);
      ++i;
    }
  }
  routing_ = routing;
  adc_waypoint_ = LaneWaypoint();
  stop_for_destination_ = false;
  return true;
}

const routing::RoutingResponse &PncMap::routing_response() const {
  return routing_;
}

bool PncMap::ValidateRouting(const RoutingResponse &routing) {
  const int num_road = routing.road_size();
  if (num_road == 0) {
    AERROR << "Route is empty.";
    return false;
  }
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    AERROR << "Routing does not have request.";
    return false;
  }
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    if (!waypoint.has_id() || !waypoint.has_s()) {
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  return true;
}

int PncMap::SearchForwardWaypointIndex(int start,
                                       const LaneWaypoint &waypoint) const {
  int i = std::max(start, 0);
  while (
      i < static_cast<int>(route_indices_.size()) &&
      !RouteSegments::WithinLaneSegment(route_indices_[i].segment, waypoint)) {
    ++i;
  }
  return i;
}

int PncMap::SearchBackwardWaypointIndex(int start,
                                        const LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                     waypoint)) {
    --i;
  }
  return i;
}

int PncMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

int PncMap::GetWaypointIndex(const LaneWaypoint &waypoint) const {
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

bool PncMap::PassageToSegments(routing::Passage passage,
                               RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  for (const auto &lane : passage.segment()) {
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << lane.id();
      return false;
    }
    segments->emplace_back(lane_ptr, std::max(0.0, lane.start_s()),
                           std::min(lane_ptr->total_length(), lane.end_s()));
  }
  return !segments->empty();
}

std::vector<int> PncMap::GetNeighborPassages(const routing::RoadSegment &road,
                                             int start_passage) const {

  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  std::vector<int> result;
  // 找到当前所在的road_segment的passage
  const auto &source_passage = road.passage(start_passage);
  result.emplace_back(start_passage);
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }
  if (source_passage.can_exit()) {  // No need to change lane
    return result;
  }
  // Passage转RouteSegment
  RouteSegments source_segments;
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "Failed to convert passage to segments";
    return result;
  }
  /*如果下一个查询点(routing_waypoint_index_[next_routing_waypoint_index_].waypoint)
  在当前通道中，不需要变道，直接返回车辆所在的车道
  */
  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
      source_segments.IsWaypointOnSegment(
          routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
    ADEBUG << "Need to pass next waypoint[" << next_routing_waypoint_index_
           << "] before change lane";
    return result;
  }
  /*
    如果车辆再左转车道或者右转车道，从高精地图hd map中查询当前车道对应左侧或者右侧的所有车道线，
    然后去和当前RoadSegment.passage()去做对比，找到两者共同包含的车道，就是最终的邻接车道。
  */
  std::unordered_set<std::string> neighbor_lanes;
  if (source_passage.change_lane_type() == routing::LEFT) {
    for (const auto &segment : source_segments) {
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(left_id.id());
      }
    }
  } else if (source_passage.change_lane_type() == routing::RIGHT) {
    for (const auto &segment : source_segments) {
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(right_id.id());
      }
    }
  }
  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road.passage(i);
    for (const auto &segment : target_passage.segment()) {
      if (neighbor_lanes.count(segment.id())) {
        result.emplace_back(i);
        break;
      }
    }
  }
  return result;
}
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              std::list<RouteSegments> *const route_segments) {
    // 计算前视距离=min(车速×8s(默认)，250m)
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());
    // 后视距离50m默认
  double look_backward_distance = FLAGS_look_backward_distance;
  return GetRouteSegments(vehicle_state, look_backward_distance,
                          look_forward_distance, route_segments);
}

bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments) {
  // 1.更新车辆状态(更新adc_waypoint_,adc_waypoint_index_,next_routing_waypoint_index_)
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  // Vehicle has to be this close to lane center before considering change
  // lane
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }
  // route_index = {road_index, passage_index, lane_index};
  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_index = route_index[0];
  const int passage_index = route_index[1];
  const auto &road = routing_.road(road_index);
  // Raw filter to find all neighboring passages
  // 2.计算临近通道 
  auto drive_passages = GetNeighborPassages(road, passage_index);
  // 3.创建车辆当前可行驶区域
  for (const int index : drive_passages) {
    const auto &passage = road.passage(index);
    RouteSegments segments;
    // Passage转RoutSegment
    if (!PassageToSegments(passage, &segments)) {
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }
    const PointENU nearest_point =
        index == passage_index
            ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
            : PointFactory::ToPointENU(adc_state_);
    common::SLPoint sl;
    LaneWaypoint segment_waypoint;
    // 3.1 将当前车辆的坐标投影到Passage
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }
    // 3.2 对临近通道是否可驶入进行检查
    if (index != passage_index) {
      if (!segments.CanDriveFrom(adc_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }
    route_segments->emplace_back();
    const auto last_waypoint = segments.LastWaypoint();
    // 3.3 生成RouteSegments
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
        /*
            原本车辆在passage中的投影点累计距离为sl.s(注意这个s是投影点在passage段起点的累计距离，
            而非在整个lane的累计距离)，扩展后前向增加forward_length，后向增加backward_length
        */
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    // 3.4 设置RouteSegments属性
    route_segments->back().SetCanExit(passage.can_exit());
    route_segments->back().SetNextAction(passage.change_lane_type());
    const std::string route_segment_id = absl::StrCat(road_index, "_", index);
    route_segments->back().SetId(route_segment_id);
    route_segments->back().SetStopForDestination(stop_for_destination_);
    if (index == passage_index) {
      route_segments->back().SetIsOnSegment(true);
      route_segments->back().SetPreviousAction(routing::FORWARD);
    } else if (sl.l() > 0) {
      route_segments->back().SetPreviousAction(routing::RIGHT);
    } else {
      route_segments->back().SetPreviousAction(routing::LEFT);
    }
  }
  return !route_segments->empty();
}

// 根据车辆状态找到在<对应车道>上的投影点adc_waypoint_
bool PncMap::GetNearestPointFromRouting(const VehicleState &state,
                                        LaneWaypoint *waypoint) const {
  waypoint->lane = nullptr;
  std::vector<LaneInfoConstPtr> lanes;
  const auto point = PointFactory::ToPointENU(state);
  std::vector<LaneInfoConstPtr> valid_lanes;
  // 把all_lane_ids_中记录的车道id都拿出来，加到valid_lanes
  for (auto lane_id : all_lane_ids_) {
    hdmap::Id id = hdmap::MakeMapId(lane_id);
    // 根据id查询车道
    auto lane = hdmap_->GetLaneById(id);
    if (nullptr != lane) {
      valid_lanes.emplace_back(lane);
    }
  }

  // Get nearest_waypoints for current position
  std::vector<LaneWaypoint> valid_way_points;
  for (const auto &lane : valid_lanes) {
    // 过滤掉不在range_lane_ids_中的车道
    if (range_lane_ids_.count(lane->id().id()) == 0) {
      continue;
    }
    double s = 0.0;
    double l = 0.0;
    {  // 将车辆xy投影到车道中得到sl
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        continue;
      }
      // Use large epsilon to allow projection diff
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
    }
    // 把在每个车道的投影点都加入valid_way_points中
    valid_way_points.emplace_back();
    auto &last = valid_way_points.back();
    last.lane = lane;
    last.s = s;
    last.l = l;
    ADEBUG << "distance:" << std::fabs(l);
  }
  if (valid_way_points.empty()) {
    AERROR << "Failed to find nearest point: " << point.ShortDebugString();
    return false;
  }

  // Choose the lane with the right heading if there is more than one candiate
  // lanes. If there is no lane with the right heading, choose the closest one.
  size_t closest_index = 0;
  int right_heading_index = -1;
  // The distance as the sum of the lateral and longitude distance, to estimate
  // the distance from the vehicle to the lane.
  double distance = std::numeric_limits<double>::max();
  double lane_heading = 0.0;
  double vehicle_heading = state.heading();
  for (size_t i = 0; i < valid_way_points.size(); i++) {
    double distance_to_lane = std::fabs(valid_way_points[i].l);
    if (valid_way_points[i].s > valid_way_points[i].lane->total_length()) {
      distance_to_lane +=
          (valid_way_points[i].s - valid_way_points[i].lane->total_length());
    } else if (valid_way_points[i].s < 0.0) {
      distance_to_lane -= valid_way_points[i].s;
    }
    if (distance > distance_to_lane) {
      distance = distance_to_lane;
      closest_index = i;
    }
    lane_heading = valid_way_points[i].lane->Heading(valid_way_points[i].s);
    if (std::abs(common::math::AngleDiff(lane_heading, vehicle_heading)) <
        M_PI_2) {
      // 车道和车辆的heading相差不超过90度 -> right heading
      // Choose the lane with the closest distance to the vehicle and with the
      // right heading.
      if (-1 == right_heading_index || closest_index == i) {
        waypoint->lane = valid_way_points[i].lane;
        waypoint->s = valid_way_points[i].s;
        waypoint->l = valid_way_points[i].l;
        right_heading_index = i;
      }
    }
  }
  // Use the lane with the closest distance to the current position of the
  // vehicle.
  if (-1 == right_heading_index) {
    waypoint->lane = valid_way_points[closest_index].lane;
    waypoint->s = valid_way_points[closest_index].s;
    waypoint->l = valid_way_points[closest_index].l;
    AWARN << "Find no lane with the right heading, use the cloesest lane!";
  }
  return true;
}

LaneInfoConstPtr PncMap::GetRouteSuccessor(LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (range_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

LaneInfoConstPtr PncMap::GetRoutePredecessor(LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  for (size_t i = 1; i < route_indices_.size(); ++i) {
    auto &lane = route_indices_[i].segment.lane->id();
    if (predecessor_ids.count(lane.id()) != 0) {
      preferred_id = lane;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

// 向前向后截取RouteSegments
bool PncMap::ExtendSegments(const RouteSegments &segments,
                            const common::PointENU &point, double look_backward,
                            double look_forward,
                            RouteSegments *extended_segments) {
  common::SLPoint sl;
  LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR << "point: " << point.ShortDebugString() << " is not on segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

bool PncMap::ExtendSegments(const RouteSegments &segments, double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const {
    /*
        sl.s是投影点相对于passage(first_lane_segment.start_s)的累计距离，
        而不是相对于整个规划路径road(first_passage.first_lane_segment.start_s)的累计距离，这很重要
    */
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  // 将完整的RouteSegment属性拷贝到要截取的RouteSegment中
  truncated_segments->SetProperties(segments);

  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  std::unordered_set<std::string> unique_lanes;
  static constexpr double kRouteEpsilon = 1e-3;
  // Extend the trajectory towards the start of the trajectory.
  if (start_s < 0) { // 当后向查询起始点小于0，说明需要用到这个laneSegment所在lane的前面的部分
    const auto &first_segment = *segments.begin(); // 获得passage的第一个LaneSegment的所属lane
    auto lane = first_segment.lane;
    double s = first_segment.start_s;
    double extend_s = -start_s; // extend_s为需要从s前截取的道路段长度，初始化为-start_s
    std::vector<LaneSegment> extended_lane_segments; 
    while (extend_s > kRouteEpsilon) { // 每次循环(截取)以后extend_s都会减小，直至到0
      if (s <= kRouteEpsilon) { // 1. s < 0，自己这条lane不够用了，需要在查询这条lane对应的前置车道，把前置车道整个拿过来
        lane = GetRoutePredecessor(lane); // 获取当前lane的前置车道
        if (lane == nullptr ||
            unique_lanes.find(lane->id().id()) != unique_lanes.end()) {
          break;
        }
        s = lane->total_length();
      } else { // 2. 如果s > 0，说明当前的lane还够用，截取道路段
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);
        extend_s -= length;
        s -= length; // 每次截取完，s减去截取的长度，若s < 0，就会到1.中寻求前置的lane
        unique_lanes.insert(lane->id().id());
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }
  // 下面部分就是正常的passage中LaneSegment截取，根据start_s和end_s
  bool found_loop = false;
  double router_s = 0;
  for (const auto &lane_segment : segments) {
    /* router_s代表已经累计截取到了的LaneSegment长度，
       如果当前正在截取第3个LaneSegment，那么router_s就是前两个LaneSegment的长度和
    */
    /*
        找到这个LaneSegment需要截取的起点和终点
        1. router_s还没累加到start_s
            1.1 start_s不在当前laneSegment里，此时不满足adjusted_start_s < adjusted_end_s，不截取
            1.2 start_s在当前laneSegment里， 那么adjusted_start_s = start_s - router_s + lane_segment.start_s
                开始截取
        2. router_s累加超过start_s
            adjusted_start_s = lane_segment.start_s，从laneSegment头开始截取
    */
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->id().id() ==
              lane_segment.lane->id().id()) {
        truncated_segments->back().end_s = adjusted_end_s;
      } else if (unique_lanes.find(lane_segment.lane->id().id()) ==
                 unique_lanes.end()) {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                         adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->id().id());
      } else {
        found_loop = true;
        break;
      }
    }
    /*
       判断是否截取结束，如果结束了那么可以退出，否则就需要继续截取
    */
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }
  if (found_loop) {
    return true;
  }
  // Extend the trajectory towards the end of the trajectory.
  /*
    当最后循环最后一次最后一个LaneSegment还是没有结束(router_s < end_s)，
    那么就需要新增加后置车道继续处理
  */
  if (router_s < end_s && !truncated_segments->empty()) {
    auto &back = truncated_segments->back();
    if (back.lane->total_length() > back.end_s) { // 当前lane足够用
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }
  auto last_lane = segments.back().lane;
  while (router_s < end_s - kRouteEpsilon) { // 还是不够找后置车道线
    last_lane = GetRouteSuccessor(last_lane);
    if (last_lane == nullptr ||
        unique_lanes.find(last_lane->id().id()) != unique_lanes.end()) {
      break;
    }
    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes.insert(last_lane->id().id());
    router_s += length;
  }
  return true;
}

// 将一个laneSegment离散化成一组MapPathPoint
void PncMap::AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                const double end_s,
                                std::vector<MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) { // 封装中间点point
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) { // 封装段起点waypoint
        points->emplace_back(segment.start() + segment.unit_direction() *
                                                   (start_s - accumulate_s),
                             lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) { // 封装段终点waypoint
        points->emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

}  // namespace hdmap
}  // namespace apollo
