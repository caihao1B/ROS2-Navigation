#include "collision_volume.hpp"
#include "cooperative_shutdown.hpp"
#include "local_surface.hpp"
#include "observation_confirmation.hpp"
#include "risk_evidence.hpp"
#include "self_mask.hpp"
#include "slope_observability.hpp"
#include "step_reference.hpp"
#include "surface_statistics.hpp"
#include "terrain_observations.hpp"
// Observed geometry only. Raw world geometry and actor scripts are not inputs.
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <array>
#include <deque>
#include <fstream>
#include <map>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <terrain_perception/risk.hpp>
using J = nlohmann::json;
using V = Eigen::Vector3d;
using T = Eigen::Isometry3d;
T trans(const nav_msgs::msg::Odometry &m) {
  auto &p = m.pose.pose;
  T t = T::Identity();
  t.translation() << p.position.x, p.position.y, p.position.z;
  t.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                  p.orientation.y, p.orientation.z)
                   .normalized()
                   .toRotationMatrix();
  return t;
}
struct Cell {
  std::deque<std::array<double, 3>> observations;
  double t = -100, h = 0, top = 0, slope = 0, rough = 0, step = 0;
  int points = 0;
  std::array<double, 2> risk_t{{-100., -100.}};
  std::array<int, 2> confirm{{0, 0}};
  std::array<int, 2> cost{{-1, -1}}, clear{{0, 0}};
};
class Terrain : public rclcpp::Node {
  J cfg_;
  std::array<double, 4> self_box_;
  double res_, half_, ox_, oy_;
  int n_;
  std::vector<Cell> cells_;
  std::array<terrain::Profile, 2> profiles_;
  std::deque<nav_msgs::msg::Odometry> odom_;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> pending_clouds_;
  double last_ = -1, last_details_ = -1;
  bool epoch_bad_ = false;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr objects_, details_,
      health_;
  std::array<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr, 2>
      grids_, confirmed_grids_;
  double val(const char *k) { return cfg_.at(k).get<double>(); }
  int index(double x, double y) {
    int a = int(std::floor((x - ox_) / res_)),
        b = int(std::floor((y - oy_) / res_));
    return a >= 0 && b >= 0 && a < n_ && b < n_ ? b * n_ + a : -1;
  }
  V center(int i, double z = 0) {
    return V((i % n_ + .5) * res_ + ox_, (i / n_ + .5) * res_ + oy_, z);
  }
  void string(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p,
              const J &d) {
    std_msgs::msg::String m;
    m.data = d.dump();
    p->publish(m);
  }
  void fail(double t, const std::string &reason) {
    string(health_,
           {{"t", t}, {"valid", false}, {"reason", reason}, {"max_speed", 0.}});
  }
  void process_synchronized_clouds() {
    while (!pending_clouds_.empty()) {
      const auto &sample = pending_clouds_.front();
      const auto t = rclcpp::Time(sample->header.stamp).seconds();
      const double age = now().seconds() - t;
      if (age <= val("max_cloud_age_s") &&
          (odom_.empty() ||
           rclcpp::Time(odom_.back().header.stamp).seconds() < t))
        return;
      auto ready = std::move(pending_clouds_.front());
      pending_clouds_.pop_front();
      try {
        cloud(*ready);
      } catch (const std::exception &e) {
        fail(t, std::string("PROCESSING_EXCEPTION: ") + e.what());
      }
    }
  }
  void cloud(const sensor_msgs::msg::PointCloud2 &m) {
    auto begin = std::chrono::steady_clock::now();
    double t = rclcpp::Time(m.header.stamp).seconds();
    if (t <= last_) {
      if (t < last_)
        epoch_bad_ = true;
      fail(t, "NONMONOTONIC_SENSOR_TIME");
      return;
    }
    last_ = t;
    if (epoch_bad_ || m.header.frame_id != "lidar_link" ||
        now().seconds() - t > val("max_cloud_age_s") ||
        t > now().seconds() + .05 || odom_.empty()) {
      fail(t, "TIME_FRAME_OR_ODOMETRY_UNAVAILABLE");
      return;
    }
    auto it =
        std::min_element(odom_.begin(), odom_.end(), [t](auto &a, auto &b) {
          return std::abs(rclcpp::Time(a.header.stamp).seconds() - t) <
                 std::abs(rclcpp::Time(b.header.stamp).seconds() - t);
        });
    double dt = std::abs(rclcpp::Time(it->header.stamp).seconds() - t);
    if (dt > val("max_odom_delta_s")) {
      fail(t, "ODOMETRY_EPOCH_MISMATCH");
      return;
    }
    T o = trans(*it);
    if (!o.matrix().allFinite()) {
      fail(t, "INVALID_ODOMETRY");
      return;
    }
    int sx = std::lround((o.translation().x() - (ox_ + half_)) / res_),
        sy = std::lround((o.translation().y() - (oy_ + half_)) / res_);
    if (std::abs(sx) > 10 || std::abs(sy) > 10) {
      std::vector<Cell> shifted(cells_.size());
      for (int y = 0; y < n_; y++)
        for (int x = 0; x < n_; x++) {
          int px = x + sx, py = y + sy;
          if (px >= 0 && py >= 0 && px < n_ && py < n_)
            shifted[y * n_ + x] = cells_[py * n_ + px];
        }
      cells_.swap(shifted);
      ox_ += sx * res_;
      oy_ += sy * res_;
    }
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(m, input);
    auto clean = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (auto &p : input) {
      V v(p.x, p.y, p.z);
      if (!v.allFinite() || v.norm() < val("min_range_m") ||
          v.norm() > val("range_m"))
        continue;
      v += V(cfg_["lidar_xyz"][0], cfg_["lidar_xyz"][1], cfg_["lidar_xyz"][2]);
      if (v.x() > self_box_[0] && v.x() < self_box_[1] &&
          v.y() > self_box_[2] && v.y() < self_box_[3] &&
          v.z() < val("self_z_max"))
        continue;
      v = o * v;
      if (index(v.x(), v.y()) < 0)
        continue;
      clean->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
    }
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(clean);
    vox.setLeafSize(val("voxel_m"), val("voxel_m"), val("voxel_m"));
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    vox.filter(*filtered);
    auto low = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (auto &p : *filtered)
      if (p.z < o.translation().z() + .35 && p.z > o.translation().z() - .6)
        low->push_back(p);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(val("ground_max_angle_deg") * M_PI / 180);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(80);
    seg.setDistanceThreshold(val("ground_distance_m"));
    seg.setInputCloud(low);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients plane;
    seg.segment(inliers, plane);
    if (inliers.indices.size() < size_t(cfg_["min_ground_points"].get<int>()) ||
        plane.values.size() != 4 || std::abs(plane.values[2]) < .7) {
      fail(t, "GROUND_REFERENCE_UNVERIFIED");
      return;
    }
    auto floor = [&](double x, double y) {
      return -(plane.values[0] * x + plane.values[1] * y + plane.values[3]) /
             plane.values[2];
    };
    std::map<int, std::vector<V>> ray_bins;
    for (auto &p : *clean) {
      int i = index(p.x, p.y);
      if (i >= 0 && surface::inCollisionVolume(p.z - floor(p.x, p.y),
                                               val("collision_ceiling_m")))
        ray_bins[i].push_back(V(p.x, p.y, p.z));
    }
    std::map<int, std::vector<double>> bins;
    for (auto &p : *filtered) {
      int i = index(p.x, p.y);
      if (i >= 0 && surface::inCollisionVolume(p.z - floor(p.x, p.y),
                                               val("collision_ceiling_m")))
        bins[i].push_back(p.z);
    }
    std::map<int, double> h, observed_top;
    for (auto &[i, z] : bins) {
      std::sort(z.begin(), z.end());
      auto pos = center(i);
      double f = floor(pos.x(), pos.y());
      auto stat = patrol::surfaceStatistics(
          ray_bins[i], z[size_t(.25 * (z.size() - 1))] - f,
          z[size_t(.8 * (z.size() - 1))] - f, f, val("terrain_height_sigma_m"),
          val("obstacle_min_height_m"));
      auto obs =
          inspection::groundObservation(cells_[i].observations, t, stat.ground,
                                        stat.top, val("obstacle_min_height_m"));
      if (obs.qualified) {
        h[i] = obs.ground + f;
        observed_top[i] = obs.top;
      }
    }
    J rows = J::array();
    int lethal = 0;
    double closest = 100.;
    for (auto &[i, z] : bins) {
      if (!h.count(i)) {
        auto &c = cells_[i];
        for (int k = 0; k < 2; ++k)
          surface::observeRisk(-1, t, cfg_["clear_frames"].get<int>(),
                               c.cost[k], c.clear[k], c.risk_t[k]);
        continue;
      }
      auto pos = center(i);
      double ground = floor(pos.x(), pos.y()), height = observed_top[i];
      std::vector<V> near;
      for (int dy = -4; dy <= 4; dy++)
        for (int dx = -4; dx <= 4; dx++) {
          int x = i % n_ + dx, y = i / n_ + dy;
          if (x < 0 || y < 0 || x >= n_ || y >= n_)
            continue;
          int j = y * n_ + x;
          if (h.count(j) && std::abs(h[j] - h[i]) < .25)
            near.push_back(center(j, h[j]));
        }
      V mean = V::Zero(), normal = V(0, 0, 1);
      double rough = .1, slope = 0, step = 0;
      bool planar = false;
      double slope_sigma = 90.;
      bool slope_observed = false;
      auto local =
          surface::localSurface(near, center(i, h[i]), val("ground_distance_m"),
                                val("terrain_height_sigma_m"));
      if (local.valid) {
        mean = local.mean;
        normal = local.normal;
        rough = local.rough;
        slope = local.slope;
        slope_sigma = local.sigma;
        slope_observed = slope_sigma <= val("terrain_slope_sigma_deg");
        planar = rough < .02 && slope < 15.;
        step = std::max(local.step, std::max(0., height - (h[i] - ground)));
      } else {
        rough = .01;
        step = std::abs(height);
      }
      // A measured depression remains hazardous even with a planar bottom.
      if (height < -val("obstacle_min_height_m"))
        step = std::max(step, std::abs(height));
      // A positive, nearly horizontal island is a step, not a new ground datum.
      if (height > .045 && slope < 1.5)
        step = std::max(step, height);
      Cell &c = cells_[i];
      const double observation_gap = t - c.t;
      c.t = t;
      c.h = h[i] - ground;
      c.top = height;
      c.slope = slope;
      c.rough = rough;
      c.step = step;
      c.points = z.size();
      for (int k = 0; k < 2; k++) {
        double reference_slope =
            std::acos(std::clamp(double(plane.values[2]), 0., 1.)) * 180 / M_PI;
        int cost = slope_observed ? terrain::risk(slope, step, rough, height,
                                                  planar, profiles_[k])
                                  : surface::unobservableSlopeRisk(
                                        height, reference_slope, profiles_[k]);
        c.confirm[k] = nav_control::confirmLethal(c.confirm[k], cost == 100,
                                                  observation_gap);
        surface::observeRisk(cost, t, cfg_["clear_frames"].get<int>(),
                             c.cost[k], c.clear[k], c.risk_t[k]);
      }
      if (surface::riskAt(c.cost[0], c.risk_t[0], t, val("unknown_after_s")) ==
          100) {
        lethal++;
        V body = o.inverse() * center(i, h[i]);
        if (body.x() > 0 && std::abs(body.y()) < val("protective_half_width_m"))
          closest = std::min(closest, body.x());
      }
      if (t - last_details_ >= .95 ||
          (surface::riskAt(c.cost[0], c.risk_t[0], t, val("unknown_after_s")) ==
               100 &&
           (o.inverse() * pos).head<2>().norm() < 2.))
        rows.push_back(
            {pos.x(), pos.y(), c.h, c.top, slope, rough, step, c.points,
             surface::riskAt(c.cost[0], c.risk_t[0], t, val("unknown_after_s")),
             surface::riskAt(c.cost[1], c.risk_t[1], t, val("unknown_after_s")),
             slope_sigma, slope_observed, t - c.risk_t[0], t - c.risk_t[1],
             c.cost[0], c.cost[1]});
    }
    for (int k = 0; k < 2; k++) {
      nav_msgs::msg::OccupancyGrid grid;
      grid.header = m.header;
      grid.header.frame_id = "odom";
      grid.info.resolution = res_;
      grid.info.width = grid.info.height = n_;
      grid.info.origin.position.x = ox_;
      grid.info.origin.position.y = oy_;
      grid.info.origin.orientation.w = 1.;
      grid.data.resize(cells_.size());
      for (size_t i = 0; i < cells_.size(); i++)
        grid.data[i] = surface::riskAt(cells_[i].cost[k], cells_[i].risk_t[k],
                                       t, val("unknown_after_s"));
      grids_[k]->publish(grid);
      for (size_t i = 0; i < cells_.size(); ++i)
        grid.data[i] =
            (t - cells_[i].t < .15 && cells_[i].confirm[k] >= 3) ? 100 : -1;
      confirmed_grids_[k]->publish(grid);
    }
    auto elevated = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZRGB> colored;
    for (auto &p : *filtered) {
      int i = index(p.x, p.y);
      pcl::PointXYZRGB q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;
      double ht = p.z - floor(p.x, p.y);
      int active_cost =
          i >= 0 ? surface::riskAt(cells_[i].cost[0], cells_[i].risk_t[0], t,
                                   val("unknown_after_s"))
                 : -1;
      q.r = active_cost == 100 ? 240 : 30;
      q.g = active_cost >= 0 && active_cost < 100 ? 190 : 70;
      q.b = ht > .4 ? 170 : 60;
      colored.push_back(q);
      if (ht > .16 &&
          surface::inCollisionVolume(ht, val("collision_ceiling_m")))
        elevated->push_back(p);
    }
    sensor_msgs::msg::PointCloud2 pc;
    pcl::toROSMsg(colored, pc);
    pc.header = m.header;
    pc.header.frame_id = "odom";
    color_pub_->publish(pc);
    J clusters = J::array();
    if (elevated->size() > 8) {
      auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
      tree->setInputCloud(elevated);
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(.3);
      ec.setMinClusterSize(8);
      ec.setMaxClusterSize(12000);
      ec.setSearchMethod(tree);
      ec.setInputCloud(elevated);
      std::vector<pcl::PointIndices> groups;
      ec.extract(groups);
      for (auto &g : groups) {
        V lo = V::Constant(1e6), hi = V::Constant(-1e6), sum = V::Zero();
        for (int i : g.indices) {
          auto &p = (*elevated)[i];
          V v(p.x, p.y, p.z);
          lo = lo.cwiseMin(v);
          hi = hi.cwiseMax(v);
          sum += v;
        }
        V size = hi - lo, ctr = (hi + lo) / 2;
        if (size.x() > 3. || size.y() > 3. || size.z() < .25)
          continue;
        double base = floor(ctr.x(), ctr.y());
        clusters.push_back(
            {{"position", {ctr.x(), ctr.y(), ctr.z()}},
             {"size", {size.x(), size.y(), hi.z() - base}},
             {"minimum", {lo.x(), lo.y(), base}},
             {"maximum", {hi.x(), hi.y(), hi.z()}},
             {"points", g.indices.size()},
             {"covariance", {.0036, 0, 0, 0, .0036, 0, 0, 0, .01}}});
      }
    }
    string(objects_, {{"t", t},
                      {"frame", "odom"},
                      {"objects", clusters},
                      {"source", "PCL_EUCLIDEAN_OBSERVED_POINTS"}});
    double ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - begin)
                    .count();
    string(health_, {{"t", t},
                     {"valid", true},
                     {"reason", closest < val("protective_front_m")
                                    ? "GEOMETRIC_PROTECTIVE_STOP"
                                    : "OBSERVED_GEOMETRY_AVAILABLE"},
                     {"max_speed", closest < val("protective_front_m")
                                       ? 0.
                                       : val("qualified_speed_mps")},
                     {"closest_front_m", closest},
                     {"input_points", input.size()},
                     {"filtered_points", filtered->size()},
                     {"collision_ceiling_m", val("collision_ceiling_m")},
                     {"ground_inliers", inliers.indices.size()},
                     {"compute_ms", ms},
                     {"odom_delta_s", dt},
                     {"instantaneous_scan", true}});
    if (t - last_details_ >= .95 || closest < val("protective_front_m")) {
      last_details_ = t;
      string(details_,
             {{"t", t},
              {"frame", "odom"},
              {"columns",
               {"x", "y", "ground_relative_m", "top_relative_m", "slope_deg",
                "roughness_m", "step_m", "points", "diff_cost",
                "ackermann_cost", "slope_sigma_deg", "slope_observed",
                "diff_risk_age_s", "ackermann_risk_age_s", "diff_stored_cost",
                "ackermann_stored_cost"}},
              {"cells", rows},
              {"ground_plane", plane.values},
              {"lethal_cells", lethal},
              {"compute_ms", ms}});
    }
  }

public:
  Terrain() : Node("terrain_terrain") {
    std::ifstream f(declare_parameter<std::string>("config_file", ""));
    f >> cfg_;
    self_box_ = cfg_.at("self_mask").get<std::array<double, 4>>();
    res_ = val("resolution_m");
    half_ = val("width_m") / 2;
    n_ = std::lround(2 * half_ / res_);
    ox_ = oy_ = -half_;
    cells_.resize(n_ * n_);
    int k = 0;
    for (std::string name : {"diff", "ackermann"}) {
      auto p = cfg_["profiles"][name];
      profiles_[k] = {p["max_step_m"],
                      p["max_slope_deg"],
                      p["max_roughness_m"],
                      p["clearance_m"],
                      p["tool_width_m"],
                      val("terrain_slope_sigma_deg"),
                      val("terrain_height_sigma_m"),
                      val("obstacle_min_height_m")};
      confirmed_grids_[k] = create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/nav_control/terrain/confirmed/" + name, 2);
      grids_[k++] = create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/terrain/terrain/" + name, rclcpp::QoS(2).reliable());
    }
    color_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/terrain/terrain/colored", 2);
    objects_ = create_publisher<std_msgs::msg::String>("/terrain/geometry", 10);
    details_ =
        create_publisher<std_msgs::msg::String>("/terrain/terrain/cells", 5);
    health_ =
        create_publisher<std_msgs::msg::String>("/terrain/terrain/health", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/localization/odometry", 100,
        [this](nav_msgs::msg::Odometry::SharedPtr m) {
          odom_.push_back(*m);
          while (odom_.size() > 250)
            odom_.pop_front();
          process_synchronized_clouds();
        });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensors/perception/points", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {
          if (pending_clouds_.size() >= 3) {
            fail(rclcpp::Time(pending_clouds_.front()->header.stamp).seconds(),
                 "CLOUD_SYNC_QUEUE_FULL");
            pending_clouds_.pop_front();
          }
          pending_clouds_.push_back(m);
          process_synchronized_clouds();
        });
  }
};
int main(int argc, char **argv) {
  patrol::initialize(argc, argv);
  try {
    patrol::spin(std::make_shared<Terrain>());
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 2;
  }
  rclcpp::shutdown();
}
