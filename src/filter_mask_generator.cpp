#include "ros_geofence/filter_mask_generator.hpp"
#include <cmath>
#include <algorithm>
#include <iomanip>

namespace ros_geofence
{

FilterMaskGenerator::FilterMaskGenerator()
: Node("filter_mask_generator"),
  proj_(nullptr)
{
  this->declare_parameter<std::string>("geojson_file", "zone.geojson");
  this->declare_parameter<std::string>("output_pgm", "filter_mask.pgm");
  this->declare_parameter<std::string>("output_yaml", "filter_mask.yaml");
  this->declare_parameter<double>("resolution", 0.5);
  this->declare_parameter<double>("padding", 50.0);

  geojson_file_ = this->get_parameter("geojson_file").as_string();
  output_pgm_ = this->get_parameter("output_pgm").as_string();
  output_yaml_ = this->get_parameter("output_yaml").as_string();
  resolution_ = this->get_parameter("resolution").as_double();
  padding_ = this->get_parameter("padding").as_double();

  RCLCPP_INFO(this->get_logger(), "Filter Mask Generator initialized");
  RCLCPP_INFO(this->get_logger(), "  Input: %s", geojson_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output PGM: %s", output_pgm_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output YAML: %s", output_yaml_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Resolution: %.2f m/cell", resolution_);
  RCLCPP_INFO(this->get_logger(), "  Padding: %.2f m", padding_);
}

FilterMaskGenerator::~FilterMaskGenerator()
{
  if (proj_) {
    proj_destroy(proj_);
    proj_ = nullptr;
  }
}

bool FilterMaskGenerator::load_geojson_zone()
{
  try {
    std::ifstream file(geojson_file_);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open GeoJSON file: %s", geojson_file_.c_str());
      return false;
    }

    nlohmann::json geojson_data;
    file >> geojson_data;

    auto coordinates = geojson_data["features"][0]["geometry"]["coordinates"][0];
    zone_polygon_.clear();
    zone_polygon_utm_.clear();

    // Calculate average lat/lon for UTM zone determination
    double avg_lon = 0.0, avg_lat = 0.0;
    int count = 0;
    for (const auto& coord : coordinates) {
      avg_lon += coord[0].get<double>();
      avg_lat += coord[1].get<double>();
      count++;
    }
    avg_lon /= count;
    avg_lat /= count;

    // Determine UTM zone
    int zone_number = static_cast<int>(std::floor((avg_lon + 180.0) / 6.0)) + 1;
    bool is_northern = avg_lat >= 0;
    int epsg_code = is_northern ? 32600 + zone_number : 32700 + zone_number;
    std::string utm_zone = "EPSG:" + std::to_string(epsg_code);

    proj_ = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", utm_zone.c_str(), nullptr);
    if (!proj_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize projection for %s", utm_zone.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Using UTM zone: %s", utm_zone.c_str());

    // Convert all coordinates to UTM
    for (const auto& coord : coordinates) {
      Point geo_p, utm_p;
      geo_p.x = coord[0].get<double>();
      geo_p.y = coord[1].get<double>();
      zone_polygon_.push_back(geo_p);

      PJ_COORD coord_in = proj_coord(geo_p.y, geo_p.x, 0, 0);
      PJ_COORD coord_out = proj_trans(proj_, PJ_FWD, coord_in);
      utm_p.x = coord_out.xy.x;
      utm_p.y = coord_out.xy.y;
      zone_polygon_utm_.push_back(utm_p);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu points from GeoJSON", zone_polygon_.size());
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading GeoJSON: %s", e.what());
    return false;
  }
}

bool FilterMaskGenerator::point_in_polygon(const Point& p, const std::vector<Point>& polygon)
{
  int n = polygon.size();
  bool inside = false;

  for (int i = 0, j = n - 1; i < n; j = i++) {
    if (((polygon[i].y > p.y) != (polygon[j].y > p.y)) &&
        (p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) /
                 (polygon[j].y - polygon[i].y) + polygon[i].x)) {
      inside = !inside;
    }
  }

  return inside;
}

FilterMaskGenerator::GridBounds FilterMaskGenerator::calculate_bounds()
{
  GridBounds bounds;
  
  if (zone_polygon_utm_.empty()) {
    return bounds;
  }

  bounds.min_x = zone_polygon_utm_[0].x;
  bounds.max_x = zone_polygon_utm_[0].x;
  bounds.min_y = zone_polygon_utm_[0].y;
  bounds.max_y = zone_polygon_utm_[0].y;

  for (const auto& p : zone_polygon_utm_) {
    bounds.min_x = std::min(bounds.min_x, p.x);
    bounds.max_x = std::max(bounds.max_x, p.x);
    bounds.min_y = std::min(bounds.min_y, p.y);
    bounds.max_y = std::max(bounds.max_y, p.y);
  }

  // Add padding
  bounds.min_x -= padding_;
  bounds.max_x += padding_;
  bounds.min_y -= padding_;
  bounds.max_y += padding_;

  RCLCPP_INFO(this->get_logger(), "Grid bounds (UTM): [%.2f, %.2f] x [%.2f, %.2f]",
              bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y);

  return bounds;
}

void FilterMaskGenerator::write_pgm_file(const std::string& path, 
                                         const std::vector<uint8_t>& data,
                                         int width, int height)
{
  std::ofstream file(path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open PGM file for writing: " + path);
  }

  // Write PGM header (P5 = binary grayscale)
  file << "P5\n";
  file << "# Generated by ros_geofence filter_mask_generator\n";
  file << width << " " << height << "\n";
  file << "255\n";

  // Write pixel data
  file.write(reinterpret_cast<const char*>(data.data()), data.size());
  file.close();

  RCLCPP_INFO(this->get_logger(), "Saved PGM file: %s (%dx%d)", path.c_str(), width, height);
}

void FilterMaskGenerator::write_yaml_file(const std::string& path, int width, int height,
                                          double resolution, double origin_x, double origin_y)
{
  std::ofstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open YAML file for writing: " + path);
  }

  file << std::fixed << std::setprecision(6);
  file << "image: " << output_pgm_ << "\n";
  file << "resolution: " << resolution << "\n";
  file << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
  file << "negate: 0\n";
  file << "occupied_thresh: 0.65\n";
  file << "free_thresh: 0.196\n";
  file << "mode: trinary\n";
  file << "# Map frame_id for TF transformation\n";
  file << "# Width: " << width << " cells\n";
  file << "# Height: " << height << " cells\n";
  file << "# Generated by ros_geofence filter_mask_generator\n";

  file.close();

  RCLCPP_INFO(this->get_logger(), "Saved YAML file: %s", path.c_str());
}

bool FilterMaskGenerator::generate_pgm_mask(const std::string& pgm_path, 
                                            const std::string& yaml_path)
{
  GridBounds bounds = calculate_bounds();
  
  // Calculate grid dimensions
  int width = static_cast<int>(std::ceil((bounds.max_x - bounds.min_x) / resolution_));
  int height = static_cast<int>(std::ceil((bounds.max_y - bounds.min_y) / resolution_));

  RCLCPP_INFO(this->get_logger(), "Generating grid: %dx%d cells", width, height);

  // Create the mask data
  // 255 = free (white), 0 = occupied/forbidden (black)
  std::vector<uint8_t> mask_data(width * height, 0);

  int total_cells = width * height;
  int cells_inside = 0;

  // For each cell, check if its center is inside the polygon
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      // Calculate cell center in UTM coordinates
      Point cell_center;
      cell_center.x = bounds.min_x + (col + 0.5) * resolution_;
      cell_center.y = bounds.min_y + (row + 0.5) * resolution_;

      // Check if inside polygon
      bool inside = point_in_polygon(cell_center, zone_polygon_utm_);

      // PGM format: row 0 is at the top
      // We need to flip Y axis (map origin is at bottom-left)
      int pgm_row = height - 1 - row;
      int index = pgm_row * width + col;

      if (inside) {
        mask_data[index] = 255;  // Free (white)
        cells_inside++;
      } else {
        mask_data[index] = 0;    // Forbidden (black)
      }
    }

    // Progress indicator
    if ((row + 1) % 100 == 0 || row == height - 1) {
      int progress = static_cast<int>(100.0 * (row + 1) / height);
      RCLCPP_INFO(this->get_logger(), "Progress: %d%%", progress);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Cells inside zone: %d / %d (%.1f%%)", 
              cells_inside, total_cells, 100.0 * cells_inside / total_cells);

  // Write files
  write_pgm_file(pgm_path, mask_data, width, height);
  write_yaml_file(yaml_path, width, height, resolution_, bounds.min_x, bounds.min_y);

  return true;
}

bool FilterMaskGenerator::generate()
{
  RCLCPP_INFO(this->get_logger(), "Starting mask generation...");

  if (!load_geojson_zone()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load GeoJSON zone");
    return false;
  }

  if (!generate_pgm_mask(output_pgm_, output_yaml_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to generate mask");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Mask generation completed successfully!");
  return true;
}

}  // namespace ros_geofence

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros_geofence::FilterMaskGenerator>();
  
  bool success = node->generate();
  
  rclcpp::shutdown();
  return success ? 0 : 1;
}
