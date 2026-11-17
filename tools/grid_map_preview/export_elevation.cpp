// Use upstream grid_map_pcl's per-column clustering without transforming XYZ.
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "usage: nav_grid_map_pcl_export INPUT.pcd PARAMETERS.yaml OUTPUT.csv\n";
    return 2;
  }
  try {
    grid_map::GridMapPclLoader loader(rclcpp::get_logger("nav_grid_map_preview"));
    loader.loadParameters(argv[2]);
    loader.loadCloudFromPcdFile(argv[1]);
    loader.preProcessInputCloud();
    loader.initializeGridMapGeometryFromInputCloud();
    loader.addLayerFromInputCloud("elevation");
    const auto &map = loader.getGridMap();
    std::ofstream output(argv[3]);
    if (!output) throw std::runtime_error("cannot open elevation output");
    output << "x,y,z\n" << std::setprecision(12);
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position xy;
      map.getPosition(*it, xy);
      output << xy.x() << ',' << xy.y() << ',' << map.at("elevation", *it) << '\n';
    }
    if (!output) throw std::runtime_error("cannot write elevation output");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
