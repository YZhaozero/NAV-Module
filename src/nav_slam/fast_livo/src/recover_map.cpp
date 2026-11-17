#include "map_spool.h"
#include <iostream>

int main(int argc, char **argv) {
  std::filesystem::path temporary;
  try {
    if (argc != 3) throw std::runtime_error("Usage: recover_map CHECKPOINT_DIRECTORY OUTPUT.pcd");
    auto spool = MapSpool::recover(argv[1]);
    const std::filesystem::path output(argv[2]);
    if (!output.is_absolute() || output.extension() != ".pcd" || std::filesystem::exists(output))
      throw std::runtime_error("Choose a new absolute .pcd path");
    std::filesystem::create_directories(output.parent_path());
    std::string pattern=output.string()+".XXXXXX";
    std::vector<char> name(pattern.begin(),pattern.end());name.push_back('\0');
    int fd=mkstemp(name.data());if(fd<0)throw std::runtime_error("Cannot create output");
    close(fd);temporary=name.data();
    spool->exportPcd(temporary.string(),spool->voxel(),[](const MapSpool::Sink &){});
    MapSpool::commitPcd(temporary,output);
    try { spool->markExported(output.string()); }
    catch(const std::exception &e) { std::cerr << "Map saved, recovery receipt unavailable: " << e.what() << '\n'; }
    std::cout << output.string() << '\n';return 0;
  } catch(const std::exception &error) {
    if(!temporary.empty()){std::error_code ignored;std::filesystem::remove(temporary,ignored);}
    std::cerr << error.what() << '\n';return 1;
  }
}
