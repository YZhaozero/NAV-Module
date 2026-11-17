// Offline driver for the same proposal implementation used by ICPLocalizer.
#include "../src/localizers/correlative_search.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <iostream>
#include <stdexcept>

static CorrelativeSearch::Cloud load(const char* path,float leaf) {
  CorrelativeSearch::Cloud::Ptr raw(new CorrelativeSearch::Cloud);
  if(pcl::io::loadPCDFile(path,*raw)<0) throw std::runtime_error("cannot load input PCD");
  pcl::VoxelGrid<pcl::PointXYZI> filter;filter.setLeafSize(leaf,leaf,leaf);filter.setInputCloud(raw);
  CorrelativeSearch::Cloud output;filter.filter(output);return output;
}
int main(int argc,char** argv) {
  if(argc<3){std::cerr<<"usage: correlative_search_probe MAP.pcd QUERY.pcd [QUERY.pcd ...]\n";return 2;}
  cv::setNumThreads(1);
  auto begin=std::chrono::steady_clock::now();
  CorrelativeSearch matcher;matcher.build(load(argv[1],.2f));
  std::cout<<"{\"prepare_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<"}"<<std::endl;
  for(int i=2;i<argc;++i){
    auto q=load(argv[i],.12f);begin=std::chrono::steady_clock::now();
    const auto r=matcher.searchWithDiagnostics(q,Eigen::Matrix3f::Identity());
    std::cout<<"{\"index\":"<<i-2<<",\"unique\":"<<(r.unique?"true":"false")
      <<",\"gap\":"<<r.gap<<",\"resolution\":"<<r.resolution
      <<",\"layers\":"<<r.layer_count<<",\"heights\":"<<r.height_hypotheses
      <<",\"seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()
      <<",\"proposals\":[";
    for(size_t n=0;n<r.proposals.size();++n){
      if(n)std::cout<<",";
      std::cout<<"{\"score\":"<<r.proposals[n].score<<",\"pose\":[";
      for(int row=0;row<4;++row){if(row)std::cout<<",";std::cout<<"[";
        for(int col=0;col<4;++col){if(col)std::cout<<",";std::cout<<r.proposals[n].pose(row,col);}
        std::cout<<"]";}
      std::cout<<"]}";
    }
    std::cout<<"]}"<<std::endl;
  }
}
