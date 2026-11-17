#include "map_spool.h"
#include <cassert>
#include <iostream>
#include <sstream>
#include <sys/wait.h>
#include <signal.h>
#include <thread>
#include <limits>
#include <system_error>

static void check(bool value) { if (!value) throw std::runtime_error("map spool regression failed"); }
static std::vector<MapSpool::Point> read(const std::string &name) {
  std::ifstream input(name,std::ios::binary);std::string line;size_t count=0;
  while(std::getline(input,line)) {
    if(line.rfind("POINTS ",0)==0)count=std::stoull(line.substr(7));
    if(line=="DATA binary")break;
  }
  std::vector<MapSpool::Point> points(count);
  input.read(reinterpret_cast<char *>(points.data()),count*sizeof(MapSpool::Point));
  check(size_t(input.gcount())==count*sizeof(MapSpool::Point));
  check(input.peek()==EOF);
  return points;
}
int main() {
  char root[]="/tmp/nav-map-spool-test-XXXXXX";check(mkdtemp(root)!=nullptr);
  MapSpool spool(root,1.0);
  spool.spill([](const MapSpool::Sink &sink) {sink({-0.2f,0,0,1});sink({1.1f,0,0,2});});
  spool.spill([](const MapSpool::Sink &sink) {sink({1.2f,0,0,3});sink({2.1f,0,0,4});});
  const std::string path=std::string(root)+"/map.pcd";
  spool.exportPcd(path,1.0,[](const MapSpool::Sink &sink) {sink({3.1f,0,0,5});});
  auto points=read(path);check(points.size()==4);check(points[0][0]<0);check(points[1][3]==3);
  spool.exportPcd(path,2.0,[](const MapSpool::Sink &sink) {sink({3.1f,0,0,5});});
  points=read(path);check(points.size()==3);check(std::abs(points[2][0]-2.6f)<.0001f);
  // Save a prefix while capture appends. Export and its marker must describe
  // exactly the prefix; never hide the subsequently committed checkpoint.
  const auto prefix = spool.snapshot();
  spool.spill([](const auto &sink) { sink({4.1f,0,0,6}); });
  MapSpool::ExportStats prefix_stats;
  MapSpool::exportSnapshot(prefix,path,1.0,[](const auto &) {},&prefix_stats);
  check(read(path).size()==3 && prefix_stats.input_points==4 && prefix_stats.output_points==3);
  check(!spool.markExportedIfCurrent(prefix,path));
  check(!std::filesystem::exists(spool.directory()+"/exported.txt"));
  MapSpool::commitReceipt(path,prefix,7,prefix_stats);
  std::ifstream receipt(path+".commit.json");
  std::string receipt_text((std::istreambuf_iterator<char>(receipt)),{});
  check(receipt_text.find("\"checkpoint_chunks\":2")!=std::string::npos &&
        receipt_text.find("\"archive_generation\":7")!=std::string::npos);
  spool.spill([](const auto &sink) { sink({5.1f,0,0,7}); });
  check(std::filesystem::exists(path+".commit.json")); // Independent completion receipt.
  // Concurrent immutable exports have separate disk indexes.
  std::exception_ptr export_failure;
  const auto parallel = std::string(root)+"/parallel.pcd";
  std::thread exporting([&] {
    try { MapSpool::exportSnapshot(prefix,parallel,1.,[](const auto &){}); }
    catch (...) { export_failure=std::current_exception(); }
  });
  spool.exportPcd(path,1.,[](const auto &){});
  exporting.join(); if(export_failure)std::rethrow_exception(export_failure);
  check(read(parallel).size()==3 && read(path).size()==5);

  // Millions of repeated measurements must be consolidated before SQL, while
  // preserving the last observed coordinates/intensity without averaging.
  MapSpool revisits(root,1.);
  for (int visit=0;visit<20;++visit) revisits.spill([&](const auto &sink) {
    for (int i=0;i<69000;++i) sink({float(i)+.125f+float(visit)*.03125f,.125f,1.f,float(visit)});
  });
  MapSpool::ExportStats visit_stats;
  revisits.exportPcd(path,1.,[](const auto &){},&visit_stats);
  points=read(path);
  check(points.size()==69000 && visit_stats.input_points==1380000 &&
        visit_stats.sql_rows==69000 && visit_stats.batches==1 && visit_stats.peak_unique_voxels==69000);
  for (int i=0;i<69000;++i)
    check(points[i]==MapSpool::Point({float(i)+.125f+19.f*.03125f,.125f,1.f,19.f}));
  // Cross the DISTINCT-key budget, then revisit a previously flushed voxel.
  // Compare against an independent unbounded last-observation reference map.
  MapSpool crossed(root,1.);
  std::map<std::array<int64_t,3>,MapSpool::Point> expected;
  crossed.spill([&](const auto &sink) {
    for (int i=-100;i<262200;++i) {
      MapSpool::Point p{float(i)+.25f,-.25f,1.f,float(i)};
      expected[{i,-1,1}]=p; sink(p);
    }
    MapSpool::Point p{-.125f,-.125f,1.25f,999.f}; expected[{-1,-1,1}]=p; sink(p);
  });
  MapSpool::ExportStats cross_stats;
  crossed.exportPcd(path,1.,[](const auto &){},&cross_stats);
  points=read(path);check(points.size()==expected.size());
  size_t at=0;for(const auto &entry:expected)check(points[at++]==entry.second);
  check(cross_stats.partition_passes==1 && cross_stats.sql_rows==expected.size() &&
        cross_stats.peak_unique_voxels==262144 && cross_stats.low_space_fallbacks==0 &&
        cross_stats.depth_fallbacks==0);
  // The real long-capture counterexample: more unique keys than RAM, repeated
  // across many input chunks. Each stored voxel should reach SQL only once.
  for(int visit=0;visit<4;++visit) crossed.spill([&](const auto &sink) {
    for(int i=-100;i<262200;++i) {
      MapSpool::Point p{float(i)+.125f+visit*.125f,-.375f,1.25f,float(visit+500)};
      expected[{i,-1,1}]=p;sink(p);
    }
  });
  crossed.exportPcd(path,1.,[](const auto &){},&cross_stats);
  points=read(path);at=0;for(const auto &entry:expected)check(points[at++]==entry.second);
  check(points.size()==expected.size() && cross_stats.sql_rows==expected.size() &&
        cross_stats.input_points==expected.size()*5+1 && cross_stats.partition_passes==1 &&
        cross_stats.peak_unique_voxels<=262144 && cross_stats.depth_fallbacks==0);
  // Force recursive repartitioning using a small memory budget, then force
  // the depth guard and low-disk-space slow path. All retain identical values.
  MapSpool nested(root,1.);
  std::map<std::array<int64_t,3>,MapSpool::Point> nested_expected;
  for(int visit=0;visit<6;++visit) nested.spill([&](const auto &sink) {
    for(int i=-80;i<80;++i) {
      MapSpool::Point p{float(i)+.125f+visit*.0625f,-.25f,1.f,float(visit)};
      nested_expected[{i,-1,1}]=p;sink(p);
    }
  });
  MapSpool::ExportLimits tiny;tiny.cached_voxels=8;tiny.partition_bits=1;
  MapSpool::ExportStats nested_stats;
  MapSpool::exportSnapshot(nested.snapshot(),path,1.,[](const auto &){},&nested_stats,{},&tiny);
  points=read(path);at=0;for(const auto &entry:nested_expected)check(points[at++]==entry.second);
  check(nested_stats.max_partition_depth>1 && nested_stats.depth_fallbacks==0 &&
        nested_stats.sql_rows==nested_expected.size() && nested_stats.peak_unique_voxels==8 &&
        nested_stats.peak_temporary_bytes<=nested_stats.source_bytes*2);
  tiny.partition_depth=1;
  MapSpool::exportSnapshot(nested.snapshot(),path,1.,[](const auto &){},&nested_stats,{},&tiny);
  points=read(path);at=0;for(const auto &entry:nested_expected)check(points[at++]==entry.second);
  check(nested_stats.depth_fallbacks>0 && nested_stats.max_partition_depth==1 &&
        nested_stats.peak_unique_voxels==8);
  tiny.partition_depth=10;tiny.free_space_reserve=std::numeric_limits<uint64_t>::max();
  MapSpool::exportSnapshot(nested.snapshot(),path,1.,[](const auto &){},&nested_stats,{},&tiny);
  points=read(path);at=0;for(const auto &entry:nested_expected)check(points[at++]==entry.second);
  check(nested_stats.low_space_fallbacks==1 && nested_stats.partition_files==0 && nested_stats.sql_rows>160);
  // Coarse output must average the final fine-voxel observations, rather than
  // averaging duplicates (which would bias frequently visited locations).
  tiny.free_space_reserve=64*1024*1024;
  MapSpool::exportSnapshot(nested.snapshot(),path,2.,[](const auto &){},&nested_stats,{},&tiny);
  points=read(path);check(points.size()==80);
  for(size_t n=0;n<points.size();++n) {
    const float x=float(int(n)*2-80)+.125f+5*.0625f+.5f;
    check(points[n]==MapSpool::Point({x,-.25f,1.f,5.f}));
  }
  // Cancel after temp partitions exist; an injected source write failure must
  // likewise clean all temporary buckets without damaging the source chunks.
  auto partitionExists=[&] {
    for(const auto &entry:std::filesystem::directory_iterator(nested.directory()))
      if(entry.is_directory() && entry.path().filename().string().find("-parts")!=std::string::npos)return true;
    return false;
  };
  bool partition_cancelled=false;
  try {
    MapSpool::exportSnapshot(nested.snapshot(),path,1.,[](const auto &){},nullptr,partitionExists,&tiny);
  }catch(const std::exception &){partition_cancelled=true;}
  check(partition_cancelled && !partitionExists());
  bool disk_failed=false;
  try {
    MapSpool::exportSnapshot(nested.snapshot(),path,1.,[](const auto &){
      throw std::system_error(std::make_error_code(std::errc::no_space_on_device));
    },nullptr,{},&tiny);
  }catch(const std::exception &){disk_failed=true;}
  check(disk_failed && !partitionExists());
  nested.exportPcd(path,1.,[](const auto &){});
  points=read(path);at=0;for(const auto &entry:nested_expected)check(points[at++]==entry.second);
  int cancel_checks=0; bool cancelled=false;
  const auto cancelled_path=std::string(root)+"/cancelled.pcd";
  try {
    MapSpool::exportSnapshot(crossed.snapshot(),cancelled_path,1.,[](const auto &){},nullptr,
                            [&]{return ++cancel_checks>=5;});
  } catch(const std::exception &) {cancelled=true;}
  check(cancelled && cancel_checks==5 && !std::filesystem::exists(cancelled_path));
  for(const auto &entry:std::filesystem::directory_iterator(crossed.directory()))
    check(entry.path().filename().string().rfind("export-",0)!=0);
  crossed.exportPcd(path,1.,[](const auto &){}); // Cancellation did not damage raw chunks.
  check(read(path).size()==expected.size());
  // More than the former one-million-voxel cap, without an in-memory point set.
  MapSpool large(root,.15);
  for(int block=0;block<3;++block)large.spill([&](const MapSpool::Sink &sink) {
    for(int i=0;i<400000;++i)sink({float((i+block*400000)%2000),float((i+block*400000)/2000),1,1});
  });
  large.exportPcd(path,.15,[](const MapSpool::Sink &){});
  check(read(path).size()==1200000);
  std::ofstream corrupt(large.directory()+"/points-0.bin",std::ios::binary|std::ios::app);corrupt.put('x');corrupt.close();
  bool failed=false;try{large.exportPcd(path,.15,[](const MapSpool::Sink &){});}catch(const std::exception &){failed=true;}
  check(failed);
  const auto crash_root=std::string(root)+"/crash";
  const auto child=fork();check(child>=0);
  if(child==0) {
    MapSpool crash(crash_root,.05);
    crash.spill([](const auto &sink){sink({-10,0,1,1});});
    crash.spill([](const auto &sink){sink({10,0,1,2});});
    kill(getpid(),SIGKILL);_exit(1);
  }
  int status=0;check(waitpid(child,&status,0)==child && WIFSIGNALED(status) && WTERMSIG(status)==SIGKILL);
  const auto crash_dir=std::filesystem::directory_iterator(crash_root)->path().string();
  {
    auto recovered=MapSpool::recover(crash_dir);
    recovered->exportPcd(path,.05,[](const auto &){});
    check(read(path).size()==2);
  }
  std::filesystem::remove(crash_dir+"/points-1.bin");
  failed=false;try{MapSpool::recover(crash_dir);}catch(const std::exception &){failed=true;}
  check(failed); // Even a missing LAST chunk must not look like a full recovery.
  std::string recover_dir;
  {
    MapSpool interrupted(root,.05);
    interrupted.spill([](const auto &sink){sink({-2,0,0,1});sink({2,0,0,2});});
    interrupted.spill([](const auto &sink){sink({4,0,0,3});});
    recover_dir=interrupted.directory();
    failed=false;
    try {MapSpool::recover(recover_dir);}catch(const std::exception &){failed=true;}
    check(failed); // Must not read a currently owned acquisition.
  }
  std::ofstream(recover_dir+"/points-2.bin.partial").put('x');
  {
    auto recovered=MapSpool::recover(recover_dir);
    check(recovered->voxel()==.05);
    recovered->exportPcd(path,.05,[](const auto &){});
    check(read(path).size()==3);
    recovered->markExported(path);
    recovered->spill([](const auto &sink){sink({6,0,0,4});});
    check(!std::filesystem::exists(recover_dir+"/exported.txt"));
  }
  std::filesystem::rename(recover_dir+"/points-1.bin",recover_dir+"/points-9.bin");
  failed=false;try {MapSpool::recover(recover_dir);}catch(const std::exception &){failed=true;}
  check(failed);
  std::cout<<"map spool: negative coordinates, last observation, coarsening, 1200000 points, truncated input passed\n";
  std::filesystem::remove_all(root);
}
