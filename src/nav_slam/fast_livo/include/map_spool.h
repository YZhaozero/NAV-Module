#pragma once

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include <sqlite3.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/file.h>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <map>
#include <unordered_map>
#include <sstream>

// A full-resolution spill store. Only the live preview is bounded by RAM.
// Chunks survive an interrupted capture and can be exported without loading
// the entire map. SQLite performs the final voxel merge with a 4 MiB cache.
class MapSpool {
 public:
  using Point = std::array<float, 4>;
  using Sink = std::function<void(const Point &)>;
  using Source = std::function<void(const Sink &)>;
  using Cancel = std::function<bool()>;
  // Chunk files are immutable after their durable rename. A snapshot contains
  // only their paths, never the full map; new checkpoints may append meanwhile.
  struct Snapshot {
    std::string directory;
    double voxel;
    std::vector<std::string> chunks;
  };
  struct ExportStats {
    uint64_t input_points = 0, sql_rows = 0, batches = 0, output_points = 0;
    size_t peak_unique_voxels = 0;
    uint64_t source_bytes = 0, partition_bytes_written = 0, peak_temporary_bytes = 0;
    uint64_t partition_files = 0, partition_passes = 0, low_space_fallbacks = 0, depth_fallbacks = 0;
    unsigned max_partition_depth = 0;
  };
  struct ExportLimits {
    // Smaller limits support deterministic recursion/fallback regression tests.
    // Production uses this fixed memory budget; callers cannot increase it.
    size_t cached_voxels = 262144;
    unsigned partition_bits = 6, partition_depth = 10;
    uint64_t free_space_reserve = 64 * 1024 * 1024;
  };
  explicit MapSpool(std::string root, double voxel) : root_(root), voxel_(voxel) {}
  ~MapSpool() { if (lock_fd_ >= 0) close(lock_fd_); }
  MapSpool(const MapSpool &) = delete;
  MapSpool &operator=(const MapSpool &) = delete;
  double voxel() const { return voxel_; }
  static void commitPcd(const std::filesystem::path &temporary, const std::filesystem::path &output) {
    // Refuse replacement and durably publish the completed file before a
    // receipt can hide this session from the recovery list.
    std::filesystem::create_hard_link(temporary, output);
    syncAncestors(output.parent_path());
    std::filesystem::remove(temporary);
    syncDirectoryPath(output.parent_path().string());
  }
  // Recover one closed session only. Never join different coordinate frames.
  static std::unique_ptr<MapSpool> recover(const std::string &directory) {
    namespace fs = std::filesystem;
    const auto dir = fs::canonical(directory);
    auto result = std::make_unique<MapSpool>(dir.parent_path().string(), .15);
    result->directory_ = dir.string();
    result->lock();
    std::ifstream metadata(dir / "map.meta");
    long long committed = -1;
    if (metadata) {
      int version;
      if (!(metadata >> version >> result->voxel_) || version != 1 ||
          !std::isfinite(result->voxel_) || result->voxel_ < .01)
        throw std::runtime_error("Invalid map recovery metadata");
      if (!(metadata >> committed) || committed < 0)
        throw std::runtime_error("Invalid map checkpoint count");
    } // Legacy sessions used a fixed 0.15 m archive.
    std::map<size_t, std::string> chunks;
    for (const auto &entry : fs::directory_iterator(dir)) {
      const auto name = entry.path().filename().string();
      if (name.rfind("points-",0)!=0 || entry.path().extension()!=".bin") continue;
      const auto number = name.substr(7, name.size()-11);
      if (number.empty() || number.find_first_not_of("0123456789")!=std::string::npos || entry.is_symlink())
        throw std::runtime_error("Invalid map chunk name");
      const size_t index = std::stoull(number);
      if (!chunks.emplace(index, entry.path().string()).second) throw std::runtime_error("Duplicate map chunk");
    }
    for (const auto &entry : chunks) {
      if (entry.first != result->chunks_.size()) throw std::runtime_error("Missing map chunk");
      result->chunks_.push_back(entry.second);
    }
    if (committed >= 0 && size_t(committed) > result->chunks_.size())
      throw std::runtime_error("Missing committed map checkpoint");
    if (result->empty()) throw std::runtime_error("No completed map checkpoints");
    return result;
  }
  void markExported(const std::string &path) {
    std::ofstream output(directory_ + "/exported.partial");
    output.exceptions(std::ios::badbit | std::ios::failbit);
    output << path << '\n'; output.close();
    syncAndRelease(directory_ + "/exported.partial");
    std::filesystem::rename(directory_ + "/exported.partial", directory_ + "/exported.txt");
    syncDirectory();
  }
  bool empty() const { return chunks_.empty(); }
  const std::string &directory() const { return directory_; }
  Snapshot snapshot() {
    ensureDirectory();
    return {directory_, voxel_, chunks_};
  }
  bool matches(const Snapshot &snapshot) const {
    return directory_ == snapshot.directory && voxel_ == snapshot.voxel &&
        chunks_ == snapshot.chunks;
  }
  bool markExportedIfCurrent(const Snapshot &snapshot, const std::string &path) {
    if (!matches(snapshot)) return false;
    markExported(path);
    return true;
  }
  // This durable output receipt is independent of the recovery-list marker:
  // later checkpoints invalidate exported.txt, but cannot invalidate this file.
  static void commitReceipt(const std::filesystem::path &path, const Snapshot &snapshot,
                            uint64_t generation, const ExportStats &stats) {
    const auto receipt = path.string() + ".commit.json";
    auto temporary = temporaryPath(receipt + ".XXXXXX");
    try {
      std::ofstream output(temporary, std::ios::trunc);
      output.exceptions(std::ios::badbit | std::ios::failbit);
      output << "{\"schema_version\":1,\"output_path\":" << jsonString(path.string())
             << ",\"bytes\":" << std::filesystem::file_size(path)
             << ",\"points\":" << stats.output_points
             << ",\"source_spool\":" << jsonString(snapshot.directory)
             << ",\"checkpoint_chunks\":" << snapshot.chunks.size()
             << ",\"archive_generation\":" << generation << "}\n";
      output.close();
      syncAndRelease(temporary);
      commitPcd(temporary, receipt);
    } catch (...) {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
      throw;
    }
  }

  void spill(const Source &source) {
    ensureDirectory();
    // Invalidate the receipt before committing new data. A crash between chunk
    // rename and receipt removal must not hide the new unexported checkpoint.
    if (std::filesystem::remove(directory_ + "/exported.txt")) syncDirectory();
    const auto path = directory_ + "/points-" + std::to_string(chunks_.size()) + ".bin";
    const auto temporary = path + ".partial";
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    output.exceptions(std::ios::badbit | std::ios::failbit);
    source([&](const Point &p) { output.write(reinterpret_cast<const char *>(p.data()), sizeof(p)); });
    output.close();
    syncAndRelease(temporary);
    std::filesystem::rename(temporary, path);
    syncDirectory();
    chunks_.push_back(path);
    writeMetadata();
  }

  void exportPcd(const std::string &path, double leaf, const Source &current,
                 ExportStats *stats = nullptr) {
    exportSnapshot(snapshot(), path, leaf, current, stats);
  }
  static void exportSnapshot(const Snapshot &snapshot, const std::string &path, double leaf,
                             const Source &current, ExportStats *stats = nullptr,
                             const Cancel &cancel = {}, const ExportLimits *requested_limits = nullptr) {
    if (!std::isfinite(leaf) || leaf < snapshot.voxel)
      throw std::runtime_error("Export resolution is finer than the stored map");
    const ExportLimits limits = requested_limits ? *requested_limits : ExportLimits{};
    if (!limits.cached_voxels || limits.cached_voxels > 262144 ||
        !limits.partition_bits || limits.partition_bits > 6 ||
        limits.partition_depth > 10 || limits.partition_bits * limits.partition_depth > 64)
      throw std::runtime_error("Invalid bounded map export limits");
    ExportStats measured;
    for (const auto &chunk : snapshot.chunks) measured.source_bytes += std::filesystem::file_size(chunk);
    auto checkCancelled = [&]() {
      if (cancel && cancel()) throw std::runtime_error("Map export cancelled; checkpoints preserved");
    };
    checkCancelled();
    sqlite3 *db = nullptr;
    sqlite3_stmt *insert = nullptr, *rows = nullptr;
    // Recovery and independent snapshots must never share a mutable index.
    const auto index = temporaryPath(snapshot.directory + "/export-XXXXXX");
    std::string partitions;
    auto cleanup = [&]() {
      std::error_code ignored;
      std::filesystem::remove(index, ignored);
      std::filesystem::remove(index + "-journal", ignored);
      if (!partitions.empty()) std::filesystem::remove_all(partitions, ignored);
    };
    auto sql = [&](const char *statement) {
      if (sqlite3_exec(db, statement, nullptr, nullptr, nullptr) != SQLITE_OK)
        throw std::runtime_error(std::string("Map merge failed: ") + sqlite3_errmsg(db));
    };
    try {
      if (sqlite3_open(index.c_str(), &db) != SQLITE_OK)
        throw std::runtime_error("Cannot create disk map index");
      if (cancel) sqlite3_progress_handler(db, 4096,
          [](void *context) -> int {
            try { return (*static_cast<const Cancel *>(context))() ? 1 : 0; }
            catch (...) { return 1; }
          }, const_cast<Cancel *>(&cancel));
      sql("PRAGMA cache_size=-4096; PRAGMA temp_store=FILE; PRAGMA journal_mode=DELETE;"
          "DROP TABLE IF EXISTS points; CREATE TABLE points(ix INTEGER,iy INTEGER,iz INTEGER,"
          "x REAL,y REAL,z REAL,i REAL,PRIMARY KEY(ix,iy,iz)) WITHOUT ROWID; BEGIN;");
      if (sqlite3_prepare_v2(db, "INSERT OR REPLACE INTO points VALUES(?,?,?,?,?,?,?)", -1, &insert, nullptr) != SQLITE_OK)
        throw std::runtime_error(sqlite3_errmsg(db));
      // Keep small maps in RAM; once the unique-key budget is exceeded,
      // partition observations by a complete voxel-key hash instead of repeatedly
      // flushing the same keys to SQL. Every key stays in one ordered stream.
      using Key = std::array<int64_t,3>;
      struct Entry { Key key; Point point; };
      struct KeyHash {
        static uint64_t mix(uint64_t x) {
          x += UINT64_C(0x9e3779b97f4a7c15);
          x = (x ^ (x >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
          x = (x ^ (x >> 27)) * UINT64_C(0x94d049bb133111eb);
          return x ^ (x >> 31);
        }
        static uint64_t full(const Key &key) {
          uint64_t h = UINT64_C(0x43b0d7e5c32f197a);
          for (auto value : key) h = mix(h ^ mix(uint64_t(value)));
          return h;
        }
        size_t operator()(const Key &key) const { return size_t(full(key)); }
      };
      using Cache = std::unordered_map<Key,Point,KeyHash>;
      std::vector<Entry> batch;
      batch.reserve(limits.cached_voxels);
      auto keyFor = [&](const Point &p) {
        Key key;
        for (int k=0;k<3;++k) {
          if (!std::isfinite(p[k]) || std::abs(p[k])>1e6)
            throw std::runtime_error("Invalid point in map checkpoint");
          key[k]=int64_t(std::floor(p[k]/snapshot.voxel));
        }
        return key;
      };
      auto flush = [&](Cache &latest) {
        if (latest.empty()) return;
        checkCancelled();
        for (const auto &entry : latest) batch.push_back({entry.first,entry.second});
        std::sort(batch.begin(),batch.end(),[](const Entry &a,const Entry &b){return a.key<b.key;});
        for (size_t n=0;n<batch.size();++n) {
          if ((n & 4095)==0) checkCancelled();
          for (int k=0;k<3;++k) sqlite3_bind_int64(insert,k+1,batch[n].key[k]);
          for (int k=0;k<4;++k) sqlite3_bind_double(insert,k+4,batch[n].point[k]);
          if (sqlite3_step(insert)!=SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(db));
          sqlite3_reset(insert);
        }
        measured.sql_rows+=batch.size();++measured.batches;
        latest.clear();batch.clear();
      };
      auto readPoints = [&](const std::string &file, const Sink &sink) {
        std::ifstream input(file,std::ios::binary);
        if (!input) throw std::runtime_error("Cannot read map spill: "+file);
        std::array<Point,4096> block;
        while (input) {
          checkCancelled();
          input.read(reinterpret_cast<char *>(block.data()),sizeof(block));
          const auto bytes=input.gcount();
          if (bytes%sizeof(Point) || input.bad() || (input.fail() && !input.eof()))
            throw std::runtime_error("Truncated map spill: "+file);
          for (size_t n=0;n<size_t(bytes)/sizeof(Point);++n) sink(block[n]);
        }
        input.close();releaseReadCache(file);
      };
      struct BucketSet {
        std::array<std::ofstream,64> streams;
        std::array<std::string,64> paths;
        std::array<uint64_t,64> sizes{};
        unsigned count;
        explicit BucketSet(unsigned n):count(n) {}
        void close() {
          for (unsigned n=0;n<count;++n) if (streams[n].is_open()) streams[n].close();
        }
        void flush() {
          for (unsigned n=0;n<count;++n) if (streams[n].is_open()) streams[n].flush();
        }
      };
      uint64_t temporary_bytes=0, next_space_check=8*1024*1024, partition_id=0;
      auto available = [&] { return std::filesystem::space(snapshot.directory).available; };
      std::function<void(const Source &,unsigned,uint64_t,const std::string &)> merge;
      merge = [&](const Source &source,unsigned depth,uint64_t source_bytes,const std::string &parent_file) {
        checkCancelled();
        measured.max_partition_depth=std::max(measured.max_partition_depth,depth);
        auto latest=std::make_unique<Cache>();latest->reserve(limits.cached_voxels);
        std::unique_ptr<BucketSet> buckets;
        bool sql_fallback=false;
        uint64_t seen=0;
        auto writeBucket = [&](const Key &key,const Point &p) {
          const unsigned bucket=unsigned((KeyHash::full(key)>>(depth*limits.partition_bits)) &
                                        ((uint64_t(1)<<limits.partition_bits)-1));
          auto &stream=buckets->streams[bucket];
          if (!stream.is_open()) {
            buckets->paths[bucket]=partitions+"/bucket-"+std::to_string(partition_id++)+".bin";
            stream.exceptions(std::ios::badbit|std::ios::failbit);
            stream.open(buckets->paths[bucket],std::ios::binary|std::ios::trunc);
            ++measured.partition_files;
          }
          if (measured.partition_bytes_written>=next_space_check) {
            checkCancelled();buckets->flush();
            if (available()<limits.free_space_reserve)
              throw std::runtime_error("Insufficient map export temporary space; checkpoints preserved");
            next_space_check=measured.partition_bytes_written+8*1024*1024;
          }
          stream.write(reinterpret_cast<const char *>(p.data()),sizeof(p));
          buckets->sizes[bucket]+=sizeof(p);temporary_bytes+=sizeof(p);
          measured.partition_bytes_written+=sizeof(p);
          measured.peak_temporary_bytes=std::max(measured.peak_temporary_bytes,temporary_bytes);
        };
        source([&](const Point &p) {
          if ((seen++ & 4095)==0) checkCancelled();
          const auto key=keyFor(p);
          if (buckets) {writeBucket(key,p);return;}
          auto found=latest->find(key);
          if (found!=latest->end()) {found->second=p;return;}
          if (latest->size()==limits.cached_voxels) {
            if (!sql_fallback) {
              if (depth>=limits.partition_depth) {
                sql_fallback=true;++measured.depth_fallbacks;
              } else {
                const auto free=available();
                // Preflight is conservative for a closed file source. A generic
                // in-memory Source has no declared length, so dynamic checks
                // remain necessary. Low space keeps the bounded slow SQL path.
                const auto estimate=std::max(source_bytes,uint64_t(latest->size()*sizeof(Point)));
                if (free<limits.free_space_reserve || estimate>free-limits.free_space_reserve) {
                  sql_fallback=true;++measured.low_space_fallbacks;
                }
              }
            }
            if (sql_fallback) flush(*latest);
            else {
              if (partitions.empty()) {
                const auto directory=index+"-parts";
                if (!std::filesystem::create_directory(directory))
                  throw std::runtime_error("Cannot create private map partition directory");
                partitions=directory;
              }
              buckets=std::make_unique<BucketSet>(1u<<limits.partition_bits);
              ++measured.partition_passes;
              // Each cached key is unique, so their inter-key order is irrelevant;
              // all later observations are appended in their original order.
              for (const auto &entry:*latest) writeBucket(entry.first,entry.second);
              latest.reset();
              writeBucket(key,p);return;
            }
          }
          latest->emplace(key,p);
          measured.peak_unique_voxels=std::max(measured.peak_unique_voxels,latest->size());
        });
        auto releaseParent = [&] {
          if (!parent_file.empty()) {
            std::filesystem::remove(parent_file);temporary_bytes-=source_bytes;
          }
        };
        if (!buckets) {flush(*latest);releaseParent();return;}
        buckets->close(); // Close all writers before recursion opens a reader.
        // The caller finishes consuming this source before visiting its children;
        // remove that parent bucket now to keep the temporary peak below two
        // complete streams. Original capture chunks are NEVER removed here.
        releaseParent();
        std::vector<std::pair<std::string,uint64_t>> children;
        for (unsigned n=0;n<buckets->count;++n)
          if (!buckets->paths[n].empty()) children.emplace_back(buckets->paths[n],buckets->sizes[n]);
        buckets.reset(); // No parent cache/64 stream buffers remain on recursion.
        for (const auto &child:children) {
          merge([&](const Sink &sink){readPoints(child.first,sink);},depth+1,child.second,child.first);
        }
      };
      merge([&](const Sink &sink) {
        const Sink counted=[&](const Point &p){++measured.input_points;sink(p);};
        for (const auto &chunk:snapshot.chunks) readPoints(chunk,counted);
        current(counted);
      },0,measured.source_bytes,{});
      sql("COMMIT");
      checkCancelled();
      sqlite3_finalize(insert); insert=nullptr;
      // SQLite's built-in math functions are not enabled on every distribution.
      sqlite3_create_function_v2(db,"map_cell",2,SQLITE_UTF8|SQLITE_DETERMINISTIC,nullptr,
          [](sqlite3_context *ctx,int,sqlite3_value **v) {
            sqlite3_result_int64(ctx,int64_t(std::floor(sqlite3_value_double(v[0])/sqlite3_value_double(v[1]))));
          },nullptr,nullptr,nullptr);
      std::string query = "SELECT x,y,z,i FROM points";
      if (leaf > snapshot.voxel) {
        if (sqlite3_prepare_v2(db,"CREATE TEMP TABLE reduced AS SELECT avg(x) x,avg(y) y,avg(z) z,avg(i) i FROM points GROUP BY map_cell(x,?),map_cell(y,?),map_cell(z,?)",-1,&rows,nullptr)!=SQLITE_OK)
          throw std::runtime_error(sqlite3_errmsg(db));
        for(int k=1;k<=3;++k) sqlite3_bind_double(rows,k,leaf);
        if(sqlite3_step(rows)!=SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(db));
        sqlite3_finalize(rows);rows=nullptr;
        query="SELECT x,y,z,i FROM reduced";
      }
      const auto count_query="SELECT count(*) FROM ("+query+")";
      if(sqlite3_prepare_v2(db,count_query.c_str(),-1,&rows,nullptr)!=SQLITE_OK || sqlite3_step(rows)!=SQLITE_ROW)
        throw std::runtime_error(sqlite3_errmsg(db));
      const auto count=sqlite3_column_int64(rows,0);
      measured.output_points = uint64_t(count);
      sqlite3_finalize(rows);rows=nullptr;
      if(!count) throw std::runtime_error("No map points received");
      std::ofstream output(path,std::ios::binary|std::ios::trunc);
      output.exceptions(std::ios::badbit|std::ios::failbit);
      output << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH " << count
             << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << count << "\nDATA binary\n";
      if(sqlite3_prepare_v2(db,query.c_str(),-1,&rows,nullptr)!=SQLITE_OK) throw std::runtime_error(sqlite3_errmsg(db));
      int status;
      uint64_t written = 0;
      while((status=sqlite3_step(rows))==SQLITE_ROW) {
        if ((written++ & 4095) == 0) checkCancelled();
        Point p;for(int k=0;k<4;++k)p[k]=float(sqlite3_column_double(rows,k));
        output.write(reinterpret_cast<const char *>(p.data()),sizeof(p));
      }
      if(status!=SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(db));
      output.close();syncAndRelease(path);
      checkCancelled();
      sqlite3_finalize(rows);rows=nullptr;sqlite3_close(db);db=nullptr;
      cleanup();
      if (stats) *stats = measured;
    } catch (...) {
      if(insert)sqlite3_finalize(insert);
      if(rows)sqlite3_finalize(rows);
      if(db)sqlite3_close(db);
      cleanup();
      throw;
    }
  }

 private:
  static std::string temporaryPath(const std::string &pattern) {
    std::vector<char> name(pattern.begin(), pattern.end()); name.push_back('\0');
    const int fd = mkstemp(name.data());
    if (fd < 0) throw std::runtime_error("Cannot create temporary map file");
    close(fd);
    return name.data();
  }
  static std::string jsonString(const std::string &value) {
    std::ostringstream output;
    output << '"';
    for (unsigned char c : value) {
      if (c == '"' || c == '\\') output << '\\' << char(c);
      else if (c < 32) output << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c);
      else output << char(c);
    }
    output << '"';
    return output.str();
  }
  static void releaseReadCache(const std::string &path) {
    // Chunks were already fdatasync'ed before publication. A reader only needs
    // to release its clean cache; another sync does not make the snapshot safer.
    const int fd = open(path.c_str(), O_RDONLY | O_NOFOLLOW);
    if (fd < 0) throw std::runtime_error("Cannot reopen map spill: " + path);
    posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
    close(fd);
  }
  void ensureDirectory() {
    if(!directory_.empty())return;
    std::filesystem::create_directories(root_);
    std::string pattern=root_+"/capture-XXXXXX";
    std::vector<char> name(pattern.begin(),pattern.end());name.push_back('\0');
    if(!mkdtemp(name.data()))throw std::runtime_error("Cannot create map spill directory");
    directory_=name.data();
    // fsync of the new directory alone does not persist its parent entry.
    // Include newly created ancestor directories on a first-ever capture.
    syncAncestors(root_);
    lock();
    writeMetadata();
    syncDirectory();
  }
  static void syncAncestors(const std::filesystem::path &path) {
    for (auto parent=std::filesystem::absolute(path); !parent.empty();) {
      syncDirectoryPath(parent.string());
      auto next=parent.parent_path();
      if(next==parent) break;
      parent=next;
    }
  }
  void writeMetadata() {
    std::ofstream metadata(directory_ + "/map.meta.partial");
    metadata.exceptions(std::ios::badbit | std::ios::failbit);
    metadata << "1 " << std::setprecision(17) << voxel_ << ' ' << chunks_.size() << '\n'; metadata.close();
    syncAndRelease(directory_ + "/map.meta.partial");
    std::filesystem::rename(directory_ + "/map.meta.partial", directory_ + "/map.meta");
    syncDirectory();
  }
  void lock() {
    lock_fd_=open((directory_+"/active.lock").c_str(),O_CREAT|O_RDWR|O_NOFOLLOW,0600);
    if(lock_fd_<0 || flock(lock_fd_,LOCK_EX|LOCK_NB)) throw std::runtime_error("Map capture or recovery is still running");
  }
  void syncDirectory() {
    syncDirectoryPath(directory_);
  }
  static void syncDirectoryPath(const std::string &path) {
    const int fd=open(path.c_str(),O_RDONLY|O_DIRECTORY);
    if(fd<0)throw std::runtime_error("Cannot open map checkpoint directory");
    const int result=fsync(fd);close(fd);
    if(result)throw std::runtime_error("Cannot sync map checkpoint directory");
  }
  static void syncAndRelease(const std::string &path) {
    const int fd=open(path.c_str(),O_RDONLY|O_NOFOLLOW);
    if(fd<0)throw std::runtime_error("Cannot sync map file: "+path);
    const int result=fdatasync(fd);
    if(!result)posix_fadvise(fd,0,0,POSIX_FADV_DONTNEED);
    close(fd);
    if(result)throw std::runtime_error("Map file sync failed: "+path);
  }
  std::string root_, directory_;
  double voxel_;
  std::vector<std::string> chunks_;
  int lock_fd_ = -1;
};
