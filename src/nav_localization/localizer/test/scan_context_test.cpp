#include "../src/localizers/scan_context.h"
#include <cassert>
#include <chrono>
#include <iostream>
#include <random>

int main() {
    ScanContextManager sc;
    std::mt19937 rng(20261005);
    std::uniform_real_distribution<double> value(0.1, 10.);
    auto matrix = [&]() {
        Eigen::MatrixXd result(20, 60);
        for (int c=0; c<60; ++c)
            for (int r=0; r<20; ++r) result(r,c) = value(rng);
        return result;
    };
    auto legacy = [&](const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
        std::pair<double,int> best{1000000., 0};
        for (int s=0; s<60; ++s) {
            const double distance = sc.distDirectSC(a, sc.circshift(b,s));
            if (distance < best.first) best = {distance,s};
        }
        return best;
    };
    for (int trial=0; trial<100; ++trial) {
        Eigen::MatrixXd a=matrix(), b=matrix();
        if (trial%2) for (int c=0; c<60; c+=3) a.col(c).setZero();
        if (trial%3) for (int c=1; c<60; c+=4) b.col(c).setZero();
        const auto old=legacy(a,b), optimized=sc.distanceBtnScanContext(a,b);
        assert(std::abs(old.first-optimized.first) < 1e-12);
        assert(old.second==optimized.second);
    }
    const Eigen::MatrixXd empty=Eigen::MatrixXd::Zero(20,60);
    assert(sc.distanceBtnScanContext(empty,matrix()).first==1.);
    assert(sc.distanceBtnScanContext(empty,empty).second==0);

    // Asymmetric 3D scene, sector/ring centers avoid quantization boundaries.
    auto world = ScanContextManager::CloudType::Ptr(new ScanContextManager::CloudType);
    for (int c=0; c<60; ++c) for (int r=0; r<20; ++r) {
        const double theta=-M_PI+(c+.5)*2*M_PI/60., radius=(r+.5)*4.;
        pcl::PointXYZI point;
        point.x=radius*std::cos(theta); point.y=radius*std::sin(theta);
        point.z=value(rng); point.intensity=0.; world->push_back(point);
    }
    const auto world_sc=sc.makeScanContext(world);
    for (int degrees : {0, 42, 90, 180, 270}) {
        const Eigen::Matrix3f map_body =
            (Eigen::AngleAxisf(degrees*M_PI/180.,Eigen::Vector3f::UnitZ()) *
             Eigen::AngleAxisf(.51,Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(-.12,Eigen::Vector3f::UnitX())).toRotationMatrix();
        // Odometry's heading is unrelated to map north; only gravity is usable.
        const Eigen::Matrix3f odom_body =
            Eigen::AngleAxisf(.73,Eigen::Vector3f::UnitZ()) * map_body;
        const Eigen::Matrix3f level=ScanContextManager::gravityLevelRotation(odom_body);
        auto query=ScanContextManager::CloudType::Ptr(new ScanContextManager::CloudType);
        for (const auto &point : *world) {
            const Eigen::Vector3f body=map_body.transpose()*point.getVector3fMap();
            pcl::PointXYZI leveled;
            leveled.getVector3fMap()=level*body; leveled.intensity=0.;
            query->push_back(leveled);
        }
        const auto match=sc.distanceBtnScanContext(sc.makeScanContext(query),world_sc);
        const Eigen::Matrix3f recovered =
            Eigen::AngleAxisf(sc.mapFromQueryYaw(match.second),Eigen::Vector3f::UnitZ()) * level;
        assert(match.first<1e-10);
        assert((recovered-map_body).norm()<1e-5);
    }
    // Informational microbenchmark, never a timing-dependent test assertion.
    const Eigen::MatrixXd a=matrix(), b=matrix();
    volatile double checksum=0.;
    auto measure=[&](bool old) {
        const auto start=std::chrono::steady_clock::now();
        for (int i=0; i<500; ++i)
            checksum += old ? legacy(a,b).first : sc.distanceBtnScanContext(a,b).first;
        return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    };
    const double old_s=measure(true), new_s=measure(false);
    std::cout << "100 score equivalence cases, empty inputs, 5 rotated/tilted clouds passed; "
              << "500 pairs legacy_s=" << old_s << " optimized_s=" << new_s
              << " checksum=" << checksum << std::endl;
}
