#include "../src/localizers/correlative_search.h"
#include <cassert>
#include <stdexcept>
int main() {
    CorrelativeSearch search;CorrelativeSearch::Cloud map,query;
    for(int i=0;i<80;++i) for(float z:{1.f,4.f,10.f}) {
        pcl::PointXYZI a;a.x=i*.2f-8;a.y=-4;a.z=z;map.push_back(a);
        a.x=-8;a.y=i*.1f-4;map.push_back(a);
        a.x=3;a.y=i*.08f;map.push_back(a);
    }
    const Eigen::Matrix3f rotation=Eigen::AngleAxisf(M_PI/3,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    for(const auto& p:map) {
        const Eigen::Vector3f q=rotation.transpose()*(Eigen::Vector3f(p.x,p.y,p.z)-Eigen::Vector3f(4,-3,0));
        pcl::PointXYZI a;a.x=q.x();a.y=q.y();a.z=q.z();query.push_back(a);
    }
    search.build(map);auto matches=search.search(query,Eigen::Matrix3f::Identity());
    assert(!matches.empty());const auto pose=matches[0].pose;
    assert((pose.block<3,1>(0,3)-Eigen::Vector3f(4,-3,0)).norm()<.8);
    assert(std::abs(Eigen::AngleAxisf(pose.block<3,3>(0,0)*rotation.transpose()).angle())<.06);
    // Cancel after entering the heading/layer loop, then reuse the same cache.
    int checkpoints = 0;
    bool cancelled = false;
    try {
        search.searchWithDiagnostics(query, Eigen::Matrix3f::Identity(), [&] {
            if (++checkpoints == 10) throw std::runtime_error("cancel test");
        });
    } catch (const std::runtime_error&) { cancelled = true; }
    assert(cancelled && checkpoints == 10);
    const auto retry = search.search(query, Eigen::Matrix3f::Identity());
    assert(!retry.empty() && retry.front().pose.isApprox(pose));
    search.clear();assert(search.search(query,Eigen::Matrix3f::Identity()).empty());
}
