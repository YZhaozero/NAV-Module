#include "../src/tracking_policy.h"
#include <cassert>
#include <limits>
int main() {
    Eigen::Matrix3d r = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto prediction = tracking::predict(r, {10,20,0}, r, {2,0,0});
    assert((prediction.block<3,1>(0,3).isApprox(Eigen::Vector3f(10,22,0))));
    assert((prediction.block<3,3>(0,0).isApprox((r*r).cast<float>())));
    auto result = prediction;
    result(0,3) += .2;
    assert(tracking::acceptable(prediction,result,.01,.1,.5,15,false));
    result(0,3) += 2;
    assert(!tracking::acceptable(prediction,result,.01,.1,.5,15,false));
    assert(tracking::acceptable(prediction,result,.01,.1,.5,15,true));
    assert(!tracking::acceptable(prediction,result,.2,.1,.5,15,true));
    result(0,0) = std::numeric_limits<float>::quiet_NaN();
    assert(!tracking::acceptable(prediction,result,.01,.1,.5,15,true));
    assert(tracking::valid(true,1,.05,0,5,false));
    assert(!tracking::valid(true,6,.05,0,5,false)); // fresh odom cannot renew map match
    assert(!tracking::valid(true,1,2,0,5,false));
    assert(!tracking::valid(true,1,.05,3,5,false));
    assert(!tracking::valid(true,1,.05,0,5,true));
    assert(!tracking::valid(false,0,.05,0,5,false));
    // Field replay reproduced 0.93 s compute plus transfer producing a 1.029 s
    // source age. A fresh live stream keeps that bounded result usable.
    assert(tracking::observationFresh(1.35,.1,5.));
    assert(!tracking::observationFresh(1.35,1.01,5.));
    assert(!tracking::observationFresh(5.01,.1,5.));
    assert(!tracking::observationFresh(1.35,.1,.7));
    assert(!tracking::observationFresh(-.11,.1,5.));
    assert(!tracking::observationFresh(
        1.,std::numeric_limits<double>::quiet_NaN(),5.));
    assert(tracking::observationFresh(20.,.1,120.));
    // Field failures had valid geometry but live input age 1.67 / 2.20 s.
    assert(tracking::canAwaitFreshScan(1.67,1.67,120.,true));
    assert(tracking::canAwaitFreshScan(8.21,2.20,120.,true));
    assert(!tracking::observationFresh(8.21,2.20,120.));
    assert(!tracking::valid(false,0.,.1,0,5.,true));
    assert(!tracking::canAwaitFreshScan(8.21,2.20,120.,false));
    assert(!tracking::canAwaitFreshScan(121.,.1,120.,true));
    assert(!tracking::canAwaitFreshScan(-1.,.1,120.,true));
    assert(!tracking::canAwaitFreshScan(1.,-.2,120.,true));
    assert(!tracking::canAwaitFreshScan(1.,std::numeric_limits<double>::quiet_NaN(),120.,true));
    assert(tracking::needsFreshConfirmation(true,false,12.,true));
    assert(!tracking::needsFreshConfirmation(true,true,2.,true));
    assert(tracking::needsFreshConfirmation(true,true,2.,false));
    assert(!tracking::needsFreshConfirmation(false,false,2.,true));
}
