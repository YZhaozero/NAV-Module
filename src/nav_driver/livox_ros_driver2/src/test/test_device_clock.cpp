#include "comm/device_clock.h"
#include <iostream>
#include <stdexcept>
using livox_ros::DeviceClock;
void check(bool value) { if(!value) throw std::runtime_error("clock invariant failed"); }
int main() {
  const uint64_t wall=1800000000000000000ULL, mono=100000000000ULL, raw=50000000000ULL;
  DeviceClock clock;
  auto a=clock.map(raw,wall,mono,0,true);
  auto b=clock.map(raw+5000000,wall+505000000,mono+505000000,0,true);
  check(b.stamp-a.stamp==5000000); // 500 ms delivery stall, no fake gap.
  auto lidar=clock.map(raw+6000000,wall+505000001,mono+505000001,0,false);
  check(lidar.stamp-a.stamp==6000000); // shared clock, not one anchor per stream.
  auto c=clock.map(raw+10000000,wall+505000002,mono+505000002,0,true);
  check(c.stamp-b.stamp==5000000);
  auto late=clock.map(raw+9000000,wall+505000003,mono+505000003,0,true);
  check(!late.accept && late.event==DeviceClock::Late);
  auto lost=clock.map(raw+300000000,wall+600000000,mono+600000000,0,true);
  check(lost.stamp-c.stamp>=289970000); // actual missing samples are not filled.
  auto reboot=clock.map(1000000,wall+700000000,mono+700000000,0,true);
  check(reboot.epoch==2 && reboot.event==DeviceClock::Restart);
  auto stepped=clock.map(6000000,wall+1705000000,mono+705000000,0,true);
  check(stepped.event==DeviceClock::WallStep && stepped.epoch==3);
  auto ptp=clock.map(wall+2000000000,wall+2000000001,mono+1000000000,1,true);
  check(ptp.stamp==wall+2000000000 && ptp.event==DeviceClock::ModeChange);
  auto gps=clock.map(wall+2005000000,wall+2005000001,mono+1005000000,2,true);
  check(gps.stamp==wall+2005000000);
  DeviceClock drift;
  uint64_t previous=0;
  for(uint64_t i=0;i<120000;++i) { // 10 minutes, device clock +50 ppm.
    auto r=drift.map(raw+i*5000250,wall+i*5000000,mono+i*5000000,0,true);
    if(previous) check(r.stamp-previous>=4999750 && r.stamp-previous<=5000750);
    previous=r.stamp;
  }
  check(std::llabs(int64_t(previous)-int64_t(wall+119999ULL*5000000))<1000000);
  std::cout << "clock stall/loss/shared clock/reorder/reboot/wall step/PTP/GPS/drift passed\n";
}
