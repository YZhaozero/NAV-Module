#include "map_preview.h"
#include <stdexcept>
#include <iostream>
int main() {
  MapPreview map(.05,40);
  for(int region=-1;region<=1;++region) for(int n=0;n<1000;++n)
    map.add({float(region*100+n%10*.01),float(n/10*.01),1});
  bool left=false,right=false;
  for(const auto &entry:map.points()) {left|=entry.second.x < -90;right|=entry.second.x>90;}
  if(map.points().size()>40 || !left || !right)throw std::runtime_error("Old/new region lost");
  MapPreview rgb(.05,100);
  rgb.add({1,2,3,255,20,30,true});rgb.add({1,2,3});rgb.add({-4,2,3});
  if(rgb.points().size()!=2 || rgb.points().at({20,40,60}).r!=255)
    throw std::runtime_error("Geometry erased colour or uncoloured geometry");
  MapPreview one(.01,1);one.add({-1,-1,-1});one.add({1,1,1});
  if(one.points().size()!=1)throw std::runtime_error("Preview budget violated");
  std::cout<<"whole-scene preview, RGB overlay, negative coordinates, bounded memory passed\n";
}
