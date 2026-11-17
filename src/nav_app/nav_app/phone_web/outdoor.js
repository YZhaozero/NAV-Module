'use strict';
const $=id=>document.getElementById(id);
let map,marker,line,trackLines=[],savedTrackLines=[],points=[],draft=null,selectedStart=null,selectedEnd=null,picking='end',lastState=null,editing=false,satellite=false,polling=false,titleDirty=false;
const say=text=>{$('message').textContent=text;};
let geoChoiceDirty=false,geoSignature='';
let followHeading=true,followPosition=false,routeSignature='',trackSignature='',savedTrackSignature='';
function updateHeading(state){
  const yaw=state?.position?.heading_enu_rad;
  const valid=!!state?.bd09&&!state.simulation&&Number.isFinite(yaw);
  const bearing=valid?((90-yaw*180/Math.PI)%360+360)%360:0;
  $('heading-status').textContent=valid?`设备朝向 ${Math.round(bearing)}°`:'方向待确认';
  $('heading').textContent=followHeading?'朝向跟随':'北向朝上';
  $('heading').setAttribute('aria-pressed',String(followHeading));
  if(map&&typeof map.setHeading==='function')map.setHeading(followHeading&&valid?bearing:0,{noAnimation:true});
  if(map&&followPosition&&state?.bd09)map.setCenter(point(state.bd09.point),{noAnimation:true});
}
$('heading').onclick=()=>{followHeading=!followHeading;updateHeading(lastState);};
$('geo-map').onchange=()=>{geoChoiceDirty=true;};
async function geoControl(action){
  try{await api('map',{action,id:$('geo-map').value||null});geoChoiceDirty=false;await poll();
    if(action==='select'&&map&&lastState?.saved_map_track?.track_bd09?.length){
      followPosition=false;map.setViewport(lastState.saved_map_track.track_bd09.map(v=>point(v.point)));
    }
    say(action==='select'?'地图选择已保存':action==='confirm'?'已确认新位置':'定位操作已提交，请查看定位状态');}catch(e){say(e.message);}
}
for(const action of ['select','start','stop','confirm','anchor'])$('geo-'+action).onclick=()=>geoControl(action);
document.addEventListener('securitypolicyviolation',event=>{
  $('map-message').textContent='地图资源加载失败，请刷新；持续失败请联系设备管理员';
  console.warn('Map resource blocked',event.effectiveDirective,event.blockedURI.split('?')[0]);
});
async function api(path,data,timeout=12000){
  const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),timeout);
  try{
    const response=await fetch('/navigation/'+path,{method:data?'POST':'GET',credentials:'same-origin',cache:'no-store',signal:controller.signal,
      headers:data?{'Content-Type':'application/json'}:{},body:data?JSON.stringify(data):undefined});
    if(response.status===401){location.replace('/');throw new Error('请重新连接设备');}
    const result=await response.json();
    if(!response.ok)throw new Error(result.message||'设备暂时无法处理，请重试');
    return result;
  }finally{clearTimeout(timer);}
}
function point(p){return NAVBaidu.point(p);}
function renderSavedTrack(state){
  const saved=!state.simulation&&state.saved_map_track?.map_id===state.geographic?.selected_map_id?state.saved_map_track:null;
  if(saved){
    const source=saved.reference_source==='manual_map_selection'?'手动参考':saved.reference_source==='trajectory_alignment'?'轨迹对齐参考':'卫星与雷达参考';
    const count=saved.counts?.track_wgs84||0;
    const detail=saved.track_bd09?.length?`紫线为已保存轨迹${(saved.counts?.track_bd09||0)<count?'，部分路段仅保留经纬度，可导出查看':''}`:count?'经纬度轨迹可导出；缺少保存的百度坐标，暂不能上图':'这份地图没有可显示的地理轨迹';
    $('geo-status').textContent+=`。${detail}（${source}，未经实测验证）`;
  }
  if(!map)return;
  const signature=JSON.stringify(saved&&[saved.map_id,saved.track_bd09]);
  if(signature===savedTrackSignature)return;
  savedTrackSignature=signature;
  for(const line of savedTrackLines)map.removeOverlay(line);savedTrackLines=[];
  let segment=[],previous=null;
  const flush=()=>{if(segment.length>1){const line=new BMap.Polyline(segment.map(point),{strokeColor:'#8855ba',strokeWeight:4,strokeOpacity:.75,strokeStyle:'dashed'});map.addOverlay(line);savedTrackLines.push(line);}segment=[];};
  for(const sample of saved?.track_bd09||[]){
    if(previous&&(sample.stamp<=previous.stamp||sample.stamp-previous.stamp>10||sample.segment!==previous.segment||sample.epoch!==previous.epoch))flush();
    segment.push(sample.point);previous=sample;
  }
  flush();
}
function drawRoute(){
  if(!map)return;
  const route=editing?draft:lastState?.route;
  const signature=JSON.stringify(route);
  if(signature===routeSignature)return;
  routeSignature=signature;
  if(line){map.removeOverlay(line);line=null;}
  if(route&&route.points.length>1){line=new BMap.Polyline(route.points.map(point),{strokeColor:'#376dce',strokeWeight:6,strokeOpacity:.9});map.addOverlay(line);}
  $('route-summary').textContent=route?`${editing?'待保存 · ':''}${route.title} · ${(route.distance_m/1000).toFixed(2)} 公里${route.source==='manual'?' · 手工路线':''}`:'还没有路线。请先选择起点和终点。';
}
function showPins(){
  if(!map)return;
  for(const name of ['startPin','endPin'])if(showPins[name])map.removeOverlay(showPins[name]);
  for(const [name,p,text]of [['startPin',selectedStart,'起'],['endPin',selectedEnd,'终']])if(p){
    const pin=new BMap.Marker(point(p));pin.setLabel(new BMap.Label(text,{offset:new BMap.Size(15,-5)}));map.addOverlay(pin);showPins[name]=pin;
  }
}
function acceptClick(p){
  if(lastState?.guiding||lastState?.simulation){say('请先结束引导并退出模拟，再修改路线');return;}
  if(picking==='draw'){
    points.push(p);draft={points:points.slice(),title:$('title').value||'手工路线',source:'manual',coordinate_system:'BD09',distance_m:0};
    for(let i=1;i<points.length;i++)draft.distance_m+=map.getDistance(point(points[i-1]),point(points[i]));
    editing=true;drawRoute();say(`已标记 ${points.length} 个点，完成后点“保存路线”`);return;
  }
  if(picking==='start'){selectedStart=p;$('origin').value='地图选定的起点';picking='end';say('起点已选，请继续选择终点');}
  else{selectedEnd=p;$('destination').value='地图选定的终点';say('终点已选，可以规划步行路线');}
  showPins();
}
async function resolve(text){
  if(!text.trim())throw new Error('请输入地点名称或在地图上选点');
  return new Promise((done,fail)=>{
    const timer=setTimeout(()=>fail(new Error('地点搜索超时，请在地图上选点')),10000);
    const search=new BMap.LocalSearch(map,{onSearchComplete:result=>{
      clearTimeout(timer);
      if(search.getStatus()!==BMAP_STATUS_SUCCESS||!result.getCurrentNumPois()){fail(new Error('未找到地点，请补充城市和详细名称'));return;}
      const poi=result.getPoi(0);done({position:[poi.point.lng,poi.point.lat],title:poi.title});
    }});search.search(text.trim());
  });
}
$('origin').addEventListener('input',()=>{selectedStart=null;});
$('destination').addEventListener('input',()=>{selectedEnd=null;});
$('title').addEventListener('input',()=>{titleDirty=true;});
$('pick-start').onclick=()=>{picking='start';$('draw').setAttribute('aria-pressed','false');say('请在地图上轻点起点');};
$('pick-end').onclick=()=>{picking='end';$('draw').setAttribute('aria-pressed','false');say('请在地图上轻点终点');};
$('draw').onclick=()=>{picking=picking==='draw'?'end':'draw';$('draw').setAttribute('aria-pressed',String(picking==='draw'));if(picking==='draw'){points=[];draft=null;editing=true;selectedStart=selectedEnd=null;showPins();drawRoute();say('依次点击沿途位置画线，完成后保存路线');}};
$('undo').onclick=()=>{if(picking!=='draw'||!points.length)return;points.pop();draft=null;if(points.length){const p=points.pop();acceptClick(p);}else{editing=true;drawRoute();say('已撤销所有手工点');}};
$('search').onsubmit=async event=>{
  event.preventDefault();if(!map)return;say('正在规划…');$('plan').disabled=true;
  try{
    if(!selectedStart){const r=await resolve($('origin').value);selectedStart=r.position;$('origin').value=r.title;}
    if(!selectedEnd){const r=await resolve($('destination').value);selectedEnd=r.position;$('destination').value=r.title;}
    showPins();draft=await api('plan',{start:selectedStart,end:selectedEnd,title:$('title').value||'步行路线'});
    editing=true;drawRoute();map.setViewport(draft.points.map(point));say('路线已规划，请核对起终点，再保存到设备');
  }catch(e){say(e.name==='AbortError'?'规划超时，请重试':e.message);}finally{$('plan').disabled=false;}
};
$('save-route').onclick=async()=>{
  if(!draft){say('请先规划或画一条路线');return;}
  try{draft=await api('route',{...draft,title:$('title').value});editing=false;titleDirty=false;lastState=await api('state');drawRoute();say('路线已保存到设备，重新打开仍可使用');}catch(e){say(e.message);}
};
async function control(action){try{const simulated=!!lastState?.simulation;await api('control',{action});say(simulated?(action==='stop'?'已退出模拟':'模拟行走已开始'):action==='start'?'步行引导已开始':action==='stop'?'引导已结束，实走轨迹已保存':'实走轨迹已保存到设备');await poll();}catch(e){say(e.message);}}
async function simulate(action){try{await api('simulation',{action});say(action==='exit'?'已退出模拟，恢复真实设备定位':'模拟状态已更新');await poll();}catch(e){say(e.message);}}
$('sim-enter').onclick=()=>simulate('enter');
document.querySelectorAll('[data-sim]').forEach(button=>{button.onclick=()=>simulate(button.dataset.sim);});
$('start').onclick=()=>control('start');$('stop').onclick=()=>control('stop');$('save-track').onclick=()=>control('save');
$('locate').onclick=()=>{if(!map)return;if(!lastState?.bd09){say('设备尚无有效位置，请等待定位恢复');return;}followPosition=true;map.panTo(point(lastState.bd09.point));selectedStart=lastState.bd09.point;$('origin').value='设备当前位置';showPins();};
$('satellite').onclick=()=>{if(!map)return;satellite=!satellite;map.setMapType(satellite?BMAP_HYBRID_MAP:BMAP_NORMAL_MAP);$('satellite').setAttribute('aria-pressed',String(satellite));};
$('campus').onclick=()=>{
  if(!map)return;
  followPosition=false;
  // User-supplied Baidu MC link; this is not EPSG:3857 or a GNSS position.
  const p=map.mercatorToLnglat(12953465.523501843,4837876.262720998);
  map.centerAndZoom(point(p),20);say('已定位到机械学院附近，请在右侧道路选择起终点');
};
function render(state){
  renderJourney(state.journey);
  lastState=state;$('connection').textContent='设备已连接';$('position-status').textContent=state.simulation?state.observation_reason:(state.navigation?.reason||state.observation_reason);$('alignment-status').textContent=[state.alignment_reason,state.motion_status].filter(Boolean).join(' · ');
  const simulated=!!state.simulation;
  const geo=state.geographic;
  if(geo){
    $('geo-status').textContent=geo.map_reason+(geo.map_catalog_errors?'；部分地图信息无法读取':'');
    const signature=JSON.stringify(geo.maps.map(m=>[m.id,m.title,m.georeferenced]));
    if(signature!==geoSignature){
      const previous=$('geo-map').value;
      $('geo-map').replaceChildren(new Option('首次采集（无预建地图）',''),...geo.maps.map(m=>new Option(m.title+(m.georeferenced?' · 已关联地理位置':' · 本地地图'),m.id)));
      if(geoChoiceDirty)$('geo-map').value=previous;
      geoSignature=signature;
    }
    if(!geoChoiceDirty)$('geo-map').value=geo.selected_map_id||'';
    $('geo-select').disabled=simulated||state.guiding||state.local_navigation_active;
    $('geo-anchor').hidden=!geo.maps.some(m=>m.id===geo.selected_map_id&&!m.georeferenced);
    $('geo-anchor').disabled=simulated;
    $('geo-start').disabled=simulated||!geo.selected_map_id||state.mode!=='walking';
    $('geo-confirm').hidden=!geo.can_confirm_position||simulated;
  }
  renderSavedTrack(state);
  $('simulation-banner').hidden=!simulated;$('sim-controls').hidden=!simulated;$('sim-enter').hidden=simulated;
  $('sim-enter').disabled=state.local_navigation_active||!state.route||state.guiding||state.mode!=='walking';
  $('save-track').disabled=simulated;$('position-legend').textContent=simulated?'● 模拟位置':'● 实际位置';
  if(simulated)$('simulation-panel').open=true;
  $('start').textContent=simulated?'模拟行走':'开始引导';$('stop').textContent=simulated?'退出模拟':'结束引导';
  $('start').disabled=state.local_navigation_active||state.guiding||!state.route||!state.bd09||state.mode!=='walking';$('stop').disabled=!state.guiding&&!simulated;
  $('guide').hidden=!state.guiding;
  const g=state.guidance;
  if(state.guiding){$('remaining').textContent=g?g.near_destination?'已到目的地附近':`剩余约 ${Math.round(g.remaining_m)} 米`:'引导暂停';
    $('direction').textContent=g?g.uncertain?'路线进度未确认，请查看地图':g.off_route_m>20?`偏离路线约 ${Math.round(g.off_route_m)} 米，请查看地图`:g.instruction||`下一段朝向 ${g.bearing_deg}°（正北为 0°）`:'当前位置未确认，请停下并等待恢复';}
  if(!editing){draft=state.route;if(state.route&&!titleDirty)$('title').value=state.route.title;drawRoute();}
  if(state.save_error||state.conversion_error)say(state.save_error||state.conversion_error);
  updateHeading(state);
  if(!map)return;
  if(state.bd09){if(!marker){
    const icon=new BMap.Icon('data:image/svg+xml,'+encodeURIComponent('<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24"><circle cx="12" cy="12" r="8" fill="#126956" stroke="white" stroke-width="3"/></svg>'),new BMap.Size(24,24));
    marker=new BMap.Marker(point(state.bd09.point),{icon});map.addOverlay(marker);
  }else marker.setPosition(point(state.bd09.point));}
  else if(marker){map.removeOverlay(marker);marker=null;}
  const nextTrack=JSON.stringify(state.track_bd09||[]);
  if(nextTrack===trackSignature)return;
  trackSignature=nextTrack;
  for(const p of trackLines)map.removeOverlay(p);trackLines=[];
  let segment=[];
  const flush=()=>{if(segment.length>1){const l=new BMap.Polyline(segment.map(point),{strokeColor:'#126956',strokeWeight:4,strokeOpacity:.65});map.addOverlay(l);trackLines.push(l);}segment=[];};
  let prior=0;
  for(const sample of state.track_bd09||[]){if(sample.stamp-prior>10)flush();segment.push(sample.point);prior=sample.stamp;}flush();
}
async function poll(){
  if(polling||document.hidden)return;polling=true;
  try{render(await api('state',null,3500));}catch(e){$('connection').textContent='连接中断，正在重连';$('position-status').textContent='设备状态暂不可用';$('start').disabled=true;$('guide').hidden=false;$('remaining').textContent='引导暂停';$('direction').textContent='等待连接恢复';updateHeading(null);if(marker&&map){map.removeOverlay(marker);marker=null;}}
  finally{polling=false;}
}
let journeySignature='';
async function journeyCommand(data){
  try{await api('journey',data,30000);await poll();say('全程路线已更新');}catch(e){say(e.message);}
}
function renderJourney(j){
  if(!j)return;
  $('journey-status').textContent=j.error||j.reason;
  $('journey-transition').hidden=j.phase!=='transition';
  const signature=JSON.stringify(j.network.places);
  if(signature!==journeySignature){
    for(const id of ['journey-start','journey-end']){
      const previous=$(id).value;
      $(id).replaceChildren(new Option('请选择地点',''),...j.network.places.map(p=>new Option(p.name+(p.domain==='BD09'?' · 道路':' · 地图'),p.id)));
      $(id).value=previous;
    }
    journeySignature=signature;
  }
  document.querySelector('[data-journey="start"]').disabled=j.phase!=='preview';
  document.querySelector('[data-journey="stop"]').disabled=!['guiding','lost','transition','deviated'].includes(j.phase);
}
document.querySelectorAll('[data-journey]').forEach(b=>b.onclick=()=>journeyCommand({action:b.dataset.journey}));
$('journey-plan').onclick=()=>journeyCommand({action:'plan',start:$('journey-start').value,end:$('journey-end').value});
$('place-local').onclick=()=>journeyCommand({action:'place',name:$('place-name').value,domain:'local'});
$('place-road').onclick=()=>journeyCommand({action:'place',name:$('place-name').value,domain:'BD09',point:selectedEnd});
$('portal-link').onclick=()=>journeyCommand({action:'link',from:$('journey-start').value,to:$('journey-end').value,kind:'portal',name:$('place-name').value||'地图入口',passage_confirmed:$('portal-confirm').checked});
$('road-link').onclick=()=>journeyCommand({action:'link',from:$('journey-start').value,to:$('journey-end').value,kind:'road'});
window.startOutdoorMap=()=>{
  try{
    map=new BMap.Map('map',{center:new BMap.Point(116.310,39.992),zoom:16});
    map.addEventListener('dragstart',()=>{followPosition=false;});
    map.addEventListener('click',e=>{const p=e.latlng||e.point;if(p)acceptClick([p.lng,p.lat]);});
    $('map-message').textContent='拖动浏览地图，轻点选择终点';
    if(lastState?.route){drawRoute();map.setViewport(lastState.route.points.map(point));}
    else $('campus').click();
    poll();
  }catch(e){$('map-message').textContent='地图加载失败，请检查网络或应用授权';}
};
async function boot(){
  try{
    const config=await api('config');await poll();
    if(!config.browser_key){$('map-message').textContent='地图服务未配置，请联系设备管理员';return;}
    try{await NAVBaidu.load(config.browser_key);window.startOutdoorMap();}
    catch(error){$('map-message').textContent=error.message;}
  }catch(e){say('无法连接设备，请返回设备屏幕重新连接');}
}
document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});window.addEventListener('online',poll);setInterval(poll,500);boot();
