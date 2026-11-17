/* Shared live view. Alignment, route validity and guidance belong to Python. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  let state = null, bridge = null, map = null, lastDraw = 0, centered = null;
  let receivedAt=0,following=true,userMoved=false,wasGuiding=false,centeredFresh=false;
  let toastTimer, connected = false, assistPending = false, queue = Promise.resolve();
  let manualDraft = null;
  let selecting = 0;
  let mapPress=null,ignoreMapClickUntil=0,redrawPending=false;
  const mapPointers=new Set();
  let journey=null,journeyReadAt=0,confirmedSession=null,observedTrial=null,journeyPending=null;
  const trialId=s=>String(s?.trial?.trial_id || '');
  function rememberManual(){try{if(manualDraft)sessionStorage.setItem('nav-walking-manual',JSON.stringify(manualDraft));else sessionStorage.removeItem('nav-walking-manual');}catch(e){}}
  function clearConfirmations(){confirmedSession=null;for(const id of ['confirm','confirm-restart','confirm-discard'])$(id).hidden=true;}
  let mapStyle = 'street', mapFailure = '';
  try { if(window.localStorage.getItem('nav-walking-map-style')==='satellite')mapStyle='satellite'; } catch(e) {}
  const ink = {fixed:'#38dbb3', float:'#be91ff', fusion:'#75c9ff', lidar:'#f6b85a', route:'#518dff'};
  const gnssColor=row=>row.source==='phone_continuity'?ink.fusion:row.quality===4?ink.fixed:ink.float;
  function toast(message) { $('toast').textContent=message; $('toast').hidden=false;
    clearTimeout(toastTimer); toastTimer=setTimeout(()=>{$('toast').hidden=true;},5000); }
  function sheet(open) { $('sheet').hidden=!open; $('shade').hidden=!open;
    if(open)$('close').focus(); else $('details').focus(); }
  async function request(path, data) {
    const controller=new AbortController(), timer=setTimeout(()=>controller.abort(),5000);
    try {
      const response=await fetch('/navigation/'+path,{signal:controller.signal,cache:'no-store',
        ...(data?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)}:{})});
      const result=await response.json();
      if(response.status===401){location.href='/?next=%2Fnavigation';throw Error('请先输入设备上的配对码');}
      if(!response.ok)throw Error(result.message||'操作未完成，请重试');
      return result;
    } finally {clearTimeout(timer);}
  }
  const qtCall=(name,arg)=>new Promise((resolve,reject)=>{
    const callback=raw=>{try{const value=JSON.parse(raw);if(value.error)reject(Error(value.error));else resolve(value);}catch(e){reject(e);}};
    if(arg===undefined)bridge[name](callback);else bridge[name](JSON.stringify(arg),callback);
  });
  function act(action, values={}) {
    const requestedTrial=trialId(state);
    queue=queue.then(()=>sendAction(action,values,requestedTrial));
    return queue;
  }
  async function sendAction(action, values={},requestedTrial=trialId(state)) {
    if(!connected){toast('设备连接中，请稍候');return;}
    if(requestedTrial!==trialId(state)){toast('行程已变化，请重新选择操作');return false;}
    if(!bridge){
      if(action==='begin' && state?.runtime?.capture_active)return journeyAction('join_navigation');
      const destinations={begin:'/?view=prepare',camera:'/?view=camera',satellites:'/?tab=device#satellite-card',back:'/'};
      if(destinations[action]){location.href=destinations[action];return true;}
      if(['save','restart','discard'].includes(action))return journeyAction(action);
    }
    try {const payload={action,trial_id:trialId(state),...values};
      await (bridge?qtCall('command',payload):request('experience/control',payload));
      if(['begin','guide','stop','edit','replan','save','restart','discard','camera','satellites'].includes(action))sheet(false);
      await poll();
      if(['select','undo','delete','loop','edit','replan'].includes(action))refreshMap();
      return true;
    } catch(e){toast(e.message||'操作未完成，请重试');return false;}
  }
  async function journeyRead(force=false){
    if(bridge || (!force&&Date.now()-journeyReadAt<1000))return;
    const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),5000);
    try{const response=await fetch('/journey/state',{cache:'no-store',credentials:'same-origin',signal:abort.signal});if(response.status===401){location.href='/?next=%2Fnavigation';return;}if(!response.ok)throw Error('行程状态暂不可用');const value=await response.json();journey=value;journeyReadAt=Date.now();
      if(journeyPending){const receipt=(value.commands||[]).find(row=>row.id===journeyPending.id);if(receipt){toast(receipt.message || ({queued:'设备已收到',running:'正在执行',succeeded:'操作已完成',failed:'操作未完成'})[receipt.status]);if(['succeeded','failed'].includes(receipt.status))journeyPending=null;}}
    }finally{clearTimeout(timer);}
  }
  async function journeyAction(action){
    if(!journey?.session_id || journey.available===false || journey.age_s>5 || Date.now()-journeyReadAt>5000){toast('正在读取新鲜的行程状态，请稍后重试');return false;}
    if(journeyPending){toast('前一个操作尚待确认，请在行程页查看进度');return false;}
    const bytes=new Uint8Array(16);crypto.getRandomValues(bytes);const id=[...bytes].map(v=>v.toString(16).padStart(2,'0')).join(''),payload={id,session_id:journey.session_id,action,confirmed:true,protocol_version:2};journeyPending=payload;
    const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),7000);
    try{const response=await fetch('/journey/command',{method:'POST',credentials:'same-origin',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload),signal:abort.signal});const result=await response.json();if(!response.ok){if(response.status>=400&&response.status<500)journeyPending=null;throw Error(result.message || '操作未完成');}toast('设备已收到，执行进度将在行程页显示');sheet(false);await journeyRead(true);await poll();return true;}
    catch(e){toast(e.name==='AbortError'?'操作是否送达尚未确认，请在行程页核对状态':e.message);return false;}finally{clearTimeout(timer);}
  }
  function openConfirmation(id){confirmedSession={trial:trialId(state),session:journey?.session_id};$(id).hidden=false;}
  function confirmAction(action){
    const previous=confirmedSession;clearConfirmations();
    if(!previous || previous.trial!==trialId(state) || (!bridge&&previous.session!==journey?.session_id)){toast('当前行程已变化，请重新确认');return;}
    return act(action);
  }
  function label(id,value) {$(id).textContent=value;}
  const number=(v,d=2)=>Number.isFinite(v)?v.toFixed(d):'—';
  function renderAssist(s) {
    const a=s.route_assist, enabled=!!(a&&a.enabled);
    const toggle=$('route-assist'), cue=$('assist-cue');
    toggle.setAttribute('aria-checked',String(enabled));
    toggle.disabled=!a||assistPending;
    label('assist-toggle-state',assistPending?'…':enabled?'开':'关');
    label('assist-status',!a?'路口辅助暂不可用':!enabled?'已关闭':a.message||'已开启，等待路线和运动跟踪');
    const runtime=s.runtime||{};
    cue.hidden=!(enabled&&a.available&&s.guiding&&a.message&&!runtime.saving&&!runtime.preparing&&!runtime.error);
    cue.textContent=cue.hidden?'':'路口辅助 · '+a.message;
    cue.classList.toggle('warning',!cue.hidden&&['missed_turn','wrong_turn','early_turn','off_route','junction_uncertain'].includes(a.state));
  }
  function render(s,received=false) {
    const id=trialId(s);
    if(observedTrial!==null&&observedTrial!==id){manualDraft=null;centered=null;userMoved=false;following=true;centeredFresh=false;rememberManual();clearConfirmations();toast('当前行程已变化，已关闭上一段的待确认操作');}
    if(observedTrial===null){try{const draft=JSON.parse(sessionStorage.getItem('nav-walking-manual')||'null');if(draft&&draft.trial_id===id)manualDraft=draft;}catch(e){}}
    observedTrial=id;
    state=s;
    if(received){receivedAt=Date.now();connected=true;}
    document.body.classList.toggle('manual-view',!!manualDraft);
    $('map').parentElement.classList.toggle('manual-positioning',!!manualDraft);
    if(manualDraft){label('manual-hint',manualDraft.point?'将十字移到设备正前方5米以外的位置':'停稳设备，拖动或点选地图，将十字对准当前位置');label('manual-next',manualDraft.point?'确认朝向':'确认位置');}
    if(s.guiding&&!wasGuiding){following=true;userMoved=false;centered=null;}
    wasGuiding=!!s.guiding;
    const t=s.trial,v=s.visual,g=s.guidance,a=t.alignment||t.preview||{};
    const runtime=s.runtime||{}, stage=s.phase==='idle'?'align':s.phase;
    const routeWarnings=s.route?.coverage?.warnings||[];
    const routeWarning=routeWarnings.find(w=>w.role==='destination')||routeWarnings[0];
    $('route-warning').hidden=!routeWarning||!!runtime.saving||!!runtime.preparing||!!runtime.restarting;
    label('route-warning',routeWarning?.message||'');
    ['align','plan','navigate'].forEach(k=>$('step-'+k).classList.toggle('active',k===stage));
    let headline=t.phase, subtitle=t.reason;
    if(runtime.restarting){headline='正在恢复运动跟踪';subtitle='正在保存本段并准备新的路段，请保持设备稳定';}
    else if(runtime.saving){headline='正在保存地图';subtitle='正在整理地图与本次选择的数据，请保持通电';}
    else if(runtime.preparing){headline='正在准备设备';subtitle='请保持设备稳定，等待运动跟踪启动';}
    else if(runtime.error){headline='采集需要处理';subtitle=runtime.error;}
    else if(!t.active){headline='准备开始';subtitle=s.message||'开始行程后，可用卫星对齐或手动确认位置与方向';}
    else if(s.locating){headline='正在确认手动位置';subtitle='请保持设备稳定，正在关联地图坐标';}
    else if(s.phase==='navigate'){
      headline=g?(g.instruction||'沿步行路线前行'):'定位暂缺，请停下确认';
      subtitle=g?'剩余 '+number(g.remaining_m,0)+' 米 · 离路线 '+number(g.off_route_m,1)+' 米':'等待定位恢复，采集继续';
      if(g&&routeWarnings.length&&Number.isFinite(g.selected_destination_distance_m))
        subtitle='路线剩余 '+number(g.remaining_m,0)+' 米 · 距所选终点 '+number(g.selected_destination_distance_m,g.selected_destination_distance_m<10?1:0)+' 米';
    } else if(s.phase==='plan'){
      headline=s.busy?'正在规划步行路线':s.route?'路线已准备好':'点选途经点';
      subtitle=!t.odometry_live?'运动跟踪暂未更新，可继续选点；恢复后开始引导':s.message||'轻点一次加一点，最后一个点为终点';
    } else if(t.aligned){subtitle=t.alignment_method==='manual'?'位置与方向已手动确认 · 运动跟踪接续':a.geographic_accuracy==='approximate'?'手机近似定位 · 位置不确定度约 '+number(a.absolute_horizontal_sigma_m,1)+' 米':'拟合差 '+number(a.residual_m)+' 米 · '+(t.rtk_enabled?'卫星与雷达联合定位':'雷达独立跟踪');}
      else if(!t.aligned){
        subtitle=t.reason||'正在等待可用于对齐的位置与运动观测';
        // An absent observation cannot improve by asking the user to keep walking.
        // Keep specific quality/timing rejection reasons; replace only the initial hint.
        if(!t.satellite_live&&!v.gnss_fresh&&(!t.reason||t.reason==='启动后缓慢走一段，自动对准卫星与雷达轨迹'))
          subtitle='暂无可用卫星位置，可点“手动定位”确认位置与方向；建图继续';
      }
      label('headline',headline);label('subtitle',subtitle);renderAssist(s);
      $('position-uncertainty').hidden=a.geographic_accuracy!=='approximate';
      label('position-uncertainty',(a.absolute_anchor_source==='system_gps_window'?'普通定位约束':a.absolute_anchor_source==='raw_window'?'卫星与雷达联合':'近似定位')+' · 约 '+number(a.absolute_horizontal_sigma_m,1)+' 米不确定度');
    $('progress').hidden=stage!=='align'||!t.active;
      const progress=t.aligned?100:t.phone_heading_edges?(t.phone_heading_pending?60+Math.min(30,(t.phone_heading_check_edges||0)*5):Math.min(55,t.phone_heading_edges*5)):Math.min(85,t.stable_pairs/15*60)+Math.min(14,t.verification_count/3*14);
    $('progress').firstElementChild.style.width=progress+'%';
    label('detail-status',[s.conversion_error,s.message,t.reason,...routeWarnings.map(w=>w.message)].filter((value,index,all)=>value&&all.indexOf(value)===index).join(' · '));
    $('quality').value=t.quality_policy;$('quality').disabled=s.guiding;
    const motion=s.phone_motion||{};
    $('phone-motion').value=motion.enabled?'motion':'native';$('phone-motion').disabled=s.guiding;
    label('phone-motion-status',(motion.absolute_window&&motion.absolute_window.observations?motion.absolute_window.reason:motion.reason||'等待手机融合定位')+(motion.fresh&&Number.isFinite(motion.horizontal_sigma_m)?' · 手机位置不确定度约 '+number(motion.horizontal_sigma_m,1)+' 米':''));
    $('fusion-legend').hidden=!(v.gnss||[]).some(r=>r.source==='phone_continuity');
    $('manual-place').disabled=!t.active||s.guiding||s.locating||runtime.preparing||runtime.saving||runtime.restarting;
    $('manual-place').hidden=!!manualDraft||!!s.guiding;$('summary').hidden=!!manualDraft;
    label('manual-place',t.alignment_method==='manual'?'重新确认位置':'手动定位');
    $('manual-panel').hidden=!manualDraft;$('manual-crosshair').hidden=!manualDraft;
    $('manual').value=s.manual?'manual':'auto';$('manual').disabled=s.guiding;
    label('rtk','RTK参与定位：'+(t.rtk_enabled?'开':'关'));$('rtk').disabled=!t.active;
    $('route-tools').hidden=stage!=='plan';$('replan').disabled=!s.can_plan;
    $('save').disabled=!(runtime.can_save===undefined?t.active&&!runtime.saving:runtime.can_save)||!!runtime.restarting;
    label('save',runtime.restarting?'恢复中…':runtime.saving?'保存中…':'保存结束');
    $('restart-tracking').hidden=!runtime.can_restart||!t.active||!!t.odometry_live||!!manualDraft;
    $('restart-tracking').disabled=!!runtime.saving||!!runtime.restarting;
    $('discard').disabled=!runtime.can_discard;
    $('plan').disabled=s.busy||s.waypoints.length<2||!s.can_plan;
    label('plan',s.busy?'正在规划…':s.manual?'生成手绘路线':'规划步行路线');
    renderRouteEditor();
    $('metrics').replaceChildren();
      const heading=!!t.phone_heading_edges;
      const entries=[heading?['运动观测',t.phone_heading_edges]:['有效配对',t.pairs],heading?['后续行走复核',t.aligned?'已通过':(t.phone_heading_check_edges||0)+' / 6']:['稳定关键点',t.stable_pairs],
        heading?['方向差',number(a.validation_yaw_error_rad*180/Math.PI,1)+'°']:['方向复核',Math.min(t.verification_count,3)+' / 3'],
        heading?['位置不确定度',number(a.absolute_horizontal_sigma_m)+' 米']:['轨迹拟合差',number(a.residual_m)+' 米'],['当前对准角',number(a.yaw*180/Math.PI,1)+'°'],
      ['卫星轨迹',v.gnss_fresh?'正在更新':'暂无新位置'],['雷达轨迹',v.lidar_fresh?'正在更新':'等待更新']];
    entries.forEach(([name,value])=>{const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=name;dd.textContent=value;$('metrics').append(dt,dd);});
    label('speeds',['gnss','lidar'].map(k=>{const speed=v['velocity_'+k];if(!speed)return '';
      const axes=k==='gnss'||v.velocity_frame==='east_north'?['东','北']:['X','Y'];
      return (k==='gnss'?'卫星':'雷达')+' '+number(speed.speed_m_s)+' 米/秒 · '+axes[0]+number(speed.components[0])+' / '+axes[1]+number(speed.components[1]);}).filter(Boolean).join('；')+'（位置差分）');
    const listKey=JSON.stringify(s.waypoints);
    if($('waypoints').dataset.key!==listKey){$('waypoints').dataset.key=listKey;$('waypoints').replaceChildren();
      s.waypoints.forEach((p,i)=>{const li=document.createElement('li');li.textContent=i?'途经点 '+i:'当前位置起点';
        if(i){const button=document.createElement('button');button.textContent='删除';button.onclick=()=>act('delete',{index:i});li.append(button);}$('waypoints').append(li);});}
    let primary,secondary;
    if(!t.active){primary=[runtime.capture_active?'添加地理引导':'开始行程','begin',!runtime.preparing];secondary=['相机 / 点云','camera',true];}
    else if(stage==='align'){primary=['规划路线','edit',s.can_plan];secondary=['相机 / 点云','camera',true];}
    else if(stage==='plan'){primary=s.route?['开始引导','guide',s.can_guide]:[s.busy?'规划中…':'规划步行路线','plan',s.can_plan&&!s.busy&&s.waypoints.length>=2];secondary=['撤销一点','undo',s.waypoints.length>1];}
    else{primary=[g&&g.near_destination?'完成引导':'结束引导','stop',true];secondary=[t.rtk_enabled?'关闭RTK':'开启RTK','rtk',true];}
    secondary=['相机 / 点云','camera',true];
    for(const [id,item] of [['primary',primary],['secondary',secondary]]){label(id,item[0]);$(id).dataset.action=item[1];$(id).disabled=!item[2]||(id==='primary'&&(!!runtime.saving||!!runtime.restarting||!!manualDraft||!!s.locating));}
    if(!connected)renderDisconnected();
    if(Date.now()-lastDraw>=1000){lastDraw=Date.now();refreshMap();}
  }
  function renderDisconnected(){label('headline','设备连接已中断');label('subtitle','正在重新连接，设备上的采集不会因此停止');
    $('phone-motion').disabled=true;
    $('primary').disabled=true;$('secondary').disabled=true;$('route-assist').disabled=true;
    $('assist-cue').hidden=true;label('assist-status','设备连接中断，正在重新读取辅助状态');
  }
  async function poll(){try{render(await (bridge?qtCall('read'):request('experience')),true);if(!bridge)try{await journeyRead();}catch(e){/* Map browsing remains available while journey state reconnects. */}}catch(e){
    connected=false;renderDisconnected();
    refreshMap();
  }}
  const point=p=>new BMap.Point(p[0],p[1]);
  const validPoint=p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite)&&Math.abs(p[0])<=180&&Math.abs(p[1])<=90;
  function mapLocation(){
    if(!state)return null;
    const elapsed=Math.max(0,(Date.now()-receivedAt)/1000),p=state.position,r=state.map_reference,v=state.visual;
    const mapAge=elapsed+(Number.isFinite(state.map_position_age_s)?state.map_position_age_s:0);
    if(connected&&mapAge<=1.5&&state.bd09&&p&&validPoint(state.bd09.point))
      return {point:state.bd09.point,fresh:true,label:p.source==='gnss'?(p.quality===4?'卫星固定解':p.quality===5?'卫星浮点解':'卫星粗定位'):'我在这里',source:p.source};
    if(r&&validPoint(r.point)){
      const age=Math.max(0,Number(r.age_s)||0)+elapsed;
      const fresh=connected&&r.fresh===true&&age<=1.5;
      return {point:r.point,fresh,age_s:age,label:fresh?(r.quality===4?'卫星固定解':r.quality===5?'卫星浮点解':'卫星粗定位'):'历史卫星位置',source:'gnss'};
    }
    if(connected&&elapsed<=1.5&&v.gnss_fresh&&v.gnss_map.length){const last=v.gnss_map[v.gnss_map.length-1];
      return {point:last.point,fresh:true,label:last.quality===4?'卫星固定解':'卫星浮点解',source:'gnss'};}
    return null;
  }
  function locationNote(location){if(!location)return '暂无卫星参考位置，请在地图上查找并确认当前位置';
    if(!location.fresh){const age=location.age_s||0;return '历史卫星位置 · '+(age>=3600?Math.floor(age/3600)+'小时前':age>=60?Math.floor(age/60)+'分钟前':Math.ceil(age)+'秒前')+'，仅供查找，不是当前位置';}
    return location.label==='卫星粗定位'?'卫星粗定位仅供参考，请按实际位置微调':location.label;
  }
  function pauseFollowing(){following=false;userMoved=true;if(manualDraft){manualDraft.touched=true;rememberManual();}renderCenter();}
  function renderCenter(){
    label('center',manualDraft?.point?'已选位置':following&&state?.guiding?'跟随中':'我的位置');
    $('center').setAttribute('aria-label',manualDraft?.point?'回到已选择的位置':'回到我的位置');
    $('center').setAttribute('aria-pressed',String(!!following&&!!state?.guiding&&!manualDraft));
  }
  function focusLocation(p,zoom){if(!map||!validPoint(p))return;
    if(zoom!==undefined&&map.getZoom()!==zoom)map.centerAndZoom(point(p),zoom);else map.panTo(point(p),{noAnimation:true});
    // Keep the current-position symbol inside the visible map, below the
    // summary and away from the control column. Manual crosshair stays central.
    if(!manualDraft&&map.panBy){const top=$('summary').hidden?12:$('summary').offsetHeight+24;
      if(Number.isFinite(top))map.panBy(-32,Math.max(0,(top-36)/2),{noAnimation:true});}
  }
  function recenter(){if(!map)return drawMetric();
    if(manualDraft?.point){focusLocation(manualDraft.point);return;}
    const location=mapLocation();if(!location){toast('暂无卫星参考位置，请拖动地图查找当前位置');return;}
    userMoved=false;following=true;if(manualDraft)manualDraft.touched=false;
    focusLocation(location.point,manualDraft?20:19);centered=trialId(state);centeredFresh=location.fresh;
    if(!location.fresh||location.label==='卫星粗定位')toast(locationNote(location));renderCenter();
  }
  function renderMapStyle() {
    for(const [id,type] of [['map-street','street'],['map-satellite','satellite']]) {
      $(id).disabled=!map;$(id).setAttribute('aria-pressed',String(mapStyle===type));
    }
  }
  function setMapStyle(type, remember=true) {
    if(!map)return false;
    try {
      map.setMapType(type==='satellite'?BMAP_HYBRID_MAP:BMAP_NORMAL_MAP);
    } catch(e) {
      console.warn('Walking map layer change failed',e);
      mapFailure='底图切换未成功，请稍后重试';
      renderMapStyle();label('map-note',mapFailure);toast(mapFailure);return false;
    }
    // Changing the layer preserves the SDK viewport. Do not reinitialize it
    // during the asynchronous tile change or reinterpret an overlay error as
    // a failed layer switch / device disconnection.
    mapStyle=type;mapFailure='';
    if(remember)try{window.localStorage.setItem('nav-walking-map-style',type);}catch(e){}
    renderMapStyle();refreshMap();return true;
  }
  function split(rows,byQuality=false){const groups=[];let prev=null;
    rows.forEach(row=>{if(!prev||row.stamp-prev.stamp>2||(byQuality&&(row.quality!==prev.quality||row.source!==prev.source)))groups.push([]);
      groups[groups.length-1].push(row);prev=row;});return groups;}
  function fit(){if(!map||!state)return;const rows=[...state.visual.gnss_map,...state.visual.lidar_map].map(r=>r.point);
    rows.push(...state.waypoints,...(state.route?state.route.points:[]));if(!rows.length)return;
    // Fit first, then offset into the uncovered area of the small screen.
    const viewport=map.getViewport(rows.map(point));
    const height=$('map').clientHeight, top=$('summary').offsetHeight+20, bottom=68;
    const available=Math.max(40,height-top-bottom);
    const pixels=rows.map(p=>map.pointToPixel(point(p))), xs=pixels.map(p=>p.x), ys=pixels.map(p=>p.y);
    const factor=Math.min(($('map').clientWidth-120)/Math.max(1,Math.max(...xs)-Math.min(...xs)),
      available/Math.max(1,Math.max(...ys)-Math.min(...ys)));
    const zoom=Math.floor(Math.min(20,Math.max(3,map.getZoom()+Math.log2(factor)-.25)));
    const centerPixel=map.pointToPixel(viewport.center), ratio=Math.pow(2,zoom-map.getZoom());
    const center=map.pixelToPoint(new BMap.Pixel(centerPixel.x+30/ratio,centerPixel.y-(top-bottom)/2/ratio));
    map.setViewport({center,zoom},{enableAnimation:false});}
  function draw(){if(!state)return;const v=state.visual;
    if(map){$('metric').hidden=true;map.clearOverlays();
      const line=(points,color,dashed=false,width=4)=>{if(points.length>1)map.addOverlay(new BMap.Polyline(points.map(point),{strokeColor:color,strokeWeight:width,strokeOpacity:.9,strokeStyle:dashed?'dashed':'solid'}));};
      if(state.route)line(state.route.points,ink.route,false,7);
      for(const rows of split(v.gnss_map,true))line(rows.map(r=>r.point),gnssColor(rows[0]));
      for(const rows of split(v.lidar_map))line(rows.map(r=>r.point),ink.lidar,!v.aligned,3);
      state.waypoints.forEach((p,i)=>{const marker=new BMap.Marker(point(p));marker.setLabel(new BMap.Label(String(i+1),{offset:new BMap.Size(12,-8)}));map.addOverlay(marker);});
      const location=mapLocation();
      if(location){const marker=new BMap.Label('<span class="user-marker'+(location.fresh?'':' historical')+'"><i></i><b>'+location.label+'</b></span>',{position:point(location.point),offset:new BMap.Size(-12,-12)});
        marker.setStyle({border:'0',background:'transparent',overflow:'visible'});map.addOverlay(marker);}
      if(manualDraft&&manualDraft.point)map.addOverlay(new BMap.Circle(point(manualDraft.point),1,{strokeColor:ink.lidar,fillColor:ink.lidar,fillOpacity:1}));
      if(location&&!userMoved&&!(manualDraft?.touched)&&!manualDraft?.point){
        const initial=centered!==trialId(state),upgrade=location.fresh&&!centeredFresh;
        if(initial||upgrade||(state.guiding&&following&&location.fresh)){
          focusLocation(location.point,initial?manualDraft?20:19:undefined);centered=trialId(state);centeredFresh=location.fresh;}}
      renderCenter();
      label('map-note',mapFailure||(manualDraft?locationNote(location):location&&!location.fresh?locationNote(location):location?.label==='卫星粗定位'?locationNote(location):v.unoriented?'雷达方向待确认':v.preview?'虚线为对齐预览':!v.aligned&&!v.gnss_map.length?'请手动确认位置，或等待合格卫星位置':''));
    }else{$('metric').hidden=false;drawMetric();}
  }
  function refreshMap(){
    // Removing SDK overlays under a held finger can change its event target.
    if(mapPointers.size){redrawPending=true;return true;}
    try{draw();return true;}
    catch(e){
      console.warn('Walking map overlay update failed',e);
      label('map-note','轨迹画面暂未更新，正在重试');
      return false;
    }
  }
  // Phone taps tolerate ordinary finger jitter. A drag, pinch, cancellation or
  // long press never adds a waypoint; the SDK still owns all map navigation.
  $('map').addEventListener('pointerdown',event=>{
    if(event.pointerType!=='touch'&&event.pointerType!=='pen')return;
    if(!map||manualDraft||state?.phase!=='plan'||state.guiding)return;
    mapPointers.add(event.pointerId);ignoreMapClickUntil=Date.now()+1000;
    if(mapPointers.size!==1){mapPress=null;return;}
    const box=$('map').getBoundingClientRect();
    const p=map.pixelToPoint(new BMap.Pixel(event.clientX-box.left,event.clientY-box.top));
    mapPress={id:event.pointerId,x:event.clientX,y:event.clientY,at:Date.now(),
      trial:trialId(state),point:[p.lng,p.lat],moved:0};
  },true);
  document.addEventListener('pointermove',event=>{
    if(mapPress&&event.pointerId===mapPress.id)mapPress.moved=Math.max(mapPress.moved,
      Math.hypot(event.clientX-mapPress.x,event.clientY-mapPress.y));
  },true);
  function endMapPress(event){
    if(!mapPointers.has(event.pointerId))return;
    const pressed=mapPress;mapPointers.delete(event.pointerId);mapPress=null;
    ignoreMapClickUntil=Date.now()+1000;
    if(event.type==='pointerup'&&pressed&&pressed.id===event.pointerId&&!mapPointers.size
       &&Math.max(pressed.moved,Math.hypot(event.clientX-pressed.x,event.clientY-pressed.y))<=10
       &&Date.now()-pressed.at<=600&&pressed.trial===trialId(state)&&!manualDraft){
      selectRoutePoint(pressed.point);
    }
    if(!mapPointers.size&&redrawPending){redrawPending=false;setTimeout(refreshMap,0);}
  }
  document.addEventListener('pointerup',endMapPress,true);
  document.addEventListener('pointercancel',endMapPress,true);
  document.addEventListener('visibilitychange',()=>{if(document.hidden){mapPress=null;mapPointers.clear();ignoreMapClickUntil=Date.now()+1000;}});
  function renderRouteEditor(){
    const editing=!!state&&state.phase==='plan'&&!state.guiding&&!manualDraft;
    document.body.classList.toggle('route-editing',editing);
    $('route-editor').hidden=!editing;$('route-crosshair').hidden=!editing||!map;
    label('route-count',selecting?'正在添加…':'已选 '+Math.max(0,(state?.waypoints.length||0)-1)+' 个途经点');
    $('route-add-center').disabled=!editing||!map||!connected||!!selecting||state.waypoints.length>=20;
    $('route-undo').disabled=!editing||!connected||!!selecting||state.waypoints.length<2;
  }
  async function selectRoutePoint(p){
    if(!connected){toast('设备连接中，请稍候');return;}
    if(!state||state.phase!=='plan'||state.guiding){
      toast(state?.guiding?'正在引导，请先从菜单选择“从当前位置重规划”':state?.can_plan?'请先点击“规划路线”，再选择途经点':'请先完成位置与方向对齐，再选择途经点');return;
    }
    const trial=trialId(state);selecting++;renderRouteEditor();
    try{if(await act('select',{point:p})){
      if(trial===trialId(state))toast('已添加第 '+Math.max(1,state.waypoints.length-1)+' 个途经点，可继续选点');
    }}finally{selecting--;renderRouteEditor();}
  }
  function drawMetric(){const canvas=$('metric'),r=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
    canvas.width=r.width*dpr;canvas.height=r.height*dpr;const c=canvas.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,r.width,r.height);
    if(!state)return;const v=state.visual,gnss=v.separate_axes?[]:v.gnss,rows=[...v.lidar,...gnss];if(!rows.length)return;
    const xs=rows.map(p=>p.point[0]),ys=rows.map(p=>p.point[1]),top=$('summary').offsetHeight+40,bottom=85;
    const minX=Math.min(...xs)-2,maxX=Math.max(...xs)+2,minY=Math.min(...ys)-2,maxY=Math.max(...ys)+2;
    const scale=Math.max(.01,Math.min((r.width-130)/(maxX-minX),Math.max(30,r.height-top-bottom)/(maxY-minY)));
    const project=p=>[(p[0]-(minX+maxX)/2)*scale+(r.width-65)/2,top+(Math.max(30,r.height-top-bottom))/2-(p[1]-(minY+maxY)/2)*scale];
    function path(rows,color,dashed){if(!rows.length)return;c.beginPath();c.strokeStyle=color;c.lineWidth=3;c.setLineDash(dashed?[6,5]:[]);
      rows.forEach((row,i)=>{const p=project(row.point);if(i)c.lineTo(...p);else c.moveTo(...p);});c.stroke();}
    split(v.lidar).forEach(g=>path(g,ink.lidar,!v.aligned));split(gnss,true).forEach(g=>path(g,gnssColor(g[0]),false));
    label('map-note',mapFailure||(v.separate_axes?'雷达局部轨迹 · 等待合格卫星位置':'底图未加载 · 东向右，北向上'));
  }
  function loadMap(key){if(!key){mapFailure='地图未配置，仍可查看轨迹';label('map-note',mapFailure);renderMapStyle();return;}
    window.walkingMapReady=()=>{try{map=new BMap.Map('map',{enableMapClick:false});
      const location=mapLocation(),chosen=validPoint(manualDraft?.point)?manualDraft.point:null;
      map.centerAndZoom(point(chosen||(location?location.point:[104.,35.])),chosen?20:location?19:5);
      map.enableScrollWheelZoom(true);map.disableDoubleClickZoom();map.disableAutoResize();
      map.addEventListener('dragstart',pauseFollowing);
      map.addEventListener('click',e=>{if(Date.now()<ignoreMapClickUntil||mapPointers.size||!e.point)return;if(manualDraft){pauseFollowing();map.panTo(e.point,{noAnimation:true});return;}selectRoutePoint([e.point.lng,e.point.lat]);});
      $('metric').hidden=true;setMapStyle(mapStyle,false);renderMapStyle();refreshMap();renderRouteEditor();}catch(e){map=null;mapFailure='百度底图未能打开，请检查地图服务后重试';renderMapStyle();label('map-note',mapFailure);refreshMap();}};
    // This view is entirely 2D. JSAPI 4.0 (20261021193447) changes the layer,
    // then throws on its missing WebGL feature manager in Qt software rendering.
    // The 3.0 raster API supports these same public map/overlay operations.
    const script=document.createElement('script');script.src='https://api.map.baidu.com/api?v=3.0&s=1&ak='+encodeURIComponent(key)+'&callback=walkingMapReady';script.onerror=()=>{mapFailure='百度底图加载失败，请检查网络；轨迹仍在记录';renderMapStyle();label('map-note',mapFailure);};document.head.append(script);
  }
  $('details').onclick=()=>sheet(true);$('close').onclick=()=>sheet(false);$('shade').onclick=()=>sheet(false);
  function manualView(){
    document.body.classList.toggle('manual-view',!!manualDraft);
    $('map').parentElement.classList.toggle('manual-positioning',!!manualDraft);
    $('manual-panel').hidden=!manualDraft;$('manual-crosshair').hidden=!manualDraft;
    if(manualDraft){
      label('manual-hint',manualDraft.point?'将十字移到设备正前方5米以外的位置':'停稳设备，拖动或点选地图，将十字对准当前位置');
      label('manual-next',manualDraft.point?'确认朝向':'确认位置');
    }
    if(state)render(state);refreshMap();
  }
  $('manual-place').onclick=()=>{
    if(!map){toast('底图尚未加载，请检查地图连接后重试');return;}
    sheet(false);manualDraft={point:null,trial_id:trialId(state),touched:false};userMoved=false;rememberManual();manualView();recenter();
  };
  $('manual-cancel').onclick=()=>{manualDraft=null;rememberManual();manualView();};
  $('route-add-center').onclick=()=>{if(map){const p=map.getCenter();selectRoutePoint([p.lng,p.lat]);}};
  $('route-undo').onclick=()=>act('undo');
  $('map-street').onclick=()=>setMapStyle('street');$('map-satellite').onclick=()=>setMapStyle('satellite');
  for(const [id,x,y] of [['manual-north',0,72],['manual-south',0,-72],['manual-west',72,0],['manual-east',-72,0]]){
    $(id).onclick=()=>{if(map&&manualDraft){pauseFollowing();map.panBy(x,y,{noAnimation:true});}};
  }
  $('manual-next').onclick=async()=>{
    if(!manualDraft||!map)return;
    if(manualDraft.trial_id!==trialId(state)){manualDraft=null;rememberManual();manualView();toast('行程已变化，请重新确认位置');return;}
    const center=map.getCenter(), chosen=[center.lng,center.lat];
    if(!manualDraft.point){manualDraft.point=chosen;rememberManual();manualView();return;}
    $('manual-next').disabled=true;
    const ok=await act('manual_anchor',{point:manualDraft.point,direction:chosen});
    $('manual-next').disabled=false;
    if(ok){manualDraft=null;rememberManual();}manualView();
  };
  $('primary').onclick=()=>act($('primary').dataset.action);$('secondary').onclick=()=>{const action=$('secondary').dataset.action;act(action,action==='rtk'?{value:!state.trial.rtk_enabled}:{});};
  $('quality').onchange=()=>act('quality',{value:$('quality').value});$('manual').onchange=()=>act('manual',{value:$('manual').value==='manual'});
  $('phone-motion').onchange=()=>act('phone_motion',{value:$('phone-motion').value==='motion'});
  $('rtk').onclick=()=>act('rtk',{value:!state.trial.rtk_enabled});$('replan').onclick=()=>act('replan');$('plan').onclick=()=>act('plan');
  $('route-assist').onclick=async()=>{
    if(!connected||assistPending||!state||!state.route_assist)return;
    assistPending=true;renderAssist(state);
    try{await act('route_assist',{value:!state.route_assist.enabled});}
    finally{assistPending=false;if(connected)renderAssist(state);}
  };
  document.querySelectorAll('[data-action]').forEach(b=>{if(b.id!=='primary'&&b.id!=='secondary')b.onclick=()=>act(b.dataset.action);});
  $('save').onclick=()=>{openConfirmation('confirm');$('cancel-save').focus();};$('cancel-save').onclick=()=>{clearConfirmations();$('save').focus();};
  $('confirm-save').onclick=()=>confirmAction('save');
  $('back').onclick=$('home').onclick=()=>{if(bridge)act('back');else location.href='/';};
  $('restart-tracking').onclick=()=>{openConfirmation('confirm-restart');$('cancel-restart').focus();};
  $('cancel-restart').onclick=()=>{clearConfirmations();$('restart-tracking').focus();};
  $('confirm-restart-action').onclick=()=>confirmAction('restart');
  $('discard').onclick=()=>{openConfirmation('confirm-discard');$('cancel-discard').focus();};
  $('cancel-discard').onclick=()=>{clearConfirmations();$('discard').focus();};
  $('confirm-discard-action').onclick=()=>confirmAction('discard');
  $('center').onclick=recenter;$('overview').onclick=()=>{pauseFollowing();sheet(false);if(map)fit();else drawMetric();};
  $('zoom-in').onclick=()=>{if(map){if(manualDraft)pauseFollowing();map.zoomIn();}};
  $('zoom-out').onclick=()=>{if(map){if(manualDraft)pauseFollowing();map.zoomOut();}};
  document.addEventListener('keydown',e=>{if(e.key==='Escape'){if(!$('confirm-discard').hidden)$('cancel-discard').click();else if(!$('confirm-restart').hidden)$('cancel-restart').click();else if(!$('confirm').hidden)$('cancel-save').click();else sheet(false);}
    if(e.key==='Tab'){const dialog=!$('confirm-discard').hidden?$('confirm-discard'):!$('confirm-restart').hidden?$('confirm-restart'):!$('confirm').hidden?$('confirm'):!$('sheet').hidden?$('sheet'):null;if(!dialog)return;
      const controls=[...dialog.querySelectorAll('button,select')].filter(el=>!el.disabled&&el.getClientRects().length);
      const first=controls[0],last=controls[controls.length-1];if(e.shiftKey&&document.activeElement===first){e.preventDefault();last.focus();}else if(!e.shiftKey&&document.activeElement===last){e.preventDefault();first.focus();}}});
  // Own the resize instead of the SDK's timer: its raster resize shifts the
  // geographic center. Preserve the point under the manual-position crosshair.
  new ResizeObserver(()=>{
    if(!map){drawMetric();return;}
    const center=map.getCenter(),zoom=map.getZoom();map.checkResize();
    const resized=map.getCenter();
    if(center&&resized&&(center.lng!==resized.lng||center.lat!==resized.lat))map.centerAndZoom(center,zoom);
  }).observe($('map'));
  async function boot(){const config=bridge?await qtCall('config'):await request('config');loadMap(config.browser_key);await poll();
    // Only one outstanding snapshot; hidden views do not spend map/CPU bandwidth.
    async function tick(){if(!document.hidden)await poll();setTimeout(tick,500);}setTimeout(tick,500);}
  function start(){boot().catch(e=>{toast(e.message);label('headline','正在重新连接设备');setTimeout(start,3000);});}
  if(window.qt&&window.qt.webChannelTransport){const script=document.createElement('script');script.src='qrc:///qtwebchannel/qwebchannel.js';script.onload=()=>new QWebChannel(qt.webChannelTransport,channel=>{bridge=channel.objects.walking;start();});document.head.append(script);}
  else {document.body.classList.add('standalone');start();}
})();
