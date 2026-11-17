'use strict';
window.NAVTrajectory = (() => {
  let map = null, marker = null, lines = [], state = null, key = '', loading = false;
  let context = null, lastUpdate = 0, follow = true, zoom = 18, trackSignature = '';
  const message = text => { document.getElementById('message').textContent = text || ''; };
  function removeMarker() { if (map && marker) map.removeOverlay(marker); marker = null; }
  function removeTracks() { if (map) for (const line of lines) map.removeOverlay(line); lines = []; trackSignature = ''; }
  function render() {
    if (!state) return;
    if (!state.point) { removeMarker(); message(state.message || '等待有效卫星位置'); return; }
    if (!map) return;
    const point = NAVBaidu.point(state.point);
    if (!marker) { marker = new BMap.Marker(point); map.addOverlay(marker); }
    else marker.setPosition(point);
    if (follow) map.setCenter(point, {noAnimation:true});
    const signature = JSON.stringify(state.segments || []);
    if (signature !== trackSignature) {
      removeTracks(); trackSignature = signature;
      for (const segment of state.segments || []) {
        if (segment.length < 2) continue;
        const line = new BMap.Polyline(segment.map(NAVBaidu.point), {strokeColor:'#126956',strokeWeight:4,strokeOpacity:.8});
        map.addOverlay(line); lines.push(line);
      }
    }
    message(state.message);
  }
  function boot() {
    if (loading || map || !key || !state || !state.point) return;
    loading = true; message('正在加载百度网页地图…');
    NAVBaidu.load(key).then(() => {
      loading = false;
      if (!state || !state.point) return;
      try {
        map = new BMap.Map('map', {center:NAVBaidu.point(state.point),zoom});
        map.addEventListener('dragstart', () => { follow = false; });
        render();
      } catch (_) { message('网页地图初始化失败，请刷新；坐标和记录仍可使用'); }
    }).catch(error => { loading = false; message(error.message); });
  }
  function update(value) {
    lastUpdate = Date.now();
    if (context !== value.context) {
      context = value.context; follow = true; removeMarker(); removeTracks();
    }
    state = value; render(); boot();
  }
  function view(level, recenter) {
    zoom = level; if (recenter) follow = true;
    if (map) { map.setZoom(zoom); render(); } else boot();
  }
  window.addEventListener('offline', () => { message('网络已断开，底图可能不完整；坐标和记录仍可使用'); });
  window.addEventListener('online', boot);
  setInterval(() => {
    if (state && !state.history && Date.now()-lastUpdate > 2500) {
      state = {...state,point:null}; removeMarker(); message('位置更新已中断，等待恢复');
    }
  }, 500);
  return {configure(value) {key=value;boot();}, update, view};
})();
