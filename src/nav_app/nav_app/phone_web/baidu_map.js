/* Shared by the existing phone map and the embedded trajectory page. */
'use strict';
window.NAVBaidu = (() => {
  let loading = null, sequence = 0;
  function load(key) {
    if (window.BMap && window.BMap.Map) return Promise.resolve(window.BMap);
    if (!key) return Promise.reject(new Error('网页地图密钥未配置'));
    if (loading) return loading;
    loading = new Promise((resolve, reject) => {
      const name = '__navBaiduReady' + (++sequence);
      const script = document.createElement('script');
      let settled = false;
      const finish = (error) => {
        if (settled) return;
        settled = true; clearTimeout(timer);
        // Keep a harmless callback for an SDK response that arrives after timeout.
        window[name] = () => {};
        if (error) { script.remove(); loading = null; reject(error); }
        else resolve(window.BMap);
      };
      window[name] = () => finish(window.BMap && window.BMap.Map ? null : new Error('网页地图加载失败'));
      const timer = setTimeout(() => finish(new Error('地图加载超时，请检查网络和网页地图授权后刷新')), 15000);
      script.src = 'https://api.map.baidu.com/api?v=4.0&s=1&ak=' + encodeURIComponent(key) + '&callback=' + name;
      script.onerror = () => finish(new Error('地图网络不可用，请联网后刷新'));
      document.head.appendChild(script);
    });
    return loading;
  }
  const point = p => new BMap.Point(p[0], p[1]);
  return {load, point};
})();
