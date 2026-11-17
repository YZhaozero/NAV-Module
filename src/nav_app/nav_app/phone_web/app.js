"use strict";
const $ = id => document.getElementById(id);
let connected = false, generation = 0, frame = 0, updated = 0, sending = false, pending = 0, gesture = null, objectUrl = null, editable = false;
let frameRequest = null;
let fieldId = "";
let revealInput = false;
let landscapeMode = false, landscapeEpoch = 0, landscapeRotated = false, landscapeFullscreen = false;
function cancelGesture() {
  const current = gesture; gesture = null;
  if (current) { try { $("screen").releasePointerCapture(current.id); } catch (_) {} }
}
function layoutLandscape() {
  cancelGesture();
  if (!landscapeMode) return;
  const stage = $("remote-stage"), width = window.innerWidth, height = window.innerHeight;
  landscapeRotated = width < height;
  stage.classList.toggle("landscape-rotated", landscapeRotated);
  stage.style.setProperty("--landscape-width", (landscapeRotated ? height : width) + "px");
  stage.style.setProperty("--landscape-height", (landscapeRotated ? width : height) + "px");
}
function unlockOrientation() { try { window.screen?.orientation?.unlock?.(); } catch (_) {} }
function releaseFullscreen() {
  if (document.fullscreenElement === $("remote-shell")) {
    try { Promise.resolve(document.exitFullscreen?.()).catch(() => {}); } catch (_) {}
  }
}
function exitLandscape() {
  landscapeEpoch++; landscapeMode = false; landscapeRotated = false; landscapeFullscreen = false;
  cancelGesture(); unlockOrientation(); releaseFullscreen();
  document.body.classList.remove("remote-landscape");
  $("remote-stage").classList.remove("landscape-mode", "landscape-rotated");
  $("remote-stage").style.removeProperty("--landscape-width");
  $("remote-stage").style.removeProperty("--landscape-height");
  $("landscape").textContent = "横屏操作"; $("landscape").setAttribute("aria-pressed", "false");
}
async function enterLandscape() {
  if (!connected || landscapeMode) return;
  const run = ++landscapeEpoch, stage = $("remote-stage"), shell = $("remote-shell");
  landscapeMode = true; document.body.classList.add("remote-landscape"); stage.classList.add("landscape-mode");
  $("landscape").textContent = "退出横屏"; $("landscape").setAttribute("aria-pressed", "true");
  layoutLandscape();
  // The fallback is already interactive while the browser handles permission/orientation.
  try {
    // Browsers force transform:none on :fullscreen; rotate only its inner stage.
    if (!shell.requestFullscreen) return;
    await shell.requestFullscreen();
    if (run !== landscapeEpoch || !landscapeMode) { if (!landscapeMode) releaseFullscreen(); return; }
    landscapeFullscreen = document.fullscreenElement === shell;
    if (landscapeFullscreen && window.screen?.orientation?.lock) await window.screen.orientation.lock("landscape");
  } catch (_) {
    // Unsupported fullscreen/locking keeps the rotated controls and inverse touch mapping.
  } finally {
    if (run === landscapeEpoch && landscapeMode) layoutLandscape();
    else if (!landscapeMode) { unlockOrientation(); releaseFullscreen(); }
  }
}
function clearDraft() { $("text").value = ""; fieldId = ""; revealInput = false; $("text-form").hidden = true; }
async function request(path, data) {
  const abort = new AbortController();
  const timeout = setTimeout(() => abort.abort(), path === "/pair" ? 7000 : 3000);
  try {
    const response = await fetch(path, {method: data ? "POST" : "GET", cache:"no-store", credentials:"same-origin", headers:data ? {"Content-Type":"application/json"} : {}, body:data ? JSON.stringify(data) : undefined, signal:abort.signal});
    // Keep the deadline active while reading the body. A dropped hotspot can
    // deliver headers and then leave an incomplete error response forever.
    const body = await response.text();
    if (!response.ok) {
      let message = "连接失败";
      try {message = JSON.parse(body).message || message;} catch (_) {}
      const error = new Error(message); error.status = response.status; throw error;
    }
    return response;
  } catch (error) {
    if (error.status) throw error;
    throw new Error(path === "/pair" ? "配对请求未确认，请保持连接设备 Wi-Fi，刷新页面后重试" :
      data ? "操作未确认，请查看设备最新画面后重试" : "连接暂时中断，正在重新获取画面");
  } finally {
    clearTimeout(timeout);
  }
}
function block(message) { if (!connected) clearDraft(); $("blocked").textContent = message; $("blocked").hidden = false; gesture = null; }
function disconnected(message) { exitLandscape(); connected = false; generation++; frame = 0; pending = 0; block("连接已结束"); $("remote").hidden = true; $("pairing").hidden = false; $("connection").textContent = "等待配对"; $("message").textContent = message; }
function resume() {
  clearDraft();
  if (document.hidden) return;
  generation++;
  if (frameRequest) frameRequest.abort();
  connected = true; frame = 0; updated = 0; pending = 0; editable = false;
  $("pairing").hidden = true; $("remote").hidden = false;
  block("正在重新连接，获取最新画面…");
  poll(generation);
}
async function poll(run) {
  let failures = 0;
  while (connected && generation === run) {
    const start = performance.now();
    let url = null;
    try {
      const abort = new AbortController(); frameRequest = abort;
      const timeout = setTimeout(() => abort.abort(), 4000);
      let response, blob;
      try {
        response = await fetch("/frame", {cache:"no-store", credentials:"same-origin", signal:abort.signal});
        if (!response.ok) {const error = new Error("连接已结束，请查看设备上的配对码"); error.status=response.status; throw error;}
        blob = await response.blob();
      } finally {clearTimeout(timeout); if (frameRequest === abort) frameRequest = null;}
      if (generation !== run) break;
      const nextFrame = Number(response.headers.get("X-Frame-Id"));
      url = URL.createObjectURL(blob);
      const image = new Image(); image.src = url;
      let decodeTimer;
      try {await Promise.race([image.decode(), new Promise((_, reject) => {decodeTimer=setTimeout(() => reject(new Error("画面加载超时")),3000);})]);}
      finally {clearTimeout(decodeTimer);}
      if (!connected || generation !== run) break;
      failures = 0;
      // Keep the touched image and its coordinates fixed until the finger lifts.
      if (!gesture) {
        $("screen").src = url; if (objectUrl) URL.revokeObjectURL(objectUrl); objectUrl = url;
        url = null;
        frame = nextFrame; updated = performance.now(); editable = response.headers.get("X-Editable") === "1";
        const nextField = response.headers.get("X-Input-Field") || "";
        const changedField = fieldId !== nextField;
        if (changedField || !editable) clearDraft();
        fieldId = nextField;
        const secret = response.headers.get("X-Input-Type") === "password";
        $("text").type = secret && !revealInput ? "password" : "text";
        $("reveal-input").hidden = !secret;
        $("reveal-input").textContent = revealInput ? "隐藏密码" : "显示密码";
        if (changedField && editable) {
          $("text-form").hidden = false;
          $("text-form").scrollIntoView({block:"nearest"});
        }
        $("text").setAttribute("autocapitalize", "none");
        $("text").setAttribute("autocorrect", "off");
        $("text").spellcheck = false;
        if (Number(response.headers.get("X-Input-Processed")) >= pending) pending = 0;
        $("blocked").hidden = !sending && !pending;
        $("connection").textContent = "已连接 · 同步设备画面";
        if (response.headers.get("X-Input-Rejected") === "1") $("feedback").textContent = "设备界面已变化，刚才的操作未执行，请重试。";
      }
    } catch (error) {
      if (!connected || generation !== run) break;
      if (error.status === 401) {disconnected(error.message); break;}
      failures++;
      clearDraft();
      frame = 0; editable = false;
      block("连接中断或画面未更新，操作已暂停"); $("connection").textContent = "等待设备画面";
    } finally {if(url) URL.revokeObjectURL(url);}
    await new Promise(resolve => setTimeout(resolve, failures ? Math.min(2000, 250 * failures) : Math.max(50, 140 - (performance.now() - start))));
  }
}
$("pair-form").addEventListener("submit", async event => {
  event.preventDefault(); const button = event.submitter; button.disabled = true; $("message").textContent = "";
  try { await request("/pair", {code:$("code").value.trim()}); $("code").value = ""; resume(); }
  catch(error) {$("message").textContent = error.message || "无法连接，请检查 Wi-Fi";}
  finally {button.disabled = false;}
});
async function input(data, sourceFrame=frame) {
  if (!connected || sending || pending || !sourceFrame || performance.now() - updated > 1500) return false;
  sending = true; pending = sourceFrame; block("正在同步操作…"); $("feedback").textContent = "";
  const run = generation;
  try {await request("/input", {...data, frame:sourceFrame}); if(run !== generation) return false; frame = 0; return true;}
  catch(error) {if(run !== generation) return false; pending = 0; frame = 0; if(error.status === 401) disconnected(error.message); else $("feedback").textContent = error.message || "操作未确认，请查看最新画面"; return false; }
  finally {sending = false;}
}
function point(event) {
  const box = $("screen").getBoundingClientRect();
  const x = (event.clientX-box.left)/box.width, y = (event.clientY-box.top)/box.height;
  // A clockwise 90° CSS rotation maps source (u,v) to display (1-v,u).
  const source = landscapeRotated ? [y,1-x] : [x,y];
  return source.map(value => Math.min(1,Math.max(0,value)));
}
$("screen").addEventListener("pointerdown", event => {
  if (gesture || event.button !== 0 || !frame || sending || pending || performance.now()-updated > 1500) return;
  event.preventDefault(); $("screen").setPointerCapture(event.pointerId); gesture = {id:event.pointerId, frame, points:[point(event)], started:performance.now()};
});
$("screen").addEventListener("pointermove", event => {if(gesture && gesture.id === event.pointerId && gesture.points.length < 63) gesture.points.push(point(event));});
$("screen").addEventListener("pointerup", event => {
  if (!gesture || gesture.id !== event.pointerId) return;
  const done = gesture; gesture = null; done.points.push(point(event));
  if (performance.now()-done.started < 1500) input({kind:"gesture", points:done.points}, done.frame);
  else $("feedback").textContent = "触摸时间过长，请重新轻点或拖动。";
});
$("screen").addEventListener("pointercancel", cancelGesture);
$("screen").addEventListener("lostpointercapture", cancelGesture);
$("fit").onclick = () => {cancelGesture();document.body.classList.toggle("expanded"); $("fit").textContent = document.body.classList.contains("expanded") ? "恢复大小" : "放大画面";};
$("landscape").onclick = () => landscapeMode ? exitLandscape() : enterLandscape();
$("keyboard").onclick = () => {if(!editable) {$("feedback").textContent="请先轻点设备画面中的输入框。"; return;} $("text-form").hidden=false; $("text").focus(); $("text-form").scrollIntoView({block:"nearest"});};
async function submitDraft() {
  const value = $("text").value;
  if (await input({kind:"text", text:value, replace:true, commit:true})) $("text").value="";
}
$("text-form").onsubmit = async event => {event.preventDefault(); await submitDraft();};
$("enter").onclick = () => $("text").value ? submitDraft() : input({kind:"key",key:"Enter"});
$("backspace").onclick = () => {if($("text").value) $("text").value=Array.from($("text").value).slice(0,-1).join(""); else input({kind:"key",key:"Backspace"});};
$("reveal-input").onclick = () => {revealInput=!revealInput; $("text").type=revealInput ? "text" : "password"; $("reveal-input").textContent=revealInput ? "隐藏密码" : "显示密码";};
$("escape").onclick = () => input({kind:"key",key:"Escape"});
$("disconnect").onclick = async () => {let message="已断开连接"; try{await request("/disconnect",{});} catch{message="网络已中断，请在设备上确认任务状态";} finally{disconnected(message);}};
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {clearDraft();exitLandscape();}
  if(document.hidden) {generation++; frame=0; gesture=null; if(frameRequest) frameRequest.abort(); block("回到页面后等待画面更新");}
  else if(connected) resume();
});
window.addEventListener("online", () => {if(connected) resume();});
window.addEventListener("pageshow", event => {if(event.persisted && connected) resume();});
window.addEventListener("offline", () => {clearDraft(); frame=0; block("网络已断开，恢复后会自动连接");});
window.addEventListener("pagehide", exitLandscape);
window.addEventListener("resize", layoutLandscape);
window.addEventListener("orientationchange", layoutLandscape);
window.visualViewport?.addEventListener("resize", layoutLandscape);
window.screen?.orientation?.addEventListener?.("change", layoutLandscape);
document.addEventListener("fullscreenchange", () => {
  cancelGesture();
  if (document.fullscreenElement === $("remote-shell")) {
    if (!landscapeMode) {releaseFullscreen();return;}
    landscapeFullscreen = true; layoutLandscape();
  } else if (landscapeFullscreen && landscapeMode) exitLandscape();
});
setInterval(() => {if(connected && performance.now()-updated > 1500) block("正在等待最新画面，操作已暂停");},250);
// A page refresh can reuse its HttpOnly pairing cookie while the lease is still alive.
resume();
