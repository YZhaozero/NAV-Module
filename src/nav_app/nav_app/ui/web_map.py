"""Read-only embedded page: same SDK loader as the phone, no native bridge."""
import html
import json
import secrets
from pathlib import Path
from urllib.parse import urlsplit
from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel


def prepare_web_engine():
    # Ubuntu's PyQt WebEngine requires the import itself before QApplication,
    # even when AA_ShareOpenGLContexts is already set. Keep it optional.
    from PyQt5.QtWidgets import QApplication
    QApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)
    try:
        from PyQt5 import QtWebEngineWidgets  # noqa: F401
    except ImportError:
        return False
    return True


def document():
    root = Path(__file__).resolve().parent.parent/'phone_web'
    nonce = secrets.token_hex(16)
    policy = (
        "default-src 'none'; script-src 'nonce-"+nonce+"' 'unsafe-eval' "
        "https://*.map.baidu.com https://*.bdimg.com https://*.bcebos.com "
        "http://api.map.baidu.com http://*.bdimg.com http://dlswbr.baidu.com; "
        "img-src data: blob: https://*.baidu.com https://*.bdimg.com https://*.bcebos.com http://*.baidu.com http://*.bdimg.com; "
        "style-src 'unsafe-inline' https://api.map.baidu.com; "
        "connect-src https://*.baidu.com https://*.bdimg.com https://*.bcebos.com http://*.baidu.com http://*.bdimg.com; "
        "worker-src blob:; font-src data:; base-uri 'none'; form-action 'none'")
    page = (root/'trajectory.html').read_text()
    page = page.replace('<meta charset="utf-8">', '<meta charset="utf-8"><meta http-equiv="Content-Security-Policy" content="'+html.escape(policy,quote=True)+'">')
    page = page.replace('<link rel="stylesheet" href="trajectory.css">',
                        '<style>'+(root/'trajectory.css').read_text()+'</style>')
    for name in ('baidu_map.js','trajectory.js'):
        # Bundled source only; keys, coordinates and user text are never HTML.
        tag = '<script src="'+name+'"'+(' defer' if name=='trajectory.js' else '')+'></script>'
        page = page.replace(tag, '<script nonce="'+nonce+'">'+(root/name).read_text()+'</script>')
    return page


def safe_origin(value):
    parsed = urlsplit(value)
    if (parsed.scheme not in ('http','https') or not parsed.hostname or parsed.username
            or parsed.password or parsed.query or parsed.fragment or parsed.path not in ('','/')):
        raise ValueError('地图网页地址无效')
    return value.rstrip('/')


def create_view(parent):
    # Startup preloads the optional renderer; older devices retain recording.
    # Never disable Chromium's sandbox or attach a native command bridge.
    from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage, QWebEngineProfile, QWebEngineSettings
    from PyQt5.QtWebEngineCore import QWebEngineUrlRequestInterceptor
    class Requests(QWebEngineUrlRequestInterceptor):
        base=QUrl()
        def interceptRequest(self, info):
            url=info.requestUrl(); host=url.host().lower()
            # setHtml has a synthetic main-frame request at its HTTP base URL.
            # Only that exact document is allowed; local API/subresource access
            # remains denied and no phone authentication cookie is shared.
            if info.resourceType()==info.ResourceTypeMainFrame and url==self.base:return
            remote=url.scheme() in ('https','http') and any(
                host==domain or host.endswith('.'+domain) for domain in ('baidu.com','bdimg.com','bcebos.com'))
            if not remote and url.scheme() not in ('data','blob'):info.block(True)
    class Page(QWebEnginePage):
        def acceptNavigationRequest(self,url,kind,main):
            return bool(main and (url.scheme()=='data' or url==self.base))
        def createWindow(self,kind):return None
        def chooseFiles(self,*args):return []
        def javaScriptConsoleMessage(self,*args):pass  # SDK messages may include AKs.
    view=QWebEngineView(parent); view.setContextMenuPolicy(Qt.NoContextMenu)
    # The view/page is destroyed before its sibling profile on parent teardown.
    profile=QWebEngineProfile(parent)  # off-the-record; never shares phone cookies
    profile.setHttpCacheType(QWebEngineProfile.MemoryHttpCache)
    profile.setHttpCacheMaximumSize(16*1024*1024)
    profile.downloadRequested.connect(lambda item:item.cancel())
    interceptor=Requests(profile); profile.setUrlRequestInterceptor(interceptor)
    page=Page(profile,view); page.base=QUrl()
    for option in (QWebEngineSettings.JavascriptCanOpenWindows,QWebEngineSettings.LocalContentCanAccessFileUrls,
                   QWebEngineSettings.JavascriptCanAccessClipboard,QWebEngineSettings.PluginsEnabled):
        page.settings().setAttribute(option,False)
    view.setPage(page)
    view._map_profile,view._map_interceptor=profile,interceptor
    return view


class WebMap(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(120,100)
        self.body=QVBoxLayout(self); self.body.setContentsMargins(0,0,0,0)
        self.message=QLabel('等待有效卫星位置'); self.message.setWordWrap(True)
        self.message.setAlignment(Qt.AlignCenter); self.message.setTextFormat(Qt.PlainText)
        self.message.setStyleSheet('font-size: 15px'); self.body.addWidget(self.message)
        self.view=None; self.failed=False; self.ready=False; self.identity=None
        self.payload=dict(point=None,segments=[],history=False,context=0,message='等待有效卫星位置')

    def ensure(self,origin,key):
        if not key:
            self.message.setText('网页地图密钥未配置；请沿用手机地图的配置'); return False
        if self.failed:return False
        if self.view is None:
            try:self.view=create_view(self)
            except (ImportError,RuntimeError):
                self.failed=True
                self.message.setText('设备缺少网页地图显示组件，请安装 python3-pyqt5.qtwebengine 后重启应用。坐标和记录仍可使用。')
                return False
            self.body.addWidget(self.view,1); self.message.hide()
            self.view.loadFinished.connect(self.loaded)
            self.view.page().renderProcessTerminated.connect(self.crashed)
        identity=(safe_origin(origin),key)
        if identity!=self.identity:
            self.identity=identity; self.ready=False
            self.view.page().base=QUrl(identity[0]+'/trajectory')
            self.view._map_interceptor.base=self.view.page().base
            self.view.setHtml(document(),self.view.page().base)
        return True

    def crashed(self,*args):
        self.ready=False; self.message.show()
        self.message.setText('网页地图显示中断，请点击“刷新位置”重试；记录仍在继续。')

    def loaded(self,ok):
        self.ready=ok
        if not ok:self.crashed();return
        self.message.hide()
        self.run('configure',self.identity[1]); self.send(self.payload)

    def run(self,method,*args):
        if self.ready:
            self.view.page().runJavaScript('window.NAVTrajectory && window.NAVTrajectory.'+method+
                '('+','.join(json.dumps(a,ensure_ascii=True,allow_nan=False) for a in args)+');')

    def send(self,payload):
        self.payload=payload
        if not self.failed and self.view is None:self.message.setText(payload.get('message') or '等待网页地图')
        self.run('update',payload)

    def reset_view(self,zoom):
        if self.view and not self.ready and self.identity:
            identity=self.identity; self.identity=None; self.ensure(*identity)
        self.run('view',zoom,True)
