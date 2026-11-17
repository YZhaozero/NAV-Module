"""Local HTTP/Qt integration; no sensors, ROS nodes or physical robot."""
import http.client
import json
import time
import pytest
from nav_app.phone_remote import RemoteState, RemoteServer, RemoteError
from nav_app.ui.phone_remote import configured_public_url


def test_pairing_expiry_exclusivity_and_rate_limit():
    now = [0.]
    state = RemoteState(lambda: now[0])
    code = state.code
    token = state.pair(code)
    with pytest.raises(RemoteError):
        state.pair(code)
    now[0] = 6
    assert state.controller() == token
    assert not state.live()
    with pytest.raises(RemoteError) as error:
        state.frame(token)
    assert error.value.status == 503  # Needs a fresh image, not another pairing.
    for _ in range(3):
        with pytest.raises(RemoteError):
            state.pair("invalid")
    with pytest.raises(RemoteError) as error:
        state.pair(state.code)
    assert error.value.status == 429
    now[0] = 61
    state.revoke()
    assert state.pair(state.code)
    now[0] += state.SESSION_SECONDS + 1
    assert not state.controller()
    state.revoke()
    now[0] += 301
    with pytest.raises(RemoteError):
        state.pair(state.code)


def test_sleep_discards_input_but_keeps_auth_and_reconnects_with_fresh_frame():
    now = [0.]
    state = RemoteState(lambda: now[0])
    token = state.pair(state.code)
    state.publish(b"old", "view", 800, 480)
    state.submit(token, dict(frame=1, kind="key", key="Enter"))
    now[0] = 20
    with pytest.raises(RemoteError) as error:
        state.frame(token)  # HTTP may arrive before the next GUI tick.
    assert error.value.status == 503 and state.interruptions == 1
    assert not state.take()
    state.publish(b"new", "view", 800, 480)
    assert state.frame(token)[0] == b"new"
    with pytest.raises(RemoteError):
        state.submit(token, dict(frame=1, kind="key", key="Enter"))
    assert state.processed == 1
    state.submit(token, dict(frame=2, kind="key", key="Enter"))
    assert len(state.take()) == 1


def test_visible_pairing_code_can_replace_only_an_offline_phone():
    now = [0.]
    state = RemoteState(lambda: now[0])
    first = state.pair(state.code)
    with pytest.raises(RemoteError) as error:
        state.pair(state.code)
    assert error.value.status == 409
    now[0] = 6
    second = state.pair(state.code)
    assert second != first and state.controller() == second
    with pytest.raises(RemoteError) as error:
        state.frame(first)
    assert error.value.status == 401


@pytest.mark.parametrize("command", [
    {"kind":"gesture", "points":[[float("nan"),0]]},
    {"kind":"gesture", "points":[[1.1,0]]},
    {"kind":"gesture", "points":[[True,0]]},
    {"kind":"gesture", "points":[]},
    {"kind":"gesture", "points":[[0,0]]*65},
    {"kind":"key", "key":"F4"},
    {"kind":"text", "text":"unauthorized field"},
    {"kind":"execute", "command":"anything"},
])
def test_reject_invalid_or_unfocused_input(command):
    state = RemoteState()
    token = state.pair(state.code)
    state.publish(b"image", "view", 800, 480)
    with pytest.raises(RemoteError):
        state.submit(token, dict(frame=1, **command))
    assert not state.take()


def test_expiring_consumed_frames_and_revoked_queued_input():
    now = [0.]
    state = RemoteState(lambda:now[0])
    token = state.pair(state.code)
    state.publish(b"image", "view", 800, 480)
    command = dict(kind="gesture", points=[[.5,.5]], frame=1)
    state.submit(token, command)
    with pytest.raises(RemoteError):
        state.submit(token, command)
    state.revoke()
    assert state.take() == []
    token = state.pair(state.code)
    state.publish(b"image", "view", 800, 480)
    now[0] = 2.1
    with pytest.raises(RemoteError):
        state.submit(token, dict(command, frame=2))
    with pytest.raises(RemoteError):
        state.frame(token)


def call(server, method, path, data=None, token="", headers=None):
    connection = http.client.HTTPConnection("127.0.0.1", server.port, timeout=2)
    values = {"Content-Type":"application/json"}
    if token:
        values["Cookie"] = "nav_remote=" + token
    values.update(headers or {})
    connection.request(method, path, body=json.dumps(data) if data is not None else None, headers=values)
    response = connection.getresponse()
    result = response.status, dict(response.getheaders()), response.read()
    connection.close()
    return result


def test_http_pair_cookie_assets_and_host_origin_boundaries():
    state = RemoteState()
    server = RemoteServer(state,"127.0.0.1",0,[])
    try:
        assert call(server,"GET","/")[0] == 200
        assert call(server,"GET","/frame")[0] == 401
        assert call(server,"POST","/input",dict(kind="key",key="Enter",frame=1))[0] == 401
        assert call(server,"POST","/pair",dict(code=state.code), headers={"Origin":"https://elsewhere.example"})[0] == 403
        assert call(server,"GET","/",headers={"Host":"rebind.example"})[0] == 403
        assert call(server,"POST","/pair",dict(code=state.code),headers={"Content-Type":"text/plain"})[0] == 415
        status, headers, _ = call(server,"POST","/pair",dict(code=state.code))
        assert status == 200 and "HttpOnly" in headers["Set-Cookie"] and "SameSite=Strict" in headers["Set-Cookie"]
        state.publish(b"jpeg", "view",800,480)
        assert call(server,"GET","/frame", token=state.session)[2] == b"jpeg"
        assert call(server,"GET","/../phone_remote.py",token=state.session)[0] == 404
        assert call(server,"POST","/disconnect",{},state.session)[0] == 200
        assert not state.controller()
    finally:
        server.close()


def test_phone_gnss_proxy_forwards_ciphertext_to_pinned_local_service(tmp_path,monkeypatch):
    from pathlib import Path
    from types import SimpleNamespace
    import nav_app.phone_remote as remote
    certificate=tmp_path/'.config/nav/phone-gnss/device.crt'
    certificate.parent.mkdir(parents=True);certificate.write_text('test certificate')
    monkeypatch.setattr(Path,'home',lambda:tmp_path)
    context=object();calls=[]
    monkeypatch.setattr(remote.ssl,'create_default_context',lambda **kw:context
        if kw['cafile']==str(certificate) else None)
    class Response:
        status=200
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def read(self,size):return b'{"ciphertext":"response"}'
    def forward(request,**kwargs):
        calls.append((request,kwargs));return Response()
    monkeypatch.setattr(remote.urllib.request,'urlopen',forward)
    server=RemoteServer(RemoteState(),'127.0.0.1',0,[])
    try:
        payload={'nonce':'opaque','ciphertext':'opaque'}
        status,_,body=call(server,'POST','/api/phone-gnss',payload)
        assert status==200 and json.loads(body)=={'ciphertext':'response'}
        request,options=calls[0]
        assert request.full_url=='https://127.0.0.1:8830/v1/sealed'
        assert json.loads(request.data)==payload
        assert options==dict(context=context,timeout=3)
        assert call(server,'POST','/api/phone-gnss',payload,
            headers={'Host':'rebind.example'})[0]==403
        assert len(calls)==1
    finally:server.close()


def test_journey_api_plans_but_does_not_start_without_real_position(tmp_path):
    from test_journey import network
    from nav_app.outdoor import OutdoorState
    from types import SimpleNamespace
    net,_,_=network(tmp_path)
    outdoor=OutdoorState(tmp_path/'session');outdoor.maps=net.maps;outdoor.network=net
    state=RemoteState();server=RemoteServer(state,'127.0.0.1',0,[],outdoor=outdoor,map_services=SimpleNamespace(browser_key=''))
    try:
        assert call(server,'POST','/navigation/journey',{'action':'plan'})[0]==401
        call(server,'POST','/pair',{'code':state.code})
        status,_,body=call(server,'POST','/navigation/journey',{'action':'plan','start':'road-start','end':'room'},state.session)
        assert status==200 and len(json.loads(body)['plan']['segments'])==3
        assert call(server,'POST','/navigation/journey',{'action':'start'},state.session)[0]==400
        assert outdoor.journey.phase=='preview' and not outdoor.guiding
    finally:server.close()


def test_network_change_updates_allowed_hosts_and_removes_old_address():
    server = RemoteServer(RemoteState(), "127.0.0.1", 0, ["192.168.1.20"])
    try:
        assert call(server, "GET", "/", headers={"Host": f"192.168.1.20:{server.port}"})[0] == 200
        server.update_hosts(["10.42.0.1"])
        assert call(server, "GET", "/", headers={"Host": f"10.42.0.1:{server.port}"})[0] == 200
        assert call(server, "GET", "/", headers={"Host": f"192.168.1.20:{server.port}"})[0] == 403
    finally:
        server.close()


def test_explicit_forwarded_origin_survives_lan_updates_without_wildcard():
    state = RemoteState()
    server = RemoteServer(state, "127.0.0.1", 0, [], public_url="http://demo.example:11080")
    try:
        server.update_hosts(["10.42.0.1"])
        host = {"Host": "demo.example:11080", "Origin": "http://demo.example:11080"}
        assert call(server, "GET", "/", headers=host)[0] == 200
        assert call(server, "GET", "/frame", headers=host)[0] == 401
        assert call(server, "POST", "/pair", {"code": state.code}, headers=host)[0] == 200
        for bad in ("demo.example:8000", "elsewhere.example:11080", "demo.example.evil:11080"):
            assert call(server, "GET", "/", headers={"Host": bad})[0] == 403
        assert call(server, "POST", "/input", {}, headers=dict(host, Origin="http://elsewhere.example"))[0] == 403
    finally:
        server.close()


@pytest.mark.parametrize("url", ["https://demo.example", "http://user:pass@demo.example", "http://demo.example/path", "http://demo.example:0", "http://demo.example/?q=x"])
def test_invalid_forwarded_origins_rejected(url):
    with pytest.raises(ValueError):
        RemoteServer(RemoteState(), "127.0.0.1", 0, [], public_url=url)


def test_frame_transport_reuses_http_connection():
    server = RemoteServer(RemoteState(), "127.0.0.1", 0, [])
    connection = http.client.HTTPConnection("127.0.0.1", server.port, timeout=2)
    try:
        connection.request("GET", "/")
        response = connection.getresponse()
        assert response.status == 200 and response.version == 11
        response.read()
        sock = connection.sock
        connection.request("GET", "/style.css")
        response = connection.getresponse()
        assert response.status == 200
        response.read()
        assert connection.sock is sock and sock is not None
    finally:
        connection.close()
        server.close()


