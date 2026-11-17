"""Device-side access to the two fixed Baidu APIs; secrets never enter responses."""
import json
import hashlib
import math
import time
from urllib.parse import urlencode, quote_plus
from urllib.request import Request, build_opener, HTTPRedirectHandler


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class BaiduServiceError(ValueError):
    """Only a local message and a bounded API status can leave this module."""


def require_success(result):
    status = result.get('status')
    if type(status) is not int:
        raise ValueError()
    if status == 0:
        return
    messages = {
        210: '百度地图拒绝当前网络地址（210），请核对服务端密钥的 IP 白名单',
        211: '百度地图签名校验未通过（211），请核对服务端密钥的验证方式',
        220: '百度地图页面来源未获授权（220），请核对浏览器密钥的来源白名单',
    }
    if 1 <= status <= 9999:
        raise BaiduServiceError(messages.get(status, f'百度地图请求未通过（{status}），请核对账号权限和配额'))
    raise ValueError()


def points_valid(points, maximum=10):
    return (isinstance(points, list) and 1 <= len(points) <= maximum and all(
        isinstance(p, list) and len(p) == 2 and all(type(v) in (int, float) and math.isfinite(v) for v in p)
        and abs(p[0]) <= 180 and abs(p[1]) < 85 for p in points))


def request_url(path, params, key, signing_key=''):
    # Baidu's documented SN protocol signs the exact ordered GET query. The SK
    # stays local; only its protocol-required MD5 signature is transmitted.
    params = dict(params)
    if signing_key and path == '/directionlite/v1/walking':
        # The walking API requires a current timestamp whenever SN is present.
        # Include it before signing; regenerate it on every request/retry.
        params['timestamp'] = int(time.time())
    query = path + '?' + urlencode(dict(params, ak=key))
    if signing_key:
        signature = hashlib.md5(quote_plus(query + signing_key).encode('utf-8')).hexdigest()
        query += '&sn=' + signature
    return 'https://api.map.baidu.com' + query


def request_json(path, params, key, signing_key=''):
    request = Request(request_url(path, params, key, signing_key),
                      headers={'Accept': 'application/json'})
    with build_opener(NoRedirect()).open(request, timeout=7) as response:
        raw = response.read(4 * 2**20 + 1)
    if len(raw) > 4 * 2**20:
        raise ValueError()
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise ValueError()
    return data


def call(action, data, key, request=request_json, signing_key=''):
    if not key:
        raise ValueError('本机地图服务密钥尚未配置')
    try:
        def send(path, params):
            return request(path, params, key, signing_key) if signing_key else request(path, params, key)
        if action == 'convert' and points_valid(data.get('points')):
            result = send('/geoconv/v2/', dict(coords=';'.join(f'{p[0]:.7f},{p[1]:.7f}' for p in data['points']),
                                             model=2, output='json'))
            require_success(result)
            points = [[p['x'], p['y']] for p in result.get('result', [])]
            if not points_valid(points) or len(points) != len(data['points']):
                raise ValueError()
            return dict(ok=True, coordinate_system='BD09', points=points)
        if action == 'walking' and points_valid([data.get('start'), data.get('end')]):
            a, b = data['start'], data['end']
            result = send('/directionlite/v1/walking', dict(origin=f'{a[1]:.6f},{a[0]:.6f}',
                destination=f'{b[1]:.6f},{b[0]:.6f}', coord_type='bd09ll', ret_coordtype='bd09ll', steps_info=1))
            require_success(result)
            route = result['result']['routes'][0]
            distance = route['distance']
            if type(distance) not in (float, int) or not math.isfinite(distance) or distance <= 0:
                raise ValueError()
            points = []
            for step in route['steps']:
                for item in step['path'].split(';'):
                    point = [float(v) for v in item.split(',')]
                    if not points or point != points[-1]:
                        points.append(point)
                    if len(points) > 2000:
                        raise ValueError()
            if not points_valid(points, 2000) or len(points) < 2:
                raise ValueError()
            return dict(ok=True, coordinate_system='BD09', points=points, distance_m=distance)
        raise ValueError()
    except BaiduServiceError:
        raise
    except Exception:
        # HTTP exception text can contain the AK. Never relay it to UI/logs.
        raise ValueError('地图请求未完成，请检查联网和密钥权限后重试') from None
