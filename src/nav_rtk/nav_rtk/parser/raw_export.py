"""Export one RTK evidence session to separate rover/base RTCM files.

python3 -m nav_rtk.parser.raw_export /path/to/rtk-PID-STAMP /new/output
The input is read-only; output must not exist. JSONL retains arrival timestamps,
while RTCM carries its own measurement epochs. No resampling or synthetic data.
"""
import argparse
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path

from .ntrip import RtcmDecoder
from .raw_gnss import describe_rtcm


def export_session(source, destination):
    source, destination = Path(source), Path(destination)
    files = sorted(source.glob('events-*.jsonl'))
    if not files:
        raise ValueError('请选择包含 events-*.jsonl 的 RTK 日志目录')
    destination.mkdir(mode=0o700, parents=True, exist_ok=False)
    stats = {name: dict(bytes=0, packets=0, message_types=Counter(), measurement_cells=0,
                       ephemeris_packets=0) for name in ('rover', 'base')}
    digests = {name: hashlib.sha256() for name in stats}
    decoder = RtcmDecoder()
    errors = 0
    with (destination/'rover.rtcm3').open('xb') as rover, (destination/'base.rtcm3').open('xb') as base, (destination/'packets.jsonl').open('x', encoding='utf-8') as index:
        streams = dict(rover=rover, base=base)
        for path in files:
            with path.open('rb') as stream:
                for line_number, line in enumerate(stream, 1):
                    try:
                        row = json.loads(line)
                        event = row.get('event')
                        if event not in ('rover_rtcm', 'ntrip_body_rx'):
                            continue
                        data = base64.b64decode(row['base64'], validate=True)
                        if len(data) != row['bytes']:
                            raise ValueError('Invalid recorded length')
                        side = 'rover' if event == 'rover_rtcm' else 'base'
                        packets = [data] if side == 'rover' else decoder.feed(data)
                        for packet in packets:
                            details = describe_rtcm(packet)
                            offset = streams[side].tell()
                            streams[side].write(packet)
                            digests[side].update(packet)
                            stat = stats[side]
                            stat['bytes'] += len(packet)
                            stat['packets'] += 1
                            stat['message_types'][details['message_type']] += 1
                            stat['measurement_cells'] += details.get('measurement_cells', 0)
                            stat['ephemeris_packets'] += int(details['ephemeris'])
                            index.write(json.dumps(dict(side=side, offset=offset, bytes=len(packet),
                                wall_ns=row.get('wall_ns'), mono_ns=row.get('mono_ns'),
                                source=path.name, line=line_number, **{k:v for k,v in details.items() if k != 'bytes'}))+'\n')
                    except (ValueError, KeyError, TypeError, AttributeError):
                        errors += 1
    for name in stats:
        stats[name]['sha256'] = digests[name].hexdigest()
    result = dict(source=str(source.resolve()), streams=stats, malformed_records=errors,
                  has_rover_measurements=bool(stats['rover']['measurement_cells']),
                  note='接收时间见 packets.jsonl；观测历元见 RTCM。尚未验证历元重叠、星历覆盖或完成差分解算。')
    (destination/'manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session')
    parser.add_argument('output')
    args = parser.parse_args()
    print(json.dumps(export_session(args.session, args.output), ensure_ascii=False, indent=2))
