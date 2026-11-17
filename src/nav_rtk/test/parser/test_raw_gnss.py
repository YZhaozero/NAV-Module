import base64
import json
import pytest
from nav_rtk.parser.ntrip import crc24q
from nav_rtk.parser.raw_gnss import describe_rtcm, RawGnssStatus
from nav_rtk.parser.raw_export import export_session


def msm(variant=5, invalid=False):
    # One satellite and signal, rough range 70 ms, fine PR/phase zero (valid).
    def bits(n, width):
        return f'{n:0{width}b}'
    text = bits(1070+variant,12)+bits(42,12)+bits(123456,30)+'0'*19
    text += '1'+'0'*63+'1'+'0'*31+'1'
    text += bits(70,8)
    if variant in (5,7): text += '0'*4
    text += '0'*10
    if variant in (5,7): text += '0'*14
    pr, phase = (20,24) if variant in (6,7) else (15,22)
    text += bits(1 << (pr-1) if invalid else 0,pr)+'0'*phase
    text += '0' * ((10+1+10) if variant in (6,7) else (4+1+6))
    if variant in (5,7): text += '0'*15
    text += '0' * (-len(text)%8)
    return packet(int(text,2).to_bytes(len(text)//8,'big'))


def packet(payload):
    raw=b'\xd3'+len(payload).to_bytes(2,'big')+payload
    return raw+crc24q(raw).to_bytes(3,'big')


@pytest.mark.parametrize('variant',[4,5,6,7])
def test_observations_require_valid_crc_payload_and_measurement_fields(variant):
    raw=msm(variant)
    result=describe_rtcm(raw)
    assert result['station_id']==42 and result['measurement_cells']==1
    assert describe_rtcm(msm(variant,True))['measurement_cells']==0
    with pytest.raises(ValueError): describe_rtcm(raw[:-1]+bytes([raw[-1]^1]))
    with pytest.raises(ValueError): describe_rtcm(packet(raw[3:-5]))


def test_export_distinguishes_base_from_rover_and_keeps_exact_bytes(tmp_path):
    source=tmp_path/'rtk-1-2';source.mkdir()
    raw=msm(); ephemeris=packet((1019<<4).to_bytes(2,'big')+b'\0'*59)
    rows=[]
    for event,data in [('ntrip_body_rx',raw[:7]),('ntrip_body_rx',raw[7:]),('serial_rx',raw),
                       ('rover_rtcm',raw),('rover_rtcm',ephemeris)]:
        rows.append(dict(event=event,bytes=len(data),base64=base64.b64encode(data).decode(),wall_ns=123))
    (source/'events-000000.jsonl').write_text('\n'.join(map(json.dumps,rows)))
    result=export_session(source,tmp_path/'out')
    assert result['has_rover_measurements'] and result['malformed_records']==0
    assert result['streams']['rover']['ephemeris_packets']==1
    assert (tmp_path/'out/rover.rtcm3').read_bytes()==raw+ephemeris
    assert (tmp_path/'out/base.rtcm3').read_bytes()==raw
    # Old logs with only base observations must never be labelled rover observations.
    (source/'events-000000.jsonl').write_text('\n'.join(map(json.dumps,rows[:3])))
    assert not export_session(source,tmp_path/'base-only')['has_rover_measurements']


def test_status_records_only_crc_valid_rover_packets():
    class Evidence:
        def __init__(self):self.rows=[]
        def bytes(self,*args):self.rows.append(args)
        def __call__(self,*args,**kwargs):pass
    evidence=Evidence(); status=RawGnssStatus(evidence)
    status.receive(msm()); status.receive(b'invalid')
    assert status.snapshot()['measurement_cells']==1
    assert status.snapshot()['invalid_packets']==1
    assert evidence.rows==[('rover_rtcm',msm())]
