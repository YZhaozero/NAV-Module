import base64
import json
import threading
from unittest.mock import patch
from nav_rtk.parser.evidence import EvidenceWriter


def test_streams_segments_binary_and_flush(tmp_path):
    writer=EvidenceWriter(root=tmp_path,segment_bytes=500)
    payload=b"\x00\xff$GNGGA,raw*00\r\n"*100
    writer.bytes("serial_rx",payload)
    writer("receiver_command",command="AT+OPEN ROVER",acknowledged=False)
    writer.close()
    rows=[json.loads(line) for path in sorted(writer.path.glob("events-*.jsonl")) for line in path.read_text().splitlines()]
    assert b"".join(base64.b64decode(r["base64"]) for r in rows if r["event"]=="serial_rx")==payload
    assert rows[-1]["acknowledged"] is False
    assert all(r["wall_ns"] and r["mono_ns"] for r in rows)
    status=json.loads((writer.path/"status.json").read_text())
    assert status["closed"] and status["written"]==len(rows) and status["dropped"]==0


def test_stalled_disk_does_not_block_producer_or_grow_memory(tmp_path):
    entered=threading.Event(); release=threading.Event()
    def stalled(self):
        entered.set(); release.wait(2)
    with patch.object(EvidenceWriter,"_run",stalled):
        writer=EvidenceWriter(root=tmp_path,queue_bytes=4096)
        assert entered.wait(1)
        for i in range(10000): writer.bytes("serial_rx",b"x"*1000)
        status=writer.snapshot()
        assert status["queued_bytes"]<=4096 and status["dropped"]>9900
        release.set(); writer.close()


def test_invalid_diagnostic_does_not_crash_receiver(tmp_path):
    writer=EvidenceWriter(root=tmp_path)
    writer("bad", value=float("nan"))
    writer("good", value=1)
    writer.close()
    assert writer.snapshot()["dropped"]==1 and writer.snapshot()["written"]==1


def test_disk_reserve_failure_is_explicit_and_keeps_memory_bounded(tmp_path):
    from types import SimpleNamespace
    with patch("nav_rtk.parser.evidence.shutil.disk_usage",return_value=SimpleNamespace(free=1)):
        writer=EvidenceWriter(root=tmp_path)
        writer.bytes("serial_rx",b"original bytes")
        writer.close()
    status=json.loads((writer.path/"status.json").read_text())
    assert status["failed"]==1 and status["queued_bytes"]==0
    assert status["error"] and status["written"]==0
