"""Exercise real OS signals and the real bounded evidence writer, no hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import pytest


@pytest.mark.skipif(sys.platform == 'win32', reason='Jetson POSIX signal lifecycle')
def test_repeated_sigint_during_shutdown_finishes_original_evidence(tmp_path):
    code = r'''
import pathlib,sys,time,signal,os,importlib.util
from nav_rtk.bridge import rtk_node as module
from nav_rtk.parser.evidence import EvidenceWriter
root=pathlib.Path(sys.argv[1])
if os.environ.get('RTK_MAIN_UNDER_TEST'):
 spec=importlib.util.spec_from_file_location('nav_rtk.bridge._shutdown_candidate',os.environ['RTK_MAIN_UNDER_TEST'])
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class FakeNode:
 def __init__(self):
  self.evidence=EvidenceWriter(root=root/'evidence')
  self.evidence.bytes('serial_rx',b'synthetic initial bytes')
 def destroy_node(self):
  (root/'draining').write_text('ready')
  time.sleep(.6)
  self.evidence.bytes('serial_rx',b'synthetic final bytes')
  self.evidence.close()
def spin(node):
 (root/'spinning').write_text('ready')
 while True:time.sleep(.05)
module.RtkBridge=FakeNode
module.rclpy.init=lambda **kwargs:None
module.rclpy.spin=spin
module.rclpy.ok=lambda:False
signal.signal(signal.SIGINT,signal.default_int_handler)
module.main()
'''
    process = subprocess.Popen([sys.executable, '-c', code, str(tmp_path)],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    def wait_marker(name):
        end=time.monotonic()+10
        while time.monotonic()<end and not (tmp_path/name).exists():
            assert process.poll() is None, process.communicate()
            time.sleep(.01)
        assert (tmp_path/name).exists()
    try:
        wait_marker('spinning')
        process.send_signal(signal.SIGINT)
        wait_marker('draining')
        process.send_signal(signal.SIGINT)
        _, stderr = process.communicate(timeout=10)
        assert process.returncode == 0, stderr.decode()
        status=json.loads(next((tmp_path/'evidence').glob('*/status.json')).read_text())
        assert status['closed'] and status['accepted']==status['written']==2
        assert status['queued_bytes']==status['dropped']==status['failed']==0
    finally:
        if process.poll() is None:
            process.kill();process.communicate()
