"""Small snapshots of receiver sentences, captured by the existing serial owner."""
from collections import OrderedDict
import re

from nav_rtk.parser.checksum import validate_nmea_sentence


class ReceiverDiagnostics:
    def __init__(self):
        self.messages = OrderedDict()
        self.antenna = None
        self.counts = dict.fromkeys(("GGA", "RMC", "GSA", "GSV", "GST"), 0)

    def feed(self, text, received_at):
        # Never include binary WTRAW fragments, AT commands or account settings.
        if len(text) > 512 or not re.match(r"^\$(?:[A-Z]{2}(?:GGA|RMC|GSA|GSV|GST)|QXANTSTAT),", text):
            return
        if not validate_nmea_sentence(text):
            return
        if text.startswith('$QXANTSTAT,'):
            fields = text.split('*', 1)[0].split(',')
            if len(fields) == 2 and fields[1].isdigit():
                self.antenna = dict(code=int(fields[1]), received_at=received_at, source='QXANTSTAT')
            return
        key = text[1:6]
        self.counts[key[-3:]] += 1
        self.messages[key] = dict(raw=text, received_at=received_at)
        self.messages.move_to_end(key)
        while len(self.messages) > 32:
            self.messages.popitem(last=False)

    def snapshot(self):
        return dict(messages={key: dict(value) for key, value in self.messages.items()},
                    counts=dict(self.counts), antenna=dict(self.antenna) if self.antenna else None)
