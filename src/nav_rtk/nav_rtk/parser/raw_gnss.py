"""Small RTCM output inventory; never confuse base corrections with rover RX."""
from collections import Counter
import time
import threading

EPHEMERIDES = {1019, 1020, 1042, 1044, 1045, 1046}


def describe_rtcm(packet):
    from .ntrip import crc24q
    if len(packet) < 8 or packet[0] != 0xd3 or packet[1] & 0xfc:
        raise ValueError('Invalid RTCM header')
    length = ((packet[1] & 3) << 8) | packet[2]
    if len(packet) != length + 6 or crc24q(packet[:-3]) != int.from_bytes(packet[-3:], 'big'):
        raise ValueError('Invalid RTCM length/checksum')
    payload = packet[3:-3]
    value = int.from_bytes(payload, 'big')
    bits = len(payload) * 8

    def field(offset, width):
        if offset + width > bits:
            raise ValueError('Truncated RTCM message')
        return (value >> (bits-offset-width)) & ((1 << width)-1)

    kind = field(0, 12)
    result = dict(message_type=kind, bytes=len(packet), ephemeris=kind in EPHEMERIDES)
    if kind // 10 in range(107, 113) and kind % 10 in (4, 5, 6, 7):
        satellites = field(73, 64).bit_count()
        signals = field(137, 32).bit_count()
        mask_width = satellites*signals
        mask = field(169, mask_width)
        cells = mask.bit_count()
        variant = kind % 10
        satellite_bits = 36 if variant in (5, 7) else 18
        cell_bits = {4:48, 5:63, 6:65, 7:80}[variant]
        required = 169 + satellites*signals + satellites*satellite_bits + cells*cell_bits
        if required > bits:
            raise ValueError('Truncated MSM observations')
        satellite_start = 169 + mask_width
        cell_start = satellite_start + satellites*satellite_bits
        pr_bits, phase_bits = (20, 24) if variant in (6, 7) else (15, 22)
        valid_cells = cell_index = 0
        for mask_index in range(mask_width):
            if not mask & (1 << (mask_width-mask_index-1)):
                continue
            sat = mask_index // signals
            rough = field(satellite_start + sat*8, 8)
            pr = field(cell_start + cell_index*pr_bits, pr_bits)
            phase = field(cell_start + cells*pr_bits + cell_index*phase_bits, phase_bits)
            valid_cells += int(rough != 255 and pr != 1 << (pr_bits-1)
                               and phase != 1 << (phase_bits-1))
            cell_index += 1
        result.update(station_id=field(12, 12), epoch_raw=field(24, 30),
                      satellites=satellites, signals=signals, cells=cells,
                      measurement_cells=valid_cells, msm=True)
    return result


class RawGnssStatus:
    def __init__(self, evidence):
        self.evidence = evidence
        self.lock = threading.Lock()
        self.types = Counter()
        self.total_bytes = self.msm_packets = self.cells = self.ephemerides = self.errors = 0
        self.last_received_at = None
        self.measurement_cells = 0
        self.configuration = dict(phase='pending', message='正在检查原始卫星测量输出')

    def configured(self, result):
        with self.lock:
            self.configuration = dict(result)
        self.evidence('raw_gnss_configuration', **result)

    def receive(self, packet):
        try:
            details = describe_rtcm(packet)
        except ValueError:
            with self.lock:
                self.errors += 1
            return
        with self.lock:
            self.types[details['message_type']] += 1
            self.total_bytes += len(packet)
            self.last_received_at = time.time()
            self.msm_packets += int(details.get('msm', False))
            self.cells += details.get('cells', 0)
            self.measurement_cells += details.get('measurement_cells', 0)
            self.ephemerides += int(details['ephemeris'])
        # Exact CRC-checked packets, alongside the unmodified serial RX stream.
        self.evidence.bytes('rover_rtcm', packet)

    def snapshot(self):
        with self.lock:
            return dict(configuration=dict(self.configuration), message_types=dict(self.types),
                    bytes=self.total_bytes, msm_packets=self.msm_packets,
                    measurement_cells=self.measurement_cells,
                    observation_cells=self.cells, ephemeris_packets=self.ephemerides,
                    invalid_packets=self.errors, last_received_at=self.last_received_at)
