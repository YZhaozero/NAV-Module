from nav_rtk.parser.checksum import calculate_nmea_checksum
from nav_rtk.bridge.receiver_diagnostics import ReceiverDiagnostics


def sentence(body):
    return '$' + body + '*' + calculate_nmea_checksum(body)


def test_capture_raw_no_fix_and_rmc_without_turning_them_into_positions():
    diagnostics = ReceiverDiagnostics()
    gga = sentence('GNGGA,081350.400,,,,,0,00,99.99,,,,,,')
    rmc = sentence('GNRMC,081350.400,V,,,,,,,161026,,E,N,V')
    diagnostics.feed(gga, 1.)
    diagnostics.feed(rmc, 1.01)
    result = diagnostics.snapshot()
    assert result['messages']['GNGGA'] == dict(raw=gga, received_at=1.)
    assert result['counts']['GGA'] == result['counts']['RMC'] == 1
    diagnostics.feed(gga, 2.)
    assert result['messages']['GNGGA']['received_at'] == 1., 'snapshot must not mutate later'


def test_bounded_diagnostics_rejects_binary_commands_bad_checksums_and_long_frames():
    diagnostics = ReceiverDiagnostics()
    for raw in ('AT+ACCOUNT secret', '$WTRAW,\x00\xff', '$GNGGA,invalid*00', '$GNGGA,'+'1'*600):
        diagnostics.feed(raw, 1.)
    assert not diagnostics.snapshot()['messages']
    for a in 'ABCDE':
        for b in 'ABCDEFGHIJ':
            diagnostics.feed(sentence(a+b+'GGA,081350.4,,,,,0,00,99.99,,,,,,'), 2.)
    assert len(diagnostics.snapshot()['messages']) == 32


def test_antenna_report_requires_checksum_and_is_never_inferred_from_zero_satellites():
    diagnostics = ReceiverDiagnostics()
    diagnostics.feed(sentence('GNGGA,081350.400,,,,,0,00,99.99,,,,,,'), 1.)
    assert diagnostics.snapshot()['antenna'] is None
    diagnostics.feed('$QXANTSTAT,2*00', 2.)
    assert diagnostics.snapshot()['antenna'] is None
    diagnostics.feed(sentence('QXANTSTAT,2'), 3.)
    before = diagnostics.snapshot()
    assert before['antenna'] == dict(code=2, received_at=3., source='QXANTSTAT')
    diagnostics.feed(sentence('QXANTSTAT,0'), 4.)
    assert before['antenna']['code'] == 2
    assert diagnostics.snapshot()['antenna']['code'] == 0
