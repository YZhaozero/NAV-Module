from nav_rtk.bridge.framing import is_receiver_sentence


def test_binary_fragments_in_field_log_are_not_checksum_failures():
    for fragment in (b'$\x1ao\x15\x00*75', b'$K\x04o\x00*5E', b'$\xff\x00*12',
                     b'$GNGGA,\xff*00', b'ROVER OPEN OK'):
        assert not is_receiver_sentence(fragment)


def test_real_ascii_sentences_keep_checksum_validation():
    for line in (b'$GNGGA,120000.00,,,,,0,00,99.99,,,,,,*00', b'$GNRMC,invalid*00', b'$WTRTK,1*00'):
        assert is_receiver_sentence(line)
