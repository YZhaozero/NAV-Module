"""Discriminate ASCII receiver sentences from newline-split binary WTRAW."""
import re


def is_receiver_sentence(raw):
    # A binary fragment can begin with '$' and even contain '*xx'. That alone
    # must not make it a corrupt NMEA message. Retain real ASCII checksum errors.
    return bool(re.match(rb"^\$[A-Z]{5},", raw)) and all(32 <= byte <= 126 for byte in raw)
