from dataclasses import dataclass


PREAMBLE = 0xFA
BID = 0xFF
EXTENDED_LEN = 0xFF
MTDATA2 = 0x36


@dataclass
class XbusPacket:
    bid: int
    mid: int
    payload: bytes
    extended: bool = False


class XbusParser:
    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf.extend(data)

    def parse_packets(self):
        packets = []
        while True:
            pkt = self._try_parse()
            if pkt is None:
                break
            packets.append(pkt)
        return packets

    def _resync(self):
        idx = self._buf.find(bytes([PREAMBLE]), 1)
        if idx == -1:
            self._buf.clear()
        else:
            del self._buf[:idx]

    def _try_parse(self):
        while self._buf and self._buf[0] != PREAMBLE:
            self._resync()

        if len(self._buf) < 4:
            return None

        bid = self._buf[1]
        if bid != BID:
            self._resync()
            return None

        mid = self._buf[2]
        len_byte = self._buf[3]

        if len_byte == EXTENDED_LEN:
            if len(self._buf) < 6:
                return None
            length = (self._buf[4] << 8) | self._buf[5]
            header_size = 6
            extended = True
        else:
            length = len_byte
            header_size = 4
            extended = False

        total = header_size + length + 1
        if len(self._buf) < total:
            return None

        # Two's complement checksum: sum of all bytes (BID through
        # CHECKSUM) == 0 mod 256
        packet = self._buf[1:header_size + length + 1]
        if (sum(packet) & 0xFF) != 0:
            self._resync()
            return None

        payload = bytes(self._buf[header_size:header_size + length])
        del self._buf[:total]

        return XbusPacket(bid=bid, mid=mid, payload=payload, extended=extended)


def _build_packet(bid, mid, payload, extended=False):
    if extended:
        length = len(payload)
        header = bytes([PREAMBLE, bid, mid, EXTENDED_LEN, length >> 8, length & 0xFF])
        body = bytes([bid, mid, EXTENDED_LEN, length >> 8, length & 0xFF]) + payload
    else:
        header = bytes([PREAMBLE, bid, mid, len(payload)])
        body = bytes([bid, mid, len(payload)]) + payload

    checksum = (-sum(body)) & 0xFF
    return header + payload + bytes([checksum])


def _demo():
    parser = XbusParser()

    # Standard packet
    payload = bytes([0x10, 0x10, 0x02, 0x00, 0x01])
    pkt_bytes = _build_packet(BID, MTDATA2, payload)
    parser.feed(pkt_bytes)
    packets = parser.parse_packets()

    assert len(packets) == 1
    assert packets[0].mid == MTDATA2
    assert packets[0].bid == BID
    assert not packets[0].extended
    print("xbus_parser standard: OK")

    # Extended packet
    payload2 = bytes(range(20))
    pkt_bytes2 = _build_packet(BID, MTDATA2, payload2, extended=True)
    parser.feed(pkt_bytes2)
    packets2 = parser.parse_packets()

    assert len(packets2) == 1
    assert packets2[0].extended
    assert len(packets2[0].payload) == 20
    print("xbus_parser extended: OK")

    # Bad checksum rejected
    bad = bytearray(pkt_bytes)
    bad[-1] ^= 0x01
    parser.feed(bytes(bad))
    packets3 = parser.parse_packets()
    assert len(packets3) == 0
    print("xbus_parser bad checksum rejected: OK")

    # Wrong BID rejected
    bad_bid = _build_packet(0x00, MTDATA2, payload)
    parser.feed(bad_bid)
    packets4 = parser.parse_packets()
    assert len(packets4) == 0
    print("xbus_parser bad BID rejected: OK")


if __name__ == "__main__":
    _demo()
