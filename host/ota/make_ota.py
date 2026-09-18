#!/usr/bin/env python3
"""Turn a PlatformIO firmware.bin into an .ota image the UNO R4 WiFi can take
over the air, and optionally serve it for the robot's `ota` command.

    python3 host/ota/make_ota.py firmware/.pio/build/uno_r4_wifi/firmware.bin
    python3 host/ota/make_ota.py firmware/.pio/build/uno_r4_wifi/firmware.bin --serve

The format is Arduino's (arduino-libraries/ArduinoIoTCloud, extras/tools):

    length   uint32 LE   bytes from `magic` to the end
    crc32    uint32 LE   over the same bytes
    magic    uint32 LE   board id; 0x23411002 is the UNO R4 WiFi
    version  8 bytes     all zero except 0x40 in the last: "LZSS-compressed"
    payload             the firmware, LZSS-compressed

The compressor is a port of Haruhiko Okumura's public-domain LZSS (EI=11,
EJ=4, P=1), the one Arduino ships. On this firmware its output is byte-for-byte
identical to Arduino's C encoder, and Arduino's decoder restores the image
exactly (checked 2026-09-18). That is not guaranteed for every input - any
valid stream decodes the same - so each image is also decoded again here and
compared with the input before it is written.
"""

import argparse
import http.server
import socket
import struct
import sys
import zlib
from pathlib import Path

EI, EJ, P = 11, 4, 1
N = 1 << EI              # window size
F = (1 << EJ) + 1        # longest match
START = N - F            # where the decoder's ring buffer starts writing
MAGIC_UNO_R4_WIFI = 0x23411002
VERSION = bytes([0, 0, 0, 0, 0, 0, 0, 0x40])


class BitWriter:
    def __init__(self):
        self.out = bytearray()
        self.buf = 0
        self.mask = 0x80

    def bit(self, one):
        if one:
            self.buf |= self.mask
        self.mask >>= 1
        if self.mask == 0:
            self.out.append(self.buf)
            self.buf, self.mask = 0, 0x80

    def bits(self, value, width):
        for shift in range(width - 1, -1, -1):
            self.bit((value >> shift) & 1)

    def flush(self):
        if self.mask != 0x80:
            self.out.append(self.buf)
        return bytes(self.out)


def lzss_encode(data):
    """Matches only ever point back into data already emitted, never into the
    decoder's space-filled pre-roll, so the stream stays valid however the
    search picks among equal matches."""
    w = BitWriter()
    heads = {}  # 2-byte prefix -> positions, most recent last
    i, n = 0, len(data)
    while i < n:
        best_len, best_pos = 1, 0
        limit = min(F, n - i)
        if limit >= 2:
            for j in reversed(heads.get(data[i:i + 2], ())):
                if i - j > START:  # outside what the decoder still remembers
                    break
                k = 2
                while k < limit and data[j + k] == data[i + k]:
                    k += 1
                if k > best_len:
                    best_len, best_pos = k, j
                    if k == limit:
                        break
        step = best_len if best_len > P else 1
        if best_len <= P:
            w.bit(1)
            w.bits(data[i], 8)
        else:
            w.bit(0)
            w.bits((START + best_pos) & (N - 1), EI)
            w.bits(best_len - 2, EJ)
        for q in range(i, i + step):
            if q + 2 <= n:
                heads.setdefault(data[q:q + 2], []).append(q)
        i += step
    return w.flush()


def lzss_decode(code):
    """Straight port of the reference decoder, used as the self-check."""
    ring = bytearray(b" " * N)
    r = START
    out = bytearray()
    bitpos, total = 0, len(code) * 8

    def get(width):
        nonlocal bitpos
        if bitpos + width > total:
            return None
        v = 0
        for _ in range(width):
            v = (v << 1) | ((code[bitpos >> 3] >> (7 - (bitpos & 7))) & 1)
            bitpos += 1
        return v

    while True:
        flag = get(1)
        if flag is None:
            break
        if flag:
            c = get(8)
            if c is None:
                break
            out.append(c)
            ring[r] = c
            r = (r + 1) & (N - 1)
        else:
            pos, length = get(EI), get(EJ)
            if pos is None or length is None:
                break
            for k in range(length + 2):
                c = ring[(pos + k) & (N - 1)]
                out.append(c)
                ring[r] = c
                r = (r + 1) & (N - 1)
    return bytes(out)


def make_ota(firmware):
    payload = lzss_encode(firmware)
    decoded = lzss_decode(payload)
    # Trailing pad bits can decode as a few extra bytes; everything the
    # firmware contains must come back exactly, in order, first.
    if decoded[: len(firmware)] != firmware:
        sys.exit("self-check failed: the compressed image does not decode back")
    body = struct.pack("<I", MAGIC_UNO_R4_WIFI) + VERSION + payload
    return struct.pack("<II", len(body), zlib.crc32(body) & 0xFFFFFFFF) + body


def lan_address():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("192.168.0.1", 9))  # no packet is sent; picks the LAN route
        return s.getsockname()[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bin", type=Path)
    ap.add_argument("-o", "--out", type=Path, help="default: next to the .bin")
    ap.add_argument("--serve", action="store_true", help="serve it over http for `ota`")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()

    firmware = args.bin.read_bytes()
    ota = make_ota(firmware)
    out = args.out or args.bin.with_name("tinybot.ota")
    out.write_bytes(ota)
    print(f"{out}: {len(firmware)} -> {len(ota)} bytes ({100 * len(ota) // len(firmware)} %)")

    if args.serve:
        url = f"http://{lan_address()}:{args.port}/{out.name}"
        print(f"serving {out.parent} - send the robot:\n  ota {url}")
        handler = lambda *a: http.server.SimpleHTTPRequestHandler(*a, directory=str(out.parent))
        http.server.ThreadingHTTPServer(("0.0.0.0", args.port), handler).serve_forever()


if __name__ == "__main__":
    main()
