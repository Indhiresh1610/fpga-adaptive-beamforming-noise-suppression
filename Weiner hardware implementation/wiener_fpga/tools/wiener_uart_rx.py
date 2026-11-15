#!/usr/bin/env python3
"""Receive the Wiener result from the board over the USB-UART and save it as wf_board.csv (idx,re_raw,im_raw).
   python wiener_uart_rx.py --port COM5 --start            send 'S' (reset + run), wait for the dump, save wf_board.csv
   python wiener_uart_rx.py --port COM5 --dump             send 'D' (send the last result again)
   python wiener_uart_rx.py --port COM5                    just listen (press btn_start on the board)
   python wiener_uart_rx.py --hexfile dump.bin             decode a raw byte file instead of a serial port
   Needs pyserial for --port:  pip install pyserial
   Then compare:  python wiener_check.py compare wf_expected.csv wf_board.csv
   Frame format (from the team's stream_capture_uplink): A5 5A TYPE PAYLOAD CHK, CHK = XOR of TYPE and payload,
   TYPE 'H' (10 B header), 'Y' (10 B, one per sample), 'W' (26 B), 'E' (3 B end marker)."""
import sys, time, argparse
LENS = {0x48: 10, 0x59: 10, 0x57: 26, 0x45: 3}

def parse(buf):
    ys, hdr, end, bad, skipped, i = [], None, None, 0, 0, 0
    n = len(buf)
    while i + 3 <= n:
        if buf[i] != 0xA5 or buf[i+1] != 0x5A: i += 1; skipped += 1; continue
        t = buf[i+2]
        if t not in LENS: i += 1; skipped += 1; continue
        L = LENS[t]
        if i + 3 + L + 1 > n: break
        p = bytes(buf[i+3:i+3+L]); chk = buf[i+3+L]
        c = t
        for b in p: c ^= b
        if c != chk: bad += 1; i += 1; skipped += 1; continue
        v = int.from_bytes(p, "little")
        if t == 0x59:
            d = v & ((1 << 48) - 1)
            re = d & 0xFFFFFF; im = d >> 24
            re -= (re >> 23) << 24; im -= (im >> 23) << 24          # sign-extend 24 bit
            ys.append(((v >> 48) & 0xFFFF, re, im, (v >> 72) & 1))   # idx, re, im, last
        elif t == 0x48: hdr = (p[0], p[4], p[9] & 1)      # version, status byte, overflow flag
        elif t == 0x45: end = (p[0] | p[1] << 8)
        i += 4 + L
    return ys, hdr, end, bad, skipped

def read_serial(port, baud, cmd, timeout):
    try: import serial
    except ImportError: sys.exit("pyserial is not installed: pip install pyserial")
    buf = bytearray()
    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(0.2); ser.reset_input_buffer()
        if cmd: ser.write(cmd); print("sent %r" % cmd)
        else: print("listening... press btn_start on the board")
        t0 = time.time(); last = None
        while True:
            c = ser.read(4096); now = time.time()
            if c: buf += c; last = now
            if last is None and now - t0 > timeout: sys.exit("no data within %d s: wrong port, or run not started" % timeout)
            if last is not None and now - last > 3.0: break
            if len(buf) > 40 and parse(buf)[2] is not None: break
    print("received %d bytes in %.1f s" % (len(buf), time.time() - t0))
    return buf

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port"); ap.add_argument("--hexfile"); ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--start", action="store_true"); ap.add_argument("--dump", action="store_true")
    ap.add_argument("--timeout", type=int, default=30); ap.add_argument("--out", default="wf_board.csv")
    a = ap.parse_args()
    if a.hexfile: buf = bytearray(open(a.hexfile, "rb").read())
    elif a.port: buf = read_serial(a.port, a.baud, b"S" if a.start else b"D" if a.dump else None, a.timeout)
    else: sys.exit(__doc__)
    ys, hdr, end, bad, skipped = parse(buf)
    print("samples %d, bad checksums %d, skipped bytes %d, end record: %s" % (len(ys), bad, skipped, end))
    if hdr: print("header: Wiener status 0x%02x (bit0 noise learned, bit1 gamma sat, bit3 IFFT-input sat), capture overflow %d" % (hdr[1], hdr[2]))
    if end is not None and end != len(ys): print("WARNING: end record says %d samples, decoded %d" % (end, len(ys)))
    with open(a.out, "w") as f:
        f.write("idx,re_raw,im_raw\n")
        for idx, re, im, _ in ys: f.write("%d,%d,%d\n" % (idx, re, im))
    print("wrote", a.out)

if __name__ == "__main__": main()
