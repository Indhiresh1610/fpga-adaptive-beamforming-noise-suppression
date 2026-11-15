#!/usr/bin/env python3
"""Load an input case into the board over the UART, run it, and save the Wiener output as a CSV.
   python run_case.py --port COM10 --case ..\\data\\case_5.mem              load case_5, run, receive -> wf_board.csv
   python run_case.py --port COM10 --case case_5.mem --out case5_board.csv
   python run_case.py --port COM10 --case case_5.mem --no-run               only load (then press BTN1, or use wiener_uart_rx.py --start)
   Needs pyserial:  pip install pyserial.  Put wiener_uart_rx.py in the same folder (it decodes the dump).
   Close every other program that holds the COM port (Vivado hardware terminal, PuTTY ...).
   Protocol (see top/case_loader.v):  'L' + 2048 lines x 16 bytes (MSB byte first) + 1 checksum byte (sum mod 256).
   The board has no answer for the load itself: a bad checksum or a stalled load lights LED 2 and the board then refuses
   'S'.  So 'no data' after the load = look at LED 2 and load again.
   The case file: 2048 lines of 32 hex digits (line 0 = steering vector, lines 1..2047 = snapshots) = the .mem files."""
import sys, time, argparse
import wiener_uart_rx as rx

DEPTH, WORD_BYTES = 2048, 16

def read_case(path):
    words = []
    for ln in open(path):
        t = ln.split("//")[0].strip()
        if not t: continue
        if t.startswith("@"): sys.exit("%s: address lines (@...) are not supported" % path)
        if len(t) != 2 * WORD_BYTES: sys.exit("%s: expected 32 hex digits per line, got %r" % (path, t))
        try: words.append(bytes.fromhex(t))
        except ValueError: sys.exit("%s: not hexadecimal: %r" % (path, t))
    if len(words) != DEPTH: sys.exit("%s: expected %d lines, found %d" % (path, DEPTH, len(words)))
    return words

def load(port, baud, words):
    try: import serial
    except ImportError: sys.exit("pyserial is not installed: pip install pyserial")
    payload = b"".join(words)
    chk = sum(payload) & 0xFF
    t0 = time.time()
    with serial.Serial(port, baud, timeout=1) as ser:
        time.sleep(0.2); ser.reset_input_buffer()
        ser.write(b"L" + payload + bytes([chk]))
        ser.flush()
        time.sleep(0.2)                                   # let the last byte arrive and be written
    print("loaded %d bytes (checksum 0x%02x) in %.1f s" % (len(payload), chk, time.time() - t0))

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True); ap.add_argument("--case", required=True)
    ap.add_argument("--baud", type=int, default=115200); ap.add_argument("--out", default="wf_board.csv")
    ap.add_argument("--timeout", type=int, default=30); ap.add_argument("--no-run", action="store_true")
    a = ap.parse_args()
    words = read_case(a.case)
    load(a.port, a.baud, words)
    if a.no_run: return
    buf = rx.read_serial(a.port, a.baud, b"S", a.timeout)
    ys, hdr, end, nw, bad, skipped = rx.parse(buf)
    print("samples %d, weight sets %d, bad checksums %d, skipped bytes %d, end record: %s" % (len(ys), nw, bad, skipped, end))
    if hdr: print("header: bf/Wiener status byte 0x%02x, capture overflow %d" % (hdr[1], hdr[2]))
    with open(a.out, "w") as f:
        f.write("idx,block,re_raw,im_raw\n")
        for idx, blk, re, im in ys: f.write("%d,%d,%d,%d\n" % (idx, blk, re, im))
    print("wrote", a.out)

if __name__ == "__main__": main()
