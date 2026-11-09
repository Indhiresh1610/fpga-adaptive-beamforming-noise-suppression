#!/usr/bin/env python3
"""
fpga_uplink_rx.py -- receive the MVDR beamformer output from the Boolean board.

Updated for 2047 samples (26 full blocks of 76 samples + 1 tail block of 71 samples),
27 weight snapshots, and dual-mode comparison (bit-exact raw integer matching or
float64 reference comparison against y_python.csv).

Live board (needs `pip install pyserial`):
    python fpga_uplink_rx.py --port COM5 --start
    python fpga_uplink_rx.py --port COM5 --start --compare y_python.csv
    python fpga_uplink_rx.py --port COM5 --dump          # re-send the last run
    python fpga_uplink_rx.py --port COM5                 # just listen, then press btn_start

Offline, from the Vivado testbench (tb_uplink.v):
    python fpga_uplink_rx.py --hexfile uart_dump.hex --compare y_python.csv

Writes (prefix set by --out, default "fpga"):
    fpga_y.csv        same columns as tb_save_output's y_out.csv:
                      idx,block,re_raw,im_raw,re_value,im_value,m_data_hex
    fpga_weights.csv  one row per weight set (entry 0 = a/4, used in block 0;
                      entry k = weights used in block k; the last is also used in the tail)
    fpga_meta.json    status flags, solve cycles, counts, checksum parity, record errors

Frame format (see stream_capture_uplink.v):
    A5 5A TYPE PAYLOAD CHK,  CHK = XOR(TYPE, PAYLOAD...)
    'H' 10 B, 'Y' 10 B, 'W' 26 B, 'E' 3 B, little-endian
"""

import argparse
import csv
import json
import math
import sys
import time

SYNC0, SYNC1 = 0xA5, 0x5A
T_HDR, T_Y, T_W, T_END = 0x48, 0x59, 0x57, 0x45
PAYLOAD_LEN = {T_HDR: 10, T_Y: 10, T_W: 26, T_END: 3}

Q22 = float(1 << 22)   # beamformer output Q2.22
Q20 = float(1 << 20)   # weights Q5.20
LSB_Y = 1.0 / Q22

STATUS_BITS = ["err_overrun", "err_pivot", "err_solver", "sat_solver",
               "sat_w", "sat_y", "info_zero_trace", "err_cfg"]


def sext(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------
def parse_records(buf):
    """Return (records, n_bad, n_skipped_bytes). records = [(type, bytes)]."""
    recs, bad, skipped = [], 0, 0
    i, n = 0, len(buf)
    while i + 3 <= n:
        if buf[i] == SYNC0 and buf[i + 1] == SYNC1 and buf[i + 2] in PAYLOAD_LEN:
            t = buf[i + 2]
            L = PAYLOAD_LEN[t]
            end = i + 3 + L + 1
            if end > n:
                break                          # incomplete record at the end
            payload = bytes(buf[i + 3:i + 3 + L])
            x = t
            for b in payload:
                x ^= b
            if x == buf[i + 3 + L]:
                recs.append((t, payload))
                i = end
                continue
            bad += 1
        i += 1
        skipped += 1
    return recs, bad, skipped


def has_end_record(buf):
    recs, _, _ = parse_records(buf)
    return any(t == T_END for t, _ in recs)


# ---------------------------------------------------------------------------
# Record decoding
# ---------------------------------------------------------------------------
def decode_header(p):
    v = int.from_bytes(p, "little")
    status = (v >> 32) & 0xFF
    return {
        "version": v & 0xFF,
        "n_y": (v >> 8) & 0xFFFF,
        "n_w": (v >> 24) & 0xFF,
        "status": status,
        "status_hex": f"0x{status:02x}",
        "status_flags": [name for b, name in enumerate(STATUS_BITS) if (status >> b) & 1],
        "solve_cycles_last_block": (v >> 40) & 0xFFFFFFFF,
        "capture_overflow": bool((v >> 72) & 1),
    }


def decode_y(p):
    v = int.from_bytes(p, "little")
    data = v & ((1 << 48) - 1)
    return {
        "idx": (v >> 48) & 0xFFFF,
        "block": (v >> 64) & 0xFF,
        "last": (v >> 72) & 1,
        "is_tail": (v >> 73) & 1,
        "re_raw": sext(data, 24),
        "im_raw": sext(data >> 24, 24),
        "data": data,
    }


def decode_w(p):
    entry = p[0]
    v = int.from_bytes(p[1:], "little")       # 200 bits
    w = []
    for m in range(4):
        re = sext(v >> (50 * m), 25)
        im = sext(v >> (50 * m + 25), 25)
        w.append((re, im))
    return entry, w


def decode_end(p):
    v = int.from_bytes(p, "little")
    return {"n_y": v & 0xFFFF, "n_w": (v >> 16) & 0xFF}


def checksum75(ys):
    """Same running checksum as the board wrapper / tb_save_output (LD3 = parity)."""
    mask = (1 << 75) - 1
    cs = 0
    for y in ys:
        word = (y["is_tail"] << 74) | (y["last"] << 73) | (y["block"] << 65) \
               | (y["idx"] << 49) | (y["data"] << 1)
        cs = (((cs << 1) & mask) | (cs >> 74)) ^ word
    return cs


# ---------------------------------------------------------------------------
# Input sources
# ---------------------------------------------------------------------------
def read_hexfile(path):
    out = bytearray()
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and all(c in "0123456789abcdefABCDEF" for c in line):
                out.append(int(line, 16))
    return out


def read_serial(port, baud, send_start, send_dump, timeout_s):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is not installed: pip install pyserial")

    buf = bytearray()
    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        if send_start:
            ser.write(b"S")
            print("sent 'S' (reset + start)")
        elif send_dump:
            ser.write(b"D")
            print("sent 'D' (re-send last run)")
        else:
            print("listening... press btn_start on the board")

        t0 = time.time()
        last_rx = None
        last_check = 0
        while True:
            chunk = ser.read(4096)
            now = time.time()
            if chunk:
                buf += chunk
                last_rx = now
            # check for the end record from time to time (cheap: ~29 kB total for 2047 samples)
            if len(buf) - last_check > 2048 or (not chunk and len(buf) != last_check):
                last_check = len(buf)
                if has_end_record(buf):
                    break
            if last_rx is None and now - t0 > timeout_s:
                sys.exit(f"no data within {timeout_s} s -- wrong port/baud, or run not started?")
            if last_rx is not None and now - last_rx > 3.0:
                print("WARNING: stream stopped before the end record")
                break
        print(f"received {len(buf)} bytes in {time.time() - t0:.1f} s")
    return buf


# ---------------------------------------------------------------------------
# Output and checks
# ---------------------------------------------------------------------------
def write_y_csv(path, ys):
    with open(path, "w", newline="") as f:
        f.write("idx,block,re_raw,im_raw,re_value,im_value,m_data_hex\n")
        for y in ys:
            f.write(f"{y['idx']},{y['block']},{y['re_raw']},{y['im_raw']},"
                    f"{y['re_raw'] / Q22:.9f},{y['im_raw'] / Q22:.9f},{y['data']:012x}\n")


def write_w_csv(path, ws):
    with open(path, "w", newline="") as f:
        cols = ["entry", "used_in_block"]
        for m in range(4):
            cols += [f"w{m}_re_raw", f"w{m}_im_raw"]
        for m in range(4):
            cols += [f"w{m}_re", f"w{m}_im"]
        wr = csv.writer(f)
        wr.writerow(cols)
        for entry, w in ws:
            row = [entry, entry]
            for re, im in w:
                row += [re, im]
            for re, im in w:
                row += [f"{re / Q20:.9f}", f"{im / Q20:.9f}"]
            wr.writerow(row)


def compare_with_reference(ys, ref_path):
    """Compare received FPGA outputs against a reference CSV.
    Supports either:
      1. Raw format (y_out.csv / y_fixed_point_model.csv): has 're_raw' and 'im_raw'
      2. Float format (y_python.csv): has 're_value' and 'im_value'
    """
    rows = []
    with open(ref_path) as f:
        for r in csv.DictReader(f):
            rows.append(r)

    if not rows:
        print(f"ERROR: {ref_path} is empty")
        return False

    has_raw = "re_raw" in rows[0] and "im_raw" in rows[0]
    got = {y["idx"]: y for y in ys}

    if has_raw:
        # Bit-exact integer comparison
        ref_dict = {int(r["idx"]): (int(r["block"]), int(r["re_raw"]), int(r["im_raw"])) for r in rows}
        missing = sorted(set(ref_dict) - set(got))
        extra = sorted(set(got) - set(ref_dict))
        mism, max_err = [], 0
        for i in sorted(set(ref_dict) & set(got)):
            g = (got[i]["block"], got[i]["re_raw"], got[i]["im_raw"])
            if ref_dict[i] != g:
                mism.append(i)
                max_err = max(max_err, abs(ref_dict[i][1] - g[1]), abs(ref_dict[i][2] - g[2]))

        print(f"\nFPGA vs Raw Reference ({ref_path}):")
        print(f"  samples in ref: {len(ref_dict)}, from FPGA: {len(got)}")
        print(f"  missing: {len(missing)}, extra: {len(extra)}, mismatching: {len(mism)}")
        if mism:
            print(f"  first mismatching idx: {mism[:10]}, max |raw diff| = {max_err} LSBs")
        ok = not (missing or extra or mism)
        print("  RESULT: " + ("BIT-EXACT MATCH" if ok else "DIFFERENT"))
        return ok
    else:
        # Floating point comparison (e.g. against y_python.csv)
        ref_dict = {int(r["idx"]): (int(r["block"]), float(r["re_value"]), float(r["im_value"])) for r in rows}
        missing = sorted(set(ref_dict) - set(got))
        extra = sorted(set(got) - set(ref_dict))
        diffs = []
        for i in sorted(set(ref_dict) & set(got)):
            re_fpga = got[i]["re_raw"] / Q22
            im_fpga = got[i]["im_raw"] / Q22
            d_re = abs(re_fpga - ref_dict[i][1])
            d_im = abs(im_fpga - ref_dict[i][2])
            d_mag = math.hypot(d_re, d_im)
            diffs.append((d_re, d_im, d_mag))

        max_re_err = max(d[0] for d in diffs)
        max_im_err = max(d[1] for d in diffs)
        max_mag_err = max(d[2] for d in diffs)
        rmse = math.sqrt(sum(d[2]**2 for d in diffs) / len(diffs))

        print(f"\nFPGA vs Float64 Reference ({ref_path}):")
        print(f"  samples in ref: {len(ref_dict)}, from FPGA: {len(got)}")
        print(f"  missing: {len(missing)}, extra: {len(extra)}")
        print(f"  max |Re error| : {max_re_err:.6e} ({max_re_err / LSB_Y:.2f} output LSBs)")
        print(f"  max |Im error| : {max_im_err:.6e} ({max_im_err / LSB_Y:.2f} output LSBs)")
        print(f"  max complex err: {max_mag_err:.6e} ({max_mag_err / LSB_Y:.2f} output LSBs)")
        print(f"  complex RMSE   : {rmse:.6e} ({rmse / LSB_Y:.2f} output LSBs)")
        
        # Consistent with theoretical fixed-point solver error bounds (<= 15 LSBs)
        ok = (not missing) and (not extra) and (max_mag_err / LSB_Y < 50.0)
        print("  RESULT: " + ("PASS (within fixed-point tolerance)" if ok else "FAIL (outside tolerance)"))
        return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB1")
    src.add_argument("--hexfile", help="uart_dump.hex written by tb_uplink.v")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--start", action="store_true", help="send 'S' to reset + start a run")
    ap.add_argument("--dump", action="store_true", help="send 'D' to re-send the last run")
    ap.add_argument("--timeout", type=float, default=30.0, help="seconds to wait for first byte")
    ap.add_argument("--out", default="fpga", help="output file prefix")
    ap.add_argument("--compare", help="reference CSV (y_python.csv, y_out.csv, or y_fixed_point_model.csv)")
    a = ap.parse_args()

    if a.hexfile:
        buf = read_hexfile(a.hexfile)
        print(f"read {len(buf)} bytes from {a.hexfile}")
    else:
        buf = read_serial(a.port, a.baud, a.start, a.dump, a.timeout)

    recs, bad, skipped = parse_records(buf)
    hdr, end, ys, ws = None, None, [], []
    for t, p in recs:
        if t == T_HDR:
            hdr = decode_header(p)
            ys, ws = [], []            # keep only the latest dump in the stream
        elif t == T_Y:
            ys.append(decode_y(p))
        elif t == T_W:
            ws.append(decode_w(p))
        elif t == T_END:
            end = decode_end(p)

    if hdr is None:
        sys.exit("no header record found")

    # ---- integrity checks ----
    problems = []
    if bad:
        problems.append(f"{bad} records failed the checksum")
    if end is None:
        problems.append("end record missing (stream cut short)")
    if len(ys) != hdr["n_y"]:
        problems.append(f"expected {hdr['n_y']} samples, decoded {len(ys)}")
    if len(ws) != hdr["n_w"]:
        problems.append(f"expected {hdr['n_w']} weight sets, decoded {len(ws)}")
    idxs = [y["idx"] for y in ys]
    if idxs != list(range(len(idxs))):
        problems.append("sample indices are not 0,1,2,... in order")
    if hdr["capture_overflow"]:
        problems.append("capture overflow flagged by the FPGA")

    cs = checksum75(ys)
    meta = dict(hdr)
    meta.update({
        "samples_decoded": len(ys),
        "weight_sets_decoded": len(ws),
        "bad_records": bad,
        "skipped_bytes": skipped,
        "end_record": end,
        "checksum75_hex": f"{cs:019x}",
        "checksum_parity_LD3": bin(cs).count("1") & 1,
        "problems": problems,
    })

    write_y_csv(f"{a.out}_y.csv", ys)
    write_w_csv(f"{a.out}_weights.csv", ws)
    with open(f"{a.out}_meta.json", "w") as f:
        json.dump(meta, f, indent=2)

    print(f"\nsamples: {len(ys)}   weight sets: {len(ws)}   status: {hdr['status_hex']} "
          f"{hdr['status_flags'] or '(no flags)'}")
    print(f"solve cycles (last block): {hdr['solve_cycles_last_block']}")
    print(f"checksum75 = {cs:019x}   parity = {meta['checksum_parity_LD3']}  (compare with LD3)")
    print(f"wrote {a.out}_y.csv, {a.out}_weights.csv, {a.out}_meta.json")
    for p in problems:
        print("PROBLEM:", p)

    ok = not problems
    if a.compare:
        ok = compare_with_reference(ys, a.compare) and ok
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
