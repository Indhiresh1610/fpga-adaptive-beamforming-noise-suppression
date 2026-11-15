#!/usr/bin/env python3
"""Wiener-only check: same input file -> Python model and RTL -> compare.

  python3 wiener_check.py gen  BF_FILE [--keep-block0]     writes wf_in.mem and wf_expected.csv
  python3 wiener_check.py compare wf_expected.csv wf_out.csv
  python3 wiener_check.py selftest                          checks the compare logic itself (no RTL needed)

BF_FILE : beamformer output, either the team's CSV (idx,block,re_raw,im_raw,...) or a text file "idx blk re im"
          (Q2.22 integers).  By default samples of block 0 are dropped (that is what the Wiener receives in the full
          design: block 0 uses quiescent weights and would pollute the noise estimate); --keep-block0 feeds everything.
wf_in.mem      : one line per input sample, 12 hex digits {im[23:0], re[23:0]}  -> $readmemh in tb_wiener_only.v
wf_expected.csv: the Python model output, CSV with header  idx,re_raw,im_raw,re_value,im_value  (Q2.22 ints / values),
                 aligned to the input.  The RTL testbench writes wf_out.csv with header  idx,re_raw,im_raw  (the
                 testbench removes the 48-sample latency, so idx 0 is the output for the first input sample).
                 Both files are plain CSV; compare only uses the idx, re_raw and im_raw columns."""
import sys, numpy as np
from wiener_model import wiener_fixed

def load_bf(path):
    rows = []
    for ln in open(path):
        t = ln.strip().replace(",", " ").split()
        if len(t) < 4: continue
        try: rows.append([int(x) for x in t[:4]])
        except ValueError: continue
    return np.array(rows, dtype=np.int64)

def gen(path, keep0):
    b = load_bf(path)
    if not keep0: b = b[b[:, 1] != 0]
    re, im = b[:, 2], b[:, 3]
    if np.abs(re).max() >= 1 << 23 or np.abs(im).max() >= 1 << 23: sys.exit("input exceeds 24-bit range")
    mr, mi = wiener_fixed(re, im)
    with open("wf_in.mem", "w") as f:
        for r, i in zip(re, im): f.write("%06x%06x\n" % (int(i) & 0xFFFFFF, int(r) & 0xFFFFFF))
    with open("wf_expected.csv", "w") as f:
        f.write("idx,re_raw,im_raw,re_value,im_value\n")
        for k, (r, i) in enumerate(zip(mr, mi)): f.write("%d,%d,%d,%.9f,%.9f\n" % (k, r, i, r/2**22, i/2**22))
    print("%d input samples -> wf_in.mem ; model output -> wf_expected.csv (give the testbench +NIN=%d)" % (len(re), len(re)))

def load2(path):
    """CSV with columns idx,re_raw,im_raw,... (header line skipped) -> array of [re_raw, im_raw]"""
    rows = []
    for ln in open(path):
        t = ln.strip().split(",")
        if len(t) < 3: continue
        try: rows.append([int(t[1]), int(t[2])])
        except ValueError: continue                      # header
    return np.array(rows, dtype=np.int64)

def compare(exp_path, out_path, quiet=False):
    e, o = load2(exp_path), load2(out_path)
    if len(o) != len(e):
        print("FAIL: RTL gave %d samples, expected %d" % (len(o), len(e))); return 1
    d = o - e
    ne = int(np.count_nonzero(np.any(d != 0, axis=1)))
    m = e[:, 0] + 1j*e[:, 1]; err = d[:, 0] + 1j*d[:, 1]
    snr = 10*np.log10(np.sum(np.abs(m)**2) / max(np.sum(np.abs(err)**2), 1e-30))
    if not quiet:
        print("samples %d, differing %d, max |diff| = %d LSB (Q2.22), model-vs-RTL SNR = %.1f dB" % (len(e), ne, np.abs(d).max(), snr))
        if ne:
            k = int(np.argmax(np.any(d != 0, axis=1))); print("first difference at output %d: expected %s got %s" % (k, e[k], o[k]))
    ok = snr > 50
    print("PASS" if ok else "FAIL", "(exact match)" if ne == 0 else "(within FFT-IP truncation)" if ok else "")
    return 0 if ok else 1

def selftest():
    rng = np.random.default_rng(1)
    t = np.arange(600); x = 0.1*np.exp(2j*np.pi*30*t/512) + 0.03*(rng.standard_normal(600) + 1j*rng.standard_normal(600))
    re = np.round(x.real*2**22).astype(np.int64); im = np.round(x.imag*2**22).astype(np.int64)
    mr, mi = wiener_fixed(re, im)
    k = np.arange(600)
    sv = lambda p, r, i: np.savetxt(p, np.c_[k, r, i], fmt="%d", delimiter=",", header="idx,re_raw,im_raw", comments="")
    sv("_e.csv", mr, mi); sv("_o.csv", mr + (k % 3 == 0), mi); sv("_bad.csv", mr[::-1], mi[::-1])
    a = compare("_e.csv", "_e.csv", True); b = compare("_e.csv", "_o.csv", True); c = compare("_e.csv", "_bad.csv", True)
    import os
    for f in ("_e.csv", "_o.csv", "_bad.csv"): os.remove(f)
    print("selftest:", "OK" if (a == 0 and b == 0 and c == 1) else "BROKEN")

if __name__ == "__main__":
    a = sys.argv[1:]
    if a[:1] == ["gen"] and len(a) >= 2: gen(a[1], "--keep-block0" in a)
    elif a[:1] == ["compare"] and len(a) == 3: sys.exit(compare(a[1], a[2]))
    elif a[:1] == ["selftest"]: selftest()
    else: sys.exit(__doc__)
