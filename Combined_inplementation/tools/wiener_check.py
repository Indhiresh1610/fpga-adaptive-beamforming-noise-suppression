#!/usr/bin/env python3
"""Compare the Wiener result of the board (or of the simulation) with the golden / model result.
   python wiener_check.py compare GOLDEN.csv RESULT.csv      PASS if model-vs-RTL SNR > 50 dB (FFT IP truncates)
   python wiener_check.py quality BF.csv GOLDEN.csv RESULT.csv [--tone HZ]   tone-to-residual: beamformer / model / RTL
   Files: CSV with header (idx,block,re_raw,im_raw  or  idx,re_raw,im_raw), or header-less text "idx blk re im"."""
import sys, numpy as np

def load(path):
    """-> array of rows [idx, re, im]"""
    lines = [l.strip() for l in open(path) if l.strip()]
    first = lines[0].replace(",", " ").split()
    if not first[0].lstrip("-").isdigit():                      # header: find the columns by name
        names = [x.strip().lower() for x in lines[0].split(",")]
        ci, cr, cm = names.index("idx"), names.index("re_raw"), names.index("im_raw")
        body = lines[1:]
    else:                                                        # no header: idx blk re im, or idx re im
        ncol = len(first); ci, cr, cm = (0, 2, 3) if ncol >= 4 else (0, 1, 2); body = lines
    out = []
    for l in body:
        t = l.replace(",", " ").split() if "," not in l else l.split(",")
        out.append([int(t[ci]), int(t[cr]), int(t[cm])])
    return np.array(out, dtype=np.int64)

def compare(g_path, r_path):
    g, r = load(g_path), load(r_path)
    if len(g) != len(r): print("FAIL: %d samples in result, %d in golden" % (len(r), len(g))); return 1
    if not np.array_equal(g[:, 0], r[:, 0]): print("FAIL: idx column differs (first at row %d)" % int(np.argmax(g[:, 0] != r[:, 0]))); return 1
    d = r[:, 1:] - g[:, 1:]; ne = int(np.count_nonzero(np.any(d != 0, axis=1)))
    m = g[:, 1] + 1j*g[:, 2]; e = d[:, 0] + 1j*d[:, 1]
    snr = 10*np.log10(np.sum(np.abs(m)**2) / max(np.sum(np.abs(e)**2), 1e-30))
    print("samples %d, differing %d, max |diff| = %d LSB (Q2.22), model-vs-RTL SNR = %.1f dB" % (len(g), ne, np.abs(d).max(), snr))
    if ne:
        k = int(np.argmax(np.any(d != 0, axis=1))); print("first difference at row %d (idx %d): expected %s got %s" % (k, g[k, 0], g[k, 1:], r[k, 1:]))
    ok = snr > 50; print("PASS" if ok else "FAIL", "(exact match)" if ne == 0 else "(within FFT-IP truncation)" if ok else "")
    return 0 if ok else 1

def tone_snr(z, n, f, fs=512.0):
    e = np.exp(2j*np.pi*f*n/fs); a = np.vdot(e, z)/np.vdot(e, e); r = z - a*e
    return 10*np.log10(np.abs(a)**2*len(z)/np.sum(np.abs(r)**2))

def quality(bf_path, g_path, r_path, f):
    lines = [l.strip().split(",") for l in open(bf_path) if l.strip()]
    b = np.array([[int(t[0]), int(t[1]), int(t[2]), int(t[3])] for t in lines if t[0].lstrip("-").isdigit()], dtype=np.int64)
    b = b[b[:, 1] != 0]; g, r = load(g_path), load(r_path)
    if not (len(b) == len(g) == len(r)): print("length mismatch"); return 1
    k = 48; n = b[k:, 0].astype(float)
    Z = lambda m: (m[k:, 1] + 1j*m[k:, 2])/2**22
    y = (b[k:, 2] + 1j*b[k:, 3])/2**22
    s0, s1, s2 = tone_snr(y, n, f), tone_snr(Z(g), n, f), tone_snr(Z(r), n, f)
    print("tone-to-residual (dB): beamformer %.2f   model %.2f   RTL %.2f   (RTL gain %+.2f dB, model %+.2f dB)" % (s0, s1, s2, s2 - s0, s1 - s0))
    return 0

if __name__ == "__main__":
    a = sys.argv[1:]; f = 30.0
    if "--tone" in a: i = a.index("--tone"); f = float(a[i+1]); del a[i:i+2]
    if a[:1] == ["compare"] and len(a) == 3: sys.exit(compare(a[1], a[2]))
    if a[:1] == ["quality"] and len(a) == 4: sys.exit(quality(a[1], a[2], a[3], f))
    sys.exit(__doc__)
