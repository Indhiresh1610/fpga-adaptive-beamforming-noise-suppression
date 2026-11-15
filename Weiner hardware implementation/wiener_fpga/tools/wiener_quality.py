#!/usr/bin/env python3
"""How good is the Wiener output as a filter?  (not just: does it match the model)
   python3 wiener_quality.py case2_beamformer_out.csv wf_expected.csv wf_out.csv [--tone HZ]
   Fits ONE complex tone (default 30 Hz, fs 512) to each signal and prints tone-to-residual ratio:
   beamformer input, Python model output, RTL output.  Samples: blocks >= 1, first 48 (warm-up) skipped."""
import sys, numpy as np

def rows(path):
    r = []
    for ln in open(path):
        t = ln.strip().split(",")
        try: r.append([int(x) for x in t[:4]])
        except ValueError: continue
    return np.array(r, dtype=np.int64)

def snr(z, n, f, fs=512.0):
    e = np.exp(2j*np.pi*f*n/fs); a = np.vdot(e, z)/np.vdot(e, e); r = z - a*e
    return 10*np.log10(np.abs(a)**2*len(z)/np.sum(np.abs(r)**2))

a = [x for x in sys.argv[1:] if not x.startswith("--")]
f = 30.0
if "--tone" in sys.argv: f = float(sys.argv[sys.argv.index("--tone")+1]); a.remove(sys.argv[sys.argv.index("--tone")+1])
if len(a) != 3: sys.exit(__doc__)
b = rows(a[0]); b = b[b[:, 1] != 0]
mdl = np.array([[int(x) for x in ln.split(",")[1:3]] for ln in open(a[1]).read().splitlines()[1:] if ln.strip()])
rtl = np.array([[int(x) for x in ln.split(",")[1:3]] for ln in open(a[2]).read().splitlines()[1:] if ln.strip()])
if not (len(b) == len(mdl) == len(rtl)): sys.exit("length mismatch: %d %d %d" % (len(b), len(mdl), len(rtl)))
k = 48; n = b[k:, 0].astype(float)
Z = lambda m: (m[k:, 0] + 1j*m[k:, 1]) / 2**22
y = (b[k:, 2] + 1j*b[k:, 3]) / 2**22
s0, s1, s2 = snr(y, n, f), snr(Z(mdl), n, f), snr(Z(rtl), n, f)
print("tone-to-residual (dB):  beamformer %.2f   Python model %.2f   RTL %.2f" % (s0, s1, s2))
print("RTL vs model: %+.2f dB   (RTL gain over beamformer: %+.2f dB, model: %+.2f dB)" % (s2 - s1, s2 - s0, s1 - s0))
