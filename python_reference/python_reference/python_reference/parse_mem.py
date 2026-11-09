"""parse_mem.py -- read the RTL input memory image (x_mem.mem) exactly as input_mem.v does.

Facts taken from input_mem.v (and cross-checked against the file contents):
  * one 128-bit word per line, 32 hex digits, $readmemh order (line k -> mem[k])
  * mem[0]      = steering vector a, Q2.14   (1.0 = 0x4000), NOT scaled by the input gain
  * mem[1 + n]  = snapshot x[n], Q1.15       (sample_idx n is read from address n + 1)
  * word layout {s3, s2, s1, s0}: sensor m occupies word[32*m +: 32]
  * inside a sensor field: real = [15:0], imag = [31:16], signed two's complement
"""
import numpy as np

N_ANT = 4
FRAC_X = 15      # Q1.15 input samples
FRAC_A = 14      # Q2.14 steering vector


def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def _unpack(word):
    re = [_s16(word >> (32 * m)) for m in range(N_ANT)]
    im = [_s16(word >> (32 * m + 16)) for m in range(N_ANT)]
    return re, im


def load_mem(path):
    """Return dict with integer (raw) and float views of the memory image."""
    words = []
    with open(path) as f:
        for ln, line in enumerate(f, 1):
            s = line.strip()
            if not s:
                continue
            if len(s) != 32:
                raise ValueError(f"{path}:{ln}: expected 32 hex digits, got {len(s)}")
            words.append(int(s, 16))
    if len(words) < 2:
        raise ValueError("memory image too short")
    a_re, a_im = _unpack(words[0])
    xr = np.zeros((len(words) - 1, N_ANT), dtype=np.int64)
    xi = np.zeros_like(xr)
    for n, w in enumerate(words[1:]):
        r, i = _unpack(w)
        xr[n], xi[n] = r, i
    return dict(
        n_words=len(words),
        a_re=np.array(a_re, dtype=np.int64), a_im=np.array(a_im, dtype=np.int64),   # raw Q2.14
        xr=xr, xi=xi,                                                               # raw Q1.15, shape (n_samples_in_mem, 4)
        a=(np.array(a_re) + 1j * np.array(a_im)) / 2.0 ** FRAC_A,
        X=(xr + 1j * xi).T / 2.0 ** FRAC_X,                                         # float, shape (4, n_samples_in_mem)
    )


def samples_used(mem, num_samples):
    """First num_samples snapshots (the RTL reads sample_idx 0..NUM_SAMPLES-1 only)."""
    if num_samples > mem["xr"].shape[0]:
        raise ValueError("NUM_SAMPLES exceeds samples stored in memory")
    return mem["xr"][:num_samples], mem["xi"][:num_samples], mem["X"][:, :num_samples]


if __name__ == "__main__":
    import sys
    m = load_mem(sys.argv[1])
    print("words:", m["n_words"], " samples in image:", m["xr"].shape[0])
    print("a =", m["a"])
    print("max |re|, |im| raw:", np.abs(m["xr"]).max(), np.abs(m["xi"]).max())
