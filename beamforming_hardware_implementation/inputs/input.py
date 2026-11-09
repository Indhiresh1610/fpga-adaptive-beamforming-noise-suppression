import os
import numpy as np

# ============================================================================
# AI generated block: generate one .mem file per case (case_1.mem ... case_7.mem)
#
# Re-runs this script once per case in a separate Python process, passing the
# case number and output file name through environment variables (read by the
# "AI generated hook" line below). Every case therefore starts from the
# same RNG seed, exactly as if you had edited `case` by hand and run the script.
# The launching run then exits after all cases are written.
# Set GENERATE_ALL_CASES = False to run only the single case selected below
# (that run writes case_<case>.mem).
# ============================================================================
GENERATE_ALL_CASES = True
ALL_CASES = [1, 2, 3, 4, 5, 6, 7]

if GENERATE_ALL_CASES and "MVDR_CASE" not in os.environ:
    import subprocess
    import sys

    for c in ALL_CASES:
        env = dict(os.environ, MVDR_CASE=str(c))
        subprocess.run([sys.executable, os.path.abspath(__file__)], env=env, check=True)
    sys.exit(0)
# ======================= End of AI generated block ==========================


fs = 512

# signal parameters
des_angle = 30
n = 4
des_pow = 1.0
duration = 4.0
seed = 42
noise_sigma = 0.35
d = 0.5 # in terms of lambda

# Random number generator
rng = np.random.default_rng(seed)

# AI generated cases:

case = int(os.environ.get("MVDR_CASE", 1))   # AI generated hook: the per-case block at the top sets MVDR_CASE

if case == 1:
    # Basic null steering: one strong (20 dB), well separated interferer
    des_angle = 20
    ang_intf = [-40]
    pow_intf = [100.0]

elif case == 2:
    # Full degrees of freedom: 3 interferers = M-1 nulls for a 4-element array
    des_angle = -20
    ang_intf = [-60, 15, 70]
    pow_intf = [10.0, 10.0, 10.0]

elif case == 3:
    # Overloaded array: more interferers than the array can null, two near endfire
    des_angle = 0
    ang_intf = [-75, -45, 25, 55, 85]
    pow_intf = [10.0] * 5

elif case == 4:
    # Close-in interferer: 15 deg from the desired direction, inside the main beam
    des_angle = -35
    ang_intf = [-20]
    pow_intf = [100.0]

elif case == 5:
    # High dynamic range: 40 dB interferer plus a 0 dB interferer
    des_angle = 50
    ang_intf = [-30, 10]
    pow_intf = [10000.0, 1.0]

elif case == 6:
    ang_intf = [60, -30]
    pow_intf = [10.0, 10.0]

    sweep_start = 0.3
    sweep_end = 0.9
    sweep_angle_start = 60.0
    sweep_angle_end = 20.0

elif case == 7:
    ang_intf = [40, -30]
    pow_intf = [1.0, 10.0]

    power_change_fraction = 0.5
    power_before = 1.0
    power_after = 100.0

else:
    raise ValueError("Invalid case number")

# End of AI generated case-code

# Time array
T = np.arange(int(fs * duration)) / fs
N = len(T)

# Idea: Generate desired signal components, interference components, noise and add it all together

# Generating the desired signal

m = np.arange(n)

# steering vector of desired angle
# creates 4 element vector
a_des = np.exp(-2j * np.pi * d * m * np.sin(np.deg2rad(des_angle)))

phase_des = rng.uniform(-np.pi, np.pi) # this is a random initial phase of the desired signal

# frequency offset of the desired frequency
des_freq_offset = 30

# creating the desired signal
des_signal = np.sqrt(des_pow) * np.exp(1j *(2 * np.pi * des_freq_offset * T + phase_des))

des_components = a_des[:, None] * des_signal[None, :]

# defining frequency offsets of interferers
intf_offsets = [-18, 12, -10, 22, -25, 6]

# creating the total signal with interferers

X = des_components.copy()

n_angle = len(ang_intf)

for k in range(n_angle):

    # dynamic case of angle as a function of time

    if case == 6 and k == 0:
        # AI generated:
        u = np.clip(
            (T / duration - sweep_start) / (sweep_end - sweep_start),
            0.0,
            1.0
        )

        ang_t = sweep_angle_start + (
            sweep_angle_end - sweep_angle_start
        ) * u

    # case of stationary interferer (in termms of angle)
    else:
        ang_t = np.array([ang_intf[k]]*N)

    # Dynamic case of power change
    if case == 7 and k == 0:
        # AI generated block
        pow_t = np.where(
            T < power_change_fraction * duration,
            power_before,
            power_after
        )

    # case of stationary interferer (in terms of power)
    else:
        pow_t = np.array([pow_intf[k]]*N)

    # Steering matrix for the interfering signals: only for creatin of signal
    A = np.exp(-2j * np.pi * d * m[:, None] * np.sin(np.deg2rad(ang_t))[None, :])


    # Creating the interferer signals

    phase_intf = rng.uniform(-np.pi, np.pi)

    f_intf = intf_offsets[k % len(intf_offsets)]

    # signal:
    s = np.sqrt(pow_t) * np.exp(
        1j * (2 * np.pi * f_intf * T + phase_intf)
    )

    # signal components across antennas
    intf_signals = A * s[None,:]

    X += intf_signals


# Adding the Gaussian noise
noise = (noise_sigma / np.sqrt(2.0)) * (
    rng.standard_normal((n, N))
    + 1j * rng.standard_normal((n, N))
)

X += noise

# This is the final X that we wish to send to the FPGA

# Creating the MEM FILE

PEAK      = 0.9            # largest |re| or |im| after scaling
MEM_FILE  = f"case_{case}.mem"   # output file named after the selected case
BLOCK_LEN = 76             # must match BLOCK_LEN in the RTL (mvdr_top.v)

def q(x, frac):
    """float -> 16-bit two's complement integer (0..65535), round + saturate."""
    return np.clip(np.round(x * 2**frac), -32768, 32767).astype(np.int64) & 0xFFFF

def write_mem(X, a, path, peak=PEAK):
    M, N = X.shape
    assert N & (N - 1) == 0, "N must be a power of two (set duration so fs*duration = 2^k)"
    n_used = N - 1                                   # line 0 is taken by the steering vector
    Xu = X[:, :n_used]

    g = peak / max(np.abs(Xu.real).max(), np.abs(Xu.imag).max())   # ONE global gain
    re = q(g * Xu.real.T, 15)                        # (n_used, M), Q1.15
    im = q(g * Xu.imag.T, 15)

    with open(path, "w") as f:
        # header line: steering vector, Q2.14 (1.0 = 0x4000), NOT scaled by g
        f.write("".join(f"{int(q(a[m].imag, 14)):04x}{int(q(a[m].real, 14)):04x}"
                        for m in reversed(range(M))) + "\n")
        for n in range(n_used):
            f.write("".join(f"{int(im[n, m]):04x}{int(re[n, m]):04x}"
                            for m in reversed(range(M))) + "\n")
    return g, n_used

g, n_used = write_mem(X, a_des, MEM_FILE)
print(f"case {case}: {MEM_FILE}, des_angle={des_angle}, N={N}, usable samples={n_used}, "
      f"g={g:.6f}, {BLOCK_LEN}-sample blocks={n_used // BLOCK_LEN} (+ tail {n_used % BLOCK_LEN})")