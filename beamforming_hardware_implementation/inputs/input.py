import numpy as np

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

# Time array
T = np.arange(int(fs * duration)) / fs
N = len(T)

# # CASE-1
# ang_intf = [-80, -50, -10, 50, 85, 120]
# pow_intf = [1.0] * 6

# # should add the other cases

# AI generated cases:

case = 1

if case == 1:
    des_angle = 0
    ang_intf = []
    pow_intf = []

elif case == 2:
    ang_intf = [-40, 75]
    pow_intf = [1.0, 1.0]

elif case == 3:
    ang_intf = [-80, -50, -10, 50, 85, 120]
    pow_intf = [1.0] * 6

elif case == 4:
    ang_intf = [25]
    pow_intf = [1.0]

elif case == 5:
    ang_intf = [-50, 65, 100]
    pow_intf = [100.0, 0.1, 0.1]

elif case == 6:
    duration = 8.0
    ang_intf = [60, -30]
    pow_intf = [10.0, 10.0]

    sweep_start = 0.3
    sweep_end = 0.9
    sweep_angle_start = 60.0
    sweep_angle_end = 20.0

elif case == 7:
    duration = 8.0
    ang_intf = [40, -30]
    pow_intf = [1.0, 10.0]

    power_change_fraction = 0.5
    power_before = 1.0
    power_after = 100.0

else:
    raise ValueError("Invalid case number")

# Re-create time array after selecting the case, since duration may change.
T = np.arange(int(fs * duration)) / fs
N = len(T)

# End of AI generated case-code

# # Dynamic case

# duration = 8.0
# T = np.arange(int(fs * duration)) / fs
# N = len(T)

# ang_intf = [60, -30]
# pow_intf = [10.0, 10.0]
# sweep_start = 0.3
# sweep_end = 0.9
# sweep_angle_start = 60
# sweep_angle_end = 20

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

intf_components = []

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

def q(x, frac):  # float -> 16-bit two's complement
    return np.clip(np.round(x * 2**frac), -32768, 32767).astype(np.int64) & 0xFFFF

def write_mem(X, a, x_path="x_mem.mem", a_path="a_mem.mem", peak=0.9):
    M, N = X.shape
    g = peak / max(np.abs(X.real).max(), np.abs(X.imag).max())   # weights don't care about g
    re, im = q(g * X.real.T, 15), q(g * X.imag.T, 15)            # (N, M), Q1.15
    with open(x_path, "w") as f:
        for n in range(N):
            f.write("".join(f"{int(im[n,m]):04x}{int(re[n,m]):04x}"
                            for m in reversed(range(M))) + "\n")
    with open(a_path, "w") as f:                                  # Q2.14
        for m in range(M):
            f.write(f"{int(q(a[m].imag,14)):04x}{int(q(a[m].real,14)):04x}\n")
    return g

PEAK     = 0.9            # largest |re| or |im| after scaling
MEM_FILE = "x_mem.mem"
 
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
print(f"case {case}: N={N}, lines written={n_used + 1}, usable samples={n_used}, "
      f"g={g:.6f}, 64-sample blocks={n_used // 64}")
