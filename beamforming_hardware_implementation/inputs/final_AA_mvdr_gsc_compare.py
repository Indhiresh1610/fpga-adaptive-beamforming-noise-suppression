"""Causal block-MVDR vs NLMS-GSC: final comparison on the static and dynamic testbenches.

Run:   python mvdr_gsc_compare.py        (needs numpy, matplotlib, testbench.py, testbench_dynamic.py;
                                          numba is optional but makes the NLMS loop ~100x faster)
Env:   MVDR_GSC_SHOW=0 -> save the figure only (no window)

What changed relative to AA_mvdr_gsc_realtime_1.py (static) and AA_realtime_2.py (dynamic)
-------------------------------------------------------------------------------------------
Fix 1  ONE SINR definition and ONE window length for both suites.
       SINR is exact: interference+noise power is measured on the real output waveform
       (cross terms between tones included). Window = 0.15 s (= the MVDR block length).
Fix 2  ONE (mu, lambda) pair for BOTH suites. It is chosen once on a grid, on separate tuning
       seeds, giving the two suites equal weight, and then used unchanged for every case.
       MVDR block length (0.15 s) and diagonal loading (testbench value) are fixed.
Fix 3  Monte Carlo: every case is run on EVAL_SEEDS (noise realisation AND tone phases are random
       per seed). Tables report mean +/- std across seeds.

Both algorithms are causal and are scored on their real, time-varying output:
  MVDR : block k uses weights from blocks 0..k-1 only (exponential forgetting lambda); block 0 quiescent.
  GSC  : sample-by-sample NLMS (normalised, step mu).

Outputs (OUTPUT_DIR):
  table1_static.txt        Table 1: steady-state SINR + SINR fluctuation, 15 static cases
  table2_dynamic.txt       Table 2: mean SINR, 7 dynamic scenarios
  tuning_selection.txt     how the single (mu, lambda) pair was chosen (supporting)
  sinr_over_time.png       the one figure: 4 panels, mean +/- std over seeds
"""
import os
import time

import numpy as np
import matplotlib

# SHOW_PLOTS = os.environ.get("MVDR_GSC_SHOW", "1") != "0" # it is true by default. 

# if not SHOW_PLOTS:
#     matplotlib.use("Agg")
import matplotlib.pyplot as plt

from numba import njit # numba is easily getting imported, the whole 
HAVE_NUMBA = True

# try:                                   # optional speed-up for the sample-by-sample NLMS loop
#     from numba import njit
#     HAVE_NUMBA = True
# except ImportError:                    # pure-numpy fallback (correct, but slow)
#     HAVE_NUMBA = False

# for importing required values from the testbench files created. 
from testbench import TEST_CASES, make_config
from testbench_dynamic import DYNAMIC_CASES

# ======================== CONFIGURATION ========================
OUTPUT_DIR = "results_compare"  # specifies the directory into which results are written
GSC_EPSILON = 1e-10     # prevent division by 0 in NLMS update
D_OVER_LAMBDA = 0.5     # distance between sensors as a function of input wavelength
DESIRED_OFFSET_HZ = 30.0
INTERFERER_OFFSETS_HZ = [-18.0, 12.0, -10.0, 22.0, -25.0, 6.0]   # tone offsets by interferer index

WINDOW_S = 0.15                # duration of window used to measure SINR
MVDR_BLOCK_S = 0.15            # duration of MVDR weight-update block
STATIC_EVAL_FRAC = 0.5         # static cases are scored on the second half of the record
DYN_EVAL_FRAC = 0.25           # dynamic cases are scored after the first 25 % (changes start at 30 %)
TIE_TOL_DB = 0.5               # |mean MVDR - GSC| <= this counts as a tie

TUNE_SEEDS = [101, 102, 103, 104]            # used only to choose (mu, lambda)
EVAL_SEEDS = list(range(201, 221))           # 20 Monte Carlo seeds for all reported numbers
MU_GRID = [3e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2]
LAM_GRID = [0.5, 0.8, 0.9, 0.95, 0.98]
# ===============================================================


# --------------------------- case handling ---------------------------
def _core(cfg, interferers, suite, eval_frac):
    return dict(name=cfg["name"], suite=suite, M=cfg["num_antennas"], fs=float(cfg["fs_hz"]),
                dur=cfg["duration_s"], desired_angle=cfg["desired_angle_deg"],
                desired_power=cfg["desired_power"], sigma=cfg["noise_sigma"],
                loading=cfg["mvdr_loading"], interferers=interferers, eval_frac=eval_frac)


def static_case(case):
    """Static testbench case -> common format (constant-power, fixed-angle interferers)."""
    cfg = make_config(case)
    itf = [{"angle_deg": float(a), "power_profile": [(0.0, float(p))]}
           for a, p in zip(cfg["interferer_angles_deg"], cfg["interferer_powers"])]
    return _core(cfg, itf, "static", STATIC_EVAL_FRAC)


def dynamic_case(cfg):
    return _core(cfg, cfg["interferers"], "dynamic", DYN_EVAL_FRAC)


# --------------------------- signal generation ---------------------------
# creates the steering vector for M sensors and angle angles
def steering(angles_deg, M):
    m = np.arange(M)[:, None]
    ang = np.atleast_1d(np.asarray(angles_deg, dtype=float))
    return np.exp(-1j * 2 * np.pi * D_OVER_LAMBDA * m * np.sin(np.deg2rad(ang))[None, :])

# this is to create an array i.e. the power profile, that contains information about the power at different time instances
def power_envelope(it, T, dur):
    p = np.zeros_like(T) # T is an array; this line creates an array just like T but with all 0's
    # `p` is the initial, starting power profile

    # `it` is a dictionary describing one interferer. 
    # T is array containing time of every sample
    # dur is the total duration 

    for frac, pw in sorted(it["power_profile"]):
        p[T >= frac * dur] = pw
    return p

# modelling an interferer whose direction 'sweeps' from one value to another
def angle_track(it, T, dur):
    a0, a1, f0, f1 = it["sweep"]
    # a0: starting angle; a1: ending angle; f0: frac of rec. when sweep begins; f1: frac of rec. when sweep ends
    u = np.clip((T / dur - f0) / (f1 - f0), 0.0, 1.0)
    return a0 + (a1 - a0) * u


def build_signals(c, seed):
    """Returns X (M,N) total, d (M,N) desired component only, a_des (M,). Random phases + noise per seed."""
    M, FS, dur = c["M"], c["fs"], c["dur"] # extracted from the testbench files
    T = np.arange(int(FS * dur)) / FS 
    N = T.size
    rng = np.random.default_rng(seed)
    ph = rng.uniform(-np.pi, np.pi, 1 + len(c["interferers"]))

    a_des = steering(c["desired_angle"], M)[:, 0] # desired angle's steering vector
    s_des = np.sqrt(c["desired_power"]) * np.exp(1j * (2 * np.pi * DESIRED_OFFSET_HZ * T + ph[0])) # desired signal in complex domain
    d = a_des[:, None] * s_des[None, :] # desired signal component alone at all sensors 

    # X is initialised as the desired signal alone
    X = d.copy()

    # `k` is index and `it` is corresponding interferer dictionary
    for k, it in enumerate(c["interferers"]):
        f = INTERFERER_OFFSETS_HZ[k % len(INTERFERER_OFFSETS_HZ)] # if length is greater, values are taken in a cyclic manner
        s = np.sqrt(power_envelope(it, T, dur)) * np.exp(1j * (2 * np.pi * f * T + ph[1 + k]))
        A = steering(angle_track(it, T, dur) if "sweep" in it else it["angle_deg"], M)

        # generates the sensor's final readings, including all interferer directions  
        X += A * s[None, :]

    # adding the random gaussian noise
    X += (c["sigma"] / np.sqrt(2.0)) * (rng.standard_normal((M, N)) + 1j * rng.standard_normal((M, N)))

    
    return X, d, a_des


# --------------------------- algorithms ---------------------------
def _nlms_py(Z, d, mu, eps):
    # Z is the blocking channel matrix
    K, N = Z.shape # K: no. of blocking channels & N: no. of time samples

    g = np.zeros(K, dtype=complex) #initialising the current adaptive weights for the blocking channel
    G = np.empty((K, N), dtype=complex) # initialising array to store weights at every sample
    for n in range(N):
        z = Z[:, n] # blocking channel signals at the current sample
        y = d[n] - np.vdot(g, z) # error signal at current sample
        G[:, n] = g
        g += mu * z * np.conj(y) / (np.vdot(z, z).real + eps)
    return G


if HAVE_NUMBA:
    @njit(cache=True)
    def _nlms_nb(Z, d, mu, eps):
        K, N = Z.shape
        g = np.zeros(K, dtype=np.complex128)
        G = np.empty((K, N), dtype=np.complex128)
        for n in range(N):
            y = d[n]
            zz = 0.0
            for k in range(K):
                y -= np.conj(g[k]) * Z[k, n]
                zz += Z[k, n].real ** 2 + Z[k, n].imag ** 2
            for k in range(K):
                G[k, n] = g[k]
            c = mu * np.conj(y) / (zz + eps)
            for k in range(K):
                g[k] += Z[k, n] * c
        return G
    _nlms = _nlms_nb
else:
    _nlms = _nlms_py


def gsc_prepare(X, a):
    """Quiescent weights, blocking matrix, quiescent output and blocked channels (shared by all mu)."""
    wq = a / a.size
    _, _, vh = np.linalg.svd(a.conj()[None, :], full_matrices=True)
    B = vh.conj().T[:, 1:]
    return wq, B, wq.conj() @ X, np.ascontiguousarray(B.conj().T @ X)


def gsc_weights(prep, mu):
    wq, B, dq, Z = prep
    return wq[:, None] - B @ _nlms(Z, dq, mu, GSC_EPSILON)

# for MVDR

def block_covs(X, block):
    N = X.shape[1] # number of columns i.e. timesteps
    out = []
    for s in range(0, N, block):
        Xb = X[:, s:s + block]
        out.append((Xb @ Xb.conj().T) / Xb.shape[1])
    return out


def mvdr_weights(Rbs, a, lam, loading, block, N):
    """Causal block MVDR: block k uses covariance from blocks 0..k-1 only. Block 0 is quiescent."""
    M = a.size
    W = np.empty((M, N), dtype=complex)
    R = None
    for k, Rb in enumerate(Rbs):
        s, e = k * block, min((k + 1) * block, N)
        if R is None:
            w, R = a / M, Rb
        else:
            u = np.linalg.solve(R + loading * np.trace(R).real / M * np.eye(M), a)
            w = u / np.vdot(a, u)
            R = lam * R + (1 - lam) * Rb
        W[:, s:e] = w[:, None]
    return W


# --------------------------- metric (exact, shared by both suites) ---------------------------
def win_power(x, win):
    n = x.size // win
    return (np.abs(x[:n * win]) ** 2).reshape(n, win).mean(axis=1)


def window_powers(W, X, d, win):
    """Windowed desired power and interference+noise power on the REAL output waveform.
    interference+noise = total output - desired output (linear, so cross terms are included)."""
    y = np.sum(np.conj(W) * X, axis=0)
    yd = np.sum(np.conj(W) * d, axis=0)
    return win_power(yd, win), win_power(y - yd, win)


def sinr_scored_db(pd, pi, k0):
    return 10 * np.log10(max(pd[k0:].sum(), 1e-30) / max(pi[k0:].sum(), 1e-30))


def sinr_curve_db(pd, pi):
    return 10 * np.log10(np.maximum(pd, 1e-30) / np.maximum(pi, 1e-30))


def run_case(c, seed, mus, lams):
    """One case, one seed, all requested settings. Returns {key: (pd, pi)} per window."""
    X, d, a = build_signals(c, seed)
    N = X.shape[1]
    win, block = int(WINDOW_S * c["fs"]), int(MVDR_BLOCK_S * c["fs"])
    out = {"in": (win_power(d[0], win), win_power(X[0] - d[0], win))}    # input at sensor 0
    if mus:
        prep = gsc_prepare(X, a)
        for mu in mus:
            out[("gsc", mu)] = window_powers(gsc_weights(prep, mu), X, d, win)
    if lams:
        Rbs = block_covs(X, block)
        for lam in lams:
            out[("mvdr", lam)] = window_powers(mvdr_weights(Rbs, a, lam, c["loading"], block, N), X, d, win)
    return out


def k0_of(c, n_win):
    return int(c["eval_frac"] * n_win)


# --------------------------- formatting ---------------------------
def table(headers, rows):
    cols = list(zip(*([headers] + rows)))
    w = [max(len(str(x)) for x in col) for col in cols]
    ln = lambda r: "  ".join(str(x).ljust(n) if i == 0 else str(x).rjust(n)
                             for i, (x, n) in enumerate(zip(r, w)))
    return "\n".join([ln(headers), "  ".join("-" * n for n in w)] + [ln(r) for r in rows])


def pm(v, p=2):
    """mean +/- sample std across seeds for a 1-D array."""
    return f"{np.mean(v):.{p}f} +/- {np.std(v, ddof=1):.{p}f}"


def verdict(diff):
    return "MVDR" if diff > TIE_TOL_DB else ("GSC" if diff < -TIE_TOL_DB else "tie")


# --------------------------- step 1: choose ONE (mu, lambda) ---------------------------
def tune(cases):
    acc = {}                                              # (key, suite) -> {case_name: [sinr per seed]}
    for c in cases:
        for seed in TUNE_SEEDS:
            res = run_case(c, seed, MU_GRID, LAM_GRID)
            for key, (pd, pi) in res.items():
                if key == "in":
                    continue
                acc.setdefault((key, c["suite"]), {}).setdefault(c["name"], []).append(
                    sinr_scored_db(pd, pi, k0_of(c, pd.size)))
        print(f"  tuned on: {c['name']}", flush=True)
    score = {}
    for (key, suite), per_case in acc.items():
        score[(key, suite)] = float(np.mean([np.mean(v) for v in per_case.values()]))
    keys = sorted({k for k, _ in score})
    table_rows = {"gsc": [], "mvdr": []}
    best = {}
    for algo, grid in (("gsc", MU_GRID), ("mvdr", LAM_GRID)):
        comb = {p: 0.5 * (score[((algo, p), "static")] + score[((algo, p), "dynamic")]) for p in grid}
        best[algo] = max(comb, key=comb.get)
        for p in grid:
            table_rows[algo].append([f"{p:g}" + (" *" if p == best[algo] else ""),
                                     f"{score[((algo, p), 'static')]:.2f}",
                                     f"{score[((algo, p), 'dynamic')]:.2f}", f"{comb[p]:.2f}"])
    return best["gsc"], best["mvdr"], table_rows


def tuning_text(mu, lam, rows, n_cases):
    hdr = ["", "Static mean SINR", "Dynamic mean SINR", "Combined (equal weight)"]
    L = ["SELECTION OF THE SINGLE (mu, lambda) PAIR USED IN ALL CASES", "=" * 62, "",
         f"Tuning seeds: {TUNE_SEEDS} (disjoint from the evaluation seeds {EVAL_SEEDS[0]}..{EVAL_SEEDS[-1]}).",
         f"Score = mean over cases ({n_cases[0]} static, {n_cases[1]} dynamic) of the mean-over-seeds scored SINR (dB);",
         "the two suites get equal weight. Same SINR definition and window as in the final tables.",
         f"MVDR block = {MVDR_BLOCK_S} s and diagonal loading (testbench value) are NOT tuned.", "",
         "GSC: NLMS step size mu", "-" * 22, table(["mu"] + hdr[1:], rows["gsc"]), "",
         "MVDR: covariance forgetting lambda", "-" * 34, table(["lambda"] + hdr[1:], rows["mvdr"]), "",
         "* = selected (maximum combined score).",
         f"SELECTED: GSC mu = {mu:g}, MVDR lambda = {lam:g}  (fixed for every case in Tables 1 and 2)", ""]
    return "\n".join(L)


# --------------------------- step 2: Monte Carlo evaluation ---------------------------
def evaluate(cases, mu, lam):
    R = {}
    for c in cases:
        runs = []
        for seed in EVAL_SEEDS:
            runs.append(run_case(c, seed, [mu], [lam]))
        R[c["name"]] = runs
        print(f"  evaluated: {c['name']}", flush=True)
    return R


def suite_arrays(cases, R, mu, lam):
    """Per-case, per-seed arrays: scored SINR (MVDR, GSC, input) and within-run SINR fluctuation."""
    nC, nS = len(cases), len(EVAL_SEEDS)
    A = {k: np.zeros((nC, nS)) for k in ("mvdr", "gsc", "in", "fl_mvdr", "fl_gsc")}
    for i, c in enumerate(cases):
        for j, res in enumerate(R[c["name"]]):
            for tag, key in (("mvdr", ("mvdr", lam)), ("gsc", ("gsc", mu)), ("in", "in")):
                pd, pi = res[key]
                k0 = k0_of(c, pd.size)
                A[tag][i, j] = sinr_scored_db(pd, pi, k0)
                if tag != "in":
                    A["fl_" + tag][i, j] = np.std(sinr_curve_db(pd, pi)[k0:])
    return A


def common_note(mu, lam, cases):
    c = cases[0]
    return [f"Settings (identical for every case): GSC NLMS mu = {mu:g}; MVDR lambda = {lam:g}, "
            f"block = {MVDR_BLOCK_S} s, diagonal loading = {c['loading']:g} (testbench value).",
            f"Monte Carlo: {len(EVAL_SEEDS)} seeds (random noise realisation and tone phases); "
            "'a +/- b' = mean +/- std across seeds.",
            f"SINR is exact (interference+noise measured on the output waveform), {WINDOW_S} s windows; both "
            "algorithms causal, scored on their real time-varying output.",
            "Positive MVDR-GSC = MVDR better. Differences are paired by seed."]


def table1_text(cases, A, mu, lam):
    interval = f"{STATIC_EVAL_FRAC * cases[0]['dur']:.2f}-{cases[0]['dur']:.2f} s"
    diff = A["mvdr"] - A["gsc"]
    rows = []
    for i, c in enumerate(cases):
        rows.append([c["name"], pm(A["in"][i]), pm(A["mvdr"][i]), pm(A["gsc"][i]), pm(diff[i]),
                     f"{int((diff[i] > 0).sum())}/{len(EVAL_SEEDS)}",
                     f"{A['fl_mvdr'][i].mean():.2f}", f"{A['fl_gsc'][i].mean():.2f}"])
    md = diff.mean(axis=1)
    fl_m, fl_g = A["fl_mvdr"].mean(axis=1), A["fl_gsc"].mean(axis=1)       # per case, mean over seeds
    rows.append(["MEAN over cases (fluct: MEDIAN)", pm(A["in"].mean(axis=0)), pm(A["mvdr"].mean(axis=0)),
                 pm(A["gsc"].mean(axis=0)), pm(diff.mean(axis=0)), "",
                 f"{np.median(fl_m):.2f}", f"{np.median(fl_g):.2f}"])
    hdr = ["Case", "Input SINR (dB)", "MVDR SINR (dB)", "GSC SINR (dB)", "MVDR-GSC (dB)",
           "seeds MVDR>GSC", "MVDR fluct (dB)", "GSC fluct (dB)"]
    nm, ng, nt = (int((md > TIE_TOL_DB).sum()), int((md < -TIE_TOL_DB).sum()),
                  int((np.abs(md) <= TIE_TOL_DB).sum()))
    iw = int(np.argmin(md))
    L = ["TABLE 1 - STATIC SUITE: steady-state output SINR and SINR fluctuation", "=" * 72, ""]
    L += common_note(mu, lam, cases)
    L += [f"Scored interval: {interval} (second half of each {cases[0]['dur']:.0f} s record; start-up excluded).",
          "Input SINR = desired / (interference + noise) at sensor 0, same interval.",
          "Fluct = std (dB) of the windowed output SINR inside the scored interval (within-run), averaged over seeds;",
          "  a measure of weight/output stability (relevant to the following noise-reduction stage).", "",
          table(hdr, rows), "",
          f"Win / tie / loss for MVDR (mean difference vs +/-{TIE_TOL_DB} dB): "
          f"MVDR wins {nm}, ties {nt}, GSC wins {ng}  (of {len(cases)} cases)",
          f"Mean advantage {md.mean():+.2f} dB; smallest advantage / worst case for MVDR: "
          f"{cases[iw]['name']} ({md[iw]:+.2f} dB)"]
    stable = fl_m <= fl_g + 0.02
    L.append(f"Stability: MVDR fluctuation <= GSC fluctuation (+0.02 dB slack) in {int(stable.sum())}/{len(cases)} cases.")
    for i in np.where(~stable)[0]:
        L.append(f"  Exception {cases[i]['name']}: MVDR {fl_m[i]:.2f} dB vs GSC {fl_g[i]:.2f} dB, at output SINR "
                 f"{A['mvdr'][i].mean():.1f} vs {A['gsc'][i].mean():.1f} dB. Fluct is measured in dB, so at a very high "
                 "SINR a tiny absolute change in the residual reads as a large dB swing; the mean-row uses medians for this reason.")
    L.append("")
    return "\n".join(L), md


def table2_text(cases, A, mu, lam):
    short = [f"D{i + 1}" for i in range(len(cases))]
    diff = A["mvdr"] - A["gsc"]
    rows = [["GSC"] + [pm(A["gsc"][i]) for i in range(len(cases))] + [pm(A["gsc"].mean(axis=0))],
            ["MVDR"] + [pm(A["mvdr"][i]) for i in range(len(cases))] + [pm(A["mvdr"].mean(axis=0))],
            ["MVDR-GSC"] + [pm(diff[i]) for i in range(len(cases))] + [pm(diff.mean(axis=0))],
            ["seeds MVDR>GSC"] + [f"{int((diff[i] > 0).sum())}/{len(EVAL_SEEDS)}" for i in range(len(cases))]
            + [f"{int((diff.mean(axis=0) > 0).sum())}/{len(EVAL_SEEDS)}"]]
    md = diff.mean(axis=1)
    nm, ng, nt = (int((md > TIE_TOL_DB).sum()), int((md < -TIE_TOL_DB).sum()),
                  int((np.abs(md) <= TIE_TOL_DB).sum()))
    iw = int(np.argmin(md))
    L = ["TABLE 2 - DYNAMIC SUITE: mean output SINR (dB) per scenario", "=" * 62, ""]
    L += common_note(mu, lam, cases)
    L += [f"Scored interval: after the first {DYN_EVAL_FRAC:.0%} of the {cases[0]['dur']:.0f} s record "
          "(all scenario changes happen later). Mean SINR = 10log10(sum desired power / sum interference+noise power).",
          "Mean column: average over the 7 scenarios computed per seed, then mean +/- std across seeds.", "",
          table(["Setting"] + short + ["Mean"], rows), ""]
    L += [f"{s} = {c['name']}" for s, c in zip(short, cases)]
    L += ["", f"Win / tie / loss for MVDR (+/-{TIE_TOL_DB} dB): MVDR wins {nm}, ties {nt}, GSC wins {ng} "
              f"(of {len(cases)} scenarios)",
          f"Mean advantage {md.mean():+.2f} dB; worst scenario for MVDR: {cases[iw]['name']} ({md[iw]:+.2f} dB)", ""]
    return "\n".join(L), md


# --------------------------- step 3: the one figure ---------------------------
def event_times(c):
    ev = []
    for it in c["interferers"]:
        ev += [f * c["dur"] for f, _ in it["power_profile"] if f > 0]
        if "sweep" in it:
            ev += [it["sweep"][2] * c["dur"], it["sweep"][3] * c["dur"]]
    return sorted(set(ev))


def make_figure(all_cases, R, mu, lam, md_all, path):
    names = [c["name"] for c in all_cases]
    idx = {n: i for i, n in enumerate(names)}
    static_idx = [i for i, c in enumerate(all_cases) if c["suite"] == "static"]
    med = float(np.median(md_all[static_idx]))
    picks = []
    typ = min(static_idx, key=lambda i: abs(md_all[i] - med))
    picks.append((typ, "typical static case (advantage nearest the static median)"))
    for key, why in (("Dyn2", "dynamic power step"), ("Dyn3", "dynamic angle sweep")):
        i = next(i for i, n in enumerate(names) if n.startswith(key))
        picks.append((i, why))
    shown = {i for i, _ in picks}
    worst = next(i for i in np.argsort(md_all) if i not in shown)
    picks.append((int(worst), "worst case for MVDR among the cases not already shown"))

    fig, axes = plt.subplots(len(picks), 1, figsize=(10, 3.1 * len(picks)), squeeze=False)
    for ax, (i, why) in zip(axes[:, 0], picks):
        c = all_cases[i]
        runs = R[c["name"]]
        win = int(WINDOW_S * c["fs"])
        n_win = runs[0]["in"][0].size
        t = (np.arange(n_win) + 0.5) * win / c["fs"]
        for key, lab, col in ((("mvdr", lam), f"MVDR (lambda={lam:g})", "tab:blue"),
                              (("gsc", mu), f"GSC (mu={mu:g})", "tab:orange")):
            cur = np.array([sinr_curve_db(*r[key]) for r in runs])
            m, s = cur.mean(axis=0), cur.std(axis=0, ddof=1)
            ax.plot(t, m, color=col, lw=1.5, label=lab)
            ax.fill_between(t, m - s, m + s, color=col, alpha=0.2, lw=0)
        ax.axvspan(0, c["eval_frac"] * c["dur"], color="gray", alpha=0.12, label="not scored")
        for k, te in enumerate(event_times(c)):
            ax.axvline(te, color="k", ls=":", lw=1, label="scenario change" if k == 0 else None)
        ax.set_title(f"{c['name']}  [{why}]", fontsize=9)
        ax.set_ylabel("Output SINR (dB)")
        ax.set_xlim(0, c["dur"])
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="best", ncol=2)
    for ax in axes[:, 0]:
        ax.set_xlabel("Time (s)")
    fig.suptitle(f"Output SINR vs time, mean +/- std over {len(EVAL_SEEDS)} seeds "
                 f"(one fixed setting: mu={mu:g}, lambda={lam:g})", fontsize=10)
    plt.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(path, dpi=130)


# --------------------------- main ---------------------------
def main():
    t0 = time.time()
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    if not HAVE_NUMBA:
        print("NOTE: numba not found - NLMS runs in plain numpy and will be slow. `pip install numba` is advised.")
    st = [static_case(c) for c in TEST_CASES]
    dy = [dynamic_case(c) for c in DYNAMIC_CASES]

    print("Step 1/3: choosing one (mu, lambda) on tuning seeds")
    mu, lam, rows = tune(st + dy)
    with open(os.path.join(OUTPUT_DIR, "tuning_selection.txt"), "w") as fh:
        fh.write(tuning_text(mu, lam, rows, (len(st), len(dy))))
    print(f"  selected mu={mu:g}, lambda={lam:g}")

    print("Step 2/3: Monte Carlo evaluation")
    R = evaluate(st + dy, mu, lam)
    A_st, A_dy = suite_arrays(st, R, mu, lam), suite_arrays(dy, R, mu, lam)
    t1, md_st = table1_text(st, A_st, mu, lam)
    t2, md_dy = table2_text(dy, A_dy, mu, lam)
    with open(os.path.join(OUTPUT_DIR, "table1_static.txt"), "w") as fh:
        fh.write(t1)
    with open(os.path.join(OUTPUT_DIR, "table2_dynamic.txt"), "w") as fh:
        fh.write(t2)
    print("\n" + t1 + "\n" + t2)

    print("Step 3/3: figure")
    make_figure(st + dy, R, mu, lam, np.concatenate([md_st, md_dy]),
                os.path.join(OUTPUT_DIR, "sinr_over_time.png"))
    print(f"Done in {time.time() - t0:.0f} s. Results in ./{OUTPUT_DIR}/")
    # if SHOW_PLOTS:
    plt.show()


if __name__ == "__main__":
    main()
