import csv
from collections import defaultdict

import numpy as np
from scipy.signal import stft, istft, butter, filtfilt, square
from scipy.special import ive, exp1

# =========================================================
# 0. SETTINGS
# =========================================================
FS = 16000
NPERSEG = 512
NOVERLAP = 384
N_INIT_FRAMES = 20          # noise-only frames for the noise estimate (lead-in has ~33)
LEAD_SILENCE_S = 0.3

N_SEEDS = 5
RUN_TUNING = True
TUNING_SEEDS = 2            # tuning uses separate seeds (100, 101, ...)
LSD_FLOOR_DB = 60

DEFAULT_PARAMS = {"sub_alpha": 2.0, "sub_beta": 0.01, "gate_k": 1.5, "gate_floor": 0.1}

SIGNALS = ["single_tone", "multi_tone", "chirp", "am_modulated", "square_wave"]
NOISES = ["white", "pink", "narrowband"]
SNRS = [-5, 0, 5, 10]
METHODS = ["Spectral Subtraction", "Spectral Gating", "Wiener", "MMSE-STSA", "log-MMSE"]
KEY3 = ["Wiener", "MMSE-STSA", "log-MMSE"]

METRICS = ["snr_gain_db", "segsnr_db", "sdr_db", "lsd_db", "oracle_gain_rmse"]
LOWER_IS_BETTER = {"lsd_db", "oracle_gain_rmse"}


# =========================================================
# 1. SIGNALS AND NOISE
# =========================================================
def make_signal(signal_type, duration=3.0):
    t = np.arange(int(duration * FS)) / FS
    if signal_type == "single_tone":
        x = np.sin(2 * np.pi * 1000 * t)
    elif signal_type == "multi_tone":
        x = sum(np.sin(2 * np.pi * f * t) for f in (300, 1000, 2500, 4000))
    elif signal_type == "chirp":
        f0, f1 = 200, 6000
        k = (f1 - f0) / duration
        x = np.sin(2 * np.pi * (f0 * t + 0.5 * k * t ** 2))
    elif signal_type == "am_modulated":
        x = np.sin(2 * np.pi * 1500 * t) * 0.5 * (1 + np.sin(2 * np.pi * 3 * t))
    elif signal_type == "square_wave":
        x = square(2 * np.pi * 400 * t)
    else:
        raise ValueError(f"unknown signal_type: {signal_type}")
    x = x / np.max(np.abs(x)) * 0.5
    return np.concatenate([np.zeros(int(LEAD_SILENCE_S * FS)), x])


def add_noise(clean, snr_db, noise_type, seed):
    rng = np.random.default_rng(seed)
    if noise_type == "white":
        noise = rng.standard_normal(len(clean))
    elif noise_type == "pink":
        X = np.fft.rfft(rng.standard_normal(len(clean)))
        noise = np.fft.irfft(X / np.sqrt(np.arange(1, len(X) + 1)), n=len(clean))
    elif noise_type == "narrowband":
        b, a = butter(4, [1100 / (FS / 2), 1300 / (FS / 2)], btype="band")
        noise = filtfilt(b, a, rng.standard_normal(len(clean)))
    else:
        raise ValueError(f"unknown noise_type: {noise_type}")
    noise = noise * np.sqrt(np.mean(clean ** 2) / (10 ** (snr_db / 10)) / np.mean(noise ** 2))
    return clean + noise, noise


# =========================================================
# 2. SUPPRESSION RULES
# =========================================================
def wiener_gain(xi, gamma):
    return xi / (1 + xi)


def mmse_gain(xi, gamma):
    v = np.maximum((xi / (1 + xi)) * gamma, 1e-6)
    g = (np.sqrt(np.pi * v) / (2 * gamma)) * ((1 + v) * ive(0, v / 2) + v * ive(1, v / 2))
    return np.clip(g, 0, 1)


def logmmse_gain(xi, gamma):
    v = np.maximum((xi / (1 + xi)) * gamma, 1e-6)
    return np.clip((xi / (1 + xi)) * np.exp(0.5 * exp1(v)), 0, 1)


XI_GAMMA = {"Wiener": wiener_gain, "MMSE-STSA": mmse_gain, "log-MMSE": logmmse_gain}


def enhance(noisy, method, params=None, dd_alpha=0.98):
    """Returns (enhanced_signal, applied_gain[bins, frames])."""
    p = dict(DEFAULT_PARAMS, **(params or {}))
    eps = 1e-12
    _, _, Y = stft(noisy, fs=FS, nperseg=NPERSEG, noverlap=NOVERLAP, window="hann")
    mag = np.abs(Y)
    init = mag[:, :N_INIT_FRAMES]
    noise_psd = np.mean(init ** 2, axis=1, keepdims=True)

    if method == "Spectral Subtraction":
        power = mag ** 2
        clean_pow = np.maximum(power - p["sub_alpha"] * noise_psd, p["sub_beta"] * power)
        G = np.sqrt(clean_pow / (power + eps))

    elif method == "Spectral Gating":
        init_db = 20 * np.log10(init + eps)
        thr = np.mean(init_db, axis=1, keepdims=True) + p["gate_k"] * np.std(init_db, axis=1, keepdims=True)
        G = np.where(20 * np.log10(mag + eps) > thr, 1.0, p["gate_floor"])

    elif method in XI_GAMMA:
        gain_fn = XI_GAMMA[method]
        n_bins, n_frames = Y.shape
        G = np.zeros((n_bins, n_frames))
        A_prev = np.zeros((n_bins, 1))
        for n in range(n_frames):
            Yk = Y[:, n:n + 1]
            gamma = (np.abs(Yk) ** 2) / (noise_psd + eps)
            xi = dd_alpha * (A_prev ** 2) / (noise_psd + eps) + (1 - dd_alpha) * np.maximum(gamma - 1, 0)
            xi = np.maximum(xi, 1e-6)
            g = gain_fn(xi, gamma)
            G[:, n:n + 1] = g
            A_prev = np.abs(g * Yk)
    else:
        raise ValueError(f"unknown method: {method}")

    _, x_hat = istft(G * Y, fs=FS, nperseg=NPERSEG, noverlap=NOVERLAP, window="hann")
    x_hat = x_hat[:len(noisy)]
    if len(x_hat) < len(noisy):
        x_hat = np.pad(x_hat, (0, len(noisy) - len(x_hat)))
    return x_hat, G


# =========================================================
# 3. METRICS
# =========================================================
def snr_db(clean, test):
    return 10 * np.log10(np.sum(clean ** 2) / (np.sum((clean - test) ** 2) + 1e-12))


def segmental_snr(clean, test, frame_len=0.02):
    fl = int(frame_len * FS)
    vals = []
    for i in range(0, len(clean) - fl, fl):
        c, e = clean[i:i + fl], test[i:i + fl]
        if np.sum(c ** 2) < 1e-8:
            continue
        vals.append(np.clip(10 * np.log10(np.sum(c ** 2) / (np.sum((c - e) ** 2) + 1e-12)), -10, 35))
    return float(np.mean(vals)) if vals else float("nan")


def sdr_db(clean, test):
    a = np.dot(test, clean) / (np.dot(clean, clean) + 1e-12)
    target = a * clean
    return 10 * np.log10(np.sum(target ** 2) / (np.sum((test - target) ** 2) + 1e-12))


def log_spectral_distance(clean, test, floor_db=LSD_FLOOR_DB):
    """Both spectra are floored at `floor_db` below the clean peak and silent
    frames are skipped; without this, near-zero bins become -200 dB and the
    number is meaningless for narrowband signals."""
    _, _, C = stft(clean, fs=FS, nperseg=NPERSEG, noverlap=NOVERLAP)
    _, _, E = stft(test, fs=FS, nperseg=NPERSEG, noverlap=NOVERLAP)
    Cm, Em = np.abs(C), np.abs(E)
    floor = Cm.max() * 10 ** (-floor_db / 20)
    diff = 20 * np.log10(np.maximum(Cm, floor)) - 20 * np.log10(np.maximum(Em, floor))
    frame_pow = (Cm ** 2).sum(axis=0)
    active = frame_pow > 1e-4 * frame_pow.max()
    return float(np.mean(np.sqrt(np.mean(diff[:, active] ** 2, axis=0))))


def oracle_gain_rmse(clean, noise, G_applied):
    """Distance between the applied gain and the ideal gain (possible only
    because clean signal and noise are known separately)."""
    _, _, C = stft(clean, fs=FS, nperseg=NPERSEG, noverlap=NOVERLAP)
    _, _, D = stft(noise, fs=FS, nperseg=NPERSEG, noverlap=NOVERLAP)
    nf = G_applied.shape[1]
    C, D = np.abs(C[:, :nf]) ** 2, np.abs(D[:, :nf]) ** 2
    return float(np.sqrt(np.mean((G_applied - C / (C + D + 1e-12)) ** 2)))


def all_metrics(clean, noisy, enhanced, true_noise=None, G=None):
    out = {
        "snr_gain_db": snr_db(clean, enhanced) - snr_db(clean, noisy),
        "segsnr_db": segmental_snr(clean, enhanced),
        "sdr_db": sdr_db(clean, enhanced),
        "lsd_db": log_spectral_distance(clean, enhanced),
        "oracle_gain_rmse": oracle_gain_rmse(clean, true_noise, G) if true_noise is not None else float("nan"),
    }
    return {k: round(float(v), 4) for k, v in out.items()}


def run_on_arrays(clean, noisy, true_noise=None, methods=METHODS, params=None):
    """For the REAL pipeline: pass the beamformer output as `noisy` and the
    ideal/clean reference as `clean`. Returns {method: metrics}."""
    res = {}
    for m in methods:
        enhanced, G = enhance(noisy, m, params)
        res[m] = all_metrics(clean, noisy, enhanced, true_noise, G)
    return res


# =========================================================
# 4. SWEEP
# =========================================================
def run_sweep(n_seeds=N_SEEDS, methods=METHODS, params=None, verbose=True, seed_offset=0):
    rows = []
    total = len(SIGNALS) * len(NOISES) * len(SNRS) * n_seeds * len(methods)
    done = 0
    for sig in SIGNALS:
        clean = make_signal(sig)
        for noise_type in NOISES:
            for snr in SNRS:
                for seed in range(seed_offset, seed_offset + n_seeds):
                    noisy, true_noise = add_noise(clean, snr, noise_type, seed)
                    for m in methods:
                        enhanced, G = enhance(noisy, m, params)
                        row = {"signal_type": sig, "noise_type": noise_type,
                               "input_snr_db": snr, "seed": seed, "method": m}
                        row.update(all_metrics(clean, noisy, enhanced, true_noise, G))
                        rows.append(row)
                        done += 1
                        if verbose and done % 50 == 0:
                            print(f"\r  {done}/{total} runs", end="", flush=True)
    if verbose:
        print(f"\r  {total}/{total} runs")
    return rows


# =========================================================
# 5. TABLES
# =========================================================
def print_table(table, headers):
    w = {h: max(len(str(h)), max(len(str(r[h])) for r in table)) for h in headers}
    print(" | ".join(str(h).ljust(w[h]) for h in headers))
    print("-" * (sum(w.values()) + 3 * len(headers)))
    for r in table:
        print(" | ".join(str(r[h]).ljust(w[h]) for h in headers))


def save_csv(table, path):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)
    print(f"  saved {path} ({len(table)} rows)")


def pm(vals):
    return f"{np.mean(vals):.2f} ± {np.std(vals):.2f}"


def leaderboard(rows, methods=METHODS, title="LEADERBOARD -- mean over all test cases"):
    print("\n" + "=" * 100)
    print(title)
    print("=" * 100)
    table = []
    for m in methods:
        sub = [r for r in rows if r["method"] == m]
        row = {"method": m}
        for k in METRICS:
            row[k] = round(float(np.nanmean([r[k] for r in sub])), 3)
        table.append(row)
    print_table(table, ["method"] + METRICS)
    return table


def method_by_group(rows, group_field, metric):
    """All 5 methods side by side for each value of `group_field`."""
    hint = "lower = better" if metric in LOWER_IS_BETTER else "higher = better"
    print("\n" + "=" * 100)
    print(f"ALL METHODS BY {group_field.upper()} -- {metric} ({hint})")
    print("=" * 100)
    keys = sorted({r[group_field] for r in rows})
    table = []
    for k in keys:
        row = {group_field: k}
        for m in METHODS:
            row[m] = round(float(np.mean([r[metric] for r in rows
                                          if r[group_field] == k and r["method"] == m])), 2)
        table.append(row)
    print_table(table, [group_field] + METHODS)
    return table


def gap_table(rows, group_fields, metric):
    """Wiener vs MMSE-STSA and Wiener vs log-MMSE, paired per seed, shown as
    mean ± std over seeds. POSITIVE gap = Wiener better, for every metric."""
    lower = metric in LOWER_IS_BETTER
    sgn = -1 if lower else 1
    buckets = defaultdict(lambda: defaultdict(list))
    for r in rows:
        if r["method"] in KEY3:
            buckets[(tuple(r[f] for f in group_fields), r["seed"])][r["method"]].append(r[metric])
    per_group = defaultdict(list)
    for (g, _), d in buckets.items():
        mn = {m: float(np.mean(d[m])) for m in KEY3}
        per_group[g].append((mn["Wiener"], mn["MMSE-STSA"], mn["log-MMSE"],
                             sgn * (mn["Wiener"] - mn["MMSE-STSA"]),
                             sgn * (mn["Wiener"] - mn["log-MMSE"])))
    out = []
    for g, vals in per_group.items():
        a = np.array(vals)
        row = dict(zip(group_fields, g))
        row.update({"Wiener": round(a[:, 0].mean(), 2), "MMSE-STSA": round(a[:, 1].mean(), 2),
                    "log-MMSE": round(a[:, 2].mean(), 2),
                    "gap W-MMSE": pm(a[:, 3]), "gap W-logMMSE": pm(a[:, 4])})
        out.append(row)
    return out


def show_gap_table(title, rows, group_fields, metric, sort_key=None):
    hint = "lower = better" if metric in LOWER_IS_BETTER else "higher = better"
    print("\n" + "=" * 100)
    print(f"{title} -- {metric} ({hint}); gap > 0 means Wiener is better")
    print("=" * 100)
    t = gap_table(rows, group_fields, metric)
    if sort_key:
        t.sort(key=sort_key)
    print_table(t, list(group_fields) + ["Wiener", "MMSE-STSA", "log-MMSE", "gap W-MMSE", "gap W-logMMSE"])
    return t


# =========================================================
# 6. PARAMETER SWEEP FOR THE TWO SIMPLE METHODS
# =========================================================
def tune_baselines(seeds=TUNING_SEEDS, seed_offset=100):
    """Grid search on SEPARATE seeds so tuned settings are not scored on the
    data they were picked on. Best setting = highest mean SNR gain."""
    print("\n" + "=" * 100)
    print("PARAMETER SWEEP -- Spectral Subtraction / Spectral Gating (separate tuning seeds)")
    print("=" * 100)
    grids = {
        "Spectral Subtraction": [{"sub_alpha": a, "sub_beta": b}
                                 for a in (2.0, 4.0, 6.0, 8.0, 12.0) for b in (0.001, 0.01, 0.05)],
        "Spectral Gating": [{"gate_k": k, "gate_floor": fl}
                            for k in (1.5, 2.5, 3.5, 5.0) for fl in (0.005, 0.02, 0.05, 0.1)],
    }
    best, table = {}, []
    for method, grid in grids.items():
        res = []
        for prm in grid:
            rws = run_sweep(seeds, methods=[method], params=prm, verbose=False, seed_offset=seed_offset)
            res.append({"method": method, "params": str(prm), "_p": prm,
                        "snr_gain_db": round(float(np.mean([r["snr_gain_db"] for r in rws])), 3),
                        "sdr_db": round(float(np.mean([r["sdr_db"] for r in rws])), 3)})
        res.sort(key=lambda t: -t["snr_gain_db"])
        best[method] = res[0]["_p"]
        print(f"\n{method}: top 3 settings")
        print_table(res[:3], ["method", "params", "snr_gain_db", "sdr_db"])
        table += [{k: v for k, v in t.items() if k != "_p"} for t in res]
    ref = run_sweep(seeds, methods=["Wiener"], verbose=False, seed_offset=seed_offset)
    print(f"\nWiener (default) on the same tuning runs: "
          f"{np.mean([r['snr_gain_db'] for r in ref]):.3f} dB mean SNR gain")
    print("NOTE: if a best setting sits on the EDGE of its grid, extend the grid and re-run.")
    save_csv(table, "tuning_results.csv")
    return best


def tuned_comparison(rows, best):
    tuned_rows = []
    for m, prm in best.items():
        rws = run_sweep(N_SEEDS, methods=[m], params=prm, verbose=False)
        for r in rws:
            r["method"] = f"{m} (tuned)"
        tuned_rows += rws
    names = ["Wiener", "Spectral Subtraction", "Spectral Subtraction (tuned)",
             "Spectral Gating", "Spectral Gating (tuned)"]
    t = leaderboard(rows + tuned_rows, methods=names,
                    title="DEFAULT vs TUNED simple methods (tuned on separate seeds), vs Wiener")
    save_csv(t, "leaderboard_tuned.csv")


# =========================================================
# 7. MAIN
# =========================================================
if __name__ == "__main__":
    print(f"Narrowband sweep: {len(SIGNALS)} signals x {len(NOISES)} noises x {len(SNRS)} SNRs "
          f"x {N_SEEDS} seeds x {len(METHODS)} methods")
    rows = run_sweep()
    save_csv(rows, "results_raw.csv")

    save_csv(leaderboard(rows), "leaderboard.csv")

    for metric in ("snr_gain_db", "segsnr_db", "lsd_db", "oracle_gain_rmse"):
        save_csv(method_by_group(rows, "signal_type", metric), f"methods_by_signal_{metric}.csv")

    for metric in ("snr_gain_db", "oracle_gain_rmse"):
        save_csv(show_gap_table("WIENER vs MMSE FAMILY, PER SIGNAL", rows, ("signal_type",), metric),
                 f"gap_by_signal_{metric}.csv")
        save_csv(show_gap_table("WIENER vs MMSE FAMILY, BY INPUT SNR", rows, ("input_snr_db",), metric,
                                sort_key=lambda r: r["input_snr_db"]),
                 f"gap_by_snr_{metric}.csv")
        save_csv(show_gap_table("WIENER vs MMSE FAMILY, BY NOISE TYPE", rows, ("noise_type",), metric),
                 f"gap_by_noise_{metric}.csv")

    if RUN_TUNING:
        best = tune_baselines()
        tuned_comparison(rows, best)
