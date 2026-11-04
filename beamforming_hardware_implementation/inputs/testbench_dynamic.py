"""Time-varying scenarios for the MVDR vs GSC comparison (additive to testbench.py).

Imports only BASE from testbench.py (needs num_antennas, fs_hz, seed, noise_sigma,
nlms_mu, mvdr_loading; add any your BASE lacks).

Each interferer:
  angle_deg      : direction (start direction if it moves)
  power_profile  : [(time_fraction, power), ...], piecewise constant, must start at 0.0.
                   power 0 = switched off. Times are fractions of the record.
  sweep          : optional (angle_start, angle_end, frac_start, frac_end), linear motion.
All changes happen after 30% of the record so both algorithms converge first.
"""
from testbench import BASE

DYN_DURATION_S = 6.0


def _cfg(name, interferers, desired_angle_deg=0.0, desired_power=1.0):
    cfg = dict(BASE)
    cfg.update(name=name, duration_s=DYN_DURATION_S, desired_angle_deg=desired_angle_deg,
               desired_power=desired_power, interferers=interferers)
    return cfg


def itf(angle, profile, sweep=None):
    d = {"angle_deg": float(angle), "power_profile": list(profile)}
    if sweep is not None:
        d["sweep"] = tuple(sweep)
    return d


DYNAMIC_CASES = [
    _cfg("Dyn1 second interferer switches on",
         [itf(40, [(0.0, 10.0)]),
          itf(-30, [(0.0, 0.0), (0.5, 10.0)])]),

    _cfg("Dyn2 interferer power jumps +20 dB",
         [itf(40, [(0.0, 1.0), (0.5, 100.0)]),
          itf(-30, [(0.0, 10.0)])]),

    _cfg("Dyn3 interferer sweeps 60 to 20 deg",
         [itf(60, [(0.0, 10.0)], sweep=(60, 20, 0.3, 0.9)),
          itf(-30, [(0.0, 10.0)])]),
]