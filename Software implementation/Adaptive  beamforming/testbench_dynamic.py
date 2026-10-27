# AI Generated Testbench: cases have been verified by the team
# this code deals with the dynamic cases where teh signal environment changes within the computing time-frame itself. 
# It builds on the static test-case file : testbench.py

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

    # Dyn4: a null that is no longer needed disappears, then the interferer returns.
    # Tests how each algorithm handles a vanished constraint and re-acquires it.
    _cfg("Dyn4 interferer drops out, then returns",
         [itf(40, [(0.0, 10.0), (0.4, 0.0), (0.7, 10.0)]),
          itf(-30, [(0.0, 10.0)])]),

    # # Dyn5: interferers arrive one after another until all 3 degrees of freedom
    # # (M-1 = 3) are used. Tests the limit of the array while the scene is changing.
    # _cfg("Dyn5 three interferers arrive in turn",
    #      [itf(40, [(0.0, 0.0), (0.3, 10.0)]),
    #       itf(-30, [(0.0, 0.0), (0.5, 10.0)]),
    #       itf(65, [(0.0, 0.0), (0.7, 10.0)])]),

    _cfg("Dyn5 three interferers arrive in quick succession",
     [itf(40, [(0.0, 0.0), (0.3, 10.0)]),
      itf(-30, [(0.0, 0.0), (0.35, 10.0)]),
      itf(65, [(0.0, 0.0), (0.4, 10.0)])]),

    # Dyn6: the interferer is replaced by one from a very different direction.
    # The old null must be released and a new one formed; tests re-nulling speed.
    _cfg("Dyn6 interferer hops 40 to -50 deg",
         [itf(40, [(0.0, 10.0), (0.5, 0.0)]),
          itf(-50, [(0.0, 0.0), (0.5, 10.0)])]),

    # Dyn7: same motion as Dyn3 but 5x faster (0.4 -> 0.6 of the record instead of
    # 0.3 -> 0.9). Separates tracking speed from steady-state quality.
    _cfg("Dyn7 interferer sweeps 60 to 20 deg, fast",
         [itf(60, [(0.0, 10.0)], sweep=(60, 20, 0.4, 0.6)),
          itf(-30, [(0.0, 10.0)])]),

    _cfg("Dyn8 interferer approaches desired direction",
     [itf(40, [(0.0, 10.0)],
          sweep=(40, 5, 0.3, 0.9)),
      itf(-30, [(0.0, 10.0)])]),
]