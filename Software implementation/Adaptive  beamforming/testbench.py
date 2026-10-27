# AI Generated Testbench : Testcases verified by the team
# 15 test cases consisting of edge or hard to deal with cases also. 

BASE = {
    "num_antennas": 4,
    "desired_angle_deg": 30.0,
    "desired_power": 1.0,
    "noise_sigma": 0.35,
    "duration_s": 3.0,
    "fs_hz": 40000,
    "mvdr_loading": 1e-3,
    "nlms_mu": 1e-4,
    "seed": 123,
}

TEST_CASES = [
    # 1. No interference + broadside desired signal
    {
        "name": "T01_no_interference_broadside",
        "desired_angle_deg": 0.0,
        "interferer_angles_deg": [],
        "interferer_powers": [],
    },

    # 2. Ordinary separated interference
    {
        "name": "T02_two_separated",
        "interferer_angles_deg": [-40, 75],
        "interferer_powers": [1.0, 1.0],
    },

    # 3. Three interferers
    {
        "name": "T03_three_interferers",
        "interferer_angles_deg": [-60, 0, 80],
        "interferer_powers": [1.0, 0.8, 1.2],
    },

    # 4. Four interferers: spatial-capacity stress
    {
        "name": "T04_four_interferers",
        "interferer_angles_deg": [-70, -20, 60, 100],
        "interferer_powers": [1.0, 1.0, 1.0, 1.0],
    },

    # 5. More interferers than antennas
    {
        "name": "T05_six_interferers",
        "interferer_angles_deg": [-80, -50, -10, 50, 85, 120],
        "interferer_powers": [1.0] * 6,
    },

    # 6. Interferer close to desired direction
    {
        "name": "T06_near_desired",
        "interferer_angles_deg": [25],
        "interferer_powers": [1.0],
    },

    # 7. Desired and interfering signals share a direction
    {
        "name": "T07_same_direction",
        "interferer_angles_deg": [30],
        "interferer_powers": [1.0],
    },

    # 8. Duplicate interferer directions
    {
        "name": "T08_duplicate_directions",
        "interferer_angles_deg": [-40, -40],
        "interferer_powers": [1.0, 2.0],
    },

    # 9. Angle wraparound boundary
    {
        "name": "T09_angle_boundary",
        "interferer_angles_deg": [-179, 179],
        "interferer_powers": [1.0, 1.0],
    },

    # 10. One very strong interferer
    {
        "name": "T10_strong_interferer",
        "interferer_angles_deg": [-50, 65, 100],
        "interferer_powers": [100.0, 0.1, 0.1],
    },

    # 11. Weak interference + low noise (NLMS mu and MVDR loading stay at BASE)
    {
        "name": "T11_weak_low_noise",
        "interferer_angles_deg": [-50, 65],
        "interferer_powers": [0.01, 0.01],
        "noise_sigma": 0.01,
    },

    # 12. High noise (NLMS mu and MVDR loading stay at BASE)
    {
        "name": "T12_high_noise",
        "interferer_angles_deg": [-45, 70],
        "interferer_powers": [1.0, 1.0],
        "noise_sigma": 1.0,
    },

    # 13. Near-endfire desired direction
    {
        "name": "T13_near_endfire",
        "desired_angle_deg": 85.0,
        "interferer_angles_deg": [-45, 30],
        "interferer_powers": [1.0, 1.0],
    },

    # 14. Symmetric interference directions
    {
        "name": "T14_symmetric_interference",
        "interferer_angles_deg": [-30, 90],
        "interferer_powers": [1.0, 1.0],
    },

    # 15. Unequal powers and multiple spatial directions
    {
        "name": "T15_mixed_power_interference",
        "interferer_angles_deg": [-70, -10, 55, 110],
        "interferer_powers": [5.0, 0.2, 2.0, 0.05],
    },
]


def make_config(case):
    """Merge one test case with the default configuration."""
    config = BASE.copy()
    config.update(case)

    angles = config["interferer_angles_deg"]
    powers = config["interferer_powers"]

    if len(angles) != len(powers):
        raise ValueError(
            f"{config['name']}: each interferer needs one power"
        )

    if config["num_antennas"] < 2:
        raise ValueError("Use at least two antennas")

    if config["noise_sigma"] < 0:
        raise ValueError("noise_sigma cannot be negative")

    if any(p < 0 for p in powers):
        raise ValueError("Interferer powers cannot be negative")

    if config["desired_power"] < 0:
        raise ValueError("desired_power cannot be negative")

    if not 0 < config["nlms_mu"] < 2:
        raise ValueError("nlms_mu must be between 0 and 2")

    if config["mvdr_loading"] < 0:
        raise ValueError("mvdr_loading cannot be negative")

    if config["duration_s"] <= 0 or config["fs_hz"] <= 0:
        raise ValueError("duration_s and fs_hz must be positive")

    return config


if __name__ == "__main__":
    print(f"Defined {len(TEST_CASES)} test cases")

    for case in TEST_CASES:
        cfg = make_config(case)
        print(
            f"{cfg['name']:<36} "
            f"desired={cfg['desired_angle_deg']:>6.1f} deg, "
            f"interferers={len(cfg['interferer_angles_deg'])}, "
            f"noise={cfg['noise_sigma']}, "
            f"NLMS mu={cfg['nlms_mu']}, "
            f"MVDR loading={cfg['mvdr_loading']}"
        )