#!/usr/bin/env python3
"""Offline longitudinal-control analysis for Honghu Wing V8 ULogs.

The tool separates the three cascaded loops used by PX4 fixed-wing control:

* TECS pitch setpoint -> measured pitch;
* pitch-rate setpoint -> measured body pitch rate;
* normalized pitch torque / actual elevator -> measured pitch rate.

It deliberately uses only logged signals.  It does not modify PX4 parameters,
missions, models, or ULogs.  Welch spectra are implemented with NumPy so the
analysis remains usable in the project's minimal WSL Python environment.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog


DEFAULT_BASELINE = Path(
    "/home/fly/PX4-Autopilot-NewAero-flight-data/baselines/"
    "2026-08-04-pre-cleanup/ulog/NewAero-main/2026-08-03/07_37_42.ulg"
)

TOPICS = (
    "tecs_status",
    "vehicle_attitude_groundtruth",
    "vehicle_angular_velocity_groundtruth",
    "vehicle_rates_setpoint",
    "rate_ctrl_status",
    "vehicle_torque_setpoint",
    "vehicle_local_position_groundtruth",
    "vehicle_global_position",
    "airspeed_validated",
    "honghu_v8_aero_state",
    "mission_result",
)

PARAMETERS = (
    "SYS_AUTOSTART",
    "FW_PR_P",
    "FW_PR_I",
    "FW_PR_D",
    "FW_PR_FF",
    "FW_PR_IMAX",
    "FW_DTRIM_P_VMIN",
    "FW_DTRIM_P_VMAX",
    "FW_P_TC",
    "FW_T_I_GAIN_PIT",
    "FW_T_PTCH_DAMP",
    "FW_T_ALT_TC",
    "FW_T_SEB_R_FF",
    "FW_T_RLL2THR",
    "FW_R_LIM",
    "TRIM_PITCH",
)


@dataclass
class Spectrum:
    frequency_hz: np.ndarray
    pxx: np.ndarray
    pyy: np.ndarray
    pxy: np.ndarray


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def interpolate(dataset, field: str, time_s: np.ndarray) -> np.ndarray:
    timestamp = np.asarray(dataset.data["timestamp"], dtype=float) * 1e-6
    values = np.asarray(dataset.data[field], dtype=float)
    valid = np.isfinite(timestamp) & np.isfinite(values)
    if np.count_nonzero(valid) < 2:
        return np.full_like(time_s, np.nan)
    return np.interp(time_s, timestamp[valid], values[valid])


def interpolate_step(dataset, field: str, time_s: np.ndarray) -> np.ndarray:
    """Zero-order hold interpolation for discrete state variables."""
    timestamp = np.asarray(dataset.data["timestamp"], dtype=float) * 1e-6
    values = np.asarray(dataset.data[field], dtype=float)
    valid = np.isfinite(timestamp) & np.isfinite(values)
    if np.count_nonzero(valid) < 1:
        return np.full_like(time_s, np.nan)
    timestamp = timestamp[valid]
    values = values[valid]
    indices = np.searchsorted(timestamp, time_s, side="right") - 1
    indices = np.clip(indices, 0, len(values) - 1)
    return values[indices]


def quaternion_to_roll_pitch(dataset) -> tuple[np.ndarray, np.ndarray]:
    w, x, y, z = (
        np.asarray(dataset.data[f"q[{index}]"], dtype=float)
        for index in range(4)
    )
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return roll, pitch


def moving_mean_high_pass(values: np.ndarray, samples: int) -> np.ndarray:
    samples = max(3, int(samples))
    kernel = np.ones(samples, dtype=float) / samples
    left = samples // 2
    padded = np.pad(values, (left, samples - 1 - left), mode="edge")
    return values - np.convolve(padded, kernel, mode="valid")


def detrend_linear(values: np.ndarray) -> np.ndarray:
    index = np.arange(len(values), dtype=float)
    design = np.column_stack((index, np.ones_like(index)))
    return values - design @ np.linalg.lstsq(design, values, rcond=None)[0]


def welch_cross_spectrum(
    input_values: np.ndarray,
    output_values: np.ndarray,
    sample_rate_hz: float,
    requested_segment_samples: int = 2048,
) -> Spectrum:
    segment_samples = min(requested_segment_samples, len(input_values))
    segment_samples = 2 ** int(math.floor(math.log2(segment_samples)))
    if segment_samples < 64:
        raise ValueError("not enough valid samples for frequency analysis")
    step = segment_samples // 2
    window = np.hanning(segment_samples)
    pxx = np.zeros(segment_samples // 2 + 1)
    pyy = np.zeros_like(pxx)
    pxy = np.zeros_like(pxx, dtype=complex)
    segments = 0
    for start in range(0, len(input_values) - segment_samples + 1, step):
        input_segment = detrend_linear(input_values[start:start + segment_samples]) * window
        output_segment = detrend_linear(output_values[start:start + segment_samples]) * window
        input_fft = np.fft.rfft(input_segment)
        output_fft = np.fft.rfft(output_segment)
        pxx += np.abs(input_fft) ** 2
        pyy += np.abs(output_fft) ** 2
        pxy += np.conj(input_fft) * output_fft
        segments += 1
    if segments == 0:
        raise ValueError("no complete Welch segment")
    return Spectrum(
        np.fft.rfftfreq(segment_samples, 1.0 / sample_rate_hz),
        pxx / segments,
        pyy / segments,
        pxy / segments,
    )


def transfer_at(spectrum: Spectrum, target_hz: float) -> dict[str, float]:
    index = int(np.argmin(np.abs(spectrum.frequency_hz - target_hz)))
    transfer = spectrum.pxy[index] / max(spectrum.pxx[index], np.finfo(float).tiny)
    coherence = abs(spectrum.pxy[index]) ** 2 / max(
        spectrum.pxx[index] * spectrum.pyy[index], np.finfo(float).tiny
    )
    return {
        "frequency_hz": float(spectrum.frequency_hz[index]),
        "gain": float(abs(transfer)),
        "phase_deg": float(np.angle(transfer, deg=True)),
        "coherence": float(np.clip(coherence, 0.0, 1.0)),
    }


def contiguous_runs(mask: np.ndarray) -> list[slice]:
    runs: list[slice] = []
    start = None
    for index, enabled in enumerate(mask):
        if enabled and start is None:
            start = index
        elif not enabled and start is not None:
            runs.append(slice(start, index))
            start = None
    if start is not None:
        runs.append(slice(start, len(mask)))
    return runs


def longest_valid_run(mask: np.ndarray) -> slice:
    runs = contiguous_runs(mask)
    if not runs:
        raise ValueError("ULog has no samples satisfying the cruise selection")
    return max(runs, key=lambda item: item.stop - item.start)


def percentile(values: np.ndarray, percentiles: Iterable[float]) -> list[float]:
    finite = values[np.isfinite(values)]
    return [float(value) for value in np.percentile(finite, list(percentiles))]


def worst_window_metrics(values: np.ndarray, sample_rate_hz: float,
                         window_s: float = 30.0) -> dict[str, float]:
    """Quantify local oscillations without dilution by earlier quiet flight."""
    window_samples = min(len(values), max(8, round(window_s * sample_rate_hz)))
    step_samples = max(1, round(sample_rate_hz))
    starts = range(0, len(values) - window_samples + 1, step_samples)
    window_std = []
    window_peak_to_peak = []
    for start in starts:
        segment = values[start:start + window_samples]
        window_std.append(float(np.std(segment)))
        window_peak_to_peak.append(float(np.ptp(segment)))
    if not window_std:
        window_std = [float(np.std(values))]
        window_peak_to_peak = [float(np.ptp(values))]
    first_std = window_std[0]
    last_std = window_std[-1]
    return {
        "window_s": float(window_samples / sample_rate_hz),
        "max_std": float(max(window_std)),
        "last_std": float(last_std),
        "max_peak_to_peak": float(max(window_peak_to_peak)),
        "last_to_first_std_ratio": float(last_std / max(first_std, 1e-12)),
    }


def analyze_log(path: Path, sample_rate_hz: float,
                min_agl_m: float = 60.0, min_mission_seq: int = 3,
                max_mission_seq: int = 15) -> tuple[dict, dict[str, np.ndarray]]:
    ulog = ULog(str(path), message_name_filter_list=list(TOPICS))
    datasets = {name: ulog.get_dataset(name) for name in TOPICS}
    common_topics = (
        "tecs_status",
        "vehicle_attitude_groundtruth",
        "vehicle_angular_velocity_groundtruth",
        "vehicle_rates_setpoint",
        "vehicle_local_position_groundtruth",
    )
    start_s = max(float(datasets[name].data["timestamp"][0]) * 1e-6 for name in common_topics)
    end_s = min(float(datasets[name].data["timestamp"][-1]) * 1e-6 for name in common_topics)
    time_s = np.arange(start_s, end_s, 1.0 / sample_rate_hz)

    mission_sequence = interpolate_step(datasets["mission_result"], "seq_current", time_s)
    truth_z = interpolate(datasets["vehicle_local_position_groundtruth"], "z", time_s)
    tas = interpolate(datasets["airspeed_validated"], "true_airspeed_m_s", time_s)
    valid = (
        (mission_sequence >= min_mission_seq)
        & (mission_sequence <= max_mission_seq)
        & (-truth_z > min_agl_m)
        & (tas > 30.0)
    )
    selected = longest_valid_run(valid)
    time_s = time_s[selected]
    mission_sequence = mission_sequence[selected]
    truth_z = truth_z[selected]
    tas = tas[selected]

    attitude = datasets["vehicle_attitude_groundtruth"]
    attitude_time = np.asarray(attitude.data["timestamp"], dtype=float) * 1e-6
    roll_raw, pitch_raw = quaternion_to_roll_pitch(attitude)
    roll = np.interp(time_s, attitude_time, roll_raw)
    pitch = np.interp(time_s, attitude_time, pitch_raw)

    reference_altitude_m = interpolate(
        datasets["vehicle_local_position_groundtruth"], "ref_alt", time_s
    )
    signals = {
        "time_s": time_s,
        "mission_sequence": mission_sequence,
        "roll_rad": roll,
        "pitch_rad": pitch,
        "tecs_pitch_sp_rad": interpolate(datasets["tecs_status"], "pitch_sp_rad", time_s),
        "pitch_rate_sp_rad_s": interpolate(datasets["vehicle_rates_setpoint"], "pitch", time_s),
        "pitch_rate_rad_s": interpolate(
            datasets["vehicle_angular_velocity_groundtruth"], "xyz[1]", time_s
        ),
        "pitch_torque_sp": interpolate(datasets["vehicle_torque_setpoint"], "xyz[1]", time_s),
        "elevator_deg": interpolate(datasets["honghu_v8_aero_state"], "delta_doc_deg[1]", time_s),
        "alpha_deg": interpolate(datasets["honghu_v8_aero_state"], "alpha_deg", time_s),
        "tas_m_s": tas,
        # tecs_status.altitude_sp is AMSL.  Ground-truth local z is NED and
        # must be combined with the matching local-position reference altitude
        # before computing a height-control error.
        "altitude_msl_m": reference_altitude_m - truth_z,
        "altitude_est_msl_m": interpolate(
            datasets["vehicle_global_position"], "alt", time_s
        ),
        "height_agl_m": -truth_z,
        "height_sp_m": interpolate(datasets["tecs_status"], "altitude_sp", time_s),
        "height_rate_sp_m_s": interpolate(datasets["tecs_status"], "height_rate_setpoint", time_s),
        "height_rate_m_s": -interpolate(
            datasets["vehicle_local_position_groundtruth"], "vz", time_s
        ),
        "pitch_integrator": interpolate(datasets["rate_ctrl_status"], "pitchspeed_integ", time_s),
    }
    # TECS closes its loop around the PX4 altitude estimate.  Keep the Gazebo
    # truth error as a separate estimator/model cross-check, but use the
    # controller's actual feedback signal for tuning metrics.
    signals["height_error_m"] = signals["height_sp_m"] - signals["altitude_est_msl_m"]
    signals["height_truth_error_m"] = signals["height_sp_m"] - signals["altitude_msl_m"]

    high_pass = {
        name: moving_mean_high_pass(values, round(10.0 * sample_rate_hz))
        for name, values in signals.items()
        if name not in ("time_s", "mission_sequence")
    }
    pitch_spectrum = welch_cross_spectrum(
        high_pass["pitch_rad"], high_pass["pitch_rad"], sample_rate_hz
    )
    frequency_mask = (pitch_spectrum.frequency_hz >= 0.08) & (pitch_spectrum.frequency_hz <= 1.0)
    dominant_index = np.flatnonzero(frequency_mask)[
        np.argmax(pitch_spectrum.pxx[frequency_mask])
    ]
    dominant_hz = float(pitch_spectrum.frequency_hz[dominant_index])

    transfer_pairs = {
        "tecs_pitch_to_pitch": ("tecs_pitch_sp_rad", "pitch_rad"),
        "pitch_rate_sp_to_pitch_rate": ("pitch_rate_sp_rad_s", "pitch_rate_rad_s"),
        "pitch_torque_to_pitch_rate": ("pitch_torque_sp", "pitch_rate_rad_s"),
        "elevator_to_pitch_rate": ("elevator_deg", "pitch_rate_rad_s"),
        "height_rate_sp_to_tecs_pitch": ("height_rate_sp_m_s", "tecs_pitch_sp_rad"),
        "pitch_to_height_rate": ("pitch_rad", "height_rate_m_s"),
    }
    transfers = {}
    transfer_spectra = {}
    for name, (input_name, output_name) in transfer_pairs.items():
        spectrum = welch_cross_spectrum(
            high_pass[input_name], high_pass[output_name], sample_rate_hz
        )
        transfer_spectra[name] = spectrum
        transfers[name] = transfer_at(spectrum, dominant_hz)

    pitch_error = signals["tecs_pitch_sp_rad"] - signals["pitch_rad"]
    pitch_rate_error = signals["pitch_rate_sp_rad_s"] - signals["pitch_rate_rad_s"]
    initial_parameters = {
        name: ulog.initial_parameters.get(name) for name in PARAMETERS
    }
    effective_parameters = dict(initial_parameters)
    selection_start_us = int(time_s[0] * 1e6)
    for timestamp_us, name, value in ulog.changed_parameters:
        if timestamp_us <= selection_start_us and name in effective_parameters:
            effective_parameters[name] = value
    report = {
        "ulog": str(path),
        "sha256": file_sha256(path),
        "software_version": ulog.msg_info_dict.get("ver_sw", "unknown"),
        "selection": {
            "mission_sequence_min": min_mission_seq,
            "mission_sequence_max": max_mission_seq,
            "height_min_m": min_agl_m,
            "tas_min_m_s": 30.0,
            "sample_rate_hz": sample_rate_hz,
            "samples": len(time_s),
            "duration_s": float(time_s[-1] - time_s[0]),
        },
        # pyulog.initial_parameters describes the startup BSON values.  The
        # isolated sweep applies candidates after startup, so reconstruct the
        # values effective at the first selected cruise sample from the ULog
        # parameter-change stream.
        "parameters": effective_parameters,
        "initial_parameters": initial_parameters,
        "operating_point": {
            "alpha_deg_percentiles_5_50_95": percentile(signals["alpha_deg"], (5, 50, 95)),
            "tas_m_s_percentiles_5_50_95": percentile(signals["tas_m_s"], (5, 50, 95)),
            "elevator_deg_percentiles_5_50_95": percentile(signals["elevator_deg"], (5, 50, 95)),
        },
        "dominant_pitch_frequency_hz": dominant_hz,
        "metrics": {
            "pitch_hp_std_deg": float(np.degrees(np.std(high_pass["pitch_rad"]))),
            "tecs_pitch_sp_hp_std_deg": float(np.degrees(np.std(high_pass["tecs_pitch_sp_rad"]))),
            "elevator_hp_std_deg": float(np.std(high_pass["elevator_deg"])),
            "height_error_rms_m": float(np.sqrt(np.mean(signals["height_error_m"] ** 2))),
            "height_error_hp_std_m": float(np.std(high_pass["height_error_m"])),
            "height_truth_error_rms_m": float(
                np.sqrt(np.mean(signals["height_truth_error_m"] ** 2))
            ),
            "pitch_tracking_rmse_deg": float(np.degrees(np.sqrt(np.mean(pitch_error ** 2)))),
            "pitch_rate_tracking_rmse_deg_s": float(
                np.degrees(np.sqrt(np.mean(pitch_rate_error ** 2)))
            ),
            "pitch_integrator_max_abs": float(np.max(np.abs(signals["pitch_integrator"]))),
            "pitch_hp_30s_windows_deg": {
                key: (math.degrees(value)
                      if key not in ("window_s", "last_to_first_std_ratio") else value)
                for key, value in worst_window_metrics(
                    high_pass["pitch_rad"], sample_rate_hz
                ).items()
            },
            "height_error_hp_30s_windows_m": worst_window_metrics(
                high_pass["height_error_m"], sample_rate_hz
            ),
        },
        "transfers_at_dominant_frequency": transfers,
    }
    plot_data = {**signals, **{f"hp_{name}": values for name, values in high_pass.items()}}
    for name, spectrum in transfer_spectra.items():
        plot_data[f"spectrum_{name}"] = spectrum
    return report, plot_data


def write_metrics_csv(reports: list[dict], path: Path) -> None:
    rows = []
    for report in reports:
        row = {
            "ulog": report["ulog"],
            "dominant_pitch_frequency_hz": report["dominant_pitch_frequency_hz"],
            **report["metrics"],
        }
        for name, value in report["parameters"].items():
            row[f"param_{name}"] = value
        transfer = report["transfers_at_dominant_frequency"]["pitch_rate_sp_to_pitch_rate"]
        row.update({f"rate_loop_{name}": value for name, value in transfer.items()})
        rows.append(row)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def make_plots(reports: list[dict], plot_data: list[dict[str, np.ndarray]], output_dir: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    for report, signals in zip(reports, plot_data):
        label = Path(report["ulog"]).stem
        time_s = signals["time_s"] - signals["time_s"][0]
        axes[0].plot(time_s, np.degrees(signals["hp_pitch_rad"]), label=label, linewidth=0.8)
        axes[1].plot(time_s, np.degrees(signals["hp_tecs_pitch_sp_rad"]), label=label, linewidth=0.8)
        axes[2].plot(time_s, signals["hp_height_error_m"], label=label, linewidth=0.8)
    axes[0].set_ylabel("Pitch HP [deg]")
    axes[1].set_ylabel("TECS pitch SP HP [deg]")
    axes[2].set_ylabel("Height error HP [m]")
    axes[2].set_xlabel("Selected cruise time [s]")
    axes[0].legend(ncol=2)
    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.savefig(output_dir / "longitudinal_timeseries.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
    for report, signals in zip(reports, plot_data):
        label = Path(report["ulog"]).stem
        spectrum: Spectrum = signals["spectrum_pitch_rate_sp_to_pitch_rate"]
        transfer = spectrum.pxy / np.maximum(spectrum.pxx, np.finfo(float).tiny)
        coherence = np.abs(spectrum.pxy) ** 2 / np.maximum(
            spectrum.pxx * spectrum.pyy, np.finfo(float).tiny
        )
        frequency_mask = (spectrum.frequency_hz >= 0.05) & (spectrum.frequency_hz <= 2.0)
        axes[0].semilogx(
            spectrum.frequency_hz[frequency_mask], np.abs(transfer[frequency_mask]), label=label
        )
        axes[1].semilogx(
            spectrum.frequency_hz[frequency_mask], np.clip(coherence[frequency_mask], 0, 1), label=label
        )
    axes[0].axhspan(0.85, 1.15, color="tab:green", alpha=0.12)
    axes[0].axvline(0.234, color="black", linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("|q / q_sp|")
    axes[1].set_ylabel("Coherence")
    axes[1].set_xlabel("Frequency [Hz]")
    axes[0].set_ylim(bottom=0)
    axes[1].set_ylim(0, 1.05)
    axes[0].legend(ncol=2)
    for axis in axes:
        axis.grid(True, which="both", alpha=0.3)
    fig.savefig(output_dir / "pitch_rate_closed_loop_frequency_response.png", dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("ulogs", nargs="*", type=Path, default=[DEFAULT_BASELINE])
    parser.add_argument("--sample-rate", type=float, default=20.0)
    parser.add_argument(
        "--min-agl", type=float, default=60.0,
        help="minimum truth AGL for cruise selection [m]; use 35 for the 50 m route",
    )
    parser.add_argument("--min-mission-seq", type=int, default=3)
    parser.add_argument("--max-mission-seq", type=int, default=15)
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path("analysis_outputs/honghu_v8_100kg_longitudinal_control"),
    )
    arguments = parser.parse_args()
    arguments.output_dir.mkdir(parents=True, exist_ok=True)

    reports = []
    plot_data = []
    for path in arguments.ulogs:
        report, signals = analyze_log(
            path.resolve(), arguments.sample_rate, arguments.min_agl,
            arguments.min_mission_seq, arguments.max_mission_seq
        )
        reports.append(report)
        plot_data.append(signals)
        print(
            f"{path.name}: f={report['dominant_pitch_frequency_hz']:.4f} Hz, "
            f"pitch_hp={report['metrics']['pitch_hp_std_deg']:.3f} deg, "
            f"height_rms={report['metrics']['height_error_rms_m']:.3f} m"
        )

    payload = {
        "status": "PASS",
        "method": "20 Hz resampling, longest contiguous cruise selection, 10 s moving-mean high-pass",
        "logs": reports,
    }
    (arguments.output_dir / "longitudinal_analysis.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    write_metrics_csv(reports, arguments.output_dir / "longitudinal_metrics.csv")
    make_plots(reports, plot_data, arguments.output_dir)


if __name__ == "__main__":
    main()
