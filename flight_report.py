# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025 Sun Devil Rocketry

import csv
import base64
import gzip
import html
import json
import math
from datetime import datetime, timedelta, timezone

from pathlib import Path
from typing import Any

from SDECv2.Sensor.conv_functions import isa_get_altitude_qnh, mach_number, quat_to_eul


class FlightReport:
    """
    Generator class for the post-flight data report.

    QA Classification: Non-Critical -- no impact on
    mission success.
    """

    def generate(
        self,
        preset_file: str,
        data_file: str,
        output_file: str | None,
        flight_name: str | None = None,
        flight_date: str | None = None,
    ) -> str:
        """
        Generate a self-contained HTML report and return its path.

        ``flight_name`` and ``flight_date`` identify the flight in the report.
        Barometric altitude uses the sensor pressure and temperature with the
        shared ISA QNH conversion; the Nautilus STL mesh is embedded in
        compressed form so the report remains usable without a separate model
        file.
        """
        preset_path = Path(preset_file)
        data_path = Path(data_file)
        if not preset_path.is_file():
            raise FileNotFoundError(f"Preset file not found: {preset_path}")
        if not data_path.is_file():
            raise FileNotFoundError(f"Flight data file not found: {data_path}")

        with preset_path.open("r", encoding="utf-8-sig") as preset_stream:
            json.load(preset_stream)
        rows = self._read_rows(data_path)
        baseline_note = (
            "Barometric ASL altitude is calculated with the ISA QNH pressure/temperature model "
            "when inputs are available; AGL altitude is relative to the launch reading. "
            "Logged barometric altitude is used when those inputs are unavailable."
        )

        pressure_column = self._find_column(rows[0], "Barometric Pressure", "pressure", "pres")
        temperature_column = self._find_column(
            rows[0], "Barometric Temperature", "temperature", "temp"
        )

        time_column = self._find_column(rows[0], "Time", "time", "Time (s)")
        state_column = self._find_column(rows[0], "FC State", "state")
        utc_column = self._find_column(rows[0], "GPS UTC Time", "utc_time")
        baro_altitude_column = self._find_column(rows[0], "Barometric Altitude", "alt")
        gps_altitude_column = self._find_column(rows[0], "GPS Altitude (ft)", "altg")

        times: list[float] = []
        altitudes: list[float | None] = []
        baro_altitudes: list[float | None] = []
        gps_altitudes: list[float | None] = []
        accelerations: list[float | None] = []
        velocities: list[float | None] = []
        quaternions: list[list[float]] = []
        states: list[str] = []
        utc_times: list[float | None] = []

        for row_number, row in enumerate(rows, start=2):
            timestamp = self._number(row.get(time_column)) if time_column else None
            if timestamp is None:
                raise ValueError(f"Invalid or missing flight time at CSV row {row_number}")
            times.append(timestamp)
            states.append(str(row.get(state_column, "")).strip() if state_column else "")
            utc_times.append(self._number(row.get(utc_column)) if utc_column else None)

            logged_baro_altitude = (
                self._number(row.get(baro_altitude_column)) if baro_altitude_column else None
            )
            gps_altitude_ft = self._number(row.get(gps_altitude_column)) if gps_altitude_column else None
            gps_altitudes.append(
                gps_altitude_ft * 0.3048 if gps_altitude_ft is not None and gps_altitude_ft > 0 else None
            )

            pressure = self._number(row.get(pressure_column)) if pressure_column else None
            baro_temperature = (
                self._number(row.get(temperature_column)) if temperature_column else None
            )
            if pressure is not None and pressure > 0 and baro_temperature is not None:
                calculated_altitude = isa_get_altitude_qnh(
                    pressure, baro_temperature
                )
                baro_altitude = (
                    calculated_altitude if math.isfinite(calculated_altitude) else logged_baro_altitude
                )
            else:
                baro_altitude = logged_baro_altitude
            baro_altitudes.append(baro_altitude)
            altitudes.append(None)

            axes = [
                self._number(row.get(column)) if (column := self._find_column(row, *names)) else None
                for names in (
                    ("Pre-converted Accel X", "accXconv"),
                    ("Pre-converted Accel Y", "accYconv"),
                    ("Pre-converted Accel Z", "accZconv"),
                )
            ]
            valid_axes = [value for value in axes if value is not None]
            accelerations.append(
                math.sqrt(sum(value * value for value in valid_axes)) if valid_axes else None
            )

            velocity_column = self._find_column(row, "Velocity", "velo")
            velocities.append(self._number(row.get(velocity_column)) if velocity_column else None)

            quaternion_values = [
                self._number(row.get(column)) if (column := self._find_column(row, *names)) else None
                for names in (
                    ("Unit Quaternion W", "quat_w"),
                    ("Unit Quaternion X", "quat_x"),
                    ("Unit Quaternion Y", "quat_y"),
                    ("Unit Quaternion Z", "quat_z"),
                )
            ]
            if all(value is not None for value in quaternion_values):
                quaternion = [float(value) for value in quaternion_values if value is not None]
                norm = math.sqrt(sum(value * value for value in quaternion))
                quaternions.append([value / norm for value in quaternion] if norm > 1e-9 else [1.0, 0.0, 0.0, 0.0])
            else:
                quaternions.append([1.0, 0.0, 0.0, 0.0])

        launch_index = next((index for index, state in enumerate(states) if state == "4"), None)
        transition_index = next(
            (
                index
                for index in range(1, len(states))
                if states[index - 1] == "3" and states[index] == "4"
            ),
            launch_index,
        )
        launch_time = times[launch_index] if launch_index is not None else min(times)
        relative_times = [timestamp - launch_time for timestamp in times]
        if launch_index is not None:
            launch_altitude = baro_altitudes[launch_index]
            if launch_altitude is None:
                launch_altitude = next(
                    (value for value in baro_altitudes[launch_index:] if value is not None), None
                )
        else:
            launch_altitude = next(
                (value for value in baro_altitudes if value is not None), None
            )
        if launch_altitude is not None:
            altitudes = [
                altitude - launch_altitude if altitude is not None else None
                for altitude in baro_altitudes
            ]
        mach_values: list[float | None] = []
        for index, velocity in enumerate(velocities):
            altitude = baro_altitudes[index]
            if altitude is None:
                altitude = gps_altitudes[index]
            if altitude is None:
                altitude = (altitudes[index] or 0.0) + (launch_altitude or 0.0)
            mach_values.append(mach_number(velocity, altitude) if velocity is not None else None)

        duration = max(0.0, max(times) - launch_time)
        max_altitude, apogee_index = self._maximum(altitudes)
        max_acceleration, max_accel_index = self._maximum(accelerations)
        max_velocity, max_velocity_index = self._maximum(velocities)
        apogee_asl = None
        if apogee_index is not None:
            apogee_asl = baro_altitudes[apogee_index]
            if apogee_asl is None:
                apogee_asl = gps_altitudes[apogee_index]
            if apogee_asl is None and max_altitude is not None:
                launch_asl = (
                    baro_altitudes[launch_index]
                    if launch_index is not None and baro_altitudes[launch_index] is not None
                    else next((value for value in gps_altitudes if value is not None), None)
                )
                if launch_asl is not None:
                    launch_agl = altitudes[launch_index] if launch_index is not None else 0.0
                    apogee_asl = launch_asl + max_altitude - (launch_agl or 0.0)

        launch_local_time = (
            self._format_arizona_time(utc_times[transition_index])
            if transition_index is not None
            else None
        )

        output_path = Path(output_file) if output_file else data_path.with_name(f"{data_path.stem}_report.html")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_flight_name = (flight_name or "").strip() or data_path.stem
        resolved_flight_date = (flight_date or "").strip() or "Date not provided"
        title = f"{resolved_flight_name} Post-Flight Report"

        plot_limit = 700
        plot_indices = {
            index for index in self._sample_indices(len(rows), plot_limit)
        }
        if launch_index is not None:
            # Keep the full pre-roll and the exact launch sample, even when
            # the rest of the flight is downsampled for compact charts.
            plot_indices.update(range(launch_index + 1))
        for values in (altitudes, accelerations, velocities):
            valid = [
                (values[index], index)
                for index in range(len(values))
                if values[index] is not None
            ]
            if valid:
                plot_indices.add(min(valid)[1])
                plot_indices.add(max(valid)[1])
        plot_indices = sorted(plot_indices)
        plot_times = [relative_times[index] for index in plot_indices]
        altitude_plot = [altitudes[index] for index in plot_indices]
        acceleration_plot = [accelerations[index] for index in plot_indices]
        velocity_plot = [velocities[index] for index in plot_indices]
        plots = "\n".join((
            self._svg_chart("Altitude AGL", "m", plot_times, [("Altitude", "#157a56", altitude_plot)]),
            self._svg_chart("Acceleration magnitude", "m/s²", plot_times, [("Acceleration", "#c2410c", acceleration_plot)]),
            self._svg_chart("Velocity", "m/s", plot_times, [("Velocity", "#2457a7", velocity_plot)]),
        ))

        playback_start = launch_index if launch_index is not None else 0
        viewer_index_set = {
            playback_start + index
            for index in self._sample_indices(len(rows) - playback_start, 1200)
        }
        viewer_index_set.update(
            index
            for index in range(playback_start, len(rows))
            if states[index] == "4" or (index > playback_start and states[index] != states[index - 1])
        )
        burnout_index = next(
            (
                index
                for index in range(playback_start + 1, len(states))
                if states[index - 1] == "4" and states[index] == "5"
            ),
            None,
        )
        if burnout_index is not None:
            taper_end_time = times[burnout_index] + 0.5
            taper_end_index = next(
                (
                    index
                    for index in range(burnout_index + 1, len(times))
                    if times[index] > taper_end_time
                ),
                len(times) - 1,
            )
            viewer_index_set.update(range(burnout_index, taper_end_index + 1))
        viewer_indices = sorted(viewer_index_set)
        viewer_data = [
            {
                "t": relative_times[index],
                "alt": altitudes[index],
                "v": velocities[index],
                "mach": mach_values[index],
                "q": quaternions[index],
                "e": [math.degrees(angle) for angle in quat_to_eul(*quaternions[index])],
                "state": states[index],
            }
            for index in viewer_indices
        ]
        viewer_json = json.dumps(viewer_data, separators=(",", ":"), allow_nan=False)
        viewer_json = viewer_json.replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")

        stats = [
            ("Flight time", self._format_value(duration, "s"), "—"),
            ("Apogee AGL", self._format_distance(max_altitude), self._time_description(relative_times, apogee_index)),
            ("Apogee ASL", self._format_distance(apogee_asl), self._time_description(relative_times, apogee_index)),
            ("Max acceleration", self._format_acceleration(max_acceleration), self._time_description(relative_times, max_accel_index)),
            ("Peak velocity", self._format_velocity(max_velocity), self._time_description(relative_times, max_velocity_index)),
        ]
        state_summary = f"{len(rows):,} samples · { 1000 * (duration / len(rows)) :.1f} ms/sample · {len(rows) / duration :.1f} hz"
        source_names = html.escape(f"{preset_path.name} · {data_path.name}")
        statistics_table = "\n".join(
            f'<tr><th scope="row">{html.escape(label)}</th><td>{html.escape(measurement)}</td>'
            f'<td>{html.escape(note)}</td></tr>'
            for label, measurement, note in stats
        )

        model_path = Path(__file__).resolve().parent / "resources" / "LOCKEDIN.stl"
        if not model_path.is_file():
            raise FileNotFoundError(f"Rocket model not found: {model_path}")
        original_model = model_path.read_bytes()
        model_payload = base64.b64encode(
            gzip.compress(original_model, compresslevel=6, mtime=0)
        ).decode("ascii")

        document = self._html_document(
            title=title,
            flight_name=resolved_flight_name,
            flight_date=resolved_flight_date,
            launch_local_time=launch_local_time,
            statistics_table=statistics_table,
            plots=plots,
            viewer_json=viewer_json,
            state_summary=state_summary,
            source_names=source_names,
            model_payload=model_payload,
        )
        output_path.write_text(document, encoding="utf-8")
        return str(output_path)

    @staticmethod
    def _read_rows(data_path: Path) -> list[dict[str, str]]:
        with data_path.open("r", newline="", encoding="utf-8-sig") as data_stream:
            reader = csv.DictReader(data_stream)
            if not reader.fieldnames:
                raise ValueError(f"Flight data CSV has no header: {data_path}")
            rows = list(reader)
        if not rows:
            raise ValueError(f"Flight data CSV contains no samples: {data_path}")
        return rows

    @staticmethod
    def _find_column(row: dict[str, Any], *names: str) -> str | None:
        normalized = {str(key).strip().lower(): key for key in row if key is not None}
        for name in names:
            column = normalized.get(name.strip().lower())
            if column is not None:
                return column
        return None

    @staticmethod
    def _number(value: Any) -> float | None:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    @staticmethod
    def _maximum(values: list[float | None]) -> tuple[float | None, int | None]:
        candidates = [(value, index) for index, value in enumerate(values) if value is not None]
        if not candidates:
            return None, None
        return max(candidates, key=lambda candidate: candidate[0])

    @staticmethod
    def _sample_indices(length: int, limit: int) -> list[int]:
        if length <= limit:
            return list(range(length))
        return sorted({round(index * (length - 1) / (limit - 1)) for index in range(limit)})

    @staticmethod
    def _format_distance(value: float | None) -> str:
        if value is None:
            return "Unavailable"
        return f"{value:,.2f} m ({value * 3.28084:,.2f} ft)"

    @staticmethod
    def _format_acceleration(value: float | None) -> str:
        if value is None:
            return "Unavailable"
        return f"{value:,.2f} m/s² ({value / 9.80665:.2f} g)"

    @staticmethod
    def _format_velocity(value: float | None) -> str:
        if value is None:
            return "Unavailable"
        return f"{value:,.2f} m/s ({value * 2.236936:,.2f} mph)"

    @staticmethod
    def _format_value(value: float | None, unit: str) -> str:
        return f"{value:,.2f} {unit}" if value is not None else "Unavailable"

    @staticmethod
    def _time_description(times: list[float], index: int | None) -> str:
        return f"{times[index]:.2f} s after launch" if index is not None else "—"

    @staticmethod
    def _format_arizona_time(value: float | None) -> str | None:
        if value is None or value < 0:
            return None
        if value >= 100000:
            packed_time = int(value)
            hours, remainder = divmod(packed_time, 10000)
            minutes, seconds = divmod(remainder, 100)
        else:
            hours, remainder = divmod(int(value), 3600)
            minutes, seconds = divmod(remainder, 60)
        if hours > 23 or minutes > 59 or seconds > 59:
            return None
        # Arizona (Phoenix time) stays on MST year-round; a fixed offset also
        # avoids depending on an external timezone database on Windows.
        utc_datetime = datetime(2000, 1, 2, hours, minutes, seconds, tzinfo=timezone.utc)
        arizona_timezone = timezone(timedelta(hours=-7), name="MST")
        return utc_datetime.astimezone(arizona_timezone).strftime("%H:%M:%S MST")

    @classmethod
    def _svg_chart(
        cls,
        title: str,
        unit: str,
        times: list[float],
        series: list[tuple[str, str, list[float | None]]],
    ) -> str:
        width, height = 960, 320
        left, right, top, bottom = 74, 24, 38, 48
        plot_width, plot_height = width - left - right, height - top - bottom
        available = [value for _, _, values in series for value in values if value is not None]
        if not available or not times:
            return f'<article class="chart"><h3>{html.escape(title)}</h3><p class="empty">No data available.</p></article>'
        low, high = min(available), max(available)
        value_range = high - low
        start_time, end_time = min(times), max(times)
        if math.isclose(start_time, end_time):
            end_time = start_time + 1.0

        parts = [
            f'<article class="chart"><h3>{html.escape(title)}</h3>',
            f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)} over time" '
            f'data-time-min="{start_time:.8f}" data-time-max="{end_time:.8f}" '
            f'data-plot-left="{left}" data-plot-right="{width-right}">',
        ]
        for tick in range(5):
            fraction = tick / 4
            y = top + plot_height * fraction
            value = high - value_range * fraction
            parts.append(f'<line class="grid" x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}"/>')
            parts.append(f'<text class="axis" x="{left-10}" y="{y+4:.1f}" text-anchor="end">{value:.2f}</text>')
        for tick in range(5):
            fraction = tick / 4
            x = left + plot_width * fraction
            value = start_time + (end_time - start_time) * fraction
            parts.append(f'<text class="axis" x="{x:.1f}" y="{height-15}" text-anchor="middle">{value:.1f}</text>')
        parts.append(f'<text class="axis-title" x="{width/2}" y="{height-2}" text-anchor="middle">Time (s)</text>')
        parts.append(f'<text class="axis-title" transform="translate(17 {height/2}) rotate(-90)" text-anchor="middle">{html.escape(unit)}</text>')

        for label, color, values in series:
            coordinates = []
            for timestamp, value in zip(times, values):
                if value is None:
                    continue
                x = left + (timestamp - start_time) / (end_time - start_time) * plot_width
                y_fraction = 0.5 if value_range == 0 else (high - value) / value_range
                y = top + y_fraction * plot_height
                coordinates.append(f"{x:.1f},{y:.1f}")
            if coordinates:
                parts.append(f'<polyline fill="none" stroke="{color}" stroke-width="2.2" stroke-linejoin="round" stroke-linecap="round" points="{" ".join(coordinates)}"/>')
                parts.append(f'<line x1="{left+10}" y1="{top-16}" x2="{left+30}" y2="{top-16}" stroke="{color}" stroke-width="3"/>')
                parts.append(f'<text class="legend" x="{left+36}" y="{top-12}">{html.escape(label)}</text>')
        parts.append(
            f'<line class="time-cursor" x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}"/>'
        )
        parts.extend(("</svg>", "</article>"))
        return "".join(parts)

    @staticmethod
    def _html_document(
        *,
        title: str,
        flight_name: str,
        flight_date: str,
        launch_local_time: str | None,
        statistics_table: str,
        plots: str,
        viewer_json: str,
        state_summary: str,
        source_names: str,
        model_payload: str,
    ) -> str:
        safe_title = html.escape(title)
        safe_name = html.escape(flight_name)
        safe_date = html.escape(flight_date)
        return f'''<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta name="color-scheme" content="light">
  <title>{safe_title}</title>
  <style>
    :root {{ color-scheme:light; --ink:#202a33; --muted:#5d6a75; --line:#d8dee3; --grid:#e7ebee; --accent:#176b87; --paper:#fff; }}
    * {{ box-sizing:border-box; }}
    body {{ margin:0; padding-bottom:calc(70px + env(safe-area-inset-bottom,0px)); background:var(--paper); color:var(--ink); font:15px/1.55 Arial,Helvetica,sans-serif; }}
    main {{ width:min(1080px,calc(100% - 40px)); margin:0 auto; padding:38px 0 56px; }}
    header {{ padding-bottom:20px; border-bottom:2px solid var(--ink); }} h1 {{ margin:0; font-size:clamp(30px,4vw,42px); line-height:1.15; letter-spacing:-.025em; }}
    .subtitle {{ margin:8px 0 0; color:var(--muted); font-size:16px; }} h2 {{ margin:0 0 14px; font-size:22px; }} h3 {{ margin:0 0 8px; font-size:16px; }}
    .section {{ margin-top:30px; }} .metadata {{ border-collapse:collapse; width:100%; max-width:720px; }}
    .metadata th,.metadata td {{ border-bottom:1px solid var(--line); padding:8px 10px; text-align:left; vertical-align:top; }}
    .metadata th {{ width:190px; font-weight:600; }} .muted {{ color:var(--muted); }}
    .stats {{ border-collapse:collapse; width:100%; }} .stats th,.stats td {{ border:1px solid var(--line); padding:11px 13px; text-align:left; vertical-align:top; }}
    .stats thead th {{ background:#f1f4f6; font-weight:700; }} .stats tbody th {{ width:24%; font-weight:600; }} .stats td:nth-child(2) {{ width:51%; font-variant-numeric:tabular-nums; }} .stats td:nth-child(3) {{ width:25%; }}
    .charts {{ display:grid; gap:16px; }} .chart {{ border:1px solid var(--line); padding:12px 14px 8px; background:#fff; }} .chart svg {{ width:100%; height:auto; display:block; }}
    .grid {{ stroke:var(--grid); stroke-width:1; }} .axis,.legend {{ fill:#505d67; font-size:11px; }} .axis-title {{ fill:#35434d; font-size:12px; }}
    .time-cursor {{ stroke:#c2410c; stroke-width:2; stroke-dasharray:5 4; pointer-events:none; }}
    .viewer {{ border:1px solid var(--line); padding:14px; }} .viewer-grid {{ display:grid; grid-template-columns:minmax(0,1.7fr) minmax(210px,.6fr); gap:16px; }}
    canvas {{ width:100%; height:430px; display:block; border:1px solid #cbd5dc; background:linear-gradient(180deg,#559ed1 0%,#83c4e5 48%,#d8e6e8 78%,#e5d9bd 100%); }} .viewer-side {{ display:flex; flex-direction:column; gap:12px; }}
    .telemetry {{ border:1px solid var(--line); padding:12px; }} .telemetry dt {{ color:var(--muted); font-size:13px; }} .telemetry dd {{ margin:2px 0 13px; font-size:19px; font-weight:600; font-variant-numeric:tabular-nums; }}
    .control-row {{ position:fixed; z-index:20; right:0; bottom:0; left:0; display:flex; align-items:center; gap:12px; margin:0; padding:10px max(12px,calc((100vw - 1080px)/2)); padding-bottom:calc(10px + env(safe-area-inset-bottom,0px)); border-top:1px solid var(--line); background:rgba(255,255,255,.97); box-shadow:0 -3px 14px rgba(24,39,50,.14); }}
    .control-row button {{ flex:0 0 auto; }} .control-row input[type=range] {{ min-width:0; width:100%; accent-color:var(--accent); }}
    button {{ padding:7px 14px; border:1px solid #75838d; border-radius:3px; color:var(--ink); background:#fff; font:inherit; cursor:pointer; }} button:hover {{ background:#f1f4f6; }}
    .viewer-status {{ min-height:1.5em; color:#963c17; font-size:13px; }} .note {{ margin-top:14px; color:var(--muted); font-size:13px; }}
    footer {{ margin-top:30px; padding-top:12px; border-top:1px solid var(--line); color:var(--muted); font-size:12px; }}
    @media(max-width:760px) {{ main {{ width:calc(100% - 24px); padding-top:24px; }} .viewer-grid {{ grid-template-columns:1fr; }} canvas {{ height:340px; }} .stats th,.stats td {{ padding:8px; }} }}
    </style>
</head>
<body>
<main>
    <header><h1>{safe_title}</h1></header>
    <section class="section" aria-labelledby="metadata-heading">
        <h2 id="metadata-heading">1. Identification / Metadata</h2>
        <table class="metadata"><tbody>
            <tr><th scope="row">Flight name</th><td>{safe_name}</td></tr>
            <tr><th scope="row">Date of flight</th><td>{safe_date}</td></tr>
            <tr><th scope="row">Launch time (Arizona)</th><td>{html.escape(launch_local_time or "Unavailable")}</td></tr>
            <tr><th scope="row">Recorded samples</th><td>{html.escape(state_summary)}</td></tr>
            <tr><th scope="row">Input files</th><td>{source_names}</td></tr>
        </tbody></table>
    </section>
    <section class="section" aria-labelledby="statistics-heading">
        <h2 id="statistics-heading">2. Critical Flight Statistics</h2>
        <table class="stats"><thead><tr><th scope="col">Statistic</th><th scope="col">Measurement</th><th scope="col">Time from launch</th></tr></thead><tbody>{statistics_table}</tbody></table>
    </section>
    <section class="section" aria-labelledby="profile-heading">
        <h2 id="profile-heading">3. Flight Profile</h2>
        <p class="muted">All chart times are relative to the first sample in flight state 4. The vertical marker follows the orientation playback.</p>
        <div class="charts">{plots}</div>
    </section>
    <section class="section" aria-labelledby="orientation-heading">
        <h2 id="orientation-heading">4. Orientation Playback</h2>
        <div class="viewer"><div class="viewer-grid">
            <div><canvas id="rocketCanvas" aria-label="Three-dimensional Nautilus rocket flight playback"></canvas><div id="viewerStatus" class="viewer-status" role="status"></div></div>
            <div class="viewer-side"><dl class="telemetry"><dt>Time from launch</dt><dd id="viewerTime">—</dd><dt>Altitude AGL</dt><dd id="viewerAltitude">—</dd><dt>Euler angles — yaw, pitch, roll (°)</dt><dd id="viewerEuler">—</dd><dt>Quaternion (w, x, y, z)</dt><dd id="viewerQuaternion">—</dd></dl></div>
        </div><div class="control-row" id="playbackDock" role="group" aria-label="Flight playback controls"><button id="playButton" type="button">Play</button><input id="timeSlider" type="range" min="0" max="0" step="any" value="0" aria-label="Scrub flight time"></div></div>
    </section>
</main>
<script id="flightSamples" type="application/json">{viewer_json}</script>
<script id="rocketModel" type="application/octet-stream">{model_payload}</script>
<script>
(() => {{
    const samples = JSON.parse(document.getElementById('flightSamples').textContent);
    const canvas = document.getElementById('rocketCanvas');
    const gl = canvas.getContext('webgl', {{ antialias:true, alpha:true }});
    const slider = document.getElementById('timeSlider');
    const playButton = document.getElementById('playButton');
    const status = document.getElementById('viewerStatus');
    const cursors = Array.from(document.querySelectorAll('.time-cursor'));
    let index = 0, playing = false, playbackStartTimestamp = null, playbackStartTime = 0, drawScene = null;
    slider.max = String(Math.max(samples.length - 1, 0));
    const maximumAltitude=Math.max(1,...samples.map(sample=>Math.max(0,sample.alt||0)));
    const altitudeDisplayScale=1, smokeLifetime=3.5;
    const coastTransitionSample=samples.find((sample,index)=>index>0&&samples[index-1].state==='4'&&sample.state==='5');
    const coastTransitionTime=coastTransitionSample?.t??null;

    const multiplyQ = (a,b) => [
        a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
        a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
        a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
        a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]
    ];
    const normalizeQ = q => {{ const n=Math.hypot(...q)||1; return q.map(v=>v/n); }};
    const displayQuaternion = telemetry => {{
        const frameQuat=[Math.SQRT1_2,Math.SQRT1_2,0,0], frameQuatInv=[Math.SQRT1_2,-Math.SQRT1_2,0,0], modelBaseQuat=[Math.SQRT1_2,0,Math.SQRT1_2,0];
        return normalizeQ(multiplyQ(multiplyQ(multiplyQ(frameQuat,normalizeQ(telemetry)),frameQuatInv),modelBaseQuat));
    }};
    const rotateVector = (q,v) => {{
        const u=q.slice(1),t=[2*(u[1]*v[2]-u[2]*v[1]),2*(u[2]*v[0]-u[0]*v[2]),2*(u[0]*v[1]-u[1]*v[0])];
        return [v[0]+q[0]*t[0]+u[1]*t[2]-u[2]*t[1],v[1]+q[0]*t[1]+u[2]*t[0]-u[0]*t[2],v[2]+q[0]*t[2]+u[0]*t[1]-u[1]*t[0]];
    }};
    const perspective = (fov,aspect,near,far) => {{
        const f=1/Math.tan(fov/2), nf=1/(near-far);
        return new Float32Array([f/aspect,0,0,0, 0,f,0,0, 0,0,(far+near)*nf,-1, 0,0,2*far*near*nf,0]);
    }};
    const lookAt = (eye,target,up) => {{
        const norm=v=>{{const n=Math.hypot(...v)||1;return v.map(x=>x/n);}};
        const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
        const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
        const z=norm([eye[0]-target[0],eye[1]-target[1],eye[2]-target[2]]), x=norm(cross(up,z)), y=cross(z,x);
        return new Float32Array([x[0],y[0],z[0],0, x[1],y[1],z[1],0, x[2],y[2],z[2],0, -dot(x,eye),-dot(y,eye),-dot(z,eye),1]);
    }};
    const multiplyM = (a,b) => {{
        const out=new Float32Array(16);
        for(let c=0;c<4;c++) for(let r=0;r<4;r++) out[c*4+r]=a[r]*b[c*4]+a[4+r]*b[c*4+1]+a[8+r]*b[c*4+2]+a[12+r]*b[c*4+3];
        return out;
    }};
    function shader(type,source) {{
        const result=gl.createShader(type); gl.shaderSource(result,source); gl.compileShader(result);
        if(!gl.getShaderParameter(result,gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(result)||'Shader compilation failed');
        return result;
    }}
    async function initialize() {{
        if (!gl) throw new Error('WebGL is unavailable in this browser.');
        if (!window.DecompressionStream) throw new Error('This browser cannot decompress the embedded STL model. Use a current browser.');
        const vertexSource='attribute vec3 aPosition; attribute vec3 aNormal; uniform mat4 uMvp; uniform vec4 uQuaternion; uniform vec3 uRocketPosition; uniform vec3 uMeshCenter; uniform float uModelScale; uniform mediump vec3 uShadowCenter; uniform mediump float uShadowRadius; uniform mediump float uIsShadow; uniform float uIsRocket; varying vec3 vNormal; varying vec3 vWorldPosition; varying mediump vec3 vModelPosition; vec3 qrot(vec4 q,vec3 v){{vec3 t=2.0*cross(q.xyz,v);return v+q.w*t+cross(q.xyz,t);}} void main(){{vec3 p=aPosition;vec3 n=aNormal;vModelPosition=aPosition;if(uIsRocket>0.5){{p=qrot(uQuaternion,(p-uMeshCenter)*uModelScale)+uRocketPosition;n=qrot(uQuaternion,n);}}else if(uIsShadow>0.5){{p=vec3(aPosition.x*uShadowRadius+uShadowCenter.x,uShadowCenter.y,aPosition.z*uShadowRadius+uShadowCenter.z);n=vec3(0.0,1.0,0.0);}}vNormal=n;vWorldPosition=p;gl_Position=uMvp*vec4(p,1.0);}}';
        const fragmentSource='precision mediump float; uniform vec3 uColor; uniform mediump vec3 uShadowCenter; uniform mediump float uShadowRadius; uniform float uShadowOpacity; uniform float uCloudOpacity; uniform float uIsGround; uniform float uIsShadow; uniform float uIsVegetation; uniform float uIsCloud; varying vec3 vNormal; varying vec3 vWorldPosition; varying mediump vec3 vModelPosition; void main(){{float sunDiffuse=max(dot(normalize(vNormal),normalize(vec3(-0.35,0.85,0.4))),0.0);float groundLight=0.42+0.58*sunDiffuse;float rocketLight=0.20+0.80*sunDiffuse;if(uIsCloud>0.5){{float facing=max(normalize(vNormal).z,0.0);float edgeFade=smoothstep(0.0,0.98,facing);float variation=0.94+0.06*sin(vWorldPosition.x*0.004+vWorldPosition.z*0.003);vec3 cloudColor=mix(vec3(0.70,0.78,0.82),vec3(1.0,0.99,0.95),0.45+0.55*sunDiffuse);gl_FragColor=vec4(cloudColor,uCloudOpacity*variation*edgeFade);}}else if(uIsShadow>0.5){{float radius=length(vWorldPosition.xz-uShadowCenter.xz);float edge=1.0-smoothstep(uShadowRadius*0.55,uShadowRadius,radius);gl_FragColor=vec4(vec3(0.12,0.09,0.07),edge*uShadowOpacity);}}else if(uIsGround>0.5){{float broad=0.5+0.5*sin(vWorldPosition.x*0.0011+sin(vWorldPosition.z*0.0007))*cos(vWorldPosition.z*0.0013);float grain=fract(sin(dot(floor(vWorldPosition.xz*0.018),vec2(12.9898,78.233)))*43758.5453);float patch=0.5+0.5*sin(vWorldPosition.x*0.018+vWorldPosition.z*0.011)*cos(vWorldPosition.z*0.015-vWorldPosition.x*0.008);vec3 soil=mix(vec3(0.43,0.36,0.25),vec3(0.58,0.51,0.36),0.18+0.48*broad+0.20*grain);soil=mix(soil,vec3(0.34,0.37,0.25),patch*0.18);gl_FragColor=vec4(soil*groundLight,1.0);}}else if(uIsVegetation>0.5){{float variation=fract(sin(dot(floor(vWorldPosition.xz*2.0),vec2(41.7,289.3)))*24634.6345);vec3 foliage=mix(vec3(0.16,0.22,0.10),vec3(0.37,0.38,0.18),variation);float leafLight=0.32+0.68*sunDiffuse;gl_FragColor=vec4(foliage*leafLight,1.0);}}else{{float finRadial=smoothstep(44.0,58.0,length(vModelPosition.xy));float finLongitudinal=1.0-smoothstep(-1000.0,-910.0,vModelPosition.z);float finMask=finRadial*finLongitudinal;vec3 rocketColor=mix(uColor,vec3(0.88,0.08,0.10),finMask);gl_FragColor=vec4(rocketColor*rocketLight,1.0);}}}}';
        const rocketFragmentSource=`
            precision mediump float;
            uniform vec3 uColor;
            uniform mediump vec3 uMeshCenter;
            uniform vec3 uShadowCenter;
            uniform float uShadowRadius;
            uniform float uShadowOpacity;
            uniform float uIsGround;
            uniform float uIsShadow;
            uniform float uIsVegetation;
            uniform float uIsPlume;
            uniform float uIsEmber;
            uniform float uIsSmoke;
            uniform float uSmokeTime;
            uniform float uPlumeBaseZ;
            uniform float uPlumeLength;
            uniform float uPlumeRootRadius;
            uniform float uPlumeTipRadius;
            uniform float uPlumeLayer;
            uniform float uMach;
            varying vec3 vNormal;
            varying vec3 vWorldPosition;
            varying mediump vec3 vModelPosition;
            float temporalNoise(float time,float seed,float rate){{
                float position=time*rate,cell=floor(position),fraction=fract(position);
                fraction=fraction*fraction*(3.0-2.0*fraction);
                float first=fract(sin(cell*127.1+seed*311.7)*43758.5453);
                float second=fract(sin((cell+1.0)*127.1+seed*311.7)*43758.5453);
                return mix(first,second,fraction);
            }}
            void main(){{
                float sunDiffuse=max(dot(normalize(vNormal),normalize(vec3(-0.35,0.85,0.4))),0.0);
                float groundLight=0.42+0.58*sunDiffuse;
                float rocketLight=0.20+0.80*sunDiffuse;
                if(uIsSmoke>0.5){{
                    float age=clamp(vNormal.x,0.0,1.0);
                    float seed=vNormal.y;
                    float turbulence=temporalNoise(uSmokeTime,seed+vWorldPosition.x*0.07+vWorldPosition.y*0.11,3.8);
                    vec3 smokeColor=mix(vec3(0.30,0.29,0.28),vec3(0.50,0.49,0.47),0.28+0.52*turbulence);
                    float opacity=0.16*pow(1.0-age,1.45)*(0.84+0.16*turbulence);
                    gl_FragColor=vec4(smokeColor,opacity);
                }}else if(uIsEmber>0.5){{
                    float seed=vModelPosition.x*0.071+vModelPosition.y*0.053+vModelPosition.z*0.037;
                    float flicker=0.48+0.52*temporalNoise(uSmokeTime,seed,18.0);
                    float colorShift=temporalNoise(uSmokeTime,seed+9.3,6.5);
                    vec3 ember=mix(vec3(1.0,0.16,0.015),vec3(1.0,0.88,0.20),colorShift);
                    gl_FragColor=vec4(ember,0.72*flicker);
                }}else if(uIsPlume>0.5){{
                    float progress=clamp((uPlumeBaseZ-vModelPosition.z)/uPlumeLength,0.0,1.0);
                    float radius=length(vModelPosition.xy-uMeshCenter.xy);
                    float expansion=smoothstep(0.0,0.48,progress);
                    float envelope=mix(uPlumeRootRadius,uPlumeTipRadius,expansion)*(1.0-smoothstep(0.78,1.0,progress));
                    float radial=radius/max(envelope,0.001);
                    float angle=atan(vModelPosition.y-uMeshCenter.y,vModelPosition.x-uMeshCenter.x);
                    float seed=angle*2.7+progress*17.0+uPlumeLayer*31.0;
                    float turbulence=(temporalNoise(uSmokeTime,seed,6.8)-0.5)*2.0+(temporalNoise(uSmokeTime,seed+13.1,13.2)-0.5)*0.9;
                    float eddies=temporalNoise(uSmokeTime,seed+27.4,9.1);
                    float edge=1.0-smoothstep(0.82+0.05*turbulence,1.08+0.06*turbulence,radial);
                    float tail=1.0-smoothstep(0.78,1.0,progress);
                    float flicker=0.65+0.35*eddies;
                    float heatShift=0.5+0.5*sin(uSmokeTime*6.8+progress*19.0+angle*2.0);
                    vec3 flame;
                    float opacity;
                    if(uPlumeLayer>0.5){{
                        float hotCore=1.0-smoothstep(0.08,0.68,progress);
                        flame=mix(vec3(1.0,0.32+0.16*heatShift,0.035),vec3(1.0,0.86+0.12*heatShift,0.58+0.20*heatShift),hotCore);
                        opacity=0.98*edge*(1.0-smoothstep(0.60,0.76,progress))*(0.88+0.12*eddies);
                    }}else{{
                        float cool=clamp(smoothstep(0.12,0.82,progress)+(temporalNoise(uSmokeTime,seed+43.8,7.4)-0.5)*0.18,0.0,1.0);
                        flame=mix(vec3(1.0,0.48+0.14*heatShift,0.06),vec3(0.88+0.1*heatShift,0.10+0.08*heatShift,0.018),cool);
                        flame*=0.88+0.12*eddies;
                        opacity=0.76*edge*tail*flicker;
                    }}
                    gl_FragColor=vec4(flame,opacity);
                }}else if(uIsShadow>0.5){{
                    float radius=length(vWorldPosition.xz-uShadowCenter.xz);
                    float edge=1.0-smoothstep(uShadowRadius*0.55,uShadowRadius,radius);
                    gl_FragColor=vec4(vec3(0.12,0.09,0.07),edge*uShadowOpacity);
                }}else if(uIsGround>0.5){{
                    float broad=0.5+0.5*sin(vWorldPosition.x*0.0011+sin(vWorldPosition.z*0.0007))*cos(vWorldPosition.z*0.0013);
                    float grain=fract(sin(dot(floor(vWorldPosition.xz*0.018),vec2(12.9898,78.233)))*43758.5453);
                    float patch=0.5+0.5*sin(vWorldPosition.x*0.018+vWorldPosition.z*0.011)*cos(vWorldPosition.z*0.015-vWorldPosition.x*0.008);
                    vec3 soil=mix(vec3(0.43,0.36,0.25),vec3(0.58,0.51,0.36),0.18+0.48*broad+0.20*grain);
                    soil=mix(soil,vec3(0.34,0.37,0.25),patch*0.18);
                    gl_FragColor=vec4(soil*groundLight,1.0);
                }}else if(uIsVegetation>0.5){{
                    float variation=fract(sin(dot(floor(vWorldPosition.xz*2.0),vec2(41.7,289.3)))*24634.6345);
                    vec3 foliage=mix(vec3(0.16,0.22,0.10),vec3(0.37,0.38,0.18),variation);
                    gl_FragColor=vec4(foliage*(0.32+0.68*sunDiffuse),1.0);
                }}else{{
                    float z=vModelPosition.z;
                    float radius=length(vModelPosition.xy-uMeshCenter.xy);
                    float whiteMask=max(smoothstep(41.0,46.0,radius),smoothstep(843.4,848.4,z));
                    float blackRing=smoothstep(817.0,819.0,z)*(1.0-smoothstep(842.4,844.4,z));
                    vec3 paint=mix(vec3(0.0,0.2039,0.4706),vec3(0.96,0.97,1.0),whiteMask);
                    paint=mix(paint,vec3(0.012,0.016,0.024),blackRing);
                    float finRegion=smoothstep(42.0,50.0,radius)*(1.0-smoothstep(125.0,140.0,radius))*(1.0-smoothstep(185.0,225.0,z));
                    float shockBand=1.0-smoothstep(9.0,17.0,abs(z-112.0));
                    float shockStrength=0.34*smoothstep(0.68,1.05,uMach)*finRegion*shockBand;
                    paint=mix(paint,vec3(0.78,0.90,1.0),shockStrength);
                    gl_FragColor=vec4(paint*rocketLight,1.0);
                }}
            }}
        `;
        const program=gl.createProgram(); gl.attachShader(program,shader(gl.VERTEX_SHADER,vertexSource.replace('uniform vec3 uMeshCenter;','uniform mediump vec3 uMeshCenter;'))); gl.attachShader(program,shader(gl.FRAGMENT_SHADER,rocketFragmentSource)); gl.linkProgram(program);
        if(!gl.getProgramParameter(program,gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program)||'WebGL program link failed');
        gl.useProgram(program);
        const locations={{position:gl.getAttribLocation(program,'aPosition'),normal:gl.getAttribLocation(program,'aNormal'),mvp:gl.getUniformLocation(program,'uMvp'),quat:gl.getUniformLocation(program,'uQuaternion'),rocket:gl.getUniformLocation(program,'uRocketPosition'),center:gl.getUniformLocation(program,'uMeshCenter'),modelScale:gl.getUniformLocation(program,'uModelScale'),shadowCenter:gl.getUniformLocation(program,'uShadowCenter'),shadowRadius:gl.getUniformLocation(program,'uShadowRadius'),shadowOpacity:gl.getUniformLocation(program,'uShadowOpacity'),plumeBaseZ:gl.getUniformLocation(program,'uPlumeBaseZ'),plumeLength:gl.getUniformLocation(program,'uPlumeLength'),plumeRootRadius:gl.getUniformLocation(program,'uPlumeRootRadius'),plumeTipRadius:gl.getUniformLocation(program,'uPlumeTipRadius'),plumeLayer:gl.getUniformLocation(program,'uPlumeLayer'),isRocket:gl.getUniformLocation(program,'uIsRocket'),isGround:gl.getUniformLocation(program,'uIsGround'),isShadow:gl.getUniformLocation(program,'uIsShadow'),isVegetation:gl.getUniformLocation(program,'uIsVegetation'),isPlume:gl.getUniformLocation(program,'uIsPlume'),color:gl.getUniformLocation(program,'uColor')}};
        locations.mach=gl.getUniformLocation(program,'uMach');
        locations.isEmber=gl.getUniformLocation(program,'uIsEmber');
        locations.isSmoke=gl.getUniformLocation(program,'uIsSmoke');locations.smokeTime=gl.getUniformLocation(program,'uSmokeTime');

        const encoded=document.getElementById('rocketModel').textContent.trim();
        const compressed=Uint8Array.from(atob(encoded),c=>c.charCodeAt(0));
        const modelBuffer=await new Response(new Blob([compressed]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
        const view=new DataView(modelBuffer), triangleCount=view.getUint32(80,true);
        const boundsMin=[Infinity,Infinity,Infinity],boundsMax=[-Infinity,-Infinity,-Infinity];
        for(let face=0;face<triangleCount;face++) {{
            const base=84+face*50+12;
            for(let vertex=0;vertex<3;vertex++) for(let axis=0;axis<3;axis++) {{ const value=view.getFloat32(base+vertex*12+axis*4,true); boundsMin[axis]=Math.min(boundsMin[axis],value); boundsMax[axis]=Math.max(boundsMax[axis],value); }}
        }}
        const meshCenter=boundsMin.map((v,i)=>(v+boundsMax[i])/2);
        const modelScale=0.001;
        const rocketHeight=(boundsMax[2]-boundsMin[2])*modelScale;
        // Keep every source triangle. Striding through an STL face list is not
        // a valid mesh simplification: it discards neighboring faces and leaves
        // visible pinholes across the rocket surface.
        const vertices=new Float32Array(triangleCount*18);
        let cursor=0;
        for(let face=0;face<triangleCount;face++) {{
            const base=84+face*50+12;
            const normalBase=84+face*50;
            const normal=[view.getFloat32(normalBase,true),view.getFloat32(normalBase+4,true),view.getFloat32(normalBase+8,true)];
            for(let vertex=0;vertex<3;vertex++) {{
                for(let axis=0;axis<3;axis++) vertices[cursor++]=view.getFloat32(base+vertex*12+axis*4,true);
                for(const component of normal) vertices[cursor++]=component;
            }}
        }}
        const rocketBuffer=gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER,rocketBuffer); gl.bufferData(gl.ARRAY_BUFFER,vertices,gl.STATIC_DRAW);
        const plumeBaseZ=boundsMin[2],plumeLength=650,plumeRootRadius=18,plumeTipRadius=82,plumeSegments=56,plumeAxialSegments=30;
        const smoothStep=(edge0,edge1,value)=>{{const t=Math.max(0,Math.min(1,(value-edge0)/(edge1-edge0)));return t*t*(3-2*t);}};
        const hashNoise=(cell,seed)=>{{const value=Math.sin(cell*127.1+seed*311.7)*43758.5453;return value-Math.floor(value);}};
        const temporalNoise=(time,seed,rate)=>{{const position=time*rate,cell=Math.floor(position),fraction=position-cell,smooth=fraction*fraction*(3-2*fraction);return hashNoise(cell,seed)*(1-smooth)+hashNoise(cell+1,seed)*smooth;}};
        const buildFlameLayer=core=>{{
            const progressScale=core?0.72:1.0,layerScale=core?0.56:1.0;
            const values=new Float32Array(plumeAxialSegments*plumeSegments*6*6),count=values.length/6;
            const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,values.byteLength,gl.DYNAMIC_DRAW);
            const update=(time,length,directionX,directionY,sizeScale)=>{{
                let cursor=0;
                const vertex=(angle,progress)=>{{
                    const expanding=smoothStep(0,0.48,progress),tail=1-smoothStep(0.78,1,progress);
                    let radius=(plumeRootRadius+(plumeTipRadius-plumeRootRadius)*expanding)*tail*layerScale*sizeScale;
                    if(core)radius*=1-smoothStep(0.58,0.72,progress);
                    const amplitude=core?0.075:0.19,seed=angle*2.3+progress*19+(core?41:7);
                    const wave=1+(temporalNoise(time,seed,5.4)-0.5)*amplitude*2+(temporalNoise(time,seed+17.2,11.7)-0.5)*amplitude*0.9;
                    radius*=wave;
                    const axialFlutter=1+(temporalNoise(time,seed+29.6,4.1)-0.5)*(core?0.024:0.075);
                    const x=meshCenter[0]+Math.cos(angle)*radius+progress*length*directionX,y=meshCenter[1]+Math.sin(angle)*radius+progress*length*directionY,z=plumeBaseZ+3-progress*length*axialFlutter;
                    values.set([x,y,z,Math.cos(angle),Math.sin(angle),0.12],cursor);cursor+=6;
                }};
                for(let row=0;row<plumeAxialSegments;row++){{
                    const p0=row/plumeAxialSegments*progressScale,p1=(row+1)/plumeAxialSegments*progressScale;
                    for(let segment=0;segment<plumeSegments;segment++){{
                        const a0=segment/plumeSegments*Math.PI*2,a1=(segment+1)/plumeSegments*Math.PI*2;
                        vertex(a0,p0);vertex(a1,p0);vertex(a1,p1);
                        vertex(a0,p0);vertex(a1,p1);vertex(a0,p1);
                    }}
                }}
                gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferSubData(gl.ARRAY_BUFFER,0,values);
            }};
            return {{buffer,count,update}};
        }};
        const outerFlame=buildFlameLayer(false),innerFlame=buildFlameLayer(true);
        const smokeBuffer=gl.createBuffer();
        const emberBuffer=gl.createBuffer();
        const groundSegments=200, groundExtent=1000, groundStep=groundExtent*2/groundSegments;
        const terrainHeight=(x,z)=>8*Math.sin(x*0.00065)*Math.cos(z*0.0007)+4*Math.sin((x+z)*0.0013)+0.8*Math.sin(x*0.01)*Math.cos(z*0.011);
        const emitsSmoke=emission=>emission.state==='4'||(coastTransitionTime!==null&&emission.state==='5'&&emission.t<=coastTransitionTime+0.5);
        const smokePuffs=samples.flatMap((emission,sampleIndex)=>{{
            if(!emitsSmoke(emission)||(emission.alt||0)<=0.5)return[];
            const quaternion=displayQuaternion(emission.q||[1,0,0,0]);
            const rocketCenter=[0,terrainHeight(0,0)+Math.max(0,emission.alt||0)*altitudeDisplayScale+rocketHeight*0.5,0];
            const nozzleOffset=rotateVector(quaternion,[0,0,(plumeBaseZ-meshCenter[2])*modelScale]);
            const trailOffset=rotateVector(quaternion,[0,0,0.35]);
            const center=[rocketCenter[0]+nozzleOffset[0]-trailOffset[0],rocketCenter[1]+nozzleOffset[1]-trailOffset[1],rocketCenter[2]+nozzleOffset[2]-trailOffset[2]];
            const seed=((Math.round(emission.t*1000)*92821+12345)%1000)/1000,puffs=[{{t:emission.t,center,seed}}];
            const next=samples[sampleIndex+1];
            if(next&&emitsSmoke(next)&&(next.alt||0)>0.5){{
                const nextQuaternion=displayQuaternion(next.q||[1,0,0,0]);
                const nextRocketCenter=[0,terrainHeight(0,0)+Math.max(0,next.alt||0)*altitudeDisplayScale+rocketHeight*0.5,0];
                const nextNozzleOffset=rotateVector(nextQuaternion,[0,0,(plumeBaseZ-meshCenter[2])*modelScale]);
                const nextTrailOffset=rotateVector(nextQuaternion,[0,0,0.35]);
                const nextCenter=[nextRocketCenter[0]+nextNozzleOffset[0]-nextTrailOffset[0],nextRocketCenter[1]+nextNozzleOffset[1]-nextTrailOffset[1],nextRocketCenter[2]+nextNozzleOffset[2]-nextTrailOffset[2]];
                const midpoint=center.map((value,axis)=>(value+nextCenter[axis])*0.5);
                puffs.push({{t:(emission.t+next.t)*0.5,center:midpoint,seed:(seed+0.43)%1}});
            }}
            return puffs;
        }});
        const terrainNormal=(x,z)=>{{const step=6,dx=(terrainHeight(x+step,z)-terrainHeight(x-step,z))/(2*step),dz=(terrainHeight(x,z+step)-terrainHeight(x,z-step))/(2*step),length=Math.hypot(dx,1,dz);return[-dx/length,1/length,-dz/length];}};
        const groundVertices=new Float32Array(groundSegments*groundSegments*36); let groundCursor=0;
        const appendGroundVertex=(x,z)=>{{const normal=terrainNormal(x,z);groundVertices[groundCursor++]=x;groundVertices[groundCursor++]=terrainHeight(x,z);groundVertices[groundCursor++]=z;for(const component of normal)groundVertices[groundCursor++]=component;}};
        for(let row=0;row<groundSegments;row++) for(let column=0;column<groundSegments;column++){{const x=-groundExtent+column*groundStep,z=-groundExtent+row*groundStep,x1=x+groundStep,z1=z+groundStep;appendGroundVertex(x,z);appendGroundVertex(x,z1);appendGroundVertex(x1,z1);appendGroundVertex(x,z);appendGroundVertex(x1,z1);appendGroundVertex(x1,z);}}
        const groundBuffer=gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER,groundBuffer); gl.bufferData(gl.ARRAY_BUFFER,groundVertices,gl.STATIC_DRAW);
        let vegetationSeed=0x5eed1234;
        const random=()=>{{vegetationSeed=(Math.imul(1664525,vegetationSeed)+1013904223)>>>0;return vegetationSeed/4294967296;}};
        const vegetationValues=[];
        const appendShrubClump=(cx,baseY,cz,rx,ry,rz,rotation)=>{{
            const sin=Math.sin(rotation),cos=Math.cos(rotation);
            const latitudeSegments=4,longitudeSegments=8;
            const appendVertex=(latitude,longitude)=>{{
                const unit=[Math.cos(latitude)*Math.cos(longitude),Math.sin(latitude),Math.cos(latitude)*Math.sin(longitude)];
                const localX=unit[0]*rx,localZ=unit[2]*rz;
                const x=cx+localX*cos-localZ*sin,y=baseY+ry+unit[1]*ry,z=cz+localX*sin+localZ*cos;
                const nx=unit[0]/rx*cos-unit[2]/rz*sin,ny=unit[1]/ry,nz=unit[0]/rx*sin+unit[2]/rz*cos,norm=Math.hypot(nx,ny,nz);
                vegetationValues.push(x,y,z,nx/norm,ny/norm,nz/norm);
            }};
            for(let latitude=0;latitude<latitudeSegments;latitude++)for(let longitude=0;longitude<longitudeSegments;longitude++){{
                const lat0=-Math.PI/2+latitude*Math.PI/latitudeSegments,lat1=-Math.PI/2+(latitude+1)*Math.PI/latitudeSegments;
                const lon0=longitude*2*Math.PI/longitudeSegments,lon1=(longitude+1)*2*Math.PI/longitudeSegments;
                appendVertex(lat0,lon0);appendVertex(lat1,lon0);appendVertex(lat1,lon1);
                appendVertex(lat0,lon0);appendVertex(lat1,lon1);appendVertex(lat0,lon1);
            }}
        }};
        for(let bush=0;bush<1000;bush++){{
            const nearField=bush<500,spread=nearField?320:groundExtent*0.88;
            let x=(random()*2-1)*spread,z=(random()*2-1)*spread;
            if(nearField&&Math.hypot(x,z)<24){{x+=Math.sign(x||1)*32;z+=Math.sign(z||1)*32;}}
            const baseY=terrainHeight(x,z),height=0.18+random()*0.22,rotation=random()*Math.PI*2;
            appendShrubClump(x,baseY,z,0.16+random()*0.14,height,0.16+random()*0.14,rotation);
            if(random()<0.45)appendShrubClump(x+(random()-0.5)*0.24,baseY,z+(random()-0.5)*0.24,0.13+random()*0.11,height*(0.65+random()*0.30),0.13+random()*0.11,rotation+0.8);
        }}
        const vegetationVertices=new Float32Array(vegetationValues);
        const vegetationBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,vegetationBuffer);gl.bufferData(gl.ARRAY_BUFFER,vegetationVertices,gl.STATIC_DRAW);
        const shadowSegments=48, shadowVertices=new Float32Array((shadowSegments+2)*6);let shadowCursor=0;
        const appendShadowVertex=(x,z)=>{{shadowVertices[shadowCursor++]=x;shadowVertices[shadowCursor++]=0;shadowVertices[shadowCursor++]=z;shadowVertices[shadowCursor++]=0;shadowVertices[shadowCursor++]=1;shadowVertices[shadowCursor++]=0;}};
        appendShadowVertex(0,0);for(let segment=0;segment<=shadowSegments;segment++){{const angle=segment/shadowSegments*Math.PI*2;appendShadowVertex(Math.cos(angle),Math.sin(angle));}}
        const shadowBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,shadowBuffer);gl.bufferData(gl.ARRAY_BUFFER,shadowVertices,gl.STATIC_DRAW);
        gl.enableVertexAttribArray(locations.position); gl.enableVertexAttribArray(locations.normal);
        gl.enable(gl.DEPTH_TEST); gl.disable(gl.CULL_FACE);

        drawScene=(sample)=>{{
            const ratio=window.devicePixelRatio||1, width=canvas.clientWidth, height=canvas.clientHeight;
            if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){{canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);}}
            gl.viewport(0,0,canvas.width,canvas.height); gl.clearColor(0,0,0,0); gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
            const visualAltitude=Math.max(0,sample.alt||0);
            const rocketPosition=[0,terrainHeight(0,0)+visualAltitude*altitudeDisplayScale+rocketHeight*0.5,0];
            const qTelemetry=normalizeQ(sample.q||[1,0,0,0]);
            const qDisplay=displayQuaternion(qTelemetry);
            const forward=rotateVector(qDisplay,[0,0,1]);
            const coastElapsed=coastTransitionTime===null?-1:sample.t-coastTransitionTime;
            // Puffs stay fixed at their emission point. Hide a puff only while
            // it is still ahead of the rocket along its current nose direction.
            const activePuffs=smokePuffs.filter(puff=>{{
                const age=sample.t-puff.t,puffRadius=0.085+age*0.18;
                const alongForward=(puff.center[0]-rocketPosition[0])*forward[0]+(puff.center[1]-rocketPosition[1])*forward[1]+(puff.center[2]-rocketPosition[2])*forward[2];
                return age>=0&&age<smokeLifetime&&alongForward<-(puffRadius+0.12);
            }});
            const cameraDistance=8;
            const target=[0,rocketPosition[1],0];
            const pitchRadians=(0.4+11.6*(visualAltitude/maximumAltitude))*Math.PI/180;
            const eye=[0,target[1]+Math.tan(pitchRadians)*cameraDistance,cameraDistance];
            const mvp=multiplyM(perspective(0.70,canvas.width/canvas.height,0.1,groundExtent*3),lookAt(eye,target,[0,1,0]));
            gl.uniformMatrix4fv(locations.mvp,false,mvp); gl.uniform3f(locations.rocket,...rocketPosition); gl.uniform3f(locations.center,...meshCenter); gl.uniform4f(locations.quat,0,0,0,1);
            gl.uniform1f(locations.isRocket,0);gl.uniform1f(locations.isShadow,0);gl.uniform1f(locations.isGround,1);gl.uniform1f(locations.isVegetation,0);
            gl.bindBuffer(gl.ARRAY_BUFFER,groundBuffer); gl.vertexAttribPointer(locations.position,3,gl.FLOAT,false,24,0); gl.vertexAttribPointer(locations.normal,3,gl.FLOAT,false,24,12); gl.drawArrays(gl.TRIANGLES,0,groundVertices.length/6);
            const shadowX=0,shadowZ=0,shadowY=terrainHeight(shadowX,shadowZ)+0.035,shadowRadius=rocketHeight*0.8,shadowOpacity=0.52;
            gl.uniform1f(locations.isGround,0);gl.uniform1f(locations.isShadow,1);gl.uniform3f(locations.shadowCenter,shadowX,shadowY,shadowZ);gl.uniform1f(locations.shadowRadius,shadowRadius);gl.uniform1f(locations.shadowOpacity,shadowOpacity);
            gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);gl.depthMask(false);
            gl.bindBuffer(gl.ARRAY_BUFFER,shadowBuffer);gl.vertexAttribPointer(locations.position,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(locations.normal,3,gl.FLOAT,false,24,12);gl.drawArrays(gl.TRIANGLE_FAN,0,shadowSegments+2);
            gl.depthMask(true);gl.disable(gl.BLEND);
            gl.uniform1f(locations.isShadow,0);gl.uniform1f(locations.isVegetation,1);
            gl.bindBuffer(gl.ARRAY_BUFFER,vegetationBuffer);gl.vertexAttribPointer(locations.position,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(locations.normal,3,gl.FLOAT,false,24,12);gl.drawArrays(gl.TRIANGLES,0,vegetationVertices.length/6);
            // Match the dashboard's FRD-to-Three conversion: re-express the
            // firmware quaternion by the frame similarity transform, then
            // apply the STL model's +Z-nose to +X base rotation. Composing an
            // NED/world transform directly here swaps the apparent pitch/roll.
            // qDisplay uses firmware-style (w, x, y, z); the GLSL qrot helper
            // takes conventional vector-first (x, y, z, w).
            const plumeVisible=coastTransitionTime===null?sample.state==='4':coastElapsed<0.5;
            if(plumeVisible&&(sample.alt||0)>0.5){{
                const lengthPulse=0.80+0.30*temporalNoise(sample.t,13.7,2.6)+0.12*(temporalNoise(sample.t,37.2,7.3)-0.5);
                const plumeScale=coastElapsed>=0?Math.max(0,0.5*(1-coastElapsed/0.5)):1;
                const activePlumeLength=plumeLength*lengthPulse*plumeScale;
                const activePlumeRootRadius=plumeRootRadius*plumeScale,activePlumeTipRadius=plumeTipRadius*plumeScale;
                const directionX=(temporalNoise(sample.t,21.4,3.3)-0.5)*0.075+(temporalNoise(sample.t,52.8,8.7)-0.5)*0.028;
                const directionY=(temporalNoise(sample.t,32.6,2.9)-0.5)*0.065+(temporalNoise(sample.t,71.9,7.8)-0.5)*0.024;
                gl.uniform1f(locations.isGround,0);gl.uniform1f(locations.isVegetation,0);gl.uniform1f(locations.isShadow,0);gl.uniform1f(locations.isPlume,1);gl.uniform1f(locations.isRocket,1);
                gl.uniform1f(locations.modelScale,modelScale);gl.uniform3f(locations.rocket,...rocketPosition);gl.uniform3f(locations.center,...meshCenter);
                gl.uniform4f(locations.quat,qDisplay[1],qDisplay[2],qDisplay[3],qDisplay[0]);
                gl.uniform1f(locations.plumeBaseZ,plumeBaseZ);gl.uniform1f(locations.plumeLength,activePlumeLength);gl.uniform1f(locations.plumeRootRadius,activePlumeRootRadius);gl.uniform1f(locations.plumeTipRadius,activePlumeTipRadius);
                gl.uniform1f(locations.smokeTime,sample.t);
                gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);gl.depthMask(false);gl.disable(gl.DEPTH_TEST);
                outerFlame.update(sample.t,activePlumeLength,directionX,directionY,plumeScale);innerFlame.update(sample.t,activePlumeLength,directionX,directionY,plumeScale);
                gl.uniform1f(locations.plumeLayer,0);gl.bindBuffer(gl.ARRAY_BUFFER,outerFlame.buffer);gl.vertexAttribPointer(locations.position,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(locations.normal,3,gl.FLOAT,false,24,12);gl.drawArrays(gl.TRIANGLES,0,outerFlame.count);
                gl.uniform1f(locations.plumeLayer,1);gl.bindBuffer(gl.ARRAY_BUFFER,innerFlame.buffer);gl.vertexAttribPointer(locations.position,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(locations.normal,3,gl.FLOAT,false,24,12);gl.drawArrays(gl.TRIANGLES,0,innerFlame.count);
                const emberValues=[],emberCount=18,octahedronFaces=[[0,2,4],[2,1,4],[1,3,4],[3,0,4],[2,0,5],[1,2,5],[3,1,5],[0,3,5]];
                for(let ember=0;ember<emberCount;ember++){{
                    const phase=(ember*0.61803398875)%1,progress=0.58+0.37*temporalNoise(sample.t,phase*31+5.7,1.2),angle=phase*Math.PI*2+(temporalNoise(sample.t,phase*43+11,3.4)-0.5)*2.4;
                    const expanding=smoothStep(0,0.48,progress),tail=1-smoothStep(0.78,1,progress),envelope=(activePlumeRootRadius+(activePlumeTipRadius-activePlumeRootRadius)*expanding)*tail;
                    const radial=envelope*(0.52+0.40*temporalNoise(sample.t,phase*67+19,5.2)),px=meshCenter[0]+Math.cos(angle)*radial+progress*activePlumeLength*directionX,py=meshCenter[1]+Math.sin(angle)*radial+progress*activePlumeLength*directionY,pz=plumeBaseZ+3-progress*activePlumeLength;
                    const size=(11+14*temporalNoise(sample.t,phase*79+23,8.1))*plumeScale,vertices=[[0,size*1.8,0],[size,0,0],[0,0,size*1.4],[-size,0,0],[0,0,-size*1.4],[0,-size*1.8,0]];
                    for(const face of octahedronFaces){{
                        const a=vertices[face[0]],b=vertices[face[1]],c=vertices[face[2]],u=[b[0]-a[0],b[1]-a[1],b[2]-a[2]],v=[c[0]-a[0],c[1]-a[1],c[2]-a[2]],normal=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],norm=Math.hypot(...normal)||1;
                        for(const point of [a,b,c])emberValues.push(px+point[0],py+point[1],pz+point[2],normal[0]/norm,normal[1]/norm,normal[2]/norm);
                    }}
                }}
                gl.uniform1f(locations.isPlume,0);gl.uniform1f(locations.isEmber,1);gl.bindBuffer(gl.ARRAY_BUFFER,emberBuffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(emberValues),gl.DYNAMIC_DRAW);gl.vertexAttribPointer(locations.position,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(locations.normal,3,gl.FLOAT,false,24,12);gl.drawArrays(gl.TRIANGLES,0,emberValues.length/6);gl.uniform1f(locations.isEmber,0);
                gl.enable(gl.DEPTH_TEST);gl.depthMask(true);gl.disable(gl.BLEND);
            }}
            gl.uniform1f(locations.isPlume,0);
            gl.uniform1f(locations.isVegetation,0);gl.uniform1f(locations.isShadow,0);gl.uniform1f(locations.isRocket,1);gl.uniform1f(locations.modelScale,modelScale);gl.uniform1f(locations.mach,sample.mach??0); gl.uniform4f(locations.quat,qDisplay[1],qDisplay[2],qDisplay[3],qDisplay[0]); gl.uniform3f(locations.color,0.93,0.95,0.97);
            gl.bindBuffer(gl.ARRAY_BUFFER,rocketBuffer); gl.vertexAttribPointer(locations.position,3,gl.FLOAT,false,24,0); gl.vertexAttribPointer(locations.normal,3,gl.FLOAT,false,24,12);
            gl.drawArrays(gl.TRIANGLES,0,vertices.length/6);
            const smokeValues=[];
            const appendSmokePuff=(center,age,seed)=>{{
                const ageRatio=age/smokeLifetime, rx=0.075+age*0.19, ry=0.085+age*0.18, rz=0.075+age*0.19;
                const latitudeSegments=4,longitudeSegments=8;
                const appendVertex=(latitude,longitude)=>{{
                    const unit=[Math.cos(latitude)*Math.cos(longitude),Math.sin(latitude),Math.cos(latitude)*Math.sin(longitude)];
                    smokeValues.push(center[0]+unit[0]*rx,center[1]+unit[1]*ry,center[2]+unit[2]*rz,ageRatio,seed,0);
                }};
                for(let latitude=0;latitude<latitudeSegments;latitude++)for(let longitude=0;longitude<longitudeSegments;longitude++){{
                    const lat0=-Math.PI/2+latitude*Math.PI/latitudeSegments,lat1=-Math.PI/2+(latitude+1)*Math.PI/latitudeSegments;
                    const lon0=longitude*2*Math.PI/longitudeSegments,lon1=(longitude+1)*2*Math.PI/longitudeSegments;
                    appendVertex(lat0,lon0);appendVertex(lat1,lon0);appendVertex(lat1,lon1);
                    appendVertex(lat0,lon0);appendVertex(lat1,lon1);appendVertex(lat0,lon1);
                }}
            }};
            for(const puff of activePuffs){{
                const age=sample.t-puff.t;
                appendSmokePuff(puff.center,age,puff.seed);
            }}
            if(smokeValues.length){{
                gl.uniform1f(locations.isRocket,0);gl.uniform1f(locations.isPlume,0);gl.uniform1f(locations.isSmoke,1);gl.uniform1f(locations.smokeTime,sample.t);
                gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);gl.depthMask(false);
                gl.bindBuffer(gl.ARRAY_BUFFER,smokeBuffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(smokeValues),gl.DYNAMIC_DRAW);
                gl.vertexAttribPointer(locations.position,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(locations.normal,3,gl.FLOAT,false,24,12);
                gl.drawArrays(gl.TRIANGLES,0,smokeValues.length/6);gl.depthMask(true);gl.disable(gl.BLEND);gl.uniform1f(locations.isSmoke,0);
            }}
        }};
        status.textContent='';
        setIndex(0);
    }}
    function setIndex(value) {{
        if(!samples.length) return;
        index=Math.max(0,Math.min(samples.length-1,value)); slider.value=String(index);
        const lower=Math.floor(index), upper=Math.min(lower+1,samples.length-1), fraction=index-lower;
        const first=samples[lower], second=samples[upper];
        const time=first.t+(second.t-first.t)*fraction;
        const altitude=first.alt==null?second.alt:(second.alt==null?first.alt:first.alt+(second.alt-first.alt)*fraction);
        const mach=first.mach==null?second.mach:(second.mach==null?first.mach:first.mach+(second.mach-first.mach)*fraction);
        const e=(first.e||[]).map((angle,axis)=>{{const end=(second.e||first.e)[axis]??angle;const delta=((end-angle+540)%360)-180;return angle+delta*fraction;}});
        let q1=first.q||[1,0,0,0], q2=second.q||q1;
        if(q1.reduce((sum,value,component)=>sum+value*q2[component],0)<0) q2=q2.map(value=>-value);
        const sample={{t:time,alt:altitude,mach,e,q:normalizeQ(q1.map((value,component)=>value+(q2[component]-value)*fraction)),state:fraction<0.5?first.state:second.state}};
        document.getElementById('viewerTime').textContent=time.toFixed(2)+' s';
        document.getElementById('viewerAltitude').textContent=altitude==null?'Unavailable':Number(altitude).toFixed(1)+' m ('+(Number(altitude) * 3.28084).toFixed(1)+' ft)';
        document.getElementById('viewerEuler').textContent=(sample.e||[]).map(v=>Number(v).toFixed(1)+'°').join(', ');
        document.getElementById('viewerQuaternion').textContent=(sample.q||[]).map(v=>Number(v).toFixed(3)).join(', ');
        for(const line of cursors){{
            const svg=line.ownerSVGElement, min=Number(svg.dataset.timeMin), max=Number(svg.dataset.timeMax), left=Number(svg.dataset.plotLeft), right=Number(svg.dataset.plotRight);
            const fraction=Math.max(0,Math.min(1,(time-min)/(max-min||1))), x=left+fraction*(right-left);
            line.setAttribute('x1',x.toFixed(2)); line.setAttribute('x2',x.toFixed(2));
        }}
        if(drawScene) drawScene(sample);
    }}
    function indexAtTime(time){{
        if(time<=samples[0].t) return 0;
        if(time>=samples[samples.length-1].t) return samples.length-1;
        let lower=0, upper=samples.length-1;
        while(upper-lower>1){{const middle=Math.floor((lower+upper)/2);if(samples[middle].t<=time)lower=middle;else upper=middle;}}
        const span=samples[upper].t-samples[lower].t;
        return lower+(span>0?(time-samples[lower].t)/span:0);
    }}
    function sampleTimeAtIndex(position){{
        const lower=Math.floor(position), upper=Math.min(lower+1,samples.length-1), fraction=position-lower;
        return samples[lower].t+(samples[upper].t-samples[lower].t)*fraction;
    }}
    slider.addEventListener('input',()=>{{playing=false;playbackStartTimestamp=null;playButton.textContent='Play';setIndex(Number(slider.value));}});
    playButton.addEventListener('click',()=>{{
        if(playing){{playing=false;playbackStartTimestamp=null;playButton.textContent='Play';return;}}
        if(index>=samples.length-1)setIndex(0);
        playing=true;playbackStartTimestamp=null;playButton.textContent='Pause';requestAnimationFrame(animate);
    }});
    function animate(timestamp){{
        if(!playing)return;
        if(playbackStartTimestamp===null){{playbackStartTimestamp=timestamp;playbackStartTime=sampleTimeAtIndex(index);}}
        const targetTime=playbackStartTime+(timestamp-playbackStartTimestamp)/1000;
        if(targetTime>=samples[samples.length-1].t){{setIndex(samples.length-1);playing=false;playbackStartTimestamp=null;playButton.textContent='Play';return;}}
        setIndex(indexAtTime(targetTime));
        requestAnimationFrame(animate);
    }}
    window.addEventListener('resize',()=>{{if(drawScene)drawScene(samples[index]);}});
    initialize().catch(error=>{{status.textContent='3D model could not be loaded: '+error.message;}});
}})();
</script>
</body>
</html>'''
