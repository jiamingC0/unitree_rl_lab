#!/usr/bin/env python3
"""Plot ET1 track observation dump joint references, targets, feedback, and velocity."""

from __future__ import annotations

import argparse
import html
import json
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

try:
    import yaml
except ModuleNotFoundError:
    yaml = None


ARRAY_RE = re.compile(r"^(?P<section>\w+) (?P<name>\w+) size (?P<size>\d+) values (?P<values>.*)$")
FRAME_RE = re.compile(
    r"^frame (?P<frame>\d+) episode_length (?P<episode>\d+) "
    r"time_s (?P<time>[-+0-9.eE]+) reference_frame (?P<ref_frame>\d+) "
    r"reference_time_s (?P<ref_time>[-+0-9.eE]+)$"
)


@dataclass
class FrameData:
    frame: int
    time_s: float
    reference_frame: int
    reference_time_s: float
    ref_joint_pos: np.ndarray | None = None
    obs_current: np.ndarray | None = None
    target_q: np.ndarray | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot per-joint curves from an ET1 Track observation dump."
    )
    parser.add_argument(
        "--dump",
        type=Path,
        default=Path("deploy/robots/et1/debug/nohead_poker_face_observations_20260519_120803.txt"),
        help="Path to *_observations_*.txt.",
    )
    parser.add_argument(
        "--deploy-yaml",
        type=Path,
        default=Path("deploy/robots/et1/config/policy/dance/nohead_poker_face/params/deploy.yaml"),
        help="Policy deploy.yaml used for joint order and default positions.",
    )
    parser.add_argument(
        "--xml",
        type=Path,
        default=Path("deploy/robots/et1/galbot_ET1_0423_26_sdk_collision.xml"),
        help="Robot XML used for joint names.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output directory. Defaults to <dump_stem>_joint_plots beside the dump.",
    )
    parser.add_argument(
        "--degrees",
        action="store_true",
        help="Plot degrees instead of radians.",
    )
    parser.add_argument(
        "--no-individual",
        action="store_true",
        help="Only write overview PNG and PDF, not one PNG per joint.",
    )
    parser.add_argument(
        "--no-html",
        action="store_true",
        help="Do not write interactive per-joint HTML plots.",
    )
    return parser.parse_args()


def parse_array_line(line: str, wanted_keys: set[str]) -> tuple[str, np.ndarray] | None:
    match = ARRAY_RE.match(line)
    if not match:
        return None
    key = f"{match.group('section')} {match.group('name')}"
    if key not in wanted_keys:
        return None
    expected_size = int(match.group("size"))
    values = np.fromstring(match.group("values"), sep=" ", dtype=np.float64)
    if values.size != expected_size:
        raise RuntimeError(
            f"Malformed array line for {match.group('section')} {match.group('name')}: "
            f"expected {expected_size}, got {values.size}"
        )
    return key, values


def load_dump(path: Path) -> List[FrameData]:
    frames: List[FrameData] = []
    current: FrameData | None = None
    wanted_keys = {"ref_command joint_pos", "obs obs_current", "action target_q"}

    with path.open() as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            frame_match = FRAME_RE.match(line)
            if frame_match:
                if current is not None:
                    frames.append(current)
                current = FrameData(
                    frame=int(frame_match.group("frame")),
                    time_s=float(frame_match.group("time")),
                    reference_frame=int(frame_match.group("ref_frame")),
                    reference_time_s=float(frame_match.group("ref_time")),
                )
                continue

            if current is None:
                continue

            parsed = parse_array_line(line, wanted_keys)
            if parsed is None:
                continue

            key, values = parsed
            if key == "ref_command joint_pos":
                current.ref_joint_pos = values
            elif key == "obs obs_current":
                current.obs_current = values
            elif key == "action target_q":
                current.target_q = values

    if current is not None:
        frames.append(current)

    complete = [
        frame
        for frame in frames
        if frame.ref_joint_pos is not None
        and frame.obs_current is not None
        and frame.target_q is not None
    ]
    if not complete:
        raise RuntimeError(f"No complete frames found in {path}")
    return complete


def load_policy_config(path: Path) -> Dict[str, Sequence[float]]:
    with path.open() as f:
        text = f.read()
    if yaml is not None:
        cfg = yaml.safe_load(text)
    else:
        cfg = load_minimal_policy_config(text)
    for key in ("joint_ids_map", "default_joint_pos"):
        if key not in cfg:
            raise RuntimeError(f"Missing {key} in {path}")
    return cfg


def parse_inline_list(text: str, key: str) -> List[float]:
    match = re.search(rf"^{re.escape(key)}:\s*\[(.*?)\]", text, flags=re.MULTILINE | re.DOTALL)
    if not match:
        raise RuntimeError(f"Missing or unsupported list format for {key}")
    return [float(value) for value in re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", match.group(1))]


def load_minimal_policy_config(text: str) -> Dict[str, Sequence[float]]:
    return {
        "joint_ids_map": [int(value) for value in parse_inline_list(text, "joint_ids_map")],
        "default_joint_pos": parse_inline_list(text, "default_joint_pos"),
    }


def load_joint_names(xml_path: Path, joint_ids: Sequence[int]) -> List[str]:
    root = ET.parse(xml_path).getroot()
    actuator = root.find("actuator")
    if actuator is None:
        raise RuntimeError(f"No <actuator> block found in {xml_path}")
    motor_joints = [motor.attrib["joint"] for motor in actuator.findall("motor") if "joint" in motor.attrib]
    names = []
    for joint_id in joint_ids:
        if joint_id < 0 or joint_id >= len(motor_joints):
            raise RuntimeError(f"joint_ids_map entry {joint_id} is out of XML motor range")
        names.append(motor_joints[joint_id])
    return names


def obs_offsets(joint_count: int) -> Dict[str, slice]:
    offset = 0
    terms = [
        ("command_root_ori_b_unbiased", 6),
        ("command_xy_yaw_vel", 3),
        ("command_jnt_pos", joint_count),
        ("projected_gravity", 3),
        ("base_ang_vel", 3),
        ("joint_pos_rel", joint_count),
        ("joint_vel_rel", joint_count),
        ("last_action", joint_count),
        ("command_foot_support_state", 6),
    ]
    slices: Dict[str, slice] = {}
    for name, size in terms:
        slices[name] = slice(offset, offset + size)
        offset += size
    return slices


def collect_series(
    frames: Sequence[FrameData],
    joint_ids: Sequence[int],
    default_joint_pos: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    joint_count = len(joint_ids)
    defaults = np.asarray(default_joint_pos, dtype=np.float64)
    if defaults.size != joint_count:
        raise RuntimeError(
            f"default_joint_pos has {defaults.size} values, expected {joint_count}"
        )

    slices = obs_offsets(joint_count)
    time = np.asarray([frame.time_s for frame in frames], dtype=np.float64)
    ref = []
    target = []
    actual = []
    velocity = []

    for frame in frames:
        assert frame.ref_joint_pos is not None
        assert frame.obs_current is not None
        assert frame.target_q is not None
        if frame.target_q.size != joint_count:
            raise RuntimeError(f"target_q has {frame.target_q.size} values, expected {joint_count}")
        if frame.obs_current.size < slices["command_foot_support_state"].stop:
            raise RuntimeError(
                f"obs_current has {frame.obs_current.size} values, too short for {joint_count} joints"
            )
        if max(joint_ids) >= frame.ref_joint_pos.size:
            raise RuntimeError(
                f"ref_command joint_pos has {frame.ref_joint_pos.size} values, "
                f"but joint_ids_map needs index {max(joint_ids)}"
            )

        ref.append(frame.ref_joint_pos[np.asarray(joint_ids, dtype=np.int64)])
        target.append(frame.target_q)
        actual.append(frame.obs_current[slices["joint_pos_rel"]] + defaults)
        velocity.append(frame.obs_current[slices["joint_vel_rel"]])

    return time - time[0], np.vstack(ref), np.vstack(target), np.vstack(actual), np.vstack(velocity)


def numerical_derivative(values: np.ndarray, time: np.ndarray) -> np.ndarray:
    if values.shape[0] != time.size:
        raise RuntimeError("Cannot differentiate: time and value lengths differ")
    if time.size < 2:
        return np.zeros_like(values)
    return np.gradient(values, time, axis=0)


def maybe_convert_units(
    degrees: bool,
    arrays: Sequence[np.ndarray],
) -> tuple[str, List[np.ndarray]]:
    if not degrees:
        return "rad", list(arrays)
    return "deg", [np.rad2deg(array) for array in arrays]


def plot_one_joint(
    output_path: Path | None,
    time: np.ndarray,
    joint_name: str,
    joint_index: int,
    ref: np.ndarray,
    target: np.ndarray,
    actual: np.ndarray,
    velocity: np.ndarray,
    position_derivative: np.ndarray,
    unit: str,
    pdf: PdfPages | None = None,
) -> None:
    fig, ax_pos = plt.subplots(1, 1, figsize=(12, 5.5))
    ax_vel = ax_pos.twinx()

    velocity_unit = f"{unit}/s"
    pos_lines = [
        ax_pos.plot(time, ref[:, joint_index], label="reference", linewidth=1.3)[0],
        ax_pos.plot(time, target[:, joint_index], label="policy_target_q", linewidth=1.2)[0],
        ax_pos.plot(time, actual[:, joint_index], label="observed_position", linewidth=1.0)[0],
    ]
    vel_line = ax_vel.plot(
        time,
        velocity[:, joint_index],
        label="observed_velocity",
        linewidth=1.0,
        color="tab:purple",
        alpha=0.85,
    )[0]
    diff_line = ax_vel.plot(
        time,
        position_derivative[:, joint_index],
        label="d_observed_position/dt",
        linewidth=1.0,
        color="tab:red",
        linestyle="--",
        alpha=0.85,
    )[0]
    ax_vel.axhline(0.0, color="tab:purple", linewidth=0.6, alpha=0.25)
    ax_pos.set_ylabel(f"position ({unit})")
    ax_vel.set_ylabel(f"velocity ({velocity_unit})")
    ax_pos.set_xlabel("time (s)")
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend(handles=pos_lines + [vel_line, diff_line], loc="best")

    fig.suptitle(f"{joint_index:02d} {joint_name}")
    fig.tight_layout()
    if output_path is not None:
        fig.savefig(output_path, dpi=150)
    if pdf is not None:
        pdf.savefig(fig)
    plt.close(fig)


def plot_overview(
    output_path: Path,
    time: np.ndarray,
    names: Sequence[str],
    ref: np.ndarray,
    target: np.ndarray,
    actual: np.ndarray,
    velocity: np.ndarray,
    unit: str,
) -> None:
    joint_count = len(names)
    cols = 4
    rows = int(np.ceil(joint_count / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(18, 3.1 * rows), sharex=True)
    flat_axes = np.asarray(axes).reshape(-1)

    for index, ax in enumerate(flat_axes):
        if index >= joint_count:
            ax.axis("off")
            continue
        ax.plot(time, ref[:, index], label="ref", linewidth=0.9)
        ax.plot(time, target[:, index], label="target", linewidth=0.9)
        ax.plot(time, actual[:, index], label="observed", linewidth=0.8)
        ax.set_title(f"{index:02d} {names[index]}", fontsize=9)
        ax.grid(True, alpha=0.25)
        ax.tick_params(labelsize=8)

    flat_axes[0].legend(loc="best", fontsize=8)
    fig.supxlabel("time (s)")
    fig.supylabel(f"position ({unit})")
    fig.suptitle("ET1 Track Joint Positions")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def save_csv(
    output_path: Path,
    time: np.ndarray,
    names: Sequence[str],
    ref: np.ndarray,
    target: np.ndarray,
    actual: np.ndarray,
    velocity: np.ndarray,
    position_derivative: np.ndarray,
) -> None:
    columns = ["time_s"]
    for name in names:
        columns.extend(
            [
                f"{name}.reference",
                f"{name}.target_q",
                f"{name}.observed_position",
                f"{name}.observed_velocity",
                f"{name}.d_observed_position_dt",
            ]
        )

    data_columns = [time]
    for index in range(len(names)):
        data_columns.extend(
            [
                ref[:, index],
                target[:, index],
                actual[:, index],
                velocity[:, index],
                position_derivative[:, index],
            ]
        )
    data = np.column_stack(data_columns)

    np.savetxt(
        output_path,
        data,
        delimiter=",",
        header=",".join(columns),
        comments="",
        fmt="%.9g",
    )


def compact_values(values: np.ndarray) -> List[float]:
    return [round(float(value), 7) for value in values]


def write_joint_html(
    output_path: Path,
    time: np.ndarray,
    joint_name: str,
    joint_index: int,
    ref: np.ndarray,
    target: np.ndarray,
    actual: np.ndarray,
    velocity: np.ndarray,
    position_derivative: np.ndarray,
    unit: str,
) -> None:
    title = f"{joint_index:02d} {joint_name}"
    payload = {
        "title": title,
        "unit": unit,
        "time": compact_values(time),
        "position": [
            {"name": "reference", "color": "#1f77b4", "y": compact_values(ref[:, joint_index])},
            {"name": "policy_target_q", "color": "#ff7f0e", "y": compact_values(target[:, joint_index])},
            {"name": "observed_position", "color": "#2ca02c", "y": compact_values(actual[:, joint_index])},
        ],
        "velocity": [
            {"name": "observed_velocity", "color": "#9467bd", "y": compact_values(velocity[:, joint_index])},
            {
                "name": "d_observed_position/dt",
                "color": "#d62728",
                "dash": True,
                "y": compact_values(position_derivative[:, joint_index]),
            },
        ],
    }
    data_json = json.dumps(payload, separators=(",", ":"))
    escaped_title = html.escape(title)
    output_path.write_text(
        f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{escaped_title}</title>
<style>
body {{ margin: 0; font-family: Arial, sans-serif; color: #222; background: #fff; }}
header {{ padding: 12px 16px 4px; }}
h1 {{ margin: 0 0 6px; font-size: 20px; font-weight: 600; }}
.hint {{ color: #666; font-size: 13px; }}
#toolbar {{ display: flex; gap: 10px; align-items: center; padding: 8px 16px; flex-wrap: wrap; }}
button {{ padding: 5px 10px; border: 1px solid #bbb; background: #f8f8f8; border-radius: 4px; cursor: pointer; }}
label {{ display: inline-flex; gap: 5px; align-items: center; font-size: 13px; }}
#chart {{ display: block; width: 100vw; height: calc(100vh - 105px); min-height: 520px; cursor: crosshair; }}
</style>
</head>
<body>
<header>
<h1>{escaped_title}</h1>
<div class="hint">Wheel: zoom time axis. Drag: pan time axis. Shift + wheel/drag: adjust y axes. Double click: reset.</div>
</header>
<div id="toolbar"><button id="reset">Reset</button><span id="legend"></span></div>
<canvas id="chart"></canvas>
<script>
const DATA = {data_json};
const canvas = document.getElementById("chart");
const ctx = canvas.getContext("2d");
const legend = document.getElementById("legend");
const allTraces = DATA.position.concat(DATA.velocity);
const visible = new Map(allTraces.map(t => [t.name, true]));
for (const t of allTraces) {{
  const label = document.createElement("label");
  const input = document.createElement("input");
  input.type = "checkbox";
  input.checked = true;
  input.onchange = () => {{ visible.set(t.name, input.checked); draw(); }};
  const swatch = document.createElement("span");
  swatch.style.cssText = `display:inline-block;width:18px;height:3px;background:${{t.color}}`;
  label.append(input, swatch, document.createTextNode(t.name));
  legend.appendChild(label);
}}
function extent(arrs) {{
  let lo = Infinity, hi = -Infinity;
  for (const arr of arrs) for (const v of arr) {{ if (v < lo) lo = v; if (v > hi) hi = v; }}
  if (!isFinite(lo) || lo === hi) return [lo - 1, hi + 1];
  const pad = (hi - lo) * 0.08;
  return [lo - pad, hi + pad];
}}
const full = {{
  x: [DATA.time[0], DATA.time[DATA.time.length - 1]],
  pos: extent(DATA.position.map(t => t.y)),
  vel: extent(DATA.velocity.map(t => t.y)),
}};
let view = JSON.parse(JSON.stringify(full));
const margin = {{left: 70, right: 76, top: 18, mid: 34, bottom: 46}};
function resize() {{
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}}
function panel() {{
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const plotW = w - margin.left - margin.right;
  const plotH = h - margin.top - margin.bottom;
  return {{x: view.x, left: margin.left, top: margin.top, width: plotW, height: plotH}};
}}
function sx(p, x) {{ return p.left + (x - p.x[0]) / (p.x[1] - p.x[0]) * p.width; }}
function sy(p, domain, y) {{ return p.top + p.height - (y - domain[0]) / (domain[1] - domain[0]) * p.height; }}
function invx(p, px) {{ return p.x[0] + (px - p.left) / p.width * (p.x[1] - p.x[0]); }}
function invy(p, domain, py) {{ return domain[0] + (p.top + p.height - py) / p.height * (domain[1] - domain[0]); }}
function ticks(lo, hi, n = 5) {{
  const out = [];
  for (let i = 0; i <= n; i++) out.push(lo + (hi - lo) * i / n);
  return out;
}}
function drawPanel(p) {{
  ctx.save();
  ctx.strokeStyle = "#333"; ctx.lineWidth = 1;
  ctx.strokeRect(p.left, p.top, p.width, p.height);
  ctx.fillStyle = "#222"; ctx.font = "12px Arial";
  ctx.textAlign = "right"; ctx.textBaseline = "middle";
  ctx.strokeStyle = "#ddd"; ctx.lineWidth = 1;
  for (const y of ticks(view.pos[0], view.pos[1])) {{
    const yy = sy(p, view.pos, y);
    ctx.beginPath(); ctx.moveTo(p.left, yy); ctx.lineTo(p.left + p.width, yy); ctx.stroke();
    ctx.fillText(y.toFixed(2), p.left - 8, yy);
  }}
  ctx.textAlign = "left";
  for (const y of ticks(view.vel[0], view.vel[1])) {{
    const yy = sy(p, view.vel, y);
    ctx.fillStyle = "#6f3fa0"; ctx.fillText(y.toFixed(2), p.left + p.width + 8, yy);
  }}
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  for (const x of ticks(p.x[0], p.x[1], 7)) {{
    const xx = sx(p, x);
    ctx.beginPath(); ctx.moveTo(xx, p.top); ctx.lineTo(xx, p.top + p.height); ctx.stroke();
    ctx.fillStyle = "#222"; ctx.fillText(x.toFixed(1), xx, p.top + p.height + 8);
  }}
  ctx.fillText("time (s)", p.left + p.width / 2, p.top + p.height + 28);
  ctx.save();
  ctx.translate(18, p.top + p.height / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.fillStyle = "#222"; ctx.fillText(`position (${{DATA.unit}})`, 0, 0);
  ctx.restore();
  ctx.save();
  ctx.translate(canvas.clientWidth - 12, p.top + p.height / 2);
  ctx.rotate(Math.PI / 2);
  ctx.fillStyle = "#6f3fa0"; ctx.fillText(`velocity (${{DATA.unit}}/s)`, 0, 0);
  ctx.restore();
  ctx.beginPath(); ctx.rect(p.left, p.top, p.width, p.height); ctx.clip();
  for (const t of DATA.position) {{
    if (!visible.get(t.name)) continue;
    ctx.strokeStyle = t.color; ctx.lineWidth = 1.5; ctx.beginPath();
    let started = false;
    for (let i = 0; i < DATA.time.length; i++) {{
      const x = DATA.time[i];
      if (x < p.x[0] || x > p.x[1]) continue;
      const px = sx(p, x), py = sy(p, view.pos, t.y[i]);
      if (!started) {{ ctx.moveTo(px, py); started = true; }} else ctx.lineTo(px, py);
    }}
    ctx.stroke();
  }}
  for (const t of DATA.velocity) {{
    if (!visible.get(t.name)) continue;
    ctx.strokeStyle = t.color; ctx.lineWidth = 1.2; ctx.beginPath();
    if (t.dash) ctx.setLineDash([6, 4]); else ctx.setLineDash([]);
    let started = false;
    for (let i = 0; i < DATA.time.length; i++) {{
      const x = DATA.time[i];
      if (x < p.x[0] || x > p.x[1]) continue;
      const px = sx(p, x), py = sy(p, view.vel, t.y[i]);
      if (!started) {{ ctx.moveTo(px, py); started = true; }} else ctx.lineTo(px, py);
    }}
    ctx.stroke();
    ctx.setLineDash([]);
  }}
  ctx.restore();
}}
function draw() {{
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  drawPanel(panel());
}}
function panelAt(x, y) {{
  const p = panel();
  return x >= p.left && x <= p.left + p.width && y >= p.top && y <= p.top + p.height ? p : null;
}}
canvas.addEventListener("wheel", e => {{
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const x = e.clientX - rect.left, y = e.clientY - rect.top;
  const p = panelAt(x, y); if (!p) return;
  const factor = e.deltaY < 0 ? 0.82 : 1.22;
  const cx = invx(p, x), cyPos = invy(p, view.pos, y), cyVel = invy(p, view.vel, y);
  if (e.shiftKey) {{
    view.pos = [cyPos + (view.pos[0] - cyPos) * factor, cyPos + (view.pos[1] - cyPos) * factor];
    view.vel = [cyVel + (view.vel[0] - cyVel) * factor, cyVel + (view.vel[1] - cyVel) * factor];
  }} else {{
    view.x = [cx + (view.x[0] - cx) * factor, cx + (view.x[1] - cx) * factor];
  }}
  draw();
}}, {{passive: false}});
let drag = null;
canvas.addEventListener("mousedown", e => {{
  const rect = canvas.getBoundingClientRect();
  const x = e.clientX - rect.left, y = e.clientY - rect.top;
  const p = panelAt(x, y); if (!p) return;
  drag = {{x, y, yOnly: e.shiftKey, view: JSON.parse(JSON.stringify(view))}};
}});
window.addEventListener("mousemove", e => {{
  if (!drag) return;
  const rect = canvas.getBoundingClientRect();
  const x = e.clientX - rect.left, y = e.clientY - rect.top;
  const p = panel();
  const dx = (x - drag.x) / p.width * (drag.view.x[1] - drag.view.x[0]);
  const dyPos = (y - drag.y) / p.height * (drag.view.pos[1] - drag.view.pos[0]);
  const dyVel = (y - drag.y) / p.height * (drag.view.vel[1] - drag.view.vel[0]);
  if (drag.yOnly) {{
    view.pos = [drag.view.pos[0] + dyPos, drag.view.pos[1] + dyPos];
    view.vel = [drag.view.vel[0] + dyVel, drag.view.vel[1] + dyVel];
  }} else {{
    view.x = [drag.view.x[0] - dx, drag.view.x[1] - dx];
  }}
  draw();
}});
window.addEventListener("mouseup", () => drag = null);
canvas.addEventListener("dblclick", () => {{ view = JSON.parse(JSON.stringify(full)); draw(); }});
document.getElementById("reset").onclick = () => {{ view = JSON.parse(JSON.stringify(full)); draw(); }};
window.addEventListener("resize", resize);
resize();
</script>
</body>
</html>
""",
        encoding="utf-8",
    )


def write_index_html(output_path: Path, html_paths: Sequence[Path]) -> None:
    links = "\n".join(
        f'<li><a href="{html.escape(path.name)}">{html.escape(path.stem)}</a></li>'
        for path in html_paths
    )
    output_path.write_text(
        f"""<!doctype html>
<html lang="en">
<head><meta charset="utf-8"><title>ET1 joint plots</title></head>
<body><h1>ET1 joint plots</h1><ul>{links}</ul></body>
</html>
""",
        encoding="utf-8",
    )


def main() -> None:
    args = parse_args()
    dump_path = args.dump
    output_dir = args.output_dir or dump_path.parent / f"{dump_path.stem}_joint_plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    cfg = load_policy_config(args.deploy_yaml)
    joint_ids = [int(value) for value in cfg["joint_ids_map"]]
    joint_names = load_joint_names(args.xml, joint_ids)
    frames = load_dump(dump_path)
    time, ref, target, actual, velocity = collect_series(frames, joint_ids, cfg["default_joint_pos"])
    position_derivative = numerical_derivative(actual, time)
    unit, converted = maybe_convert_units(args.degrees, [ref, target, actual, velocity, position_derivative])
    ref, target, actual, velocity, position_derivative = converted

    overview_path = output_dir / "overview_all_joints.png"
    pdf_path = output_dir / "per_joint_curves.pdf"
    csv_path = output_dir / "joint_curves.csv"

    plot_overview(overview_path, time, joint_names, ref, target, actual, velocity, unit)
    save_csv(csv_path, time, joint_names, ref, target, actual, velocity, position_derivative)

    with PdfPages(pdf_path) as pdf:
        html_paths = []
        for index, name in enumerate(joint_names):
            png_path = None
            if not args.no_individual:
                png_path = output_dir / f"joint_{index:02d}_{name}.png"
            plot_one_joint(
                png_path,
                time,
                name,
                index,
                ref,
                target,
                actual,
                velocity,
                position_derivative,
                unit,
                pdf=pdf,
            )
            if not args.no_html:
                html_path = output_dir / f"joint_{index:02d}_{name}.html"
                write_joint_html(
                    html_path,
                    time,
                    name,
                    index,
                    ref,
                    target,
                    actual,
                    velocity,
                    position_derivative,
                    unit,
                )
                html_paths.append(html_path)

    if not args.no_html:
        write_index_html(output_dir / "index.html", html_paths)

    print(f"frames: {len(frames)}")
    print(f"joints: {len(joint_names)}")
    print(f"overview: {overview_path}")
    print(f"pdf: {pdf_path}")
    print(f"csv: {csv_path}")
    if not args.no_html:
        print(f"html_index: {output_dir / 'index.html'}")
    if not args.no_individual:
        print(f"individual_png_dir: {output_dir}")


if __name__ == "__main__":
    main()
