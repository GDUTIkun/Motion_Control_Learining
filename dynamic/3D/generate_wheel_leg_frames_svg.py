"""Generate an editable SVG schematic for the minimal 3D wheel-leg model.

This intentionally does not use TikZ, matplotlib, mplot3d, Plotly, Blender, or
any camera-based 3D renderer.  It writes plain SVG groups and converts the SVG
to PDF/PNG with CairoSVG when available.

Stages:
  1: main robot structure + W/A/B frames, no labels
  2: add main psi, theta, l labels/arcs
  3: add two right-side inset diagrams
  4: add all final labels
"""

from __future__ import annotations

import argparse
import html
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import cairosvg
import numpy as np


CANVAS_W = 1600
CANVAS_H = 900
ROOT = Path(r"D:\Workspace\CodeWorkspace\dynamic\3D")
OUT = ROOT / "out"
OUT.mkdir(parents=True, exist_ok=True)


@dataclass(frozen=True)
class Colors:
    x: str = "#d62728"
    y: str = "#1b8e32"
    z: str = "#135ecf"
    robot: str = "#252525"
    robot2: str = "#3a3a3a"
    body_fill: str = "#e9f1fb"
    body_fill2: str = "#d8e7f7"
    aux: str = "#b9b9b9"
    angle: str = "#7b2cbf"
    actuator: str = "#e68613"
    panel: str = "#f7f7f7"


C = Colors()


@dataclass(frozen=True)
class MainGeom:
    origin: np.ndarray = field(default_factory=lambda: np.array([500.0, 665.0]))
    scale: float = 122.0
    psi: float = math.radians(28.0)
    theta: float = math.radians(15.0)
    beta: float = math.radians(16.0)
    rho: float = math.radians(13.0)
    track: float = 4.0
    leg: float = 3.55
    wheel_rx: float = 39.0
    wheel_ry: float = 82.0
    body_x: float = 1.45
    body_y: float = 1.12
    body_z: float = 0.72


G = MainGeom()


def esc(s: str) -> str:
    return html.escape(s, quote=True)


def v(x: float, y: float, z: float = 0.0) -> np.ndarray:
    return np.array([x, y, z], dtype=float)


def Rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def project3(p: np.ndarray) -> np.ndarray:
    """Fixed orthographic oblique projection, no perspective.

    Model coordinates map as:
      X -> right
      Y -> down-left
      Z -> up
    screen = O + scale * [X - 0.58Y, -Z + 0.33Y]
    """
    x, y, z = p
    return G.origin + G.scale * np.array([x - 0.58 * y, -z + 0.33 * y])


def fmt(p: np.ndarray) -> str:
    return f"{p[0]:.2f},{p[1]:.2f}"


def angle_of(vec2: np.ndarray) -> float:
    return math.atan2(vec2[1], vec2[0])


def unit2(vec2: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(vec2))
    if n < 1e-9:
        raise ValueError("zero 2D vector")
    return vec2 / n


class SVG:
    def __init__(self) -> None:
        self.parts: list[str] = []

    def add(self, s: str) -> None:
        self.parts.append(s)

    def group(self, gid: str, content: str, attrs: str = "") -> None:
        self.add(f'<g id="{esc(gid)}" {attrs}>\n{content}\n</g>\n')

    def render(self) -> str:
        defs = f"""
<defs>
  <marker id="arrow-x" markerWidth="10" markerHeight="10" refX="8.6" refY="5" orient="auto" markerUnits="strokeWidth">
    <path d="M 0 0 L 10 5 L 0 10 z" fill="{C.x}"/>
  </marker>
  <marker id="arrow-y" markerWidth="10" markerHeight="10" refX="8.6" refY="5" orient="auto" markerUnits="strokeWidth">
    <path d="M 0 0 L 10 5 L 0 10 z" fill="{C.y}"/>
  </marker>
  <marker id="arrow-z" markerWidth="10" markerHeight="10" refX="8.6" refY="5" orient="auto" markerUnits="strokeWidth">
    <path d="M 0 0 L 10 5 L 0 10 z" fill="{C.z}"/>
  </marker>
  <marker id="arrow-angle" markerWidth="11" markerHeight="11" refX="0" refY="5.5" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M 0 0 L 11 5.5 L 0 11 z" fill="{C.angle}"/>
  </marker>
  <marker id="arrow-actuator" markerWidth="13" markerHeight="13" refX="10.8" refY="6.5" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M 0 0 L 13 6.5 L 0 13 z" fill="{C.actuator}"/>
  </marker>
  <marker id="arrow-actuator-arc" markerWidth="13" markerHeight="13" refX="0" refY="6.5" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M 0 0 L 13 6.5 L 0 13 z" fill="{C.actuator}"/>
  </marker>
  <marker id="arrow-force" markerWidth="13" markerHeight="13" refX="10.8" refY="6.5" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M 0 0 L 13 6.5 L 0 13 z" fill="{C.actuator}"/>
  </marker>
  <marker id="arrow-ref" markerWidth="9" markerHeight="9" refX="7.6" refY="4.5" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M 0 0 L 9 4.5 L 0 9 z" fill="{C.aux}"/>
  </marker>
  <marker id="arrow-dim" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto" markerUnits="strokeWidth">
    <path d="M 0 0 L 8 4 L 0 8 z" fill="#555"/>
  </marker>
  <style>
    .math {{ font-family: "Cambria Math", "Times New Roman", serif; font-style: italic; }}
    .serif {{ font-family: "Times New Roman", "Noto Serif", serif; }}
  </style>
</defs>
"""
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_W}" height="{CANVAS_H}" '
            f'viewBox="0 0 {CANVAS_W} {CANVAS_H}">\n'
            f'<rect width="100%" height="100%" fill="white"/>\n'
            f'{defs}\n'
            + "".join(self.parts)
            + "\n</svg>\n"
        )


def line(p1: np.ndarray, p2: np.ndarray, color: str, width: float, extra: str = "") -> str:
    return f'<line x1="{p1[0]:.2f}" y1="{p1[1]:.2f}" x2="{p2[0]:.2f}" y2="{p2[1]:.2f}" stroke="{color}" stroke-width="{width}" stroke-linecap="round" {extra}/>\n'


def poly(points: Iterable[np.ndarray], fill: str, stroke: str, width: float, extra: str = "") -> str:
    pts = " ".join(fmt(p) for p in points)
    return f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" stroke-linejoin="round" {extra}/>\n'


def circle(p: np.ndarray, r: float, fill: str = "white", stroke: str = C.robot, width: float = 3.0, extra: str = "") -> str:
    return f'<circle cx="{p[0]:.2f}" cy="{p[1]:.2f}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" {extra}/>\n'


def ellipse(center: np.ndarray, rx: float, ry: float, rot_deg: float, stroke: str, width: float, fill: str = "none") -> str:
    return (
        f'<ellipse cx="{center[0]:.2f}" cy="{center[1]:.2f}" rx="{rx:.2f}" ry="{ry:.2f}" '
        f'transform="rotate({rot_deg:.2f} {center[0]:.2f} {center[1]:.2f})" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>\n'
    )


def text(x: float, y: float, content: str, size: int = 30, color: str = "#111", anchor: str = "middle",
         italic: bool = False, klass: str = "serif", extra: str = "") -> str:
    base_style = f"font-size:{size}px;"
    fg_style = base_style + f"fill:{color};"
    bg_style = base_style + "fill:none;stroke:white;stroke-width:6px;stroke-linejoin:round;"
    if italic:
        fg_style += "font-style:italic;"
        bg_style += "font-style:italic;"
    return (
        f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" class="{klass}" '
        f'style="{bg_style}" {extra}>{content}</text>\n'
        f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" class="{klass}" '
        f'style="{fg_style}" {extra}>{content}</text>\n'
    )


def sub_label(x: float, y: float, base: str, sub: str, size: int, color: str, anchor: str = "middle") -> str:
    subsize = int(size * 0.68)
    tspans = (
        f'<tspan>{esc(base)}</tspan>'
        f'<tspan baseline-shift="sub" font-size="{subsize}px">{esc(sub)}</tspan>'
    )
    return (
        f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" class="math" '
        f'style="font-size:{size}px;fill:none;stroke:white;stroke-width:6px;stroke-linejoin:round;">{tspans}</text>\n'
        f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" class="math" '
        f'style="font-size:{size}px;fill:{color};">{tspans}</text>\n'
    )


def frame_name(x: float, y: float, name: str, size: int = 32) -> str:
    return text(x, y, "{" + esc(name) + "}", size=size, color="#111", anchor="middle", klass="serif")


def axis_arrow(origin: np.ndarray, direction: np.ndarray, length: float, color: str, marker_id: str) -> str:
    end = origin + length * unit2(direction)
    return line(origin, end, color, 3.0, f'marker-end="url(#{marker_id})"')


def arc_path(center: np.ndarray, radius: float, start_deg: float, end_deg: float, color: str = C.angle,
             width: float = 3.0, marker: bool = True, marker_id: str = "arrow-angle") -> str:
    a0, a1 = math.radians(start_deg), math.radians(end_deg)
    p0 = center + radius * np.array([math.cos(a0), math.sin(a0)])
    p1 = center + radius * np.array([math.cos(a1), math.sin(a1)])
    sweep = 1 if end_deg > start_deg else 0
    large = 1 if abs(end_deg - start_deg) > 180 else 0
    mark = f'marker-end="url(#{marker_id})"' if marker else ""
    return (
        f'<path d="M {fmt(p0)} A {radius:.2f},{radius:.2f} 0 {large} {sweep} {fmt(p1)}" '
        f'fill="none" stroke="{color}" stroke-width="{width}" stroke-linecap="round" {mark}/>\n'
    )


def ellipse_arc_path(center: np.ndarray, rx: float, ry: float, rot_deg: float,
                     start_deg: float, end_deg: float, color: str, width: float,
                     marker_id: str = "arrow-angle") -> str:
    rot = math.radians(rot_deg)
    R = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])

    def pt(a_deg: float) -> np.ndarray:
        a = math.radians(a_deg)
        return center + R @ np.array([rx * math.cos(a), ry * math.sin(a)])

    p0 = pt(start_deg)
    p1 = pt(end_deg)
    sweep = 1 if end_deg > start_deg else 0
    large = 1 if abs(end_deg - start_deg) > 180 else 0
    return (
        f'<path d="M {fmt(p0)} A {rx:.2f},{ry:.2f} {rot_deg:.2f} {large} {sweep} {fmt(p1)}" '
        f'fill="none" stroke="{color}" stroke-width="{width}" stroke-linecap="round" '
        f'marker-end="url(#{marker_id})"/>\n'
    )


def body_vertices(center3: np.ndarray, x_axis3: np.ndarray, y_axis3: np.ndarray, z_axis3: np.ndarray) -> dict[tuple[int, int, int], np.ndarray]:
    hx, hy, hz = G.body_x / 2, G.body_y / 2, G.body_z / 2
    out: dict[tuple[int, int, int], np.ndarray] = {}
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                p3 = center3 + sx * hx * x_axis3 + sy * hy * y_axis3 + sz * hz * z_axis3
                out[(sx, sy, sz)] = project3(p3)
    return out


def draw_body(center3: np.ndarray, x_axis3: np.ndarray, y_axis3: np.ndarray, z_axis3: np.ndarray) -> str:
    c = body_vertices(center3, x_axis3, y_axis3, z_axis3)
    # Three visible faces, solid fills, few edges.
    s = ""
    s += poly([c[(-1, -1, -1)], c[(1, -1, -1)], c[(1, 1, -1)], c[(-1, 1, -1)]], C.body_fill2, C.robot2, 4)
    s += poly([c[(-1, -1, 1)], c[(1, -1, 1)], c[(1, 1, 1)], c[(-1, 1, 1)]], C.body_fill, C.robot2, 4)
    s += poly([c[(1, -1, -1)], c[(1, 1, -1)], c[(1, 1, 1)], c[(1, -1, 1)]], "#dceafa", C.robot2, 4)
    # Minimal back/top connection edges.
    for a, b in [
        ((-1, -1, -1), (-1, -1, 1)),
        ((-1, 1, -1), (-1, 1, 1)),
        ((1, -1, -1), (1, -1, 1)),
        ((1, 1, -1), (1, 1, 1)),
    ]:
        s += line(c[a], c[b], C.robot2, 3.0)
    return s


def main_geometry():
    R_WA = Rz(G.psi)
    R_AB = Ry(G.beta) @ Rx(G.rho)
    R_WB = R_WA @ R_AB
    ex, ey, ez = v(1, 0, 0), v(0, 1, 0), v(0, 0, 1)
    xA3, yA3, zA3 = R_WA @ ex, R_WA @ ey, ez
    xB3, yB3, zB3 = R_WB @ ex, R_WB @ ey, R_WB @ ez
    OA3 = v(0, 0, 0)
    CL3 = OA3 + 0.5 * G.track * yA3
    CR3 = OA3 - 0.5 * G.track * yA3
    OB3 = OA3 + G.leg * (math.sin(G.theta) * xA3 + math.cos(G.theta) * zA3)
    return {
        "R_WA": R_WA, "R_WB": R_WB,
        "xA3": xA3, "yA3": yA3, "zA3": zA3,
        "xB3": xB3, "yB3": yB3, "zB3": zB3,
        "OA3": OA3, "CL3": CL3, "CR3": CR3, "OB3": OB3,
        "OA": project3(OA3), "CL": project3(CL3), "CR": project3(CR3), "OB": project3(OB3),
        "xA2": project3(OA3 + xA3) - project3(OA3),
        "yA2": project3(OA3 + yA3) - project3(OA3),
        "zA2": project3(OA3 + zA3) - project3(OA3),
        "xB2": project3(OB3 + xB3) - project3(OB3),
        "yB2": project3(OB3 + yB3) - project3(OB3),
        "zB2": project3(OB3 + zB3) - project3(OB3),
    }


def main_structure(stage: int) -> str:
    g = main_geometry()
    OA, CL, CR, OB = g["OA"], g["CL"], g["CR"], g["OB"]
    s = ""
    # ground plane
    xA = unit2(g["xA2"])
    yA = unit2(g["yA2"])
    gp = [OA - 230 * xA - 330 * yA, OA + 470 * xA - 330 * yA, OA + 470 * xA + 330 * yA, OA - 230 * xA + 330 * yA]
    s += poly(gp, "#f8f8f8", "#d6d6d6", 2)

    # axle and wheels
    s += '<g id="main_wheels_and_axle">\n'
    s += line(CL, CR, C.robot, 5.0)
    wheel_rot = math.degrees(angle_of(g["zA2"])) + 90
    for cid, cc in [("left_wheel", CL), ("right_wheel", CR)]:
        s += f'<g id="{cid}">\n'
        s += ellipse(cc, G.wheel_rx, G.wheel_ry, wheel_rot, C.robot, 4.0, "white")
        s += circle(cc, 7.0, "white", C.robot, 3.0)
        s += line(cc - np.array([0, 46]), cc + np.array([0, 46]), C.robot, 2.0)
        s += line(cc - np.array([30, 0]), cc + np.array([30, 0]), C.robot, 2.0)
        s += '</g>\n'
    s += circle(OA, 8.0, C.robot, C.robot, 1.0)
    s += '</g>\n'

    # rod and body
    s += '<g id="main_rod_and_body">\n'
    s += line(OA, OB, C.robot2, 5.0)
    s += circle(OB, 8.5, C.robot, C.robot, 1.0)
    s += '</g>\n'

    # frames without labels at stage 1; labels added later.
    s += '<g id="frames_world">\n'
    OW = np.array([88.0, 748.0])
    s += axis_arrow(OW, np.array([1, 0]), 82, C.x, "arrow-x")
    s += axis_arrow(OW, np.array([-0.85, 0.48]), 72, C.y, "arrow-y")
    s += axis_arrow(OW, np.array([0, -1]), 112, C.z, "arrow-z")
    s += circle(OW, 4.0, "#666", "#666", 1)
    s += '</g>\n'

    s += '<g id="frames_A">\n'
    s += axis_arrow(OA, g["xA2"], 150, C.x, "arrow-x")
    s += axis_arrow(OA, g["yA2"], 145, C.y, "arrow-y")
    s += axis_arrow(OA, g["zA2"], 142, C.z, "arrow-z")
    s += '</g>\n'

    s += '<g id="frames_B">\n'
    s += axis_arrow(OB, g["xB2"], 108, C.x, "arrow-x")
    s += axis_arrow(OB, g["yB2"], 108, C.y, "arrow-y")
    s += axis_arrow(OB, g["zB2"], 108, C.z, "arrow-z")
    s += '</g>\n'
    return s


def main_labels_and_angles(stage: int) -> str:
    if stage < 2:
        return ""
    g = main_geometry()
    OA, CL, CR, OB = g["OA"], g["CL"], g["CR"], g["OB"]
    xA = unit2(g["xA2"])
    yA = unit2(g["yA2"])
    zA = unit2(g["zA2"])
    leg = unit2(OB - OA)
    leg_normal = unit2(np.array([-leg[1], leg[0]]))

    s = '<g id="main_angles_and_labels">\n'
    # psi around O_A.  Local world reference axes are translated to O_A:
    # dashed x_W -> solid x_A, with a dashed z_W reference sharing O_A.
    start_psi = angle_of(np.array([1, 0]))
    end_psi = angle_of(xA)
    s += line(OA, OA + np.array([190, 0]), C.aux, 2.0, 'stroke-dasharray="8 7" marker-end="url(#arrow-ref)"')
    s += line(OA, OA + np.array([0, -186]), C.aux, 2.0, 'stroke-dasharray="8 7" marker-end="url(#arrow-ref)"')
    s += arc_path(OA, 122, math.degrees(start_psi), math.degrees(end_psi), C.angle, 3.0)
    s += text(OA[0] + 130, OA[1] + 28, "ψ", 32, C.angle, anchor="start", italic=True, klass="math")

    # theta from +z_A to rod.
    start_th = angle_of(zA)
    end_th = angle_of(leg)
    s += line(OA, OA + 170 * zA, C.aux, 2.0, 'stroke-dasharray="7 6"')
    s += arc_path(OA, 148, math.degrees(start_th), math.degrees(end_th), C.angle, 3.0)
    mid = OA + 169 * unit2(zA + leg)
    s += text(mid[0] + 8, mid[1], "θ", 32, C.angle, anchor="start", italic=True, klass="math")

    # leg length on actual rod, no parallel dimension line.
    lm = OA + 0.52 * (OB - OA) - 34 * leg_normal
    s += text(lm[0], lm[1], "l", 32, "#333", anchor="middle", italic=True, klass="math")

    if stage >= 4:
        s += main_actuator_annotations(g, OA, CL, CR, OB, leg, leg_normal)

    if stage >= 4:
        # coordinate labels
        OW = np.array([88.0, 748.0])
        s += sub_label(OW[0] + 102, OW[1] + 21, "x", "W", 28, C.x)
        s += sub_label(OW[0] - 62, OW[1] + 60, "y", "W", 28, C.y)
        s += sub_label(OW[0] + 4, OW[1] - 126, "z", "W", 28, C.z)
        s += frame_name(OW[0] + 30, OW[1] + 76, "W", 32)
        s += sub_label(OA[0] + 172 * xA[0] + 24, OA[1] + 172 * xA[1] + 6, "x", "A", 28, C.x)
        s += sub_label(OA[0] + 165 * yA[0] - 24, OA[1] + 165 * yA[1] + 8, "y", "A", 28, C.y)
        s += sub_label(OA[0] + 150 * zA[0] + 8, OA[1] + 150 * zA[1] - 14, "z", "A", 28, C.z)
        s += sub_label(OA[0] + 198, OA[1] - 4, "x", "W", 24, "#888")
        s += sub_label(OA[0] - 16, OA[1] - 194, "z", "W", 24, "#888")
        s += frame_name(OA[0] - 42, OA[1] + 62, "A", 32)
        s += sub_label(OA[0] + 18, OA[1] - 14, "O", "A", 30, "#111", anchor="start")
        s += sub_label(OB[0] + 18, OB[1] - 10, "O", "B", 30, "#111", anchor="start")
        s += sub_label(CL[0] - 20, CL[1] + 82, "C", "L", 28, "#111")
        s += sub_label(CR[0] + 34, CR[1] - 58, "C", "R", 28, "#111")
        s += sub_label(OB[0] + 116 * unit2(g["xB2"])[0] + 20, OB[1] + 116 * unit2(g["xB2"])[1], "x", "B", 28, C.x)
        s += sub_label(OB[0] + 112 * unit2(g["yB2"])[0] - 26, OB[1] + 112 * unit2(g["yB2"])[1] + 2, "y", "B", 28, C.y)
        s += sub_label(OB[0] + 118 * unit2(g["zB2"])[0] + 10, OB[1] + 118 * unit2(g["zB2"])[1] - 12, "z", "B", 28, C.z)
        s += frame_name(OB[0] + 126, OB[1] - 80, "B", 32)
    s += '</g>\n'
    return s


def main_actuator_annotations(g: dict, OA: np.ndarray, CL: np.ndarray, CR: np.ndarray, OB: np.ndarray,
                              leg: np.ndarray, leg_normal: np.ndarray) -> str:
    s = '<g id="main_actuator_forces_and_torques">\n'
    wheel_rot = math.degrees(angle_of(g["zA2"])) + 90

    # Wheel motor torques.  Both wheel spin coordinates are defined about the wheel +y axis;
    # the visual arcs are drawn outside the rim with the same projected positive sense.
    s += ellipse_arc_path(CL, 58, 102, wheel_rot, 228, 322, C.actuator, 4.2, "arrow-actuator")
    s += sub_label(CL[0] - 88, CL[1] - 74, "τ", "L", 28, C.actuator)
    s += ellipse_arc_path(CR, 58, 102, wheel_rot, 48, 142, C.actuator, 4.2, "arrow-actuator")
    s += sub_label(CR[0] + 92, CR[1] + 70, "τ", "R", 28, C.actuator)

    # Telescopic leg force: positive direction increases l, so it points along O_A O_B.
    f0 = OA + 0.40 * (OB - OA) + 34 * leg_normal
    f1 = OA + 0.66 * (OB - OA) + 34 * leg_normal
    s += line(f0, f1, C.actuator, 4.0, 'marker-end="url(#arrow-force)"')
    fm = 0.5 * (f0 + f1) + 22 * leg_normal
    s += sub_label(fm[0], fm[1], "F", "l", 28, C.actuator)

    # Hip pitch actuator torque: one continuous orange arc with the arrowhead
    # attached to the path end.  It is intentionally a single curved arrow,
    # not a pair of action/reaction arcs.
    s += arc_path(OB, 54, 176, 306, C.actuator, 4.2, True, "arrow-actuator-arc")
    s += sub_label(OB[0] + 84, OB[1] + 2, "τ", "h", 28, C.actuator)
    s += '</g>\n'
    return s


def inset_rod(stage: int) -> str:
    if stage < 3:
        return ""
    s = '<g id="inset_rod_geometry">\n'
    # panel background
    s += '<rect x="1120" y="58" width="420" height="300" rx="18" fill="#fbfbfb" stroke="#dddddd" stroke-width="2"/>\n'
    OA = np.array([1235.0, 282.0])
    theta = math.radians(18)
    OB = OA + np.array([150 * math.sin(theta), -150 * math.cos(theta)])
    s += line(OA, OA + np.array([0, -188]), C.aux, 2.0, 'stroke-dasharray="8 7"')
    s += line(OA, OB, C.robot2, 5.0)
    s += circle(OA, 7, C.robot, C.robot, 1)
    s += circle(OB, 7, C.robot, C.robot, 1)
    s += axis_arrow(OA, np.array([1, 0]), 115, C.x, "arrow-x")
    s += axis_arrow(OA, np.array([-0.78, 0.35]), 72, C.y, "arrow-y")
    s += axis_arrow(OA, np.array([0, -1]), 105, C.z, "arrow-z")
    s += arc_path(OA, 112, -90, -90 + 18, C.angle, 3.0)
    if stage >= 4:
        s += text(1138, 90, "Rod position", 26, "#111", anchor="start")
        s += sub_label(OA[0] - 14, OA[1] + 28, "O", "A", 26, "#111")
        s += sub_label(OB[0] + 22, OB[1] - 8, "O", "B", 26, "#111")
        s += sub_label(OA[0] + 130, OA[1] + 7, "x", "A", 26, C.x)
        s += sub_label(OA[0] - 72, OA[1] + 40, "y", "A", 26, C.y)
        s += sub_label(OA[0] + 12, OA[1] - 118, "z", "A", 26, C.z)
        s += text(OA[0] + 86, OA[1] - 102, "l", 30, "#333", anchor="middle", italic=True, klass="math")
        s += text(OA[0] + 45, OA[1] - 104, "θ", 30, C.angle, anchor="start", italic=True, klass="math")
    s += '</g>\n'
    return s


def inset_attitude(stage: int) -> str:
    if stage < 3:
        return ""
    s = '<g id="inset_body_attitude">\n'
    s += '<rect x="1120" y="450" width="420" height="360" rx="18" fill="#fbfbfb" stroke="#dddddd" stroke-width="2"/>\n'

    if stage >= 4:
        s += text(1138, 484, "Body attitude", 26, "#111", anchor="start")
        s += text(1215, 526, "Pitch β", 24, "#111", anchor="middle")
        s += text(1434, 526, "Roll ρ", 24, "#111", anchor="middle")

    # Left mini-diagram: pitch beta, R_y(beta).  It is intentionally only a
    # coordinate-frame sketch: no body polygon and no fake 3D solid.
    P = np.array([1216.0, 662.0])
    xA = np.array([1.0, 0.0])
    yA = unit2(np.array([-0.72, 0.34]))
    zA = np.array([0.0, -1.0])
    beta = math.radians(28.0)
    # Pitch sign follows the right-hand-rule sketch requested here:
    # x_B1 rotates counter-clockwise from x_A and z_B1 shifts to the other side.
    xB1 = unit2(np.array([math.cos(beta), -math.sin(beta)]))
    zB1 = unit2(np.array([-math.sin(beta), -math.cos(beta)]))

    s += line(P - 44 * yA, P + 76 * yA, C.y, 3.0, 'marker-end="url(#arrow-y)"')
    s += line(P, P + 92 * xA, C.aux, 2.0, 'stroke-dasharray="7 6" marker-end="url(#arrow-ref)"')
    s += line(P, P + 98 * zA, C.aux, 2.0, 'stroke-dasharray="7 6" marker-end="url(#arrow-ref)"')
    s += axis_arrow(P, xB1, 96, C.x, "arrow-x")
    s += axis_arrow(P, zB1, 96, C.z, "arrow-z")
    s += arc_path(P, 58, -90, math.degrees(angle_of(zB1)), C.angle, 3.0)
    s += circle(P, 5.0, C.robot, C.robot, 1)

    # Right mini-diagram: roll rho.  Draw it like the pitch sketch:
    # dashed local A reference frame first, then the rolled B frame.
    R = np.array([1426.0, 662.0])
    x_ref = np.array([1.0, 0.0])
    y_ref = unit2(np.array([-0.72, 0.34]))
    z_ref = np.array([0.0, -1.0])
    xB_roll = x_ref
    yB = unit2(np.array([-0.50, 0.72]))
    zB = unit2(np.array([-0.42, -0.91]))

    s += line(R, R + 82 * x_ref, C.aux, 2.0, 'stroke-dasharray="7 6" marker-end="url(#arrow-ref)"')
    s += line(R, R + 70 * y_ref, C.aux, 2.0, 'stroke-dasharray="7 6" marker-end="url(#arrow-ref)"')
    s += line(R, R + 82 * z_ref, C.aux, 2.0, 'stroke-dasharray="7 6" marker-end="url(#arrow-ref)"')
    s += axis_arrow(R, xB_roll, 94, C.x, "arrow-x")
    s += axis_arrow(R, yB, 82, C.y, "arrow-y")
    s += axis_arrow(R, zB, 86, C.z, "arrow-z")
    # rho is shown like beta: from the reference y_A direction to the rolled y_B
    # direction.  The rotation axis is the common red x axis.
    s += arc_path(R, 54, math.degrees(angle_of(y_ref)), math.degrees(angle_of(yB)), C.angle, 3.0)
    s += circle(R, 5.0, C.robot, C.robot, 1)

    if stage >= 4:
        s += sub_label(P[0] - 60 * yA[0] - 18, P[1] - 60 * yA[1] + 6, "y", "A", 24, C.y)
        s += sub_label(P[0] + 78 * yA[0] - 8, P[1] + 78 * yA[1] + 12, "y", "B1", 22, C.y)
        s += sub_label(P[0] + 104 * xA[0] + 12, P[1] + 104 * xA[1] + 6, "x", "A", 23, "#999")
        s += sub_label(P[0] + 108 * zA[0] - 12, P[1] + 108 * zA[1] - 6, "z", "A", 23, "#999")
        s += sub_label(P[0] + 104 * xB1[0] + 10, P[1] + 104 * xB1[1] - 2, "x", "B1", 23, C.x)
        s += sub_label(P[0] + 108 * zB1[0] + 10, P[1] + 108 * zB1[1] - 8, "z", "B1", 23, C.z)
        beta_mid = P + 78 * unit2(zA + zB1)
        s += text(beta_mid[0] + 10, beta_mid[1] - 2, "β", 30, C.angle, anchor="start", italic=True, klass="math")

        s += sub_label(R[0] + 72, R[1] + 26, "x", "A", 22, "#999")
        s += sub_label(R[0] + 105, R[1] - 18, "x", "B", 24, C.x)
        s += sub_label(R[0] + 78 * y_ref[0] - 12, R[1] + 78 * y_ref[1] + 6, "y", "A", 22, "#999")
        s += sub_label(R[0] + 94 * z_ref[0] + 8, R[1] + 94 * z_ref[1] - 8, "z", "A", 22, "#999")
        s += sub_label(R[0] + 94 * yB[0] - 14, R[1] + 94 * yB[1] + 10, "y", "B", 24, C.y)
        s += sub_label(R[0] + 98 * zB[0] + 12, R[1] + 98 * zB[1] - 8, "z", "B", 24, C.z)
        rho_label = R + 78 * unit2(y_ref + yB) + np.array([-4.0, 10.0])
        s += text(rho_label[0], rho_label[1], "ρ", 30, C.angle, anchor="middle", italic=True, klass="math")
    s += '</g>\n'
    return s


def small_legend(stage: int) -> str:
    if stage < 4:
        return ""
    s = '<g id="legend_angle_force_torque">\n'
    x, y = 1138.0, 828.0
    s += arc_path(np.array([x + 18, y + 8]), 13, 200, 330, C.angle, 3.0)
    s += text(x + 52, y + 14, "angle", 21, "#333", anchor="start")
    s += arc_path(np.array([x + 165, y + 8]), 14, 200, 330, C.actuator, 4.2, True, "arrow-actuator")
    s += text(x + 202, y + 14, "torque", 21, "#333", anchor="start")
    s += line(np.array([x + 302, y + 8]), np.array([x + 344, y + 8]), C.actuator, 4.0, 'marker-end="url(#arrow-force)"')
    s += text(x + 354, y + 14, "force", 21, "#333", anchor="start")
    s += '</g>\n'
    return s


def build_svg(stage: int) -> str:
    svg = SVG()
    svg.group("layout_background", '<rect x="24" y="28" width="1060" height="824" rx="28" fill="#ffffff" stroke="#eeeeee" stroke-width="0"/>\n')
    svg.group("main_robot", main_structure(stage))
    svg.group("main_annotations", main_labels_and_angles(stage))
    svg.add(inset_rod(stage))
    svg.add(inset_attitude(stage))
    svg.add(small_legend(stage))
    return svg.render()


def write_outputs(stage: int) -> None:
    svg_text = build_svg(stage)
    svg_path = OUT / "wheel_leg_frames.svg"
    pdf_path = OUT / "wheel_leg_frames.pdf"
    png_path = OUT / "wheel_leg_frames_preview.png"
    svg_path.write_text(svg_text, encoding="utf-8")
    cairosvg.svg2pdf(bytestring=svg_text.encode("utf-8"), write_to=str(pdf_path), output_width=CANVAS_W, output_height=CANVAS_H)
    cairosvg.svg2png(bytestring=svg_text.encode("utf-8"), write_to=str(png_path), output_width=CANVAS_W, output_height=CANVAS_H)
    print(f"stage={stage}")
    print(svg_path)
    print(pdf_path)
    print(png_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", type=int, default=4, choices=[1, 2, 3, 4])
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    write_outputs(args.stage)
