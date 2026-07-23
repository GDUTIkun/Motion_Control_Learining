"""Draw the minimal 3D wheel-leg model schematic.

Outputs:
  out/wheel_leg_3d_model_schematic.svg
  out/wheel_leg_3d_model_schematic.pdf
  out/wheel_leg_3d_model_schematic.png

The figure is a 2D engineering projection of the model used in
derive_wheel_leg_body_3d_minimal_variable_l.m.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Polygon
import numpy as np


@dataclass(frozen=True)
class PhysicalParams:
    wheel_radius: float = 0.32
    wheel_track: float = 2.6
    leg_length: float = 2.25
    theta_deg: float = 12.0
    psi_deg: float = 28.0
    beta_deg: float = 18.0
    rho_deg: float = 15.0
    body_x: float = 0.70
    body_y: float = 0.88
    body_z: float = 0.42


@dataclass(frozen=True)
class StyleParams:
    axis_len_w: float = 0.90
    axis_len_a: float = 1.05
    axis_len_b: float = 0.62
    angle_radius_small: float = 0.45
    angle_radius_large: float = 0.72
    font_size: int = 11


P = PhysicalParams()
S = StyleParams()

OUT_DIR = Path(r"D:\Workspace\CodeWorkspace\dynamic\3D\out")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n == 0:
        raise ValueError("zero vector")
    return v / n


def Rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def project(p: np.ndarray) -> np.ndarray:
    """Oblique engineering projection from 3D model coordinates to 2D."""
    x, y, z = p
    return np.array([0.92 * x - 0.62 * y, 0.42 * x + 0.30 * y + 1.05 * z])


def draw_line3(ax, p0, p1, **kwargs):
    p0p, p1p = project(p0), project(p1)
    ax.plot([p0p[0], p1p[0]], [p0p[1], p1p[1]], **kwargs)


def draw_arrow3(ax, p0, p1, color, label=None, label_offset=(0.0, 0.0), lw=1.9, ms=11):
    p0p, p1p = project(p0), project(p1)
    arrow = FancyArrowPatch(
        p0p,
        p1p,
        arrowstyle="-|>",
        mutation_scale=ms,
        linewidth=lw,
        color=color,
        shrinkA=0,
        shrinkB=0,
        zorder=8,
    )
    ax.add_patch(arrow)
    if label:
        ax.text(
            p1p[0] + label_offset[0],
            p1p[1] + label_offset[1],
            label,
            color=color,
            fontsize=S.font_size,
            ha="center",
            va="center",
            zorder=20,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.72, pad=0.6),
        )


def draw_poly3(ax, points, **kwargs):
    poly = Polygon([project(p) for p in points], closed=True, **kwargs)
    ax.add_patch(poly)


def draw_axis_frame(ax, origin, axes, labels, colors, length, dashed=False, offsets=None):
    if offsets is None:
        offsets = [(0.0, 0.0)] * 3
    linestyle = "--" if dashed else "-"
    for direction, label, color, off in zip(axes, labels, colors, offsets):
        end = origin + length * unit(direction)
        p0p, p1p = project(origin), project(end)
        arr = FancyArrowPatch(
            p0p,
            p1p,
            arrowstyle="-|>",
            mutation_scale=12,
            linewidth=1.8,
            color=color,
            linestyle=linestyle,
            shrinkA=0,
            shrinkB=0,
            zorder=9,
        )
        ax.add_patch(arr)
        ax.text(
            p1p[0] + off[0],
            p1p[1] + off[1],
            label,
            color=color,
            fontsize=S.font_size,
            ha="center",
            va="center",
            zorder=20,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.72, pad=0.5),
        )


def draw_wheel(ax, center, x_axis, z_axis, radius, edgecolor="#111111"):
    t = np.linspace(0, 2 * math.pi, 160)
    circle = [center + radius * (math.cos(a) * x_axis + math.sin(a) * z_axis) for a in t]
    pts = np.array([project(p) for p in circle])
    ax.plot(pts[:, 0], pts[:, 1], color=edgecolor, linewidth=2.2, zorder=5)

    hub = [center + 0.16 * radius * (math.cos(a) * x_axis + math.sin(a) * z_axis) for a in t]
    hub_pts = np.array([project(p) for p in hub])
    ax.plot(hub_pts[:, 0], hub_pts[:, 1], color=edgecolor, linewidth=1.4, zorder=7)

    draw_line3(ax, center - 0.72 * radius * z_axis, center + 0.72 * radius * z_axis, color=edgecolor, lw=1.0, zorder=6)
    draw_line3(ax, center - 0.72 * radius * x_axis, center + 0.72 * radius * x_axis, color=edgecolor, lw=1.0, zorder=6)


def draw_arc_path(ax, pts3, color, label=None, label_index=None, label_offset=(0.0, 0.0), lw=1.6):
    pts2 = np.array([project(p) for p in pts3])
    ax.plot(pts2[:, 0], pts2[:, 1], color=color, lw=lw, zorder=12)
    p_prev, p_end = pts2[-4], pts2[-1]
    arrow = FancyArrowPatch(
        p_prev,
        p_end,
        arrowstyle="-|>",
        mutation_scale=10,
        linewidth=lw,
        color=color,
        shrinkA=0,
        shrinkB=0,
        zorder=13,
    )
    ax.add_patch(arrow)
    if label:
        idx = label_index if label_index is not None else len(pts2) // 2
        lp = pts2[idx]
        ax.text(
            lp[0] + label_offset[0],
            lp[1] + label_offset[1],
            label,
            color=color,
            fontsize=S.font_size,
            ha="center",
            va="center",
            zorder=20,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.78, pad=0.5),
        )


def body_cuboid_vertices(center, x_axis, y_axis, z_axis):
    hx, hy, hz = P.body_x / 2, P.body_y / 2, P.body_z / 2
    corners = {}
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                corners[(sx, sy, sz)] = center + sx * hx * x_axis + sy * hy * y_axis + sz * hz * z_axis
    return corners


def draw_body(ax, center, x_axis, y_axis, z_axis):
    c = body_cuboid_vertices(center, x_axis, y_axis, z_axis)
    faces = [
        [c[(-1, -1, -1)], c[(1, -1, -1)], c[(1, 1, -1)], c[(-1, 1, -1)]],
        [c[(-1, -1, 1)], c[(1, -1, 1)], c[(1, 1, 1)], c[(-1, 1, 1)]],
        [c[(1, -1, -1)], c[(1, 1, -1)], c[(1, 1, 1)], c[(1, -1, 1)]],
    ]
    for face in faces:
        draw_poly3(ax, face, facecolor="#d8ecff", edgecolor="#1c4e80", linewidth=1.2, alpha=0.72, zorder=4)

    edges = [
        ((-1, -1, -1), (1, -1, -1)), ((1, -1, -1), (1, 1, -1)),
        ((1, 1, -1), (-1, 1, -1)), ((-1, 1, -1), (-1, -1, -1)),
        ((-1, -1, 1), (1, -1, 1)), ((1, -1, 1), (1, 1, 1)),
        ((1, 1, 1), (-1, 1, 1)), ((-1, 1, 1), (-1, -1, 1)),
        ((-1, -1, -1), (-1, -1, 1)), ((1, -1, -1), (1, -1, 1)),
        ((1, 1, -1), (1, 1, 1)), ((-1, 1, -1), (-1, 1, 1)),
    ]
    for a, b in edges:
        draw_line3(ax, c[a], c[b], color="#1c4e80", lw=1.0, zorder=6)


def main():
    psi = math.radians(P.psi_deg)
    theta = math.radians(P.theta_deg)
    beta = math.radians(P.beta_deg)
    rho = math.radians(P.rho_deg)

    xW = np.array([1.0, 0.0, 0.0])
    yW = np.array([0.0, 1.0, 0.0])
    zW = np.array([0.0, 0.0, 1.0])

    R_WA = Rz(psi)
    R_AB = Ry(beta) @ Rx(rho)
    R_WB = R_WA @ R_AB

    xA, yA, zA = R_WA @ xW, R_WA @ yW, zW
    xB, yB, zB = R_WB @ xW, R_WB @ yW, R_WB @ zW

    O_A = np.array([0.0, 0.0, 0.0])
    O_W = O_A - 1.28 * xW - 0.74 * yW
    C_L = O_A + 0.5 * P.wheel_track * yA
    C_R = O_A - 0.5 * P.wheel_track * yA
    leg_dir = R_WA @ np.array([math.sin(theta), 0.0, math.cos(theta)])
    O_B = O_A + P.leg_length * leg_dir

    # Geometry checks.
    assert abs(np.linalg.norm(C_L - C_R) - P.wheel_track) < 1e-10
    assert abs(np.linalg.norm(O_B - O_A) - P.leg_length) < 1e-10
    assert abs(np.linalg.norm(xA) - 1.0) < 1e-10
    assert abs(np.dot(xA, yA)) < 1e-10
    assert abs(np.linalg.det(R_WB) - 1.0) < 1e-10

    fig, ax = plt.subplots(figsize=(9.4, 7.2), dpi=180)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_facecolor("white")

    # Ground plane guides.
    ground_corners = [
        O_A - 1.25 * xA - 1.65 * yA,
        O_A + 1.70 * xA - 1.65 * yA,
        O_A + 1.70 * xA + 1.65 * yA,
        O_A - 1.25 * xA + 1.65 * yA,
    ]
    draw_poly3(ax, ground_corners, facecolor="#f7f7f7", edgecolor="#d6d6d6", linewidth=0.8, alpha=0.60, zorder=0)

    # Reference world frame and current axle frame.
    draw_axis_frame(
        ax,
        O_W,
        [xW, yW, zW],
        [r"$x_W$", r"$y_W$", r"$z_W$"],
        ["#666666", "#666666", "#666666"],
        S.axis_len_w,
        dashed=True,
        offsets=[(-0.06, -0.06), (0.03, -0.05), (0.03, 0.04)],
    )
    ax.text(*project(O_W + np.array([-0.10, -0.10, -0.10])), r"$\{W\}$", fontsize=12, color="#444444")

    draw_axis_frame(
        ax,
        O_A,
        [xA, yA, zA],
        [r"$x_A$", r"$y_A$", r"$z_A$"],
        ["#d62728", "#2ca02c", "#1f77b4"],
        S.axis_len_a,
        offsets=[(-0.06, -0.07), (0.10, -0.02), (0.06, 0.02)],
    )
    ax.text(*project(O_A + np.array([0.05, 0.03, -0.12])), r"$O_A$", fontsize=12, color="#111111")

    # Wheels and axle.
    draw_line3(ax, C_L, C_R, color="#111111", lw=3.0, zorder=5)
    draw_wheel(ax, C_L, xA, zA, P.wheel_radius)
    draw_wheel(ax, C_R, xA, zA, P.wheel_radius)
    ax.text(*project(C_L + 0.10 * yA - 0.28 * zA), r"$C_L$", fontsize=10, ha="center")
    ax.text(*project(C_R - 0.10 * yA - 0.28 * zA), r"$C_R$", fontsize=10, ha="center")

    # Leg and hip/body.
    draw_line3(ax, O_A, O_B, color="#0b3d5c", lw=4.0, zorder=7)
    ax.scatter(*project(O_A), s=42, color="#111111", zorder=12)
    ax.scatter(*project(O_B), s=48, color="#111111", zorder=12)
    ax.text(
        *project(O_B + 0.42 * xB + 0.32 * yB - 0.22 * zB),
        r"$O_B$",
        fontsize=11,
        ha="left",
        zorder=20,
        bbox=dict(facecolor="white", edgecolor="none", alpha=0.72, pad=0.4),
    )

    draw_body(ax, O_B, xB, yB, zB)
    draw_axis_frame(
        ax,
        O_B,
        [xB, yB, zB],
        [r"$x_B$", r"$y_B$", r"$z_B$"],
        ["#d62728", "#2ca02c", "#1f77b4"],
        S.axis_len_b,
        offsets=[(0.04, -0.08), (0.08, -0.02), (0.06, 0.03)],
    )
    ax.text(*project(O_B - 0.58 * yB + 0.50 * zB), r"$\{B\}$", fontsize=13, color="#1c4e80")

    # Leg length dimension.
    side = unit(0.75 * yA - 0.12 * xA)
    draw_line3(ax, O_A + 0.10 * side, O_B + 0.10 * side, color="#555555", lw=1.0, ls="--", zorder=11)
    mid_l = 0.5 * (O_A + O_B) + 0.20 * side
    ax.text(*project(mid_l), r"$l$", fontsize=13, color="#333333", ha="center")

    # Angle arcs.
    psi_pts = [O_A + S.angle_radius_large * (math.cos(a) * xW + math.sin(a) * yW) for a in np.linspace(0, psi, 40)]
    draw_arc_path(ax, psi_pts, "#b45f06", r"$\psi$", label_offset=(0.00, -0.10), lw=1.7)

    theta_pts = [
        O_A + S.angle_radius_large * (R_WA @ np.array([math.sin(a), 0.0, math.cos(a)]))
        for a in np.linspace(0, theta, 36)
    ]
    draw_arc_path(ax, theta_pts, "#7b3294", r"$\theta$", label_offset=(0.06, 0.02), lw=1.7)

    beta_pts = [
        O_B + S.angle_radius_small * (R_WA @ Ry(a) @ zW)
        for a in np.linspace(0, beta, 36)
    ]
    draw_arc_path(ax, beta_pts, "#1f77b4", r"$\beta$", label_offset=(0.05, 0.03), lw=1.6)

    rho_pts = [
        O_B + 0.50 * xB + 0.27 * (math.cos(a) * yB + math.sin(a) * zB)
        for a in np.linspace(0, rho, 34)
    ]
    draw_arc_path(ax, rho_pts, "#2ca02c", r"$\rho$", label_offset=(0.07, 0.02), lw=1.6)

    # Rolling direction arrows.
    draw_arrow3(
        ax,
        C_R + 0.55 * P.wheel_radius * zA,
        C_R + 0.55 * P.wheel_radius * zA + 0.45 * xA,
        "#d62728",
        r"$\dot\alpha_R$",
        label_offset=(0.10, 0.02),
        lw=1.5,
        ms=9,
    )
    draw_arrow3(
        ax,
        C_L + 0.55 * P.wheel_radius * zA,
        C_L + 0.55 * P.wheel_radius * zA + 0.45 * xA,
        "#d62728",
        r"$\dot\alpha_L$",
        label_offset=(0.07, 0.04),
        lw=1.5,
        ms=9,
    )

    # Formula panel.
    panel = (
        r"$q=[x,\psi,\theta,\beta,\rho,l]^T$" + "\n"
        r"${}^WT_B={}^WT_A{}^AT_B$" + "\n"
        r"${}^Ap_B=[l\sin\theta,\ 0,\ l\cos\theta]^T$" + "\n"
        r"${}^AR_B=R_y(\beta)R_x(\rho)$"
    )
    ax.text(
        0.02,
        0.97,
        panel,
        transform=ax.transAxes,
        fontsize=10.5,
        va="top",
        ha="left",
        bbox=dict(facecolor="white", edgecolor="#bdbdbd", boxstyle="round,pad=0.45", alpha=0.92),
        zorder=30,
    )

    ax.set_title("Minimal 3D Wheel-Leg Model: Frames and Coordinates", fontsize=11.5, pad=8)

    # Fit canvas with comfortable margins.
    all_points = [O_W, O_A, C_L, C_R, O_B]
    for p in body_cuboid_vertices(O_B, xB, yB, zB).values():
        all_points.append(p)
    proj = np.array([project(p) for p in all_points])
    x_min, y_min = proj.min(axis=0) - np.array([1.00, 0.75])
    x_max, y_max = proj.max(axis=0) + np.array([1.05, 0.95])
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    base = OUT_DIR / "wheel_leg_3d_model_schematic"
    fig.savefig(base.with_suffix(".svg"), bbox_inches="tight")
    fig.savefig(base.with_suffix(".pdf"), bbox_inches="tight")
    fig.savefig(base.with_suffix(".png"), bbox_inches="tight", dpi=240)
    plt.close(fig)

    print(f"Wrote {base.with_suffix('.svg')}")
    print(f"Wrote {base.with_suffix('.pdf')}")
    print(f"Wrote {base.with_suffix('.png')}")


if __name__ == "__main__":
    main()
