from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle, FancyArrowPatch


@dataclass(frozen=True)
class WheelLegParams:
    l1: float = 3.0
    l2: float = 2.55
    c1: float = 1.35
    c2: float = 1.25
    wheel_radius: float = 0.46
    qh_deg: float = 24.0
    qk_deg: float = 38.0


def ray_from_vertical_down(angle_deg: float) -> tuple[float, float]:
    angle = math.radians(angle_deg)
    return math.sin(angle), -math.cos(angle)


def add_arrow(
    ax,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    color: str,
    lw: float = 1.6,
    mutation_scale: float = 12,
    linestyle: str = "-",
    zorder: int = 8,
):
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=mutation_scale,
        lw=lw,
        color=color,
        linestyle=linestyle,
        shrinkA=0,
        shrinkB=0,
        zorder=zorder,
    )
    ax.add_patch(arrow)
    return arrow


def add_arc(
    ax,
    center: tuple[float, float],
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    color: str,
    label: str,
    label_radius: float,
    label_shift: tuple[float, float] = (0.0, 0.0),
):
    arc = Arc(
        center,
        2 * radius,
        2 * radius,
        angle=0,
        theta1=start_deg,
        theta2=end_deg,
        color=color,
        lw=1.6,
        zorder=9,
    )
    ax.add_patch(arc)

    end = math.radians(end_deg)
    arrow_len = 0.18
    arrow_start = (
        center[0] + radius * math.cos(end - arrow_len),
        center[1] + radius * math.sin(end - arrow_len),
    )
    arrow_end = (
        center[0] + radius * math.cos(end),
        center[1] + radius * math.sin(end),
    )
    add_arrow(ax, arrow_start, arrow_end, color=color, lw=1.4, mutation_scale=10)

    mid = math.radians((start_deg + end_deg) / 2)
    label_pos = (
        center[0] + label_radius * math.cos(mid) + label_shift[0],
        center[1] + label_radius * math.sin(mid) + label_shift[1],
    )
    ax.text(*label_pos, label, color=color, fontsize=12, ha="center", va="center")


def add_signed_arc_arrow(
    ax,
    center: tuple[float, float],
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    color: str,
    lw: float = 1.6,
    mutation_scale: float = 10,
):
    steps = 60
    angles = [
        math.radians(start_deg + (end_deg - start_deg) * i / (steps - 1))
        for i in range(steps)
    ]
    xs = [center[0] + radius * math.cos(a) for a in angles]
    ys = [center[1] + radius * math.sin(a) for a in angles]
    ax.plot(xs, ys, color=color, lw=lw, zorder=9)
    add_arrow(
        ax,
        (xs[-4], ys[-4]),
        (xs[-1], ys[-1]),
        color=color,
        lw=lw,
        mutation_scale=mutation_scale,
        zorder=10,
    )


def main() -> None:
    params = WheelLegParams()

    thigh_u = ray_from_vertical_down(params.qh_deg)
    shank_angle = params.qh_deg + params.qk_deg
    shank_u = ray_from_vertical_down(shank_angle)

    hip = (0.0, 0.0)
    knee = (params.l1 * thigh_u[0], params.l1 * thigh_u[1])
    wheel_center = (
        knee[0] + params.l2 * shank_u[0],
        knee[1] + params.l2 * shank_u[1],
    )
    ground_contact = (wheel_center[0], wheel_center[1] - params.wheel_radius)
    ground_z = ground_contact[1]
    thigh_com = (params.c1 * thigh_u[0], params.c1 * thigh_u[1])
    shank_com = (
        knee[0] + params.c2 * shank_u[0],
        knee[1] + params.c2 * shank_u[1],
    )

    assert abs(math.dist(hip, knee) - params.l1) < 1e-9
    assert abs(math.dist(knee, wheel_center) - params.l2) < 1e-9
    assert abs(math.dist(wheel_center, ground_contact) - params.wheel_radius) < 1e-9
    assert abs(ground_contact[1] - ground_z) < 1e-9
    assert shank_angle == params.qh_deg + params.qk_deg

    out_dir = Path(__file__).resolve().parent
    svg_path = out_dir / "wheel_leg_angle_explanation.svg"
    png_path = out_dir / "wheel_leg_angle_explanation.png"

    fig, ax = plt.subplots(figsize=(7.8, 6.25), dpi=180)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")

    axis_color = "#1f6f8b"
    link_color = "#222222"
    guide_color = "#9aa4ad"
    qh_color = "#d1495b"
    qk_color = "#2a9d8f"
    sum_color = "#6d5bd0"
    wheel_color = "#4b5563"
    spin_color = "#c8553d"
    contact_color = "#335c67"
    com_color = "#f4a261"

    # Global coordinate frame.
    add_arrow(ax, hip, (1.45, 0.0), color=axis_color, lw=1.7)
    add_arrow(ax, hip, (0.0, 1.25), color=axis_color, lw=1.7)
    ax.text(1.57, 0.02, r"$+x$", color=axis_color, fontsize=12, ha="left", va="center")
    ax.text(0.05, 1.33, r"$+z$", color=axis_color, fontsize=12, ha="left", va="bottom")

    # Vertical-down zero references and shank reference.
    ax.plot([hip[0], hip[0]], [hip[1], -3.35], color=guide_color, lw=1.1, ls=(0, (4, 4)))
    ax.text(-0.2, -3.35, "zero direction", color=guide_color, fontsize=9, ha="right", va="top")
    ax.plot([knee[0], knee[0]], [knee[1], knee[1] - 1.45], color=guide_color, lw=1.1, ls=(0, (4, 4)))
    thigh_extension_end = (
        knee[0] + 1.55 * thigh_u[0],
        knee[1] + 1.55 * thigh_u[1],
    )
    ax.plot(
        [knee[0], thigh_extension_end[0]],
        [knee[1], thigh_extension_end[1]],
        color=guide_color,
        lw=1.35,
        ls=(0, (4, 4)),
    )

    # Ground is tangent to the wheel by construction.
    ax.plot(
        [ground_contact[0] - 1.55, ground_contact[0] + 1.65],
        [ground_z, ground_z],
        color="#7b8794",
        lw=1.2,
        zorder=1,
    )
    ax.text(
        ground_contact[0] + 1.72,
        ground_z - 0.02,
        "ground",
        color="#7b8794",
        fontsize=9,
        ha="left",
        va="top",
    )

    # Link centerlines.
    ax.plot([hip[0], knee[0]], [hip[1], knee[1]], color=link_color, lw=5.0, solid_capstyle="round", zorder=4)
    ax.plot(
        [knee[0], wheel_center[0]],
        [knee[1], wheel_center[1]],
        color=link_color,
        lw=5.0,
        solid_capstyle="round",
        zorder=4,
    )

    # Wheel body and hub marker.
    ax.add_patch(Circle(wheel_center, params.wheel_radius, facecolor="white", edgecolor=wheel_color, lw=2.0, zorder=3))
    ax.add_patch(Circle(wheel_center, params.wheel_radius * 0.22, facecolor="#f8fafc", edgecolor=wheel_color, lw=1.4, zorder=6))
    for spoke_deg in (20, 140, 260):
        spoke = math.radians(spoke_deg)
        end = (
            wheel_center[0] + params.wheel_radius * 0.82 * math.cos(spoke),
            wheel_center[1] + params.wheel_radius * 0.82 * math.sin(spoke),
        )
        ax.plot([wheel_center[0], end[0]], [wheel_center[1], end[1]], color="#6b7280", lw=1.0, zorder=5)

    # Contact point and radius dimension.
    ax.add_patch(Circle(ground_contact, 0.065, facecolor=contact_color, edgecolor=contact_color, lw=1.0, zorder=7))
    ax.plot([wheel_center[0], ground_contact[0]], [wheel_center[1], ground_contact[1]], color=guide_color, lw=1.1, ls=(0, (3, 3)), zorder=6)
    ax.text(wheel_center[0] + 0.09, (wheel_center[1] + ground_contact[1]) / 2, r"$r$", fontsize=11, color=guide_color, ha="left", va="center")

    # Joint and mass markers.
    for point, name, offset in [
        (hip, "hip", (-0.12, 0.22)),
        (knee, "knee", (0.15, 0.16)),
        (wheel_center, r"$O$", (0.11, 0.24)),
        (ground_contact, r"$C$", (0.12, -0.27)),
    ]:
        if point != ground_contact:
            ax.add_patch(Circle(point, 0.095, facecolor="white", edgecolor=link_color, lw=1.6, zorder=7))
        ax.text(point[0] + offset[0], point[1] + offset[1], name, fontsize=10, color=link_color)

    ax.add_patch(Circle(thigh_com, 0.075, facecolor=com_color, edgecolor="#7a4b15", lw=1.0, zorder=6))
    ax.add_patch(Circle(shank_com, 0.075, facecolor=com_color, edgecolor="#7a4b15", lw=1.0, zorder=6))
    ax.text(thigh_com[0] + 0.16, thigh_com[1] + 0.06, r"$p_{c1}$", fontsize=11, color="#7a4b15")
    ax.text(shank_com[0] + 0.12, shank_com[1] + 0.05, r"$p_{c2}$", fontsize=11, color="#7a4b15")

    # Kinematic arrows for the contact-point Jacobian convention.
    add_arrow(
        ax,
        (wheel_center[0] - 0.06, wheel_center[1] + params.wheel_radius + 0.24),
        (wheel_center[0] + 0.84, wheel_center[1] + params.wheel_radius + 0.24),
        color=axis_color,
        lw=1.45,
        mutation_scale=11,
    )
    ax.text(
        wheel_center[0] + 0.9,
        wheel_center[1] + params.wheel_radius + 0.24,
        r"$\dot{x}_O$",
        fontsize=11,
        color=axis_color,
        ha="left",
        va="center",
    )
    add_arrow(
        ax,
        (ground_contact[0] - 0.05, ground_contact[1] + 0.15),
        (ground_contact[0] + 0.84, ground_contact[1] + 0.15),
        color=spin_color,
        lw=1.35,
        mutation_scale=10,
    )
    ax.text(
        ground_contact[0] + 0.91,
        ground_contact[1] + 0.15,
        r"$+r\dot{q}_w^i$",
        fontsize=10,
        color=spin_color,
        ha="left",
        va="center",
    )

    # Positive wheel spin under v_c,x = xdot_O + r qdot_w.
    add_signed_arc_arrow(
        ax,
        wheel_center,
        params.wheel_radius * 0.61,
        -58.0,
        58.0,
        color=spin_color,
        lw=1.55,
        mutation_scale=10,
    )
    ax.text(
        wheel_center[0] + params.wheel_radius * 0.75,
        wheel_center[1] + 0.02,
        r"$q_w^i$",
        fontsize=12,
        color=spin_color,
        ha="left",
        va="center",
    )

    # Angle arcs. Matplotlib angles use +x as 0 deg, CCW positive.
    down_std = -90.0
    thigh_std = down_std + params.qh_deg
    shank_std = down_std + shank_angle
    add_arc(ax, hip, 0.72, down_std, thigh_std, color=qh_color, label=r"$q_h^i$", label_radius=0.95)
    add_arc(
        ax,
        knee,
        0.54,
        thigh_std,
        shank_std,
        color=qk_color,
        label=r"$q_k^i$",
        label_radius=0.76,
        label_shift=(0.05, 0.0),
    )
    add_arc(
        ax,
        knee,
        0.95,
        down_std,
        shank_std,
        color=sum_color,
        label=r"$q_h^i+q_k^i$",
        label_radius=1.31,
        label_shift=(0.32, 0.08),
    )

    all_x = [
        hip[0],
        knee[0],
        wheel_center[0],
        ground_contact[0],
        thigh_com[0],
        shank_com[0],
        thigh_extension_end[0],
        ground_contact[0] + 1.8,
        wheel_center[0] + 1.25,
    ]
    all_y = [
        hip[1],
        knee[1],
        wheel_center[1],
        ground_contact[1],
        thigh_com[1],
        shank_com[1],
        thigh_extension_end[1],
        1.4,
        ground_contact[1] - 0.3,
    ]
    ax.set_xlim(min(all_x) - 0.75, max(all_x) + 0.55)
    ax.set_ylim(min(all_y) - 0.45, max(all_y) + 0.35)

    fig.savefig(svg_path, bbox_inches="tight", pad_inches=0.08)
    fig.savefig(png_path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


if __name__ == "__main__":
    main()
