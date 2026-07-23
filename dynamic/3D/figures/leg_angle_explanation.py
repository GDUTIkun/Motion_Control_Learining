from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle, FancyArrowPatch


@dataclass(frozen=True)
class LegParams:
    l1: float = 3.0
    l2: float = 2.55
    c1: float = 1.35
    c2: float = 1.25
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


def main() -> None:
    params = LegParams()

    thigh_u = ray_from_vertical_down(params.qh_deg)
    shank_angle = params.qh_deg + params.qk_deg
    shank_u = ray_from_vertical_down(shank_angle)

    hip = (0.0, 0.0)
    knee = (params.l1 * thigh_u[0], params.l1 * thigh_u[1])
    foot = (knee[0] + params.l2 * shank_u[0], knee[1] + params.l2 * shank_u[1])
    thigh_com = (params.c1 * thigh_u[0], params.c1 * thigh_u[1])
    shank_com = (knee[0] + params.c2 * shank_u[0], knee[1] + params.c2 * shank_u[1])

    assert abs(math.dist(hip, knee) - params.l1) < 1e-9
    assert abs(math.dist(knee, foot) - params.l2) < 1e-9
    assert shank_angle == params.qh_deg + params.qk_deg

    out_dir = Path(__file__).resolve().parent
    svg_path = out_dir / "leg_angle_explanation.svg"
    png_path = out_dir / "leg_angle_explanation.png"

    fig, ax = plt.subplots(figsize=(7.2, 5.8), dpi=180)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")

    axis_color = "#1f6f8b"
    link_color = "#222222"
    guide_color = "#9aa4ad"
    qh_color = "#d1495b"
    qk_color = "#2a9d8f"
    sum_color = "#6d5bd0"
    com_color = "#f4a261"

    # Global coordinate frame.
    add_arrow(ax, hip, (1.45, 0.0), color=axis_color, lw=1.7)
    add_arrow(ax, hip, (0.0, 1.25), color=axis_color, lw=1.7)
    ax.text(1.57, 0.02, r"$+x$", color=axis_color, fontsize=12, ha="left", va="center")
    ax.text(0.05, 1.33, r"$+z$", color=axis_color, fontsize=12, ha="left", va="bottom")

    # Vertical-down zero references.
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

    # Link centerlines.
    ax.plot([hip[0], knee[0]], [hip[1], knee[1]], color=link_color, lw=5.0, solid_capstyle="round")
    ax.plot([knee[0], foot[0]], [knee[1], foot[1]], color=link_color, lw=5.0, solid_capstyle="round")

    # Joint markers and COM markers.
    for point, name, offset in [
        (hip, "hip", (-0.12, 0.22)),
        (knee, "knee", (0.15, 0.16)),
        (foot, "distal end", (0.18, -0.05)),
    ]:
        ax.add_patch(Circle(point, 0.095, facecolor="white", edgecolor=link_color, lw=1.6, zorder=5))
        ax.text(point[0] + offset[0], point[1] + offset[1], name, fontsize=10, color=link_color)

    ax.add_patch(Circle(thigh_com, 0.075, facecolor=com_color, edgecolor="#7a4b15", lw=1.0, zorder=6))
    ax.add_patch(Circle(shank_com, 0.075, facecolor=com_color, edgecolor="#7a4b15", lw=1.0, zorder=6))
    ax.text(thigh_com[0] + 0.16, thigh_com[1] + 0.06, r"$p_{c1}$", fontsize=11, color="#7a4b15")
    ax.text(shank_com[0] + 0.12, shank_com[1] + 0.05, r"$p_{c2}$", fontsize=11, color="#7a4b15")

    # Angle arcs. Matplotlib angles use +x as 0 deg, CCW positive.
    down_std = -90.0
    thigh_std = down_std + params.qh_deg
    shank_std = down_std + shank_angle
    add_arc(ax, hip, 0.72, down_std, thigh_std, color=qh_color, label=r"$q_h$", label_radius=0.95)

    # The knee angle is measured from the thigh continuation to the shank.
    add_arc(
        ax,
        knee,
        0.54,
        thigh_std,
        shank_std,
        color=qk_color,
        label=r"$q_k$",
        label_radius=0.76,
        label_shift=(0.05, 0.0),
    )

    # The shank absolute angle is measured from vertical down, so it is q_h + q_k.
    add_arc(
        ax,
        knee,
        0.95,
        down_std,
        shank_std,
        color=sum_color,
        label=r"$q_h+q_k$",
        label_radius=1.28,
        label_shift=(0.25, -0.05),
    )

    all_x = [hip[0], knee[0], foot[0], thigh_com[0], shank_com[0], thigh_extension_end[0], 1.7]
    all_y = [hip[1], knee[1], foot[1], thigh_com[1], shank_com[1], thigh_extension_end[1], 1.4]
    ax.set_xlim(min(all_x) - 0.75, max(all_x) + 0.55)
    ax.set_ylim(min(all_y) - 0.55, max(all_y) + 0.35)

    fig.savefig(svg_path, bbox_inches="tight", pad_inches=0.08)
    fig.savefig(png_path, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


if __name__ == "__main__":
    main()
