"""Shared plotting helpers for the optimization visualization scripts.

The five scripts (viz_01 .. viz_05) all draw the same kind of picture: a
2D contour plot of an objective, some constraint geometry on top, and
arrows for gradients. This module keeps that consistent so the same colour
means the same thing in every figure.

Colour convention, used everywhere:

    grey contours   the objective f
    green fill      the feasible region
    red curve       a constraint boundary (g = 0 or h = 0)
    blue arrow      grad f          -- the pull from the cost
    orange arrow    grad g / grad h -- the push from a constraint
    purple          the path an algorithm actually walks
    gold star       the optimum

Everything renders headless (Agg backend) so it works inside the container
with no display attached.
"""
import argparse
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402  (must follow matplotlib.use)

# docs/media, relative to docs/scripts/viz_common.py
DEFAULT_OUTDIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'media')

COLORS = {
    'objective': '#8a95a5',
    'objective_fill': '#eef1f5',
    'feasible': '#a8d5a8',
    'constraint': '#c0392b',
    'grad_f': '#1f77b4',
    'grad_g': '#e07b12',
    'model': '#2e86c1',
    'path': '#7d3c98',
    'optimum': '#d4a017',
    'bad': '#b03060',
    'text': '#222222',
}

plt.rcParams.update({
    'figure.dpi': 120,
    'savefig.dpi': 120,
    'font.size': 9,
    'axes.titlesize': 10,
    'axes.labelsize': 9,
    'legend.fontsize': 8,
    'axes.grid': True,
    'grid.alpha': 0.25,
    'grid.linewidth': 0.6,
})


def outdir_from_argv(description: str) -> str:
    """Standard --outdir flag, defaulting to docs/media."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--outdir', default=DEFAULT_OUTDIR,
                        help='where to write the figures (default: docs/media)')
    args = parser.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    return args.outdir


def mesh(xlim, ylim, n=400):
    """Coordinate grid for contour plots."""
    xs = np.linspace(xlim[0], xlim[1], n)
    ys = np.linspace(ylim[0], ylim[1], n)
    return np.meshgrid(xs, ys)


def plot_contours(ax, f, xlim, ylim, levels=22, n=400, fill=False,
                  color=None, alpha=0.75, linewidths=0.8, label=None):
    """Contour the scalar field `f(x, y)` (must be numpy-vectorized)."""
    X, Y = mesh(xlim, ylim, n)
    Z = f(X, Y)
    if fill:
        ax.contourf(X, Y, Z, levels=levels, cmap='Blues_r', alpha=0.35)
    cs = ax.contour(X, Y, Z, levels=levels,
                    colors=color or COLORS['objective'],
                    linewidths=linewidths, alpha=alpha)
    if label:
        _legend_proxy(ax, color or COLORS['objective'], linewidths, label,
                      alpha=alpha)
    return cs


def _legend_proxy(ax, color, lw, label, linestyle='-', alpha=1.0):
    """Add a legend entry without relying on a contour set's collections.

    A contour level with no paths (the field never crosses it inside the
    window) produces a collection matplotlib's legend handler cannot render,
    so we register an empty Line2D with the right style instead.
    """
    ax.plot([], [], color=color, lw=lw, linestyle=linestyle, alpha=alpha,
            label=label)


def shade_feasible(ax, h, xlim, ylim, n=400, label=None):
    """Shade {h(x, y) <= 0}, the feasible set of an inequality constraint."""
    X, Y = mesh(xlim, ylim, n)
    Z = h(X, Y)
    ax.contourf(X, Y, Z, levels=[-1e9, 0.0],
                colors=[COLORS['feasible']], alpha=0.45)
    ax.contour(X, Y, Z, levels=[0.0],
               colors=[COLORS['constraint']], linewidths=2.0)
    if label:
        _legend_proxy(ax, COLORS['constraint'], 2.0, label)


def draw_curve(ax, h, xlim, ylim, n=400, color=None, lw=2.0,
               linestyle='-', label=None):
    """Draw the zero level set {h(x, y) = 0} -- an equality constraint."""
    X, Y = mesh(xlim, ylim, n)
    ax.contour(X, Y, h(X, Y), levels=[0.0],
               colors=[color or COLORS['constraint']],
               linewidths=lw, linestyles=linestyle)
    if label:
        _legend_proxy(ax, color or COLORS['constraint'], lw, label, linestyle)


def draw_arrow(ax, base, vec, color, label=None, scale=1.0, lw=2.4,
               head=11, linestyle='-', zorder=5, alpha=1.0):
    """Draw `vec` as an arrow rooted at `base`.

    Gradient magnitudes vary wildly across these examples, so arrows are
    scaled for legibility. `scale` is applied uniformly within a figure so
    that *relative* lengths -- which is what carries the multiplier
    information -- stay meaningful.
    """
    base = np.asarray(base, dtype=float)
    tip = base + scale * np.asarray(vec, dtype=float)
    ax.annotate('', xy=tip, xytext=base,
                arrowprops=dict(arrowstyle='-|>', color=color, lw=lw,
                                mutation_scale=head, linestyle=linestyle,
                                shrinkA=0, shrinkB=0, alpha=alpha),
                zorder=zorder)
    if label:
        ax.plot([], [], color=color, lw=lw, linestyle=linestyle,
                alpha=alpha, label=label)
    return tip


def mark_point(ax, p, color, label=None, marker='o', size=70, zorder=6):
    ax.scatter([p[0]], [p[1]], c=[color], s=size, marker=marker,
               edgecolors='white', linewidths=1.2, zorder=zorder, label=label)


def annotate(ax, text, xy, xytext=None, fontsize=8, color=None, arrow=False):
    kwargs = dict(fontsize=fontsize, color=color or COLORS['text'],
                  ha='left', va='center', zorder=8,
                  bbox=dict(boxstyle='round,pad=0.32', fc='white',
                            ec='0.75', alpha=0.9))
    if arrow and xytext is not None:
        kwargs['arrowprops'] = dict(arrowstyle='->', color='0.45', lw=0.9)
    ax.annotate(text, xy=xy, xytext=xytext or xy, **kwargs)


def setup_axes(ax, xlim, ylim, title=None, equal=True):
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    if equal:
        ax.set_aspect('equal', adjustable='box')
    if title:
        ax.set_title(title)
    ax.set_xlabel('$x$')
    ax.set_ylabel('$y$')


def save(fig, filename, outdir, tight=True):
    path = os.path.join(outdir, filename)
    if tight:
        fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    print(f'  wrote {path}')
    return path
