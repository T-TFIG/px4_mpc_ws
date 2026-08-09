"""Part 1 -- Visualizing the Lagrange multiplier.

The claim behind the Lagrangian of MPC_explanation_my_version.md Part V
Step 6, made visible:

    at a constrained optimum, grad f and grad g are parallel.

Example problem (equality constrained):

    min  f(x, y) = (x - 3)^2 + (y - 1.5)^2     distance-squared to t = (3, 1.5)
    s.t. g(x, y) = x^2 + y^2 - 1 = 0           the unit circle

Geometrically: find the point on the unit circle closest to t. The answer is
obviously the circle point in the direction of t, so with r = |t|

    (x*, y*) = t / r,                 lambda* = r - 1

The target is deliberately far out: it makes lambda* = 2.354, so `lambda grad g`
is visibly longer than `grad g` and you can actually see the multiplier doing
its job of rescaling one gradient onto the other. All of which the script
checks numerically rather than asserting.

Why the gradients must line up: if grad f had any component *along* the
circle, you could slide that way and lower f while staying feasible -- so you
were not at the optimum. At the optimum the only surviving component of
grad f is perpendicular to the circle, i.e. parallel to grad g. That is the
entire content of the Lagrange condition, and panel (b) shows the
not-yet-optimal case where the tangential component is still there.

Run:  python3 viz_01_lagrange.py [--outdir DIR]
"""
import numpy as np

import viz_common as vc


# --- the example problem ---------------------------------------------------
TARGET = np.array([3.0, 1.5])


def f(x, y):
    return (x - TARGET[0]) ** 2 + (y - TARGET[1]) ** 2


def grad_f(p):
    return 2.0 * (np.asarray(p, dtype=float) - TARGET)


def g(x, y):
    return x ** 2 + y ** 2 - 1.0


def grad_g(p):
    x, y = p
    return np.array([2.0 * x, 2.0 * y])


# Closed-form optimum: the unit vector towards TARGET.
R_TARGET = float(np.linalg.norm(TARGET))
Z_STAR = TARGET / R_TARGET
LAMBDA_STAR = R_TARGET - 1.0

XLIM, YLIM = (-1.9, 3.7), (-1.9, 2.5)
SCALE = 0.26


def panel_optimum(ax):
    """At the optimum: grad f and grad g are anti-parallel, ratio = lambda."""
    vc.setup_axes(ax, XLIM, YLIM,
                  r'(a) At the optimum: $\nabla f = -\lambda\,\nabla g$')
    vc.plot_contours(ax, f, XLIM, YLIM, levels=18, label=r'$f$ contours')
    vc.draw_curve(ax, g, XLIM, YLIM, label=r'$g(x,y)=0$  (unit circle)')

    gf = grad_f(Z_STAR)
    gg = grad_g(Z_STAR)
    s = SCALE  # one shared display scale, so relative lengths stay honest

    # lambda * grad g drawn on top of -grad f: they coincide exactly.
    vc.draw_arrow(ax, Z_STAR, gg, vc.COLORS['grad_g'], scale=s, lw=2.0,
                  label=r'$\nabla g$')
    vc.draw_arrow(ax, Z_STAR, LAMBDA_STAR * gg, vc.COLORS['grad_g'],
                  scale=s, lw=4.5, alpha=0.45,
                  label=r'$\lambda\,\nabla g = -\nabla f$  (same length, opposite)')
    vc.draw_arrow(ax, Z_STAR, gf, vc.COLORS['grad_f'], scale=s, lw=2.4,
                  label=r'$\nabla f$')

    vc.mark_point(ax, Z_STAR, vc.COLORS['optimum'], marker='*', size=260,
                  label=r'optimum $z^*$')
    vc.mark_point(ax, TARGET, '0.35', marker='X', size=70,
                  label='unconstrained min of $f$')

    residual = np.linalg.norm(gf + LAMBDA_STAR * gg)
    vc.annotate(ax,
                f'$\\lambda^* = |t| - 1 = {LAMBDA_STAR:.4f}$\n'
                f'so $\\nabla g$ stretched by ${LAMBDA_STAR:.3f}$\n'
                f'lands exactly on $-\\nabla f$\n'
                f'$\\|\\nabla f + \\lambda\\nabla g\\| = {residual:.1e}$',
                xy=(-1.8, 2.05))
    ax.legend(loc='lower left', framealpha=0.92)


def panel_not_optimum(ax, theta=1.15):
    """Anywhere else on the circle: grad f still has a tangential component."""
    z = np.array([np.cos(theta), np.sin(theta)])
    vc.setup_axes(ax, XLIM, YLIM,
                  '(b) Not the optimum: a tangential component survives')
    vc.plot_contours(ax, f, XLIM, YLIM, levels=18)
    vc.draw_curve(ax, g, XLIM, YLIM)

    gf = grad_f(z)
    gg = grad_g(z)
    n = gg / np.linalg.norm(gg)          # outward normal to the circle
    t = np.array([-n[1], n[0]])          # unit tangent

    gf_normal = np.dot(gf, n) * n
    gf_tangent = np.dot(gf, t) * t
    s = SCALE

    vc.draw_arrow(ax, z, gf, vc.COLORS['grad_f'], scale=s, lw=2.4,
                  label=r'$\nabla f$')
    vc.draw_arrow(ax, z, gg, vc.COLORS['grad_g'], scale=s, lw=2.0,
                  label=r'$\nabla g$  (normal to the circle)')
    vc.draw_arrow(ax, z, gf_normal, '0.45', scale=s, lw=1.6, linestyle='--',
                  label='normal part of $\\nabla f$')
    vc.draw_arrow(ax, z, gf_tangent, vc.COLORS['bad'], scale=s, lw=2.6,
                  label='tangential part -- the reason this is not optimal')

    # The improving move: slide along the circle against the tangential part.
    slide = -np.sign(np.dot(gf, t)) * t
    arc = np.linspace(theta, theta + 0.55 * np.sign(slide @ t), 40)
    ax.plot(np.cos(arc), np.sin(arc), color=vc.COLORS['bad'], lw=5.0,
            alpha=0.55, solid_capstyle='round', zorder=4,
            label='improving slide along $g=0$')

    vc.mark_point(ax, z, vc.COLORS['path'], size=80, label='a feasible point')
    vc.mark_point(ax, Z_STAR, vc.COLORS['optimum'], marker='*', size=200)

    vc.annotate(ax,
                'Slide along the circle in the\n'
                'direction that opposes the\n'
                'tangential part and $f$ drops.\n'
                r'So $\nabla f \nparallel \nabla g$ here.',
                xy=(-1.8, 1.95))
    ax.legend(loc='lower left', framealpha=0.92)


def main():
    outdir = vc.outdir_from_argv(__doc__.splitlines()[0])
    fig, axes = vc.plt.subplots(1, 2, figsize=(11.0, 5.2))
    panel_optimum(axes[0])
    panel_not_optimum(axes[1])
    fig.suptitle(r'Part 1 -- The Lagrange multiplier: '
                 r'$\min\ (x-3)^2+(y-1.5)^2$  s.t.  $x^2+y^2=1$',
                 fontsize=11)

    print(f'  optimum        z* = ({Z_STAR[0]:.6f}, {Z_STAR[1]:.6f})')
    print(f'  multiplier lambda* = {LAMBDA_STAR:.6f}  (= |t| - 1)')
    print(f'  grad f(z*)        = {grad_f(Z_STAR)}')
    print(f'  grad g(z*)        = {grad_g(Z_STAR)}')
    print(f'  stationarity residual = '
          f'{np.linalg.norm(grad_f(Z_STAR) + LAMBDA_STAR * grad_g(Z_STAR)):.3e}')
    vc.save(fig, '01_lagrange_multiplier.png', outdir)


if __name__ == '__main__':
    main()
