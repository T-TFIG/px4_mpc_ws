"""Part 2 -- Visualizing the KKT conditions.

Same geometry as Part 1, but the circle becomes an *inequality*:

    min  f(x, y) = (x - c1)^2 + (y - c2)^2
    s.t. h(x, y) = x^2 + y^2 - 1 <= 0          stay inside the unit disc

Now the interesting question is whether the constraint is doing anything, and
KKT (MPC_explanation_my_version.md Part V Step 7) answers it with four
conditions:

    stationarity              grad f + nu * grad h = 0
    primal feasibility        h <= 0
    dual feasibility          nu >= 0
    complementary slackness   nu * h = 0

The last one is the combinatorial heart: for each constraint, *either* it is
tight (h = 0) *or* its multiplier is zero -- never both nonzero. The three
panels are the three cases.

  (a) target outside the disc  -> constraint ACTIVE,   h = 0, nu > 0
  (b) target inside the disc   -> constraint INACTIVE, h < 0, nu = 0
  (c) why nu >= 0 is required, by showing what nu < 0 would mean

Read stationarity as a force balance: -grad f is the pull from the cost, and
nu * grad h is the push back from the wall. At an optimum they cancel exactly.
The multiplier nu is literally how hard the wall has to push, which is why it
is called its shadow price.

Run:  python3 viz_02_kkt.py [--outdir DIR]
"""
import numpy as np

import viz_common as vc


def h(x, y):
    return x ** 2 + y ** 2 - 1.0


def grad_h(p):
    return np.array([2.0 * p[0], 2.0 * p[1]])


def make_f(target):
    tx, ty = target

    def f(x, y):
        return (x - tx) ** 2 + (y - ty) ** 2

    def grad_f(p):
        return np.array([2.0 * (p[0] - tx), 2.0 * (p[1] - ty)])

    return f, grad_f


def solve(target):
    """Closed-form solution: project the target onto the unit disc.

    Returns (z*, nu*, active). If the target is already inside, the constraint
    does nothing and nu = 0 -- which is complementary slackness choosing the
    other branch.
    """
    t = np.asarray(target, dtype=float)
    r = np.linalg.norm(t)
    if r <= 1.0:
        return t.copy(), 0.0, False
    z = t / r
    # stationarity: 2(z - t) + nu * 2z = 0  ->  nu = r - 1
    return z, r - 1.0, True


XLIM, YLIM = (-1.8, 2.8), (-1.9, 2.1)
SCALE = 0.30


def panel_active(ax):
    target = np.array([2.0, 1.0])
    f, grad_f = make_f(target)
    z, nu, _ = solve(target)

    vc.setup_axes(ax, XLIM, YLIM, '(a) ACTIVE:  $h=0$,  $\\nu>0$')
    vc.plot_contours(ax, f, XLIM, YLIM, levels=18, label='$f$ contours')
    vc.shade_feasible(ax, h, XLIM, YLIM, label='feasible set $h\\leq0$')

    gf, gh = grad_f(z), grad_h(z)
    # Force balance. The cost drives you along -grad f. For the point to sit
    # still the wall must supply the opposite force, which is -nu*grad h --
    # note the sign: grad h points OUT of the feasible set, so with nu > 0 the
    # constraint force points back IN, which is the only thing a wall can do.
    # Stationarity (grad f + nu grad h = 0) says these two cancel exactly.
    vc.draw_arrow(ax, z, -gf, vc.COLORS['grad_f'], scale=SCALE, lw=2.6,
                  label=r'$-\nabla f$  (cost pulls you outward)')
    vc.draw_arrow(ax, z, -nu * gh, vc.COLORS['grad_g'], scale=SCALE, lw=4.5,
                  alpha=0.55, label=r'$-\nu\,\nabla h$  (wall pushes back in)')
    vc.draw_arrow(ax, z, gh, '0.45', scale=SCALE * 0.5, lw=1.5, linestyle='--',
                  label=r'$\nabla h$  (outward normal)')

    vc.mark_point(ax, z, vc.COLORS['optimum'], marker='*', size=260,
                  label='optimum $z^*$')
    vc.mark_point(ax, target, '0.35', marker='X', size=70,
                  label='unconstrained min')

    res = np.linalg.norm(gf + nu * gh)
    vc.annotate(ax,
                f'$h(z^*) = {h(*z):+.2e}$  (tight)\n'
                f'$\\nu = {nu:.4f} > 0$\n'
                f'$\\nu\\,h = {nu * h(*z):+.1e}$   (slackness OK)\n'
                f'$\\|\\nabla f + \\nu\\nabla h\\| = {res:.1e}$\n'
                'The two arrows are equal and\n'
                'opposite: that IS stationarity.',
                xy=(-1.7, 1.5))
    ax.legend(loc='lower left', framealpha=0.92)


def panel_inactive(ax):
    target = np.array([0.30, 0.20])
    f, grad_f = make_f(target)
    z, nu, _ = solve(target)

    vc.setup_axes(ax, XLIM, YLIM, '(b) INACTIVE:  $h<0$,  $\\nu=0$')
    vc.plot_contours(ax, f, XLIM, YLIM, levels=18)
    vc.shade_feasible(ax, h, XLIM, YLIM)

    gf = grad_f(z)
    vc.draw_arrow(ax, z, -gf, vc.COLORS['grad_f'], scale=SCALE, lw=2.6,
                  label=r'$-\nabla f = 0$ here')

    vc.mark_point(ax, z, vc.COLORS['optimum'], marker='*', size=260,
                  label='optimum $z^*$ (interior)')

    vc.annotate(ax,
                f'$h(z^*) = {h(*z):.3f} < 0$  (slack)\n'
                r'$\nu = 0$' '\n'
                f'$\\nu\\,h = 0$\n'
                'The wall is not touched, so\n'
                'it exerts no force. Stationarity\n'
                r'collapses to $\nabla f = 0$.',
                xy=(-1.7, 1.55))
    ax.legend(loc='lower left', framealpha=0.92)


def panel_why_nu_positive(ax):
    """Why dual feasibility (nu >= 0) is a real condition, not bookkeeping."""
    target = np.array([0.30, 0.20])          # min is INSIDE the disc
    f, grad_f = make_f(target)
    z_bad = np.array([1.0, 0.0])             # a boundary point we test anyway

    vc.setup_axes(ax, XLIM, YLIM, r'(c) Why $\nu \geq 0$')
    vc.plot_contours(ax, f, XLIM, YLIM, levels=18)
    vc.shade_feasible(ax, h, XLIM, YLIM)

    gf, gh = grad_f(z_bad), grad_h(z_bad)
    # Force stationarity to hold at this wrong point and see what nu must be:
    #   grad f + nu grad h = 0  ->  nu = -(grad f . grad h)/|grad h|^2
    nu_implied = -np.dot(gf, gh) / np.dot(gh, gh)

    vc.draw_arrow(ax, z_bad, -gf, vc.COLORS['grad_f'], scale=SCALE, lw=2.6,
                  label=r'$-\nabla f$  points INTO the feasible set')
    vc.draw_arrow(ax, z_bad, gh, '0.45', scale=SCALE * 0.5, lw=1.5,
                  linestyle='--', label=r'$\nabla h$  (outward normal)')
    vc.draw_arrow(ax, z_bad, -nu_implied * gh, vc.COLORS['bad'], scale=SCALE,
                  lw=4.0, alpha=0.6,
                  label=f'$-\\nu\\,\\nabla h$ with $\\nu={nu_implied:.2f}<0$:'
                        '\nforce points OUT of the feasible set')

    vc.mark_point(ax, z_bad, vc.COLORS['bad'], marker='o', size=90,
                  label='candidate on the boundary')
    vc.mark_point(ax, target, vc.COLORS['optimum'], marker='*', size=200,
                  label='the actual optimum')
    ax.plot([z_bad[0], target[0]], [z_bad[1], target[1]],
            color=vc.COLORS['path'], lw=1.8, ls='--', alpha=0.8)

    vc.annotate(ax,
                'Force stationarity to hold here\n'
                f'and it demands $\\nu={nu_implied:.2f}<0$, i.e. a\n'
                'wall that SUCKS you into itself.\n'
                'Real walls only push inward. And\n'
                r'indeed $-\nabla f$ points into the'
                '\n'
                'feasible set, so a better point is\n'
                r'available: this is not optimal.'
                '\n'
                r'$\nu\geq0$ is what rules it out.',
                xy=(-1.7, 1.28))
    ax.legend(loc='lower left', framealpha=0.92)


def main():
    outdir = vc.outdir_from_argv(__doc__.splitlines()[0])
    fig, axes = vc.plt.subplots(1, 3, figsize=(15.5, 5.4))
    panel_active(axes[0])
    panel_inactive(axes[1])
    panel_why_nu_positive(axes[2])
    fig.suptitle('Part 2 -- KKT: complementary slackness picks a branch '
                 r'($\nu\,h=0$), dual feasibility picks a sign ($\nu\geq0$)',
                 fontsize=11)

    for name, target in [('outside', (2.0, 1.0)), ('inside', (0.30, 0.20))]:
        z, nu, active = solve(target)
        print(f'  target {name:8s} -> z*=({z[0]:.4f}, {z[1]:.4f})  '
              f'h={h(*z):+.4f}  nu={nu:.4f}  '
              f'active={active}  nu*h={nu * h(*z):+.2e}')
    vc.save(fig, '02_kkt_conditions.png', outdir)


if __name__ == '__main__':
    main()
