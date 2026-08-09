"""Part 3 -- Visualizing gradient descent, and why it zigzags.

Test function, the standard ill-conditioned quadratic:

    f(x, y) = 1/2 (x^2 + kappa * y^2)

Its Hessian is diag(1, kappa), so kappa is exactly the condition number: the
contours are ellipses `sqrt(kappa)` times longer in x than in y.

Gradient descent with exact line search:

    p_k     = -grad f(z_k)
    alpha_k = argmin_alpha f(z_k + alpha p_k)  =  (p.p) / (p.H p)   for a quadratic
    z_{k+1} = z_k + alpha_k p_k

Why it zigzags -- two facts that together explain everything:

  1. grad f is perpendicular to the contour through z_k. For a circle the
     perpendicular points straight at the centre; for a stretched ellipse it
     does NOT. It points at the *nearest wall*, which is a different place.

  2. Exact line search stops where the new gradient is perpendicular to the
     direction just travelled:  grad f(z_{k+1}) . p_k = 0. So every step is at
     right angles to the previous one. That right angle is the zigzag, and it
     is forced, not accidental.

From the closed form with z_0 = (kappa, 1) the iterates are

    z_k = ((kappa-1)/(kappa+1))^k * (kappa, (-1)^k)

so the error shrinks by ((kappa-1)/(kappa+1)) per step -- for kappa = 1 that is
0 (one step, exact), for kappa = 10 it is 0.818 (slow crawl). Conditioning, not
the algorithm, is what costs you.

Run:  python3 viz_03_gradient_descent.py [--outdir DIR]
"""
import numpy as np

import viz_common as vc


def make_quadratic(kappa):
    H = np.diag([1.0, float(kappa)])

    def f(x, y):
        return 0.5 * (x ** 2 + kappa * y ** 2)

    def grad(z):
        return H @ z

    return f, grad, H


def gradient_descent(grad, H, z0, n_steps):
    """Steepest descent with the exact line search for a quadratic."""
    z = np.array(z0, dtype=float)
    path = [z.copy()]
    for _ in range(n_steps):
        p = -grad(z)
        if np.linalg.norm(p) < 1e-14:
            break
        alpha = float(p @ p) / float(p @ (H @ p))   # exact minimizer along p
        z = z + alpha * p
        path.append(z.copy())
    return np.array(path)


def panel_path(ax, kappa, n_steps, title, equal=False, y_frac=0.35):
    f, grad, H = make_quadratic(kappa)
    z0 = np.array([float(kappa), 1.0])
    path = gradient_descent(grad, H, z0, n_steps)

    lim = 1.35 * max(abs(z0[0]), 2.0)
    # kappa = 1 is the panel that *claims* the contours are circles, so it must
    # be drawn with a true 1:1 aspect ratio or the claim is invisible. The
    # kappa = 10 panel deliberately stretches y instead, because at true scale
    # the zigzag amplitude is a few percent of the plot width and vanishes.
    xlim = (-lim, lim)
    ylim = xlim if equal else (-y_frac * lim, y_frac * lim)
    vc.setup_axes(ax, xlim, ylim, title, equal=equal)
    vc.plot_contours(ax, f, xlim, ylim, levels=np.geomspace(0.05, f(*z0), 16),
                     label='$f$ contours')

    ax.plot(path[:, 0], path[:, 1], '-o', color=vc.COLORS['path'],
            ms=3.6, lw=1.5, zorder=5, label=f'GD path ({len(path) - 1} steps)')
    for a, b in zip(path[:-1], path[1:]):
        vc.draw_arrow(ax, a, b - a, vc.COLORS['path'], lw=1.4, head=8,
                      zorder=5, alpha=0.85)

    vc.mark_point(ax, path[0], vc.COLORS['grad_f'], size=80, label='start $z_0$')
    vc.mark_point(ax, [0, 0], vc.COLORS['optimum'], marker='*', size=220,
                  label='minimum')

    # Mark the forced right angle between consecutive steps.
    if len(path) > 2:
        d0, d1 = path[1] - path[0], path[2] - path[1]
        cos = float(d0 @ d1) / (np.linalg.norm(d0) * np.linalg.norm(d1))
        vc.annotate(ax,
                    f'$\\kappa={kappa}$   ratio per step '
                    f'$=\\frac{{\\kappa-1}}{{\\kappa+1}}={((kappa - 1) / (kappa + 1)):.3f}$\n'
                    f'angle between consecutive steps $= '
                    f'{np.degrees(np.arccos(cos)):.1f}^\\circ$  (computed, not drawn:\n'
                    'the $y$ axis is stretched here so the zigzag is visible)',
                    xy=(-0.94 * lim, 0.72 * ylim[1]))
    else:
        vc.annotate(ax,
                    f'$\\kappa={kappa}$: contours really are circles\n'
                    '(1:1 aspect), so $-\\nabla f$ points straight\n'
                    'at the minimum and one step is exact.',
                    xy=(-0.94 * lim, 0.80 * ylim[1]))
    ax.legend(loc='lower left', framealpha=0.92, ncol=2)


def panel_convergence(ax):
    ax.set_title('(c) Cost of conditioning')
    for kappa, color in [(1, '#2e7d32'), (5, '#1f77b4'),
                         (10, '#e07b12'), (50, '#c0392b')]:
        f, grad, H = make_quadratic(kappa)
        path = gradient_descent(grad, H, [float(kappa), 1.0], 60)
        fvals = np.array([f(z[0], z[1]) for z in path])
        fvals = np.maximum(fvals, 1e-18)
        ax.semilogy(fvals, color=color, lw=1.8, label=f'$\\kappa={kappa}$')
    ax.set_xlabel('iteration')
    ax.set_ylabel('$f(z_k)$  (log scale)')
    ax.set_ylim(1e-14, 1e4)
    ax.grid(True, which='both', alpha=0.25)
    ax.legend(framealpha=0.92)
    vc.annotate(ax,
                'Straight lines on a log plot =\n'
                'LINEAR convergence. Every step\n'
                'multiplies the error by a constant\n'
                '< 1. Compare Part 4.',
                xy=(22, 1e-4))


def main():
    outdir = vc.outdir_from_argv(__doc__.splitlines()[0])
    fig, axes = vc.plt.subplots(1, 3, figsize=(16.0, 4.6))
    panel_path(axes[0], kappa=1, n_steps=6, equal=True,
               title=r'(a) $\kappa=1$: no zigzag, one step')
    panel_path(axes[1], kappa=10, n_steps=18,
               title=r'(b) $\kappa=10$: the zigzag')
    panel_convergence(axes[2])
    fig.suptitle('Part 3 -- Gradient descent: the zigzag is the exact line '
                 r'search forcing $\nabla f(z_{k+1}) \perp p_k$', fontsize=11)

    for kappa in (1, 10, 50):
        f, grad, H = make_quadratic(kappa)
        path = gradient_descent(grad, H, [float(kappa), 1.0], 200)
        n = int(np.argmax([f(*z) < 1e-10 for z in path]))
        print(f'  kappa={kappa:3d}  ratio={(kappa - 1) / (kappa + 1):.4f}  '
              f'steps to f<1e-10: {n if n else len(path)}')
    vc.save(fig, '03_gradient_descent.png', outdir)


if __name__ == '__main__':
    main()
