"""Part 5 -- Visualizing SQP: watching the solver think, one iteration at a time.

Example problem (nonlinear objective AND nonlinear constraint):

    min  f(x, y) = (x - 2)^2 + 3 (y - 1)^2
    s.t. c(x, y) = x^2 + y^2 - 1 = 0

SQP never solves this. At each iterate it solves an easier problem that looks
like it locally, takes the step, and repeats. One iteration is four moves:

  1. QUADRATIC MODEL of the objective, using the Hessian of the LAGRANGIAN
        B_k = grad^2 f + lambda_k grad^2 c = diag(2, 6) + lambda_k * 2I
     Not grad^2 f. The QP is about to treat the constraint as a straight line,
     so the curvature the line throws away has to be carried somewhere -- and
     `lambda * grad^2 c` is where. See MPC_solver.md Part 8.2.

  2. LINEARIZE the constraint at z_k
        c(z_k) + grad c(z_k)^T p = 0
     A circle becomes its tangent line. The `c(z_k)` term is what pulls an
     infeasible iterate back towards the circle.

  3. SOLVE THE QP for the step p. With one equality constraint this is a 3x3
     KKT system -- exactly the saddle-point system of solvers/qp.py:

        [ B_k      grad c ] [ p       ]   [ -grad f(z_k) ]
        [ grad c^T   0    ] [ lambda+ ] = [ -c(z_k)      ]

     and the QP's own multiplier `lambda+` becomes the next lambda for free.

  4. STEP to z_k + p and start over.

What to watch in the animation: the tangent line is wrong almost everywhere,
but it is right *where it matters* -- near z_k. Each step lands slightly off
the circle, and the next linearization corrects for it. The iterates approach
the true optimum from outside the constraint, never exactly on it until the
limit. That is SQP: a sequence of deliberately wrong problems whose answers
converge to the right one.

Outputs a static per-iteration grid and an animated GIF.

Run:  python3 viz_05_sqp.py [--outdir DIR] [--no-gif]
"""
import argparse
import os

import numpy as np

import viz_common as vc
from matplotlib.animation import FuncAnimation, PillowWriter


# --- the example problem ---------------------------------------------------
def f(x, y):
    return (x - 2.0) ** 2 + 3.0 * (y - 1.0) ** 2


def grad_f(z):
    return np.array([2.0 * (z[0] - 2.0), 6.0 * (z[1] - 1.0)])


HESS_F = np.diag([2.0, 6.0])


def c(z):
    return z[0] ** 2 + z[1] ** 2 - 1.0


def c_field(x, y):
    return x ** 2 + y ** 2 - 1.0


def grad_c(z):
    return np.array([2.0 * z[0], 2.0 * z[1]])


HESS_C = 2.0 * np.eye(2)

Z0 = np.array([1.60, 1.70])
XLIM, YLIM = (-1.6, 2.9), (-1.5, 2.4)


def sqp(z0, max_iter=8, tol=1e-12):
    """Run SQP, recording everything needed to draw each iteration."""
    z = np.array(z0, dtype=float)
    lam = 0.0
    steps = []
    for _ in range(max_iter):
        gf, gc = grad_f(z), grad_c(z)
        B = HESS_F + lam * HESS_C          # Hessian of the LAGRANGIAN

        # 3x3 KKT system of the QP subproblem
        K = np.zeros((3, 3))
        K[:2, :2] = B
        K[:2, 2] = gc
        K[2, :2] = gc
        rhs = np.array([-gf[0], -gf[1], -c(z)])
        sol = np.linalg.solve(K, rhs)
        p, lam_new = sol[:2], float(sol[2])

        kkt = max(np.linalg.norm(gf + lam_new * gc), abs(c(z)))
        steps.append(dict(z=z.copy(), lam=lam, B=B.copy(), gf=gf, gc=gc,
                          p=p.copy(), lam_new=lam_new, c=c(z), kkt=kkt))
        z = z + p
        lam = lam_new
        if np.linalg.norm(p) < tol:
            break
    return steps, z, lam


def draw_iteration(ax, st, stage=4, show_legend=True, xlim=None, ylim=None):
    """Draw one SQP iteration. `stage` controls how much is revealed (0..4)."""
    z, B, gf, gc, p = st['z'], st['B'], st['gf'], st['gc'], st['p']
    xlim = xlim or XLIM
    ylim = ylim or YLIM

    vc.plot_contours(ax, f, xlim, ylim, levels=16, linewidths=0.7, alpha=0.55)
    vc.draw_curve(ax, c_field, xlim, ylim, lw=2.2,
                  label=r'true constraint $c=0$')
    vc.mark_point(ax, z, vc.COLORS['path'], size=95, label=r'iterate $z_k$')

    # 1. quadratic model of the objective, drawn in absolute coordinates
    if stage >= 1:
        def model(X, Y):
            dx, dy = X - z[0], Y - z[1]
            return (f(*z) + gf[0] * dx + gf[1] * dy
                    + 0.5 * (B[0, 0] * dx * dx + 2 * B[0, 1] * dx * dy
                             + B[1, 1] * dy * dy))
        vc.plot_contours(ax, model, xlim, ylim, levels=14,
                         color=vc.COLORS['model'], linewidths=1.0, alpha=0.8,
                         label='quadratic model  '
                               r'$\frac{1}{2}p^TB_kp+\nabla f^Tp$')

    # 2. linearized constraint: {z : grad_c . (z - z_k) + c(z_k) = 0}
    if stage >= 2:
        n = gc / np.linalg.norm(gc)
        base = z - st['c'] * gc / float(gc @ gc)
        t = np.array([-n[1], n[0]])
        span = 2.0 * max(xlim[1] - xlim[0], ylim[1] - ylim[0])
        s = np.linspace(-span, span, 2)
        line = base[None, :] + s[:, None] * t[None, :]
        ax.plot(line[:, 0], line[:, 1], color=vc.COLORS['constraint'],
                lw=2.0, ls='--', alpha=0.95,
                label=r'linearized  $c(z_k)+\nabla c^Tp=0$')

    # 3. the QP solution -- the step p
    if stage >= 3:
        vc.draw_arrow(ax, z, p, vc.COLORS['grad_f'], lw=2.8, head=13,
                      label=r'QP solution  $p$')

    # 4. where we land
    if stage >= 4:
        vc.mark_point(ax, z + p, vc.COLORS['grad_f'], size=95,
                      label=r'next iterate $z_{k+1}$')

    vc.mark_point(ax, Z_STAR, vc.COLORS['optimum'], marker='*', size=230,
                  label='true optimum')
    if show_legend:
        ax.legend(loc='lower left', framealpha=0.92, fontsize=7.2)


def static_grid(steps, outdir, n_panels=6):
    n = min(n_panels, len(steps))
    fig, axes = vc.plt.subplots(2, 3, figsize=(15.5, 9.4))
    for k, ax in enumerate(axes.ravel()):
        if k >= n:
            ax.axis('off')
            continue
        st = steps[k]
        # From iteration 2 on the steps are too small to see at full scale, so
        # each panel zooms to its own step. Watch what the zoom reveals: the
        # dashed linearization and the true circle become indistinguishable.
        # That is why SQP converges -- locally, the wrong problem is the right
        # problem.
        if k < 2:
            xlim, ylim = XLIM, YLIM
            zoomed = False
        else:
            r = max(4.0 * float(np.linalg.norm(st['p'])), 6e-3)
            xlim = (st['z'][0] - r, st['z'][0] + r)
            ylim = (st['z'][1] - r, st['z'][1] + r)
            zoomed = True
        vc.setup_axes(ax, xlim, ylim,
                      f'iteration {k}:  $\\lambda_k={st["lam"]:.4f}$,  '
                      f'$\\|p\\|={np.linalg.norm(st["p"]):.3e}$'
                      + (f'   (zoom $\\pm{r:.1e}$)' if zoomed else ''))
        draw_iteration(ax, st, stage=4, show_legend=(k == 0),
                       xlim=xlim, ylim=ylim)
        vc.annotate(ax,
                    f'$c(z_k)={st["c"]:+.3e}$\n'
                    f'KKT residual $={st["kkt"]:.2e}$',
                    xy=(xlim[0] + 0.04 * (xlim[1] - xlim[0]),
                        ylim[1] - 0.08 * (ylim[1] - ylim[0])))
    fig.suptitle('Part 5 -- SQP: each panel is one quadratic model + one '
                 'linearized constraint + one QP solve', fontsize=12)
    return vc.save(fig, '05_sqp_iterations.png', outdir)


STAGE_CAPTIONS = [
    'Where we are: iterate $z_k$, off the constraint',
    r'1. Build the quadratic model using $B_k=\nabla^2f+\lambda_k\nabla^2c$',
    r'2. Linearize the constraint: the circle becomes its tangent line',
    r'3. Solve the QP $\Rightarrow$ step $p$ (a $3\times3$ KKT system)',
    r'4. Step to $z_{k+1}=z_k+p$, and repeat',
]


def animation(steps, outdir, fps=1.6):
    frames = [(k, s) for k in range(len(steps)) for s in range(5)]
    frames += [(len(steps) - 1, 4)] * 4          # hold on the final frame

    fig, ax = vc.plt.subplots(figsize=(7.4, 6.6))

    def render(idx):
        k, stage = frames[idx]
        ax.clear()
        st = steps[k]
        vc.setup_axes(ax, XLIM, YLIM)
        draw_iteration(ax, st, stage=stage, show_legend=True)
        ax.set_title(f'SQP iteration {k}   --   {STAGE_CAPTIONS[stage]}',
                     fontsize=9.5)
        vc.annotate(ax,
                    f'$z_k=({st["z"][0]:+.4f}, {st["z"][1]:+.4f})$\n'
                    f'$\\lambda_k={st["lam"]:+.4f}$\n'
                    f'$c(z_k)={st["c"]:+.3e}$\n'
                    f'KKT $={st["kkt"]:.2e}$',
                    xy=(-1.52, 2.02))
        return []

    anim = FuncAnimation(fig, render, frames=len(frames), blit=False)
    path = os.path.join(outdir, '05_sqp_animation.gif')
    anim.save(path, writer=PillowWriter(fps=fps))
    vc.plt.close(fig)
    print(f'  wrote {path}  ({len(frames)} frames)')
    return path


# Reference optimum, found by solving the KKT system to high accuracy.
def _true_optimum():
    steps, z, lam = sqp(np.array([0.8, 0.6]), max_iter=40)
    return z, lam


Z_STAR, LAMBDA_STAR = _true_optimum()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--outdir', default=vc.DEFAULT_OUTDIR)
    parser.add_argument('--no-gif', action='store_true',
                        help='skip the animated GIF (much faster)')
    args = parser.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    steps, z_final, lam_final = sqp(Z0, max_iter=7)

    print(f'  true optimum  z* = ({Z_STAR[0]:.8f}, {Z_STAR[1]:.8f}), '
          f'lambda* = {LAMBDA_STAR:.8f}')
    print(f'  {"it":>3} {"x":>12} {"y":>12} {"lambda":>11} '
          f'{"|p|":>11} {"c(z)":>12} {"KKT":>11}')
    for k, st in enumerate(steps):
        print(f'  {k:3d} {st["z"][0]:12.8f} {st["z"][1]:12.8f} '
              f'{st["lam"]:11.6f} {np.linalg.norm(st["p"]):11.3e} '
              f'{st["c"]:+12.3e} {st["kkt"]:11.3e}')
    print(f'  final: ({z_final[0]:.10f}, {z_final[1]:.10f}), '
          f'lambda = {lam_final:.10f}')

    static_grid(steps, args.outdir)
    if not args.no_gif:
        animation(steps, args.outdir)


if __name__ == '__main__':
    main()
