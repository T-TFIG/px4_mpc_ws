"""Part 4 -- Visualizing Newton's method, and comparing it with gradient descent.

Newton's idea in one line: gradient descent only knows a *slope*, so it can
pick a direction but not a distance. Newton builds a local parabola that
matches value, slope AND curvature, then jumps straight to that parabola's
minimum.

The local model at z_k is the second-order Taylor expansion

    m(p) = f(z_k) + grad f(z_k)^T p + 1/2 p^T H(z_k) p

Setting grad m = 0 gives the Newton step

    H(z_k) p = -grad f(z_k)        ->      z_{k+1} = z_k + p

Three things the panels make visible:

  (a) 1-D. Watch the parabola get fitted and the jump land on its vertex.
      f(x) = exp(-x) + x^2, convex everywhere, minimum near x = 0.3517.

  (b) 2-D on a quadratic. If f IS a quadratic, the model is not an
      approximation -- it is exact -- so Newton lands on the minimum in ONE
      step from anywhere, at any condition number. The same problem cost
      gradient descent ~18 steps in Part 3. Curvature is exactly the
      information that conditioning destroys, and Newton uses it.

  (c) A genuinely nonlinear problem, f(x,y) = (x-1)^2 + 10(y - x^2)^2, where
      the model is only local. Newton still converges QUADRATICALLY -- the
      log-error plot bends downward instead of running straight, which is the
      visual signature. Note the safeguard: H can be indefinite far from the
      minimum, where the "minimum of the parabola" is a saddle or a maximum
      and the step is garbage. We add tau*I to fix that, which is the same
      trick as `hessian_regularization` in solvers/sqp.py.

Run:  python3 viz_04_newton.py [--outdir DIR]
"""
import numpy as np

import viz_common as vc


# --- (a) the 1-D picture ---------------------------------------------------
def f1(x):
    return np.exp(-x) + x ** 2


def df1(x):
    return -np.exp(-x) + 2.0 * x


def d2f1(x):
    return np.exp(-x) + 2.0


def panel_1d(ax, x0=2.5, n_steps=3):
    ax.set_title('(a) 1-D: fit a parabola, jump to its vertex')
    xs = np.linspace(-0.9, 3.0, 400)
    ax.plot(xs, f1(xs), color='0.25', lw=2.2, label='$f(x)=e^{-x}+x^2$', zorder=4)

    colors = ['#1f77b4', '#e07b12', '#2e7d32', '#7d3c98']
    x = float(x0)
    for k in range(n_steps):
        g, hh = df1(x), d2f1(x)
        step = -g / hh
        model = f1(x) + g * (xs - x) + 0.5 * hh * (xs - x) ** 2
        c = colors[k % len(colors)]
        ax.plot(xs, model, color=c, lw=1.4, ls='--', alpha=0.85,
                label=f'model at $x_{k}$')
        ax.scatter([x], [f1(x)], c=[c], s=55, zorder=6, edgecolors='white')
        ax.annotate('', xy=(x + step, f1(x) + g * step + 0.5 * hh * step ** 2),
                    xytext=(x, f1(x)),
                    arrowprops=dict(arrowstyle='-|>', color=c, lw=1.8,
                                    mutation_scale=10), zorder=6)
        x = x + step

    ax.scatter([x], [f1(x)], marker='*', s=240, c=[vc.COLORS['optimum']],
               edgecolors='white', zorder=7, label=f'after {n_steps} steps')
    ax.set_xlim(-0.9, 3.0)
    ax.set_ylim(0, 10)
    ax.set_xlabel('$x$')
    ax.set_ylabel('$f(x)$')
    ax.legend(loc='upper left', framealpha=0.92, fontsize=7.2)
    vc.annotate(ax,
                'Each dashed parabola matches $f$ in\n'
                'value, slope and curvature at its dot.\n'
                'The arrow lands on that parabola\'s\n'
                'vertex: $p=-f\'(x)\\,/\\,f\'\'(x)$.',
                xy=(0.70, 8.35))


# --- (b) and (c) the 2-D comparison ---------------------------------------
def quad_parts(kappa=10.0):
    H = np.diag([1.0, kappa])

    def f(x, y):
        return 0.5 * (x ** 2 + kappa * y ** 2)

    def grad(z):
        return H @ z

    def hess(z):
        return H

    return f, grad, hess


def rosen_parts(b=5.0):
    def f(x, y):
        return (x - 1.0) ** 2 + b * (y - x ** 2) ** 2

    def grad(z):
        x, y = z
        return np.array([2.0 * (x - 1.0) - 4.0 * b * x * (y - x ** 2),
                         2.0 * b * (y - x ** 2)])

    def hess(z):
        x, y = z
        return np.array([[2.0 - 4.0 * b * (y - 3.0 * x ** 2), -4.0 * b * x],
                         [-4.0 * b * x, 2.0 * b]])

    return f, grad, hess


def _armijo(f, z, p, slope, alpha0=1.0, c=1e-4, max_backtracks=60):
    """Backtracking line search: shrink alpha until f drops enough.

    `slope` is the directional derivative grad_f . p, which is negative for
    any descent direction. Requiring f(z + a p) <= f(z) + c*a*slope rather
    than merely f(z + a p) < f(z) is what stops the method from taking an
    endless sequence of vanishingly small "improvements".
    """
    f0 = f(z[0], z[1])
    alpha = alpha0
    for _ in range(max_backtracks):
        if f(*(z + alpha * p)) <= f0 + c * alpha * slope:
            return alpha
        alpha *= 0.5
    return alpha


def newton(f, grad, hess, z0, n_steps, tol=1e-12):
    """Newton with a positive-definiteness safeguard.

    If the Hessian is not positive definite the model is a saddle or an
    upside-down bowl -- it has no minimum to jump to -- so we shift it to
    H + tau*I until it does. This is the same idea as `hessian_regularization`
    in solvers/sqp.py, and without it the method can step uphill.
    """
    z = np.array(z0, dtype=float)
    path = [z.copy()]
    for _ in range(n_steps):
        g = grad(z)
        if np.linalg.norm(g) < tol:
            break
        H = np.atleast_2d(hess(z))
        H = 0.5 * (H + H.T)
        min_eig = np.linalg.eigvalsh(H).min()
        tau = abs(min_eig) + 1e-3 if min_eig <= 1e-8 else 0.0
        p = np.linalg.solve(H + tau * np.eye(H.shape[0]), -g)
        z = z + _armijo(f, z, p, float(g @ p)) * p
        path.append(z.copy())
    return np.array(path)


def gradient_descent(f, grad, z0, n_steps, H=None, tol=1e-12):
    """Steepest descent. Exact line search when the problem is a quadratic
    (H given), Armijo backtracking otherwise."""
    z = np.array(z0, dtype=float)
    path = [z.copy()]
    for _ in range(n_steps):
        g = grad(z)
        if np.linalg.norm(g) < tol:
            break
        p = -g
        if H is not None:
            a = float(p @ p) / float(p @ (H @ p))     # exact for a quadratic
        else:
            a = _armijo(f, z, p, float(g @ p))
        z = z + a * p
        path.append(z.copy())
    return np.array(path)


def panel_quadratic(ax):
    kappa = 10.0
    f, grad, hess = quad_parts(kappa)
    z0 = np.array([kappa, 1.0])

    gd = gradient_descent(f, grad, z0, 18, H=np.diag([1.0, kappa]))
    nt = newton(f, grad, hess, z0, 5)

    lim = 1.35 * kappa
    xlim, ylim = (-lim, lim), (-0.35 * lim, 0.35 * lim)
    vc.setup_axes(ax, xlim, ylim,
                  r'(b) On a quadratic ($\kappa=10$): Newton needs ONE step',
                  equal=False)
    vc.plot_contours(ax, f, xlim, ylim,
                     levels=np.geomspace(0.05, f(*z0), 16))

    ax.plot(gd[:, 0], gd[:, 1], '-o', color=vc.COLORS['path'], ms=3.2, lw=1.4,
            label=f'gradient descent ({len(gd) - 1} steps)')
    ax.plot(nt[:, 0], nt[:, 1], '-o', color=vc.COLORS['grad_f'], ms=6, lw=2.6,
            label=f'Newton ({len(nt) - 1} step)')
    vc.draw_arrow(ax, nt[0], nt[1] - nt[0], vc.COLORS['grad_f'], lw=2.6, head=13)

    vc.mark_point(ax, z0, '0.35', size=70, label='start')
    vc.mark_point(ax, [0, 0], vc.COLORS['optimum'], marker='*', size=220)
    vc.annotate(ax,
                'A quadratic model of a quadratic\n'
                'is not an approximation -- it is\n'
                'the function. So the vertex of the\n'
                'model IS the minimum, at any $\\kappa$.',
                xy=(-0.95 * lim, 0.22 * lim))
    ax.legend(loc='lower left', framealpha=0.92)


def panel_nonlinear(ax):
    f, grad, hess = rosen_parts()
    z0 = np.array([-1.1, 1.3])

    gd = gradient_descent(f, grad, z0, 4000)
    nt = newton(f, grad, hess, z0, 40)

    ax.set_title('(c) Nonlinear: linear vs quadratic convergence')
    for path, color, name in [(gd, vc.COLORS['path'], 'gradient descent'),
                              (nt, vc.COLORS['grad_f'], 'Newton')]:
        gn = [max(np.linalg.norm(grad(z)), 1e-18) for z in path]
        ax.semilogy(gn, color=color, lw=1.9,
                    label=f'{name} ({len(path) - 1} steps)')
    ax.set_xlabel('iteration')
    ax.set_ylabel(r'$\|\nabla f(z_k)\|$  (log scale)')
    ax.set_xlim(0, 80)
    ax.set_ylim(1e-14, 1e2)
    ax.grid(True, which='both', alpha=0.25)
    ax.legend(framealpha=0.92, loc='upper right')
    vc.annotate(ax,
                r'$f=(x-1)^2+5(y-x^2)^2$' '\n\n'
                'Newton BENDS downward -- the number of\n'
                'correct digits roughly doubles each step\n'
                '(quadratic). GD is a straight line: a\n'
                'constant factor per step (linear).',
                xy=(23, 2e-9))


def main():
    outdir = vc.outdir_from_argv(__doc__.splitlines()[0])
    fig, axes = vc.plt.subplots(1, 3, figsize=(16.0, 4.8))
    panel_1d(axes[0])
    panel_quadratic(axes[1])
    panel_nonlinear(axes[2])
    fig.suptitle('Part 4 -- Newton: match the curvature, then jump to the '
                 r'model minimum ($H p = -\nabla f$)', fontsize=11)

    x = 2.5
    print('  1-D Newton iterates:')
    for k in range(4):
        print(f'    x_{k} = {x:.10f}   f\'= {df1(x):+.3e}')
        x -= df1(x) / d2f1(x)
    print(f'    x_4 = {x:.10f}')
    vc.save(fig, '04_newton_method.png', outdir)


if __name__ == '__main__':
    main()
