# Why the Cost Is Quadratic, and How to Weight It

A companion to Part III of [`MPC_explanation_my_version.md`](MPC_explanation_my_version.md).

Part III writes every penalty as a quadratic form, $e^\top Q e$, and calls it
"the simplest choice". It is more than simple -- it is the choice that makes
the problem solvable inside a control loop, and swapping it for something
equally reasonable-sounding can cost you an order of magnitude in solve time.

This document explains why, and then covers the practical question Part III
leaves open: what numbers actually go in $Q$ and $R$.

Nothing here is needed to follow the main derivation.

---

## Why squared, and not something else

Three candidate penalties for the same error $e$, all of which are zero at
$e=0$ and grow as $e$ grows:

| penalty | shape | what it does |
|---|---|---|
| $\lvert e \rvert$ | V | kinked at zero -- not differentiable |
| $e^2$ | parabola | smooth, curvature constant |
| $e^4$ | flat-bottomed | smooth, but nearly indifferent near zero |

Only the middle one has all the properties a solver wants.

**It is differentiable everywhere.** $\lvert e \rvert$ has a kink at $e = 0$
-- exactly where the optimum lives. Gradient-based methods do not converge to
a point where the gradient does not exist; they oscillate across it. Every
method in [`optimization_visualized.md`](optimization_visualized.md) assumes
smoothness, and the kink breaks that assumption at the one place it matters.

**Its second derivative is constant.** $\nabla^2 (e^\top Q e) = 2Q$ -- the
same matrix everywhere, known in advance, never recomputed. Newton and SQP
build a quadratic model of the objective at each iterate (Part 4 of
`optimization_visualized.md`); when the objective *is* quadratic, that model
is not an approximation but the exact function. So the Hessian block of every
QP subproblem is just $2Q$ and $2R$ laid out in blocks, assembled once.

**The penalty scales with how wrong you are.** $e^4$ is flat near zero, so
small errors are almost free and the controller stops caring once it is close
-- it drifts. $\lvert e \rvert$ has constant slope, so the *last* millimetre
costs as much to fix as the first. The parabola's slope $2Qe$ grows with the
error: a big mistake pulls hard, a small one pulls gently. That is exactly
the behaviour you want from a tracker.

### What it buys structurally

Quadratic cost plus linear constraints is a **quadratic program**, and that
is the whole reason MPC is real-time feasible:

| | |
|---|---|
| convex | one minimum, no local traps, no dependence on initialization |
| exact Newton model | SQP converges in *one* iteration on a linear model |
| fixed sparsity | $H$ is block-diagonal, built once and reused every cycle |
| warm-startable | last cycle's answer is one step away from this cycle's |

Drop any of those and the guarantees go with it. An $\lvert e \rvert$ penalty
turns the QP into a linear program with auxiliary variables; an $e^4$ penalty
makes it a genuinely nonlinear program with a Hessian that has to be
recomputed at every iterate. Both are solvable. Neither is solvable in 10 ms
with a bound you can trust.

> **The honest caveat.** Quadratic penalties are the standard choice, not the
> only defensible one. The squared penalty grows fast, so a single large error
> -- a bad measurement, a sudden reference jump -- dominates the whole sum and
> the controller lunges to fix it. Robust alternatives (Huber: quadratic near
> zero, linear far away) exist precisely for that case, and cost you the exact
> Hessian in exchange. For a position controller tracking a smooth reference,
> the quadratic is the right trade.

---

## What actually goes in $Q$ and $R$

Part III says $Q$ weights the states and $R$ weights the inputs. In practice
both are diagonal, so the real question is what each diagonal entry means.

### The units problem

The twelve states are not in the same units. Position is metres, velocity
m/s, angles radians, rates rad/s. Inputs are newtons and newton-metres. A
"squared error" summed across all of them is adding metres² to radians²,
which is meaningless as physics.

$Q$ and $R$ are what make it meaningful. Each entry carries whatever units
are needed to turn its term into a common currency -- so their **first job is
non-dimensionalization, not preference**. A workable starting rule, sometimes
called Bryson's rule, is to make one unit of "acceptable error" in each
channel contribute equally:

$$Q_{ii} = \frac{1}{(\text{largest acceptable error in state } i)^2},
\qquad
R_{jj} = \frac{1}{(\text{largest acceptable deviation in input } j)^2}$$

So if 10 cm of position error and 5° of tilt are equally unacceptable, then
$Q_{xx} = 1/0.1^2 = 100$ and $Q_{\phi\phi} = 1/0.0873^2 = 131$. The numbers
look arbitrary until you see they encode a statement you can defend in
physical terms.

From there, tune by ratio rather than by absolute value.

### Only the ratio matters

Scaling $J$ by any positive constant leaves the minimizer exactly where it
was:

$$\arg\min_{X,U} J = \arg\min_{X,U} \alpha J \quad \text{for any } \alpha > 0$$

So $(Q, R) = (100, 1)$ and $(Q, R) = (1, 0.01)$ are the *same controller*.
There is one fewer degree of freedom here than it looks like. Conventionally
you fix one -- often $R = \mathbb{I}$ -- and tune $Q$ against it, which halves
the search space and stops you chasing a knob that does nothing.

What the ratio buys:

| | behaviour |
|---|---|
| $Q \gg R$ | aggressive -- chase the reference, spend whatever it takes, risk saturation and chatter |
| $Q \ll R$ | conservative -- smooth inputs, sluggish tracking, steady-state error under disturbance |

### Two practical traps

**Yaw wraparound.** $\psi$ and $\psi + 2\pi$ are the same heading, but
$(\psi - \psi_{\text{ref}})^2$ says they are maximally different. A drone at
$+179°$ tracking a reference at $-179°$ should turn 2°, and a naive quadratic
tells it to turn 358°. The error term for any angular state has to be wrapped
into $[-\pi, \pi)$ before it is squared. This is a real bug that shows up as
an occasional violent yaw spin and is very hard to find afterwards.

**Weighting velocity as well as position.** It is tempting to weight only
position, since that is what you care about. But a cost that scores only
position is happy for the drone to reach the target at full speed and blow
straight past it -- overshoot costs nothing until it becomes position error,
by which point it is too late to prevent. Putting weight on velocity error
gives the controller a reason to arrive *settled* rather than merely to
arrive. In LQR terms this is the difference between a critically damped
response and an oscillatory one, and it is set entirely by the ratio between
the position and velocity entries of $Q$.

### Why $x_0$'s term does not matter

The first state cost, $(x_0 - x_{\text{ref},0})^\top Q (x_0 - x_{\text{ref},0})$,
is a **constant**. $x_0$ is pinned by the initial-condition constraint
$x_0 = x_{\text{measured}}$ -- it is not something the optimizer gets to
choose. Adding a constant to $J$ shifts every trajectory's score equally and
changes the argmin not at all.

Keeping it in the sum is harmless and keeps the notation uniform. But if you
ever compare $J$ values between cycles and wonder why the number jumps when
the drone is far from the reference, this is why -- and it is why the sum is
often written from $k=1$ instead.

---

## Where this connects

| | |
|---|---|
| [`MPC_explanation_my_version.md`](MPC_explanation_my_version.md#step-3-terminal-cost) Part III Step 3 | the terminal cost in depth: infinite-horizon split, Riccati, what $Q_f = P$ buys |
| [`optimization_visualized.md`](optimization_visualized.md) | why smoothness and constant curvature matter to the solver, drawn |

---

| back to | |
|---|---|
| [`MPC_explanation_my_version.md`](MPC_explanation_my_version.md) | Parts I--III |
| [`euler_angle_rates.md`](euler_angle_rates.md) | Step 4 of Part I, in full |
| [`why_rk4.md`](why_rk4.md) | Part II's claims, measured |
