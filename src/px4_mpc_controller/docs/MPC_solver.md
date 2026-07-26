# Solving the Nonlinear MPC: From Model to Numbers

## What this document covers

`MPC_explanation.md` derives the quadrotor's equations of motion from first
principles and ends, in its Part 7, by *stating* the nonlinear MPC problem we
want to solve. It stops there deliberately -- writing down an optimization
problem and actually solving it are two different subjects.

This document is the second half. It picks up exactly at that point and
answers: **given that nonlinear model, how does a solver actually turn it
into numbers, 10 or 50 times a second?**

Same rule as the companion document: nothing is dropped in as a finished
formula. Every equation is derived from the one before it, with an
explanation of where it came from and why that step is necessary.

**Important scope note.** This document is about the **full nonlinear model**
derived in `MPC_explanation.md` -- the 12-state rigid-body quadrotor. It is
*not* a description of what `mpc_solver.py` currently implements. The code
today runs a simplified point-mass model, which (as `MPC_explanation.md`
Part 5 and the closing sections explain) is a deliberate simplification. This
document describes the machinery needed for the *real* model -- the thing we
would build next.

Where the two differ matters enormously, and Part 6 is entirely about that
difference: the point-mass problem is a convex QP with strong guarantees, and
the nonlinear problem is emphatically not.

---

## Part 1: The problem we are solving

### 1.1 Recap of the model

From `MPC_explanation.md` Part 6.1, with the Coriolis term dropped per its
Part 5, the state and dynamics are:

$$x = \begin{bmatrix}\xi\\\eta\\\dot\xi\\\dot\eta\end{bmatrix}\in\mathbb{R}^{12},
\qquad
\dot x = f(x,u) = \begin{bmatrix}
\dot\xi\\
\dot\eta\\
\frac{1}{m}\big(R(\eta)[0,0,T]^T - [0,0,mg]^T\big)\\
M(\eta)^{-1}W(\eta)^T\tau_{body}
\end{bmatrix}$$

and from its Part 6.2 the true control input is the four rotor thrusts, with
$T$ and $\tau_{body}$ obtained from them through the constant mixer matrix
$\Gamma$:

$$u = \begin{bmatrix}f_1\\f_2\\f_3\\f_4\end{bmatrix},
\qquad
\begin{bmatrix}T\\\tau_\phi\\\tau_\theta\\\tau_\psi\end{bmatrix} = \Gamma u
= \begin{bmatrix}1&1&1&1\\0&l&0&-l\\-l&0&l&0\\c&-c&c&-c\end{bmatrix}
\begin{bmatrix}f_1\\f_2\\f_3\\f_4\end{bmatrix}$$

Twelve states, four inputs. Everything in NED, so $\xi_z$ is *down*.

### 1.2 What makes this hard

Three properties of $f(x,u)$ drive everything in this document:

1. **It is nonlinear.** $R(\eta)$ and $W(\eta)$ are full of $\sin$ and $\cos$
   of state variables; $M(\eta)$ likewise, and it must be *inverted*. There
   is no way to write $f(x,u) = Ax + Bu$.
2. **It is coupled.** Translational acceleration depends on orientation
   through $R(\eta)$; orientation acceleration depends on orientation through
   $M(\eta)^{-1}$. You cannot decompose the problem into independent axes.
3. **It is underactuated.** Four inputs, twelve states, six degrees of
   freedom. The drone cannot accelerate sideways directly -- it must tilt
   first, which takes time. The solver has to *discover* this sequencing.

Point 3 is the interesting one. It's why the optimizer earns its keep: the
causal chain "thrusts → torque → angular acceleration → tilt → redirected
thrust → horizontal acceleration" is something the solver must plan through,
several steps ahead, rather than compute in closed form.

### 1.3 The three things we must build

To get from $\dot x = f(x,u)$ to a number we can send to the motors:

1. **Discretize** -- turn the continuous ODE into a relation between
   consecutive time steps (Part 2).
2. **Transcribe** -- turn the infinite-dimensional optimal-control problem
   into a finite-dimensional nonlinear program (Parts 3-5).
3. **Solve** -- find the optimum of that program, fast enough to be useful
   (Parts 7-11).

---

## Part 2: Discretization

### 2.1 Why we can't do what we did before

For the point-mass model, discretization was *exact*: the double integrator
under zero-order hold integrates in closed form
($p_{k+1} = p_k + v_k\Delta t + \tfrac12 u_k\Delta t^2$), with zero
truncation error. Equivalently, $\dot x = A_cx + B_cu$ has the closed-form
solution $x_{k+1} = e^{A_c\Delta t}x_k + \big(\int_0^{\Delta t}e^{A_cs}ds\big)B_cu_k$.

That route is closed to us now. $\dot x = f(x,u)$ with $f$ nonlinear has
**no general closed-form solution** -- there's no matrix exponential to
write down, no integral we can evaluate symbolically. We must integrate
*numerically*, and numerical integration means accepting some error. The
question becomes: how much error, for how much compute?

### 2.2 The simplest option, and why it isn't good enough

**Forward Euler** approximates the solution by assuming the derivative stays
constant across the whole interval:

$$x_{k+1} = x_k + \Delta t\, f(x_k, u_k)$$

Expand the true solution as a Taylor series about $t_k$:

$$x(t_k + \Delta t) = x_k + \Delta t\,\dot x_k + \frac{\Delta t^2}{2}\ddot x_k + O(\Delta t^3)$$

Euler captures the first two terms and drops everything from
$\frac{\Delta t^2}{2}\ddot x_k$ onward, so its **local** error is
$O(\Delta t^2)$. Accumulated over $1/\Delta t$ steps, the **global** error is
$O(\Delta t)$ -- first-order accurate.

Why that's inadequate here: the rotational dynamics are *fast*. Attitude
responds on the order of tens of milliseconds, an order of magnitude quicker
than the translational motion. With $\Delta t = 0.1\,$s, Euler applied to a
fast rotational mode doesn't merely lose accuracy, it can go **unstable** --
predicting oscillations that diverge rather than settle. An optimizer handed
an unstable prediction model will plan against a fantasy.

### 2.3 Runge-Kutta 4: the standard choice

The fix is to sample the derivative at several points *within* the interval
and combine them so that more Taylor terms cancel. The classical fourth-order
Runge-Kutta method uses four samples:

$$\begin{aligned}
k_1 &= f(x_k,\; u_k) && \text{slope at the start}\\
k_2 &= f\big(x_k + \tfrac{\Delta t}{2}k_1,\; u_k\big) && \text{slope at the midpoint, stepped with } k_1\\
k_3 &= f\big(x_k + \tfrac{\Delta t}{2}k_2,\; u_k\big) && \text{midpoint again, refined with } k_2\\
k_4 &= f\big(x_k + \Delta t\,k_3,\; u_k\big) && \text{slope at the end}\\[4pt]
x_{k+1} &= x_k + \frac{\Delta t}{6}\big(k_1 + 2k_2 + 2k_3 + k_4\big)
\end{aligned}$$

The weights $\tfrac16(1,2,2,4\text{-th}\,1)$ -- i.e. $1,2,2,1$ over 6 -- are
not arbitrary: they are precisely the values that make the combination match
the true Taylor expansion through the $\Delta t^4$ term. The result is
**local error $O(\Delta t^5)$, global error $O(\Delta t^4)$**.

Concretely, halving the step size reduces Euler's error by 2× and RK4's by
16×. At $\Delta t = 0.1$ that difference is the difference between a usable
prediction and a useless one.

Note that $u_k$ is held **constant** across all four evaluations. That's the
zero-order hold again -- physically correct, since we send one command per
tick and it stands until the next.

### 2.4 What RK4 costs

Four evaluations of $f$ per step, versus one for Euler. Each evaluation
requires building $R(\eta)$, $M(\eta)$, $W(\eta)$ and **inverting** $M(\eta)$
-- the expensive part. Over an $N$-step horizon that's $4N$ evaluations per
dynamics rollout, and (crucially) the solver needs not just $f$ but its
*derivatives* with respect to $x$ and $u$, at every one of those points.

This is why Part 10 (automatic differentiation) and Part 11 (real-time
strategies) exist. Dynamics evaluation dominates the cost of nonlinear MPC.

A practical optimization worth knowing: rather than forming $M(\eta)^{-1}$
explicitly, solve the linear system $M(\eta)\,\ddot\eta = W(\eta)^T\tau_{body}$
for $\ddot\eta$. It's a $3\times3$ system, so the saving is modest, but
explicit inversion is both slower and numerically worse, and the habit
matters more as models grow.

### 2.5 Writing the discrete dynamics compactly

Whatever integrator we choose, the outcome is a discrete map

$$x_{k+1} = F(x_k, u_k)$$

where $F$ denotes "integrate $f$ from $x_k$ with $u_k$ held for $\Delta t$."
Everything from here on uses $F$; the integrator choice is encapsulated
inside it.

---

## Part 3: Transcription -- turning a trajectory into variables

### 3.1 The problem to be solved

We want to optimize over *functions* -- a continuous control signal $u(t)$
over the horizon. Computers optimize over finite lists of numbers.
**Transcription** is the process of converting the former into the latter.

### 3.2 Two approaches, and why the choice matters more now

**Single shooting.** Make only the controls $u_0,\dots,u_{N-1}$ decision
variables and obtain the states by rolling the dynamics forward:
$x_1 = F(x_0,u_0)$, $x_2 = F(F(x_0,u_0),u_1)$, and so on.

This was merely inelegant for the linear point-mass model. For a nonlinear
one it is actively dangerous, because errors *compound multiplicatively*
through the rollout. The sensitivity of the final state to the first control
is a product of Jacobians:

$$\frac{\partial x_N}{\partial u_0} = \frac{\partial F}{\partial x}\bigg|_{N-1}\frac{\partial F}{\partial x}\bigg|_{N-2}\cdots\frac{\partial F}{\partial x}\bigg|_{1}\frac{\partial F}{\partial u}\bigg|_{0}$$

For unstable or fast dynamics -- exactly our rotational modes -- these
Jacobian products grow exponentially in $N$. The optimizer sees gradients
that differ by many orders of magnitude between early and late controls, and
convergence degrades badly. This is the classic failure mode of single
shooting on nonlinear systems.

**Multiple shooting** (what to use). Make **both** states and controls
decision variables, and impose the dynamics as explicit equality constraints,
called *defect* constraints:

$$c_k(x_k,u_k,x_{k+1}) \;=\; x_{k+1} - F(x_k,u_k) \;=\; 0, \qquad k=0,\dots,N-1$$

"Defect" because the quantity measures the gap between where the dynamics say
you'd arrive and where the decision variable says you are. At the solution
all defects are zero and the trajectory is physically consistent; *during*
the solve they may be nonzero, which is exactly the point.

### 3.3 Why multiple shooting wins

- **No compounding.** Each defect couples only $x_k$, $u_k$, $x_{k+1}$. The
  long Jacobian chain is broken into $N$ short, independent pieces.
- **Sparsity.** The constraint Jacobian is block-banded. Sparse linear algebra
  exploits this so effectively that cost scales roughly *linearly* in $N$,
  making the extra variables nearly free.
- **State constraints are trivial.** A tilt limit $|\phi_k|\le\phi^{max}$ is a
  plain bound on a variable, not a nonlinear function of every preceding
  control.
- **Infeasible initialization is allowed** -- and this is a genuinely
  important practical advantage. You can seed the optimizer with a guess that
  doesn't satisfy the dynamics at all (say, a straight line from start to
  goal). The solver drives the defects to zero as it converges. Single
  shooting cannot do this: its states are always *defined* by the rollout, so
  you can only ever guess controls.
- **Parallelism.** All $N$ dynamics evaluations are independent and can be
  computed simultaneously.

### 3.4 The decision variables

$$z = \big(x_0, x_1, \dots, x_N,\; u_0, u_1, \dots, u_{N-1}\big)$$

With $n_x = 12$, $n_u = 4$, and horizon $N$:

| Quantity | Count |
|---|---|
| States | $12(N+1)$ |
| Controls | $4N$ |
| **Total** | $\mathbf{16N + 12}$ |

For $N = 20$: 332 decision variables. (Compare: 186 for the point-mass model
at the same horizon. Under twice as many variables -- the size increase is
*not* what makes this problem hard. Nonconvexity is.)

---

## Part 4: The cost function

### 4.1 General form

$$J = \sum_{k=0}^{N-1}\Big[\underbrace{(x_k - x_k^{ref})^TQ(x_k - x_k^{ref})}_{\text{state tracking}} + \underbrace{(u_k - u_{hover})^TR(u_k - u_{hover})}_{\text{control effort}}\Big] + \underbrace{(x_N - x_N^{ref})^TQ_f(x_N-x_N^{ref})}_{\text{terminal}}$$

with $Q, Q_f \succeq 0$ and $R \succ 0$ weight matrices. Three subtleties
below are specific to this model and easy to get wrong.

### 4.2 Subtlety 1: orientation error is not subtraction

For position, $\xi_k - \xi_k^{ref}$ is exactly right. For **yaw** it is not.

Suppose $\psi = +179°$ and $\psi^{ref} = -179°$. Naive subtraction gives an
error of $358°$, when the true angular distance is $2°$ in the other
direction. Feed that to an optimizer and it will command a violent, entirely
unnecessary full rotation.

Yaw lives on a circle, not a line. Two standard fixes:

$$e_\psi = \operatorname{atan2}\big(\sin(\psi - \psi^{ref}),\; \cos(\psi-\psi^{ref})\big)$$

which wraps the error to $(-\pi,\pi]$, or the smooth alternative

$$e_\psi^2 \;\longrightarrow\; 2\big(1 - \cos(\psi - \psi^{ref})\big)$$

which agrees with $e_\psi^2$ to second order near zero and has no branch cut
at all -- often preferable inside an optimizer, since `atan2` introduces a
derivative discontinuity the solver can trip over.

Roll and pitch don't need this in practice (they're bounded well away from
$\pm\pi$ by the tilt constraint in Part 5.4), but yaw genuinely does.

### 4.3 Subtlety 2: control effort is measured about *hover*, not zero

This one is critical, and getting it wrong produces a drone that refuses to
fly.

A naive control-effort term $\|u_k\|^2$ penalizes rotor thrust in absolute
terms -- so the cheapest possible control is $u = 0$, i.e. **all motors off**.
The optimizer would then permanently trade altitude against control cost,
and the drone would sag.

The fix is to penalize *deviation from the equilibrium input*: what the
motors must produce simply to hover. Derive it. At hover, $\eta = 0$,
$\dot\eta = 0$, $\dot\xi = 0$, so the dynamics require $T = mg$ and
$\tau_{body} = 0$. Substituting into the mixer:

$$\begin{aligned}
f_1+f_2+f_3+f_4 &= mg &&\text{(thrust balances weight)}\\
l(f_2 - f_4) &= 0 \;\Rightarrow\; f_2 = f_4 &&(\tau_\phi = 0)\\
l(f_3 - f_1) &= 0 \;\Rightarrow\; f_1 = f_3 &&(\tau_\theta = 0)\\
c(f_1 - f_2 + f_3 - f_4) &= 0 \;\Rightarrow\; f_1 = f_2 &&(\tau_\psi = 0,\text{ using the above})
\end{aligned}$$

All four equal, summing to $mg$:

$$\boxed{\;u_{hover} = \frac{mg}{4}\,[1,1,1,1]^T\;}$$

So the correct term is $(u_k - u_{hover})^TR(u_k-u_{hover})$: hovering is
free, and only *deviation* from hover is penalized. (This is the error
corrected in `MPC_explanation.md` Part 7, which originally had an asymmetric
$(u_k-u_{hover})^TRu_k$.)

### 4.4 Subtlety 3: the states have wildly different units

$Q$ is weighting metres against radians against metres/second against
radians/second, all summed into one scalar. A "1" in the position block and a
"1" in the attitude block do **not** mean comparable things -- 1 metre of
position error and 1 radian (57°!) of attitude error are hugely different in
severity.

The standard discipline is **Bryson's rule** as a starting point: set each
diagonal weight to the reciprocal of the square of the largest acceptable
deviation in that channel,

$$Q_{ii} = \frac{1}{(\text{max acceptable error in state } i)^2}$$

so every term contributes comparably when at its own tolerance. Tune from
there. Without something like this, weight tuning is guesswork across
incommensurable units.

### 4.5 The terminal cost, and why it is not optional here

The terminal cost $Q_f$ is what stops the optimizer from planning a
trajectory that ends in a disastrous state just outside the horizon -- e.g.
arriving at the right position while inverted and tumbling. A finite horizon
sees only $N$ steps; the terminal term is the proxy for everything after.

The principled choice is the **infinite-horizon LQR cost-to-go**: linearize
$f$ about the hover equilibrium, solve the discrete algebraic Riccati
equation

$$P = A^TPA - A^TPB(R + B^TPB)^{-1}B^TPA + Q$$

and set $Q_f = P$. This makes the finite-horizon cost approximate the
infinite-horizon one, and (together with a terminal constraint set) is the
standard route to a **provable closed-loop stability guarantee** for the MPC
controller.

This matters far more for the nonlinear model than it did for the point mass.
A point mass with a bad terminal cost tracks slightly worse; an underactuated
quadrotor with a bad terminal cost can plan itself into an attitude it cannot
recover from.

---

## Part 5: Constraints

### 5.1 Initial condition

$$x_0 = \hat x(t)$$

The measured (or estimated) state. As always, this is the single line that
makes the whole scheme a feedback controller rather than open-loop planning.

Note $\hat x$ now includes **orientation and angular velocity**, not just
position and velocity -- 12 numbers that must come from the estimator. This
is a materially heavier sensing requirement than the point-mass version, and
it is where a state estimator (the Kalman-filter work item on the project
list) becomes genuinely load-bearing rather than a nice-to-have.

### 5.2 Dynamics defects

$$x_{k+1} - F(x_k,u_k) = 0, \qquad k = 0,\dots,N-1$$

$12N$ scalar equality constraints. Unlike the point-mass case these are
**nonlinear**, and that single fact is what changes the problem class in
Part 6.

### 5.3 Rotor thrust bounds

$$0 \;\le\; f_{i,k} \;\le\; f_{max}, \qquad i = 1,\dots,4$$

The lower bound is genuinely physical, not conservatism: a fixed-pitch rotor
can push but **cannot pull**. The upper bound is the motor's maximum.

This is precisely why `MPC_explanation.md` Part 6.2 argues for formulating in
terms of $f_i$ rather than $(T, \tau_{body})$. In $(T,\tau)$ coordinates,
"each individual rotor is within its limits" maps to an awkward polytope; in
$f_i$ coordinates it is four simple bounds. Same physics, far better
conditioned.

$8N$ scalar inequalities.

### 5.4 Tilt limit

$$|\phi_k| \le \phi^{max}, \qquad |\theta_k| \le \theta^{max}$$

Two independent reasons to include this, and they are worth separating:

1. **Physical.** Beyond roughly 60° of tilt, the vertical thrust component
   $T\cos\phi\cos\theta$ can no longer support the vehicle's weight and it
   loses altitude regardless of throttle.
2. **Mathematical.** Recall from `MPC_explanation.md` Part 4 that $W(\eta)$
   becomes singular at $\theta = \pm 90°$ -- gimbal lock -- which would make
   $M(\eta)^{-1}$ blow up inside our own dynamics function. Constraining tilt
   to, say, $\pm45°$ keeps the optimizer's iterates safely away from a region
   where our chosen *representation* breaks down, independent of whether the
   physical vehicle could survive it.

Reason 2 is easy to overlook and important: the constraint is partly
protecting the solver from our modelling choice, not just protecting the
drone.

### 5.5 Count

| Kind | Count | Linear? |
|---|---|---|
| Initial condition (eq.) | 12 | yes |
| Dynamics defects (eq.) | $12N$ | **no** |
| Rotor bounds (ineq.) | $8N$ | yes |
| Tilt limits (ineq.) | $4N$ | yes |

For $N=20$: 252 equalities, 240 inequalities, 332 variables.

---

## Part 6: What kind of problem this is now

### 6.1 A nonconvex NLP

Quadratic objective, but **nonlinear equality constraints** (the defects).
That makes this a general **Nonlinear Program (NLP)** -- and because
nonlinear equalities define a curved, non-convex feasible set, it is
**nonconvex**.

This is the single most consequential difference from the point-mass
controller, so it's worth tabulating precisely what is lost:

| | Point-mass (convex QP) | Full model (nonconvex NLP) |
|---|---|---|
| Local optimum ⇒ global | **Yes** | **No** |
| Solution unique | Yes | No |
| Feasibility detectable | Reliably | Hard |
| Solve time | Predictable | Variable |
| Initial guess | Irrelevant | **Critical** |
| Iterations | Few, consistent | Variable, can fail |

Every guarantee in the left column is gone. The extra trigonometry is
incidental; *this* is the real cost of the better model.

### 6.2 What a local minimum looks like physically

Nonconvexity isn't abstract here. A concrete example: the drone must yaw
180°. Rotating left and rotating right are both locally optimal, separated by
a cost barrier. A solver started with a leftward guess converges left and
never discovers that right was equally good -- or, with an asymmetric
obstacle, that right was *better*.

Another: to reach a position quickly, the optimizer may find a solution that
tilts aggressively one way, and a different solution tilting the other way,
with a worse-cost region between them. Which one you get depends entirely on
where you started.

### 6.3 Why warm starting is now mandatory, not merely helpful

In the convex case, warm starting was an efficiency optimization -- the
solver would find the same unique global optimum regardless of where it
began.

Here, the initial guess **selects which local optimum you converge to**. Warm
starting from the previous tick's solution is therefore doing real work
beyond saving iterations: it keeps the controller in the *same basin of
attraction* from tick to tick. Without it, consecutive solves could jump
between distinct local optima, producing wildly discontinuous commands even
though each individual solve is "optimal."

This is a genuine safety consideration in nonlinear MPC, not a performance
footnote.

---

## Part 7: Optimality conditions for the NLP

### 7.1 The Lagrangian and KKT conditions

Write the problem generically with $z$ stacking all decision variables:

$$\min_z f(z) \quad\text{s.t.}\quad g(z) = 0,\quad h(z)\le 0$$

Form the Lagrangian, folding constraints into the objective with one
multiplier each:

$$\mathcal{L}(z,\lambda,\nu) = f(z) + \lambda^Tg(z) + \nu^Th(z)$$

The **Karush-Kuhn-Tucker** conditions:

$$\begin{aligned}
&\text{Stationarity:} && \nabla f(z) + \nabla g(z)^T\lambda + \nabla h(z)^T\nu = 0\\
&\text{Primal feasibility:} && g(z)=0,\quad h(z)\le 0\\
&\text{Dual feasibility:} && \nu \ge 0\\
&\text{Complementary slackness:} && \nu_i h_i(z) = 0\ \ \forall i
\end{aligned}$$

Interpretation: at an optimum the cost gradient is exactly balanced by the
constraint gradients (nothing can improve without violating something); the
multipliers price each constraint -- how much the optimum would improve per
unit of relaxation; and each inequality is either inactive with zero price,
or active and sitting exactly on its boundary.

### 7.2 The crucial difference from the convex case

For the convex QP, KKT was **necessary and sufficient**: find a KKT point and
you have provably found the global optimum.

Here, KKT is only **necessary**. A KKT point may be a local minimum, a
saddle, or even a local maximum. To confirm a local *minimum* you need the
second-order condition: the Hessian of the Lagrangian must be positive
definite on the null space of the active constraint gradients,

$$d^T\nabla^2_{zz}\mathcal{L}\,d > 0 \quad \text{for all } d\neq 0 \text{ with } \nabla g\,d = 0,\ \nabla h_{\mathcal{A}}d = 0$$

("moving in any direction that stays feasible increases the cost"). And even
this only certifies a *local* minimum -- global optimality is, in general,
not verifiable.

There's a further technical requirement: KKT conditions are only valid under
a **constraint qualification** such as LICQ (the active constraint gradients
must be linearly independent). This usually holds, but it can fail with
redundant or degenerate constraints -- another reason to prefer a clean,
non-redundant constraint formulation like the rotor-thrust bounds of
Part 5.3.

---

## Part 8: Newton-type methods

### 8.1 The core idea

The KKT conditions are a system of nonlinear equations (plus inequalities).
Newton's method is the natural tool: linearize, solve for a step, move,
repeat. Ignoring inequalities momentarily, we want $F(z,\lambda)=0$ where

$$F(z,\lambda) = \begin{bmatrix}\nabla f(z) + \nabla g(z)^T\lambda\\ g(z)\end{bmatrix}$$

Newton's step solves $\nabla F\cdot\Delta = -F$, which written out is the
**KKT system**:

$$\begin{bmatrix}
W & \nabla g^T\\
\nabla g & 0
\end{bmatrix}
\begin{bmatrix}\Delta z\\ \Delta\lambda\end{bmatrix}
= -\begin{bmatrix}\nabla_z\mathcal{L}\\ g(z)\end{bmatrix},
\qquad W = \nabla^2_{zz}\mathcal{L}$$

### 8.2 The Hessian is no longer constant -- and may be indefinite

For the point-mass QP, $W$ was the constant cost Hessian: constraints were
linear, so their second derivatives vanished and contributed nothing.

Not so here. Because $g$ is nonlinear, the Lagrangian Hessian picks up
curvature from the constraints themselves, weighted by their multipliers:

$$W = \nabla^2 f(z) + \sum_i \lambda_i\nabla^2 g_i(z) + \sum_j \nu_j\nabla^2 h_j(z)$$

Two consequences:

1. **$W$ changes every iteration** and must be recomputed -- second
   derivatives of the dynamics, which are expensive.
2. **$W$ may be indefinite.** The $\lambda_i\nabla^2g_i$ terms carry no sign
   guarantee. If $W$ is indefinite the Newton direction may not be a descent
   direction at all -- it can point uphill.

Solvers handle (2) with **regularization**: add $\delta I$ to the Hessian
block, increasing $\delta$ until the KKT matrix has the correct *inertia*
($n$ positive and $m$ negative eigenvalues, the signature of a genuine
minimum). IPOPT does exactly this, automatically. It costs extra
factorizations when it triggers -- a hidden source of variable solve time.

### 8.3 Cheaper Hessians: Gauss-Newton

Because our cost is a **sum of squares** -- write it as
$f(z) = \tfrac12\|r(z)\|^2$ with residual $r$ collecting all the weighted
tracking errors -- the exact Hessian is

$$\nabla^2 f = J_r^TJ_r + \sum_i r_i\nabla^2 r_i, \qquad J_r = \frac{\partial r}{\partial z}$$

The **Gauss-Newton approximation** discards the second term:

$$\nabla^2 f \;\approx\; J_r^TJ_r$$

Three reasons this is standard practice in tracking NMPC:

- **Always positive semidefinite** by construction, so the direction is
  always a descent direction -- no regularization needed, no indefiniteness.
- **Only first derivatives required.** No second derivatives of the dynamics
  at all: a large saving, since those are the most expensive quantities.
- **The dropped term is small when tracking well**, since it's weighted by
  the residuals $r_i$ themselves -- and in a working tracking controller the
  residuals *are* small.

The cost is convergence rate: superlinear rather than Newton's quadratic. For
real-time NMPC that trade is almost always worth it, and Gauss-Newton is what
`acados` uses by default.

(A middle option, **BFGS**, builds a PSD Hessian approximation from
successive gradients. It's common in general-purpose optimization -- IPOPT
supports it via `hessian_approximation: limited-memory` -- but for
least-squares tracking problems Gauss-Newton exploits the structure better.)

---

## Part 9: Two solver families

### 9.1 Sequential Quadratic Programming (SQP)

**Idea:** approximate the NLP by a QP at the current iterate, solve that QP
exactly, step, repeat. At iterate $z^{(i)}$:

$$\begin{aligned}
\min_{\Delta z}\quad & \tfrac12\Delta z^TW\Delta z + \nabla f(z^{(i)})^T\Delta z\\
\text{s.t.}\quad & g(z^{(i)}) + \nabla g(z^{(i)})\Delta z = 0\\
& h(z^{(i)}) + \nabla h(z^{(i)})\Delta z \le 0
\end{aligned}$$

The constraints are *linearized*; the objective is a quadratic model. Note
that this QP subproblem has exactly the structure of the point-mass problem
from the other document -- so all the machinery for solving convex QPs applies
directly at each iteration.

Strengths: excellent warm starting (the active set changes little between
ticks); very fast when started near the solution -- the typical MPC
situation. This is what makes SQP the dominant choice for real-time NMPC.

### 9.2 Interior-point (IPOPT)

**Idea:** eliminate the combinatorial difficulty of "which inequalities are
active" by replacing them with a logarithmic barrier that becomes infinite at
the boundary:

$$\min_z\; f(z) - \mu\sum_i\log\big(-h_i(z)\big)\quad\text{s.t.}\quad g(z)=0$$

Deriving what this does to the KKT conditions is illuminating. Let
$s_i \equiv -h_i(z) > 0$ (the slack -- distance to constraint $i$). Then

$$\frac{\partial}{\partial z}\big[-\mu\log s_i\big] = -\frac{\mu}{s_i}\cdot(-\nabla h_i) = \frac{\mu}{s_i}\nabla h_i$$

so the barrier problem's stationarity condition is

$$\nabla f + \nabla g^T\lambda + \sum_i\frac{\mu}{s_i}\nabla h_i = 0$$

Comparing with the true stationarity condition of Part 7.1, these are
*identical* provided we define $\nu_i \equiv \mu/s_i$, i.e.

$$\nu_i s_i = \mu$$

That is the key structural result: **the barrier problem satisfies the exact
KKT conditions, except complementary slackness is perturbed from
$\nu_is_i = 0$ to $\nu_is_i = \mu$.** The awkward either/or has become a
smooth equation. The family of solutions traced as $\mu$ varies is the
**central path**, and the algorithm is: solve for a given $\mu$, decrease
$\mu$, repeat until $\mu\to0$ recovers true complementarity.

Each inner iteration is a Newton step on the perturbed system, giving the
augmented KKT system

$$\begin{bmatrix}W + \Sigma & \nabla g^T\\ \nabla g & 0\end{bmatrix}
\begin{bmatrix}\Delta z\\\Delta\lambda\end{bmatrix} = -\begin{bmatrix}r_{dual}\\ r_{primal}\end{bmatrix},
\qquad \Sigma = \mathrm{diag}(\nu_i/s_i)$$

As an iterate approaches a boundary, $s_i\to0$, that entry of $\Sigma$
explodes, and the step is prevented from crossing -- the mechanism enforcing
strict interiority.

Globalization is by **filter line search**: rather than demanding the
objective decrease every step (too rigid when constraints must also be
satisfied), IPOPT tracks both objective value and constraint violation and
accepts a step improving either without being dominated by a previous pair.

### 9.3 Which to use

| | SQP | Interior point |
|---|---|---|
| Warm start | **Excellent** | Limited |
| Per-iteration cost | Higher (solves a QP) | Lower |
| Iteration count when warm-started | **Very low** | Moderate |
| Handles many inequalities | Less gracefully | **Well** |
| Real-time NMPC suitability | **Standard choice** | Less common |

The decisive row is warm starting. Interior-point methods must keep iterates
strictly *inside* the feasible region, so a warm start sitting exactly on a
constraint boundary -- which optimal solutions frequently do -- must first be
pushed back inside, discarding much of the benefit. SQP has no such
restriction and reuses the previous active set directly.

For an MPC loop, where consecutive problems are nearly identical, that
advantage is decisive. This is the concrete reason `MPC_explanation.md`
flagged `acados` over IPOPT for the real-dynamics controller.

---

## Part 10: Where the derivatives come from

Every method above needs $\nabla f$, $\nabla g$, and often $W$. For our
dynamics -- RK4 wrapped around a function containing $R(\eta)$, $M(\eta)^{-1}$,
$W(\eta)$ -- deriving these by hand is out of the question.

**Finite differences** ($\partial f/\partial x_i \approx [f(x+\epsilon e_i)-f(x)]/\epsilon$)
are the obvious fallback and a poor one: $n+1$ function evaluations per
Jacobian, and an unavoidable accuracy trade-off ($\epsilon$ too large gives
truncation error, too small gives catastrophic cancellation in floating
point).

**Automatic differentiation (AD)** -- what CasADi provides -- is the right
tool. AD decomposes the function into elementary operations and applies the
chain rule mechanically, producing derivatives accurate to machine precision,
with no step-size parameter. Two modes:

- **Forward mode:** propagates $\partial(\cdot)/\partial x_i$ forward through
  the graph. Cost $\propto$ number of *inputs*.
- **Reverse mode** (backpropagation): propagates sensitivities backward from
  outputs. Cost $\propto$ number of *outputs*.

For a gradient (one scalar output, hundreds of inputs) reverse mode computes
the **entire** gradient for roughly the cost of one function evaluation --
regardless of dimension. That's the difference between tractable and not.

CasADi additionally detects **sparsity patterns** automatically, so the
block-banded structure created by multiple shooting is exploited without
being told about it, and can **generate C code** for the whole evaluation --
removing Python interpreter overhead entirely, which matters a great deal in
a real-time loop.

---

## Part 11: Making it real-time

### 11.1 The budget problem

Guidance-level MPC at 10 Hz allows 100 ms per solve -- comfortable. But the
whole motivation for the full-dynamics model is to control *attitude*, and
attitude loops want 50-250 Hz: **4-20 ms per solve**, including dynamics
evaluation, derivatives, and multiple Newton iterations. Converging an NLP to
full tolerance in that window is generally not possible.

### 11.2 The Real-Time Iteration (RTI) scheme

The key insight (Diehl et al.) is that **converging fully is unnecessary**.
Consecutive MPC problems differ only slightly, so rather than iterating to
convergence on each one, perform **exactly one SQP iteration per control
tick**, warm-started from the previous. The optimizer never fully converges
on any single problem -- instead it *tracks* the moving solution across
ticks, which is all a controller actually needs.

RTI splits each tick into two phases:

- **Preparation phase** (the expensive part): evaluate dynamics, compute
  Jacobians, build and condense the QP. Crucially, this depends only on the
  *previous* solution -- so it can run **before the new measurement
  arrives**.
- **Feedback phase** (cheap): substitute the new $\hat x(t)$ and solve the
  already-prepared QP.

Because only the feedback phase sits between measurement and command, the
effective control latency drops to a fraction of the total computation --
often microseconds. This is the central trick that makes nonlinear MPC
viable on real flight hardware.

### 11.3 Condensing

The multiple-shooting QP has block-banded structure that can be handled two
ways:

- **Condensing:** eliminate the state variables using the linearized
  dynamics, leaving a smaller dense QP in the controls only ($4N$ variables
  instead of $16N+12$). Better when $N$ is small.
- **Sparse (structure-exploiting):** keep all variables and use a solver
  designed for banded KKT systems (e.g. HPIPM, or a Riccati-based recursion,
  which scales linearly in $N$). Better when $N$ is large.

The crossover is typically around $N \approx 20{-}50$ -- right at our horizon
length, so it's worth benchmarking both rather than assuming.

### 11.4 acados

`acados` packages all of the above: code-generated C, RTI, Gauss-Newton
Hessians, HPIPM as the QP solver, condensing options, with a Python interface
that accepts a CasADi model directly. Reported solve times for
quadrotor-scale NMPC are in the tens to hundreds of **microseconds** --
several orders of magnitude faster than IPOPT-in-a-Python-loop.

The migration path from our current code is unusually smooth: the CasADi
model expression is reusable as-is, and only the solver setup changes.

---

## Part 12: What to actually build

A pragmatic ordering, lowest-risk first:

1. **Write $f(x,u)$ in CasADi** exactly as derived in `MPC_explanation.md`
   Part 6.1, with $\tau_{body} = \Gamma u$ from the mixer. Verify it in
   isolation before any optimization: check that $u = u_{hover}$ at
   $\eta = 0$ produces $\dot x \approx 0$, and that a small roll torque
   produces roll acceleration of the expected sign and magnitude. Dynamics
   sign errors are the most common and most confusing failure mode; catch
   them here, not inside a solver.
2. **Wrap it in RK4** and simulate open-loop from hover. A correct model
   should stay near hover briefly and then drift -- a quadrotor is
   open-loop unstable in attitude, so *some* divergence is expected and
   correct. Divergence within a handful of timesteps indicates a bug.
3. **Build the NLP with IPOPT first.** Not because it's the right long-term
   choice, but because it's already installed, robust, and forgiving while
   the formulation is still being debugged. Test offline, not in the control
   loop.
4. **Check solve time honestly.** If it exceeds the control period, the
   controller cannot run -- and that's expected at this stage; it's data, not
   failure.
5. **Migrate to `acados` with RTI** once the formulation is verified correct.
   Optimizing before the model is right optimizes the wrong thing.
6. **Change the PX4 interface last.** Recall from `MPC_explanation.md` Part 7
   that this controller commands attitude or body rates, not
   `TrajectorySetpoint` -- so `OffboardControlMode.body_rate` (or
   `.attitude`) with `VehicleRatesSetpoint`/`VehicleAttitudeSetpoint`
   replaces the current path. Do this only once the solver is trustworthy;
   debugging a new controller *and* a new PX4 interface simultaneously is
   how projects lose days.

The one non-negotiable: **do not put an unverified nonlinear solver in the
control loop.** Steps 1-4 are all offline for exactly that reason.

---

## Summary

- The model from `MPC_explanation.md` is nonlinear, coupled, and
  underactuated. Those three properties drive everything here (Part 1).
- **Discretization must be numerical** -- no closed form exists. RK4 gives
  $O(\Delta t^4)$ global error for 4 dynamics evaluations per step; Euler's
  $O(\Delta t)$ is inadequate and can be unstable on fast rotational modes
  (Part 2).
- **Multiple shooting** is now essential, not merely preferable: single
  shooting compounds Jacobians multiplicatively across the horizon and
  conditions badly on nonlinear dynamics (Part 3).
- Cost design has three model-specific traps: **yaw error must be wrapped**,
  **control effort must be measured about $u_{hover} = \frac{mg}{4}[1,1,1,1]^T$**
  (else the optimizer prefers motors-off), and **weights span
  incommensurable units** so something like Bryson's rule is needed (Part 4).
- **Tilt constraints serve double duty**: physical (thrust can't support
  weight past ~60°) and mathematical (keeping iterates away from $W(\eta)$'s
  gimbal-lock singularity) (Part 5.4).
- The problem is a **nonconvex NLP**. Every guarantee the point-mass QP
  offered -- global optimality, uniqueness, predictable solve time,
  initial-guess independence -- is gone. Warm starting becomes a correctness
  concern, not an efficiency one, because the initial guess selects the local
  optimum (Part 6).
- **KKT conditions are now only necessary**, requiring second-order
  conditions to certify even a local minimum (Part 7).
- The Lagrangian Hessian is state-dependent and possibly indefinite;
  **Gauss-Newton** ($\nabla^2f \approx J_r^TJ_r$) sidesteps this, is always
  PSD, and needs only first derivatives (Part 8).
- **SQP beats interior-point for real-time MPC**, principally because
  interior-point methods cannot warm-start effectively from boundary
  solutions (Part 9).
- **Automatic differentiation** in reverse mode computes a full gradient for
  about the cost of one function evaluation; CasADi also handles sparsity and
  C code generation (Part 10).
- **RTI** -- one SQP iteration per tick, split into preparation and feedback
  phases -- is what makes attitude-rate NMPC feasible, cutting effective
  latency to microseconds. `acados` packages this (Part 11).
- Build order: verify dynamics offline → RK4 → IPOPT offline → measure →
  migrate to acados → change the PX4 interface last (Part 12).
