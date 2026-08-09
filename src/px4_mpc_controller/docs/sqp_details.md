# SQP: The Details Part V Leaves Open

A companion to Part V of [`MPC_explanation_my_version.md`](MPC_explanation_my_version.md).

Part V lays out the SQP skeleton: build a quadratic model, linearize both
kinds of constraint, write the KKT conditions, solve a linear system. That
skeleton is correct and it is the right thing to have in your head.

This document covers the three places where the skeleton is not yet an
algorithm -- which Hessian $B_j$ actually is, how the KKT system gets
factorized, and how you find the active set when you cannot enumerate it.

Nothing here is needed to follow the main derivation.

---

## Which Hessian

Part V approximates the objective as

$$J(z^j + d) \approx J(z^j) + \nabla J(z^j)^\top d + \tfrac{1}{2} d^\top B_j d$$

and it is natural to read $B_j$ as $\nabla^2 J$. It is not. $B_j$ is the
Hessian of the **Lagrangian**:

$$B_j = \nabla^2_{zz} \mathcal{L}
= \nabla^2 J + \sum_k \lambda_k \nabla^2 c_k + \sum_k \mu_k \nabla^2 g_k$$

### Why the constraint curvature has to be there

The QP is about to treat $c(z) = 0$ as a straight line. But the real
constraint is curved -- $c$ contains $f_d$, the RK4 map from Part II, which
is a composition of four nonlinear evaluations. Linearizing throws that
curvature away, and if nothing carries it, the model is wrong in a way that
compounds.

The clean way to see it: SQP is **Newton's method applied to the KKT
system**, not gradient descent on $J$ subject to constraints. The KKT
conditions are a system of nonlinear equations

$$F(z, \lambda, \mu) = \begin{bmatrix}
\nabla J + \nabla c^\top \lambda + \nabla g^\top \mu \\
c(z) \\
\vdots
\end{bmatrix} = 0$$

and Newton's method on $F = 0$ needs $\nabla F$. Differentiating the first
row with respect to $z$ gives exactly
$\nabla^2 J + \sum\lambda_k\nabla^2 c_k + \sum\mu_k\nabla^2 g_k$ -- the
Lagrangian Hessian, not the objective Hessian. The $\lambda\nabla^2 c$ term
is not a correction bolted on; it is what the derivative *is*.

So the choice is not "should I add constraint curvature". It is "am I doing
Newton on the KKT system, or something else".

### What it costs to get wrong

Using $\nabla^2 J$ alone still converges, and on some problems you would not
notice:

| situation | consequence of using $\nabla^2 J$ |
|---|---|
| linear constraints ($\nabla^2 c = 0$) | none -- the terms are identically zero, the two Hessians are equal |
| $\lambda \approx 0$ (constraints barely binding) | negligible -- the term is scaled by the multiplier |
| nonlinear constraints, $\lambda$ significant | **loses quadratic convergence** -- drops to linear |

The third row is this problem. The dynamics constraint is nonlinear and its
multipliers are large -- $\lambda_k$ is the shadow price of the dynamics at
step $k$, and the dynamics are the constraints that bind hardest. So the term
is neither zero nor small.

The symptom is not failure. It is an SQP that takes 30 iterations where it
should take 5, which at 100 Hz is the difference between making the deadline
and missing it.

### Gauss-Newton: the honest shortcut

Building $\nabla^2 c_k$ for every $k$ is expensive -- it is a third-order
tensor contracted with $\lambda$. A common alternative is **Gauss-Newton**:
for a least-squares objective like the MPC cost, approximate

$$B_j \approx 2\,\mathrm{blockdiag}(Q, R, \dots, Q_f)$$

dropping all constraint curvature deliberately, and accept linear
convergence in exchange for a Hessian that is constant, block-diagonal, known
in advance, and never assembled at runtime. For a tracking problem where the
iterate starts close to the solution -- which is exactly what warm starting
gives you -- this is often the right trade.

The distinction worth holding: Gauss-Newton is a *decision to drop a term you
know is there*. Writing $\nabla^2 J$ because you thought that was the formula
is a bug that happens to resemble a decision.

---

## Factorizing the KKT system

Both systems in Part V Step 8 have the same shape:

$$\begin{bmatrix} B & A^\top \\ A & 0 \end{bmatrix}
\begin{bmatrix} d \\ \lambda \end{bmatrix}
= -\begin{bmatrix} \nabla J \\ c \end{bmatrix}$$

This is a **saddle-point system**, and its structure dictates how it must be
solved.

**It is symmetric.** $B$ is symmetric, and the off-diagonal blocks are
transposes of each other by construction. So you never need to store or
factor the full matrix.

**It is indefinite.** This is the part that catches people. Even when $B$ is
positive definite, the zero block on the diagonal guarantees the whole matrix
has both positive and negative eigenvalues -- take any $v$ with $Av = 0$ and
the quadratic form $[0, v]^\top K [0, v] = 0$, while $[d, 0]$ gives something
positive. A matrix with a zero on the diagonal cannot be positive definite.

The consequence:

$$\textbf{Cholesky will fail.}$$

Not "will be inaccurate" -- will fail, with a negative square root, usually
partway through. Use $LDL^\top$ with symmetric pivoting (Bunch-Kaufman),
which is the factorization designed for symmetric indefinite matrices, or LU
with partial pivoting if you do not want to exploit symmetry. `scipy` exposes
the first as `scipy.linalg.ldl`, and `numpy.linalg.solve` will do the second.

For the correct inertia -- $n$ positive and $m$ negative eigenvalues, which
is the condition for $d$ to be a *minimizer* rather than a saddle -- $B$ must
be positive definite on the null space of $A$. When it is not, the fix is
regularization: solve with $B + \tau I$ for the smallest $\tau$ that
restores it. This is the `hessian_regularization` parameter in
`solvers/sqp.py` (on the `nonlinear-mpc` branch), and the same safeguard as
the eigenvalue shift in `viz_04_newton.py`.

---

## Finding the active set

Part V presents two cases: inactive ($\mu^{QP} = 0$) and active. Both are
correct. What neither says is **how you know which inequalities are in
$\mathcal A$**, and that is the entire difficulty of solving a QP.

### Why you cannot just try them all

With $N = 30$ and 4 inputs, the input bounds alone give $2 \times 30 \times 4
= 240$ inequality rows. Each is either active or not, so the number of
possible active sets is

$$2^{240} \approx 10^{72}$$

Enumeration is not slow, it is impossible. And the active set genuinely
changes between control cycles -- a drone at its thrust limit during a climb
has a completely different active set from one hovering.

There are two families of answer.

### Active-set methods

Guess an active set, solve the equality-constrained system Part V Step 8
Case 2 gives you, then check the answer:

- if some $\mu_i^{QP} < 0$, that constraint is pushing the wrong way -- a wall
  can only push inward -- so **drop it** from $\mathcal A$ and re-solve;
- if the resulting $d$ violates a constraint that was not in $\mathcal A$,
  **add it** and re-solve.

Each iteration changes the active set by one element and is cheap, because
the factorization can be updated rather than rebuilt. The method terminates
finitely and gives an exact solution.

The weakness is the iteration *count*. Starting from a bad guess on a problem
with 240 inequalities, you may need many exchanges, and the count is not
bounded in a way you can put in a real-time budget. Warm starting helps
enormously -- last cycle's active set is usually within a few elements of
this cycle's -- which is why active-set methods are strong in MPC
specifically and unremarkable elsewhere.

### Interior-point methods

Rather than deciding which constraints are active, **refuse to decide**.
Replace complementary slackness

$$\mu_i\, g_i = 0$$

with a relaxed version using slack variables $s_i = -g_i > 0$:

$$s_i\,\mu_i = \tau, \qquad \tau > 0$$

Now every constraint is slightly active. There is no combinatorial choice
left -- the problem is smooth, Newton's method applies directly, and one
factorization per iteration solves it. Then drive $\tau \to 0$ and the
solution converges to the true one, with the active constraints being exactly
those whose $s_i \to 0$.

The active set is *discovered*, not searched for.

| | active-set | interior-point |
|---|---|---|
| iteration count | variable, unbounded in theory | ~10-30, remarkably problem-independent |
| cost per iteration | cheap (factorization update) | one full factorization |
| warm starting | excellent | poor -- a warm start is near the boundary, exactly where the barrier is stiff |
| exact solution | yes, finitely | asymptotic, to tolerance |
| predictable timing | no | **yes** |

That last row is why interior-point tends to win for flight code. A
controller that is usually fast and occasionally slow is worse than one that
is always merely fast, because the deadline is hard and a missed cycle is a
failsafe.

`solvers/qp.py`, on the `nonlinear-mpc` branch, is built around the
interior-point formulation for this
reason -- the $s \circ \mu = \tau$ relaxation, the fraction-to-boundary rule,
and the $\Sigma = \mu/s$ barrier stiffness are all in its docstring, and
[`writing_the_solver.md`](writing_the_solver.md) stages 3--4 give the full
elimination algebra.

---

## Where this connects

| | |
|---|---|
| [`optimization_visualized.md`](optimization_visualized.md) | KKT, the sign of $\nu$, and complementary slackness drawn; Part 5 animates SQP iterating |
| [`writing_the_solver.md`](writing_the_solver.md) | the 8-stage build order, with the Newton-system elimination worked out |
| [`MPC_solver.md`](MPC_solver.md) | condensing, warm starting, and what it takes to hit the deadline |

---

| back to | |
|---|---|
| [`MPC_explanation_my_version.md`](MPC_explanation_my_version.md) | Parts I--V |
| [`euler_angle_rates.md`](euler_angle_rates.md) | Step 4 of Part I, in full |
| [`why_rk4.md`](why_rk4.md) | Part II's claims, measured |
| [`why_quadratic_cost.md`](why_quadratic_cost.md) | why the cost is squared, and picking $Q$ and $R$ |
