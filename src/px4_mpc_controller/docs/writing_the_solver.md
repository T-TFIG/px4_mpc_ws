# Writing the Solver Yourself

A build order for `solvers/qp.py` and `solvers/sqp.py`, with the algebra you
need at each step and a test that has to pass before you move on.

The theory is in
[`MPC_explanation_my_version.md`](MPC_explanation_my_version.md) Part V (KKT,
multipliers, the SQP subproblem), [`sqp_details.md`](sqp_details.md) (which
Hessian, factorization, the active set), and
[`MPC_solver.md`](MPC_solver.md) Parts 7-9 (Newton methods, SQP,
interior-point). This document is only the implementation path.

**Write them in this order.** Each stage is testable on its own, and every
stage after the first is debuggable *only* because the earlier ones are known
good.

| # | Function | Depends on |
|---|---|---|
| 1 | `qp.kkt_residuals` | nothing |
| 2 | `qp._solve_augmented`, `qp._max_step` | nothing |
| 3 | `qp.solve_qp`, equality-only path | 1, 2 |
| 4 | `qp.solve_qp`, full interior-point loop | 3 |
| 5 | the MPC problem builder (H, g, A, b, C, d) | 4 |
| 6 | `sqp` derivative helpers + `nlp_kkt_residuals` | 1 |
| 7 | `sqp.merit_function`, `merit_penalty` | 6 |
| 8 | `sqp.solve_sqp` | 4, 7 |

---

## Stage 1: `kkt_residuals`

Write this first, before any solver. It is the grader, and a grader that
shares code with the thing it grades is worthless — so keep it literal and
slow. Transcribe the four conditions and nothing else.

```
stationarity     = H z + g + A^T lam + C^T nu
primal_eq        = A z - b
primal_ineq      = max(C z - d, 0)     <- one-sided: satisfied constraints score 0
dual             = min(nu, 0)          <- one-sided: nu >= 0 is what we want
complementarity  = nu * (C z - d)      <- elementwise product
```

Return the infinity norm of each in a dict. Two traps:

- **The one-sided ones.** `C z - d` being very negative is a constraint that
  is comfortably satisfied — that is good, and must score zero. If you take
  `|C z - d|` you will "fail" every inactive constraint.
- **Empty constraint blocks.** Normalize `A=None` to a `(0, n)` array early so
  `A @ z` and `A.T @ lam` still work and return empty/zero. Special-casing
  `None` at every use site is where bugs hide.

**Test:** hand-solve `min ½‖z‖²` s.t. `z₁ + z₂ = 1`. Stationarity gives
`z = -λ[1,1]`, feasibility gives `-2λ = 1`, so `z = (0.5, 0.5)`, `λ = -0.5`.
Feed that in and every residual must be ~1e-16. Then feed in `z = (0.6, 0.4)`
with the same `λ` and watch only `stationarity` go nonzero.

---

## Stage 2: the two numerical helpers

**`_solve_augmented(M, A, rhs_z, rhs_eq)`** assembles and solves

$$\begin{bmatrix} M & A^T \\ A & 0\end{bmatrix}\begin{bmatrix}\Delta z\\ \Delta\lambda\end{bmatrix} = \begin{bmatrix}\text{rhs}_z\\ \text{rhs}_{eq}\end{bmatrix}$$

Use `np.linalg.solve`, **not** Cholesky. Even with `M` positive definite, that
zero block makes the assembled matrix *indefinite*, and the zero block is
there because equality multipliers are sign-free. Handle `m_eq == 0` by
falling through to `np.linalg.solve(M, rhs_z)` and returning an empty `lam`.

**`_max_step(x, dx, frac)`** — only components with `dx < 0` can drive `x`
toward zero, so:

```
blocking = dx < 0
if none:  return 1.0
return min(1.0, frac * min(-x[blocking] / dx[blocking]))
```

Test it directly: `_max_step([1,1], [-2,0], 0.995)` → `0.4975`.

---

## Stage 3: the equality-only QP

Do this before touching interior-point. With no inequalities there is **no
complementarity to resolve**, so the KKT conditions are just a linear system:

$$\begin{bmatrix} H & A^T \\ A & 0\end{bmatrix}\begin{bmatrix}z\\ \lambda\end{bmatrix} = \begin{bmatrix}-g\\ b\end{bmatrix}$$

One call to `_solve_augmented` and you are done — exactly, not iteratively.
This is the base case the whole method is built on, and it is worth having as
a separate code path both for speed and because if it is broken, nothing
downstream can work.

**Test:** the same hand-solved problem from Stage 1, now solved rather than
checked.

---

## Stage 4: the interior-point loop

Here is the algebra. Slacks `s = d - C z ≥ 0` turn the KKT conditions into

$$Hz + g + A^T\lambda + C^T\nu = 0,\quad Az = b,\quad Cz + s = d,\quad s\circ\nu = \tau\mathbf 1$$

with `τ > 0` the relaxation. Define residuals at the current point:

```
r_d = H z + g + A^T lam + C^T nu       # stationarity
r_p = A z - b                          # equalities
r_s = C z + s - d                      # slack definition
mu  = (s . nu) / m_in                  # average complementarity
tau = sigma * mu                       # target for this step
r_c = s * nu - tau                     # relaxed complementarity
```

Newton on that system is

$$\begin{bmatrix} H & A^T & C^T & 0\\ A&0&0&0\\ C&0&0&I\\ 0&0&S&N\end{bmatrix}\begin{bmatrix}\Delta z\\ \Delta\lambda\\ \Delta\nu\\ \Delta s\end{bmatrix} = -\begin{bmatrix}r_d\\ r_p\\ r_s\\ r_c\end{bmatrix}$$

with `S = diag(s)`, `N = diag(nu)`. Do **not** assemble that. Eliminate:

- Row 3 gives `Δs = -r_s - C Δz`.
- Substituting into row 4, with `Σ = ν/s` elementwise:
  `Δν = Σ(C Δz + r_s) - r_c/s`
- Substituting that into row 1 leaves the reduced system:

$$\begin{bmatrix} H + C^T\Sigma C & A^T\\ A & 0\end{bmatrix}\begin{bmatrix}\Delta z\\ \Delta\lambda\end{bmatrix} = \begin{bmatrix}-r_d - C^T\Sigma r_s + C^T(r_c/s)\\ -r_p\end{bmatrix}$$

which is `_solve_augmented` again, with `M = H + CᵀΣC`. In numpy,
`C.T @ (Sigma[:, None] * C)` — do not build `diag(Sigma)`.

`Σ = ν/s` is worth staring at. It is the barrier stiffness: as a constraint's
slack approaches zero, its entry blows up and dominates `M`. That is exactly
how an active constraint makes itself felt — it stiffens the Hessian in its
own direction. The active set is never chosen; it *emerges*.

**Loop body:**

1. Form residuals, call `kkt_residuals`, stop if under `tol`.
2. Build and solve the reduced system.
3. Recover `Δs`, `Δν`.
4. `alpha_p = _max_step(s, ds, frac)`, `alpha_d = _max_step(nu, dnu, frac)`.
5. Step `(z, s)` by `alpha_p` and `(lam, nu)` by `alpha_d`.

**Details that will bite you:**

- **Separate primal and dual step lengths.** Standard, and noticeably faster
  than one shared `alpha`.
- **Initialization must be strictly interior, not feasible.** `s = max(d - Cz, 1)`,
  `nu = 1`, `lam = 0`. `s` is deliberately *not* `d - Cz` — the initial `z` is
  generally infeasible, and the iteration drives `r_s → 0` on its own. Trying
  to start feasible is the Phase-1 problem you chose interior-point to avoid.
- **Regularize `H`.** Add `reg * I` (1e-9). Your MPC Hessian is positive
  *semi*definite — terminal velocity is unpenalized, so `H` has a null space.
- **`sigma = 0.1`** is a fine fixed choice; expect 15-40 iterations to 1e-8.
  Mehrotra's predictor-corrector roughly halves that and is the standard
  upgrade once this works.
- **Stall detection.** If both step lengths collapse below ~1e-12, bail with a
  status rather than spinning to `max_iter`.

**Tests, in order:**

| Problem | Expected |
|---|---|
| `min ½‖z‖²` s.t. `-z₁ ≤ -1` | `z = (1,0)`, `ν = 1` (active) |
| `min ½‖z‖²` s.t. `-z₁ ≤ 1` | `z = (0,0)`, `ν = 0` (inactive) |
| both constraints at once | check `active_set()` picks out only the tight one |

The active/inactive pair is the important one: it is complementary slackness
working. In the first, `s = 0` and `ν > 0`; in the second, `s > 0` and
`ν → 0`. One of the two is always zero, and you never told it which.

---

## Stage 5: building the MPC as a QP

Now connect it to your controller. Stack the decision variables

$$z = [x_0,\ x_1,\ \dots,\ x_N,\ u_0,\ \dots,\ u_{N-1}]$$

so `n = 6(N+1) + 3N` = 186 for `N = 20`. Write two index helpers
(`_ix(k)`, `_iu(k)`) and use them everywhere — every bug at this stage is an
indexing bug.

**Cost.** For a term `w‖y - r‖²` where `y` is a slice of `z`, matching
`½zᵀHz + gᵀz` means putting `2w` on those diagonal entries of `H` and `-2wr`
in `g`. (Check: `½(2w)y² + (-2wr)y = w(y-r)² - wr²`, and the constant does not
move the minimizer.) `H` is diagonal for this problem — build it as a vector.

**Equality constraints** are the initial condition plus the dynamics:

```
x_0 = x_measured                                  ->  6 rows,   b = x_measured
x_{k+1} - A_d x_k - B_d u_k = 0,  k = 0..N-1      ->  6N rows,  b = 0
```

with the exact zero-order-hold double integrator you already use:

$$A_d = \begin{bmatrix} I_3 & dt\,I_3\\ 0 & I_3\end{bmatrix},\qquad B_d = \begin{bmatrix}\tfrac12 dt^2 I_3\\ dt\,I_3\end{bmatrix}$$

**Inequality constraints** are the velocity and acceleration bounds, two rows
each (`+y ≤ ub` and `-y ≤ -lb`). Mirror `mpc_solver.py` exactly — it bounds
`v_k` and `u_k` for `k = 0..N-1` — which gives `12N = 240` rows for `N = 20`.

`H`, `A`, `C`, `d` are all **constant**; only `g` (the reference) and the
first six entries of `b` (the measurement) change per solve. Build the
structure once in `__init__`, as `mpc_solver.py` already does with CasADi.

**Test:** solve the same problem with your solver and with the existing
CasADi/IPOPT `MpcSolver`, from an identical state and reference. `z` should
agree to ~1e-6. Do it from a state where the acceleration bound is
saturated (start far off the circle) as well as an interior one — an
unconstrained agreement proves much less.

### A timing result you should know before you build this

Measured in the container, at `N = 20` (so `n = 186`, `m_eq = 126`,
`m_in = 240`, giving a `312 x 312` reduced KKT matrix), one interior-point
iteration in numpy costs:

| | per iteration | 30 iterations |
|---|---|---|
| form `C^T Σ C` | 4.1 ms | |
| dense LU on the 312×312 KKT matrix | 4.2 ms | |
| **total, sparse/multiple-shooting form** | **9.1 ms** | **273 ms** |
| **total, condensed form (60 variables)** | **0.41 ms** | **12 ms** |

The 10 Hz control loop has a 100 ms budget, so the straightforward
multiple-shooting form **does not fit** and the condensed form fits with 8x
margin. Condensing means eliminating the states: given `x_0` and the dynamics,
the whole trajectory is a linear function of `U`, so the only real decision
variables are the `3N = 60` controls. `MPC_solver.md` Part 11.3 covers it.

Three things follow:

1. **Build Stages 1-4 on the sparse form anyway.** It is far easier to get
   right and to debug, the equality multipliers are visible (they are the
   costates), and it is the reference implementation you will check the fast
   one against. Just do not expect to fly it.
2. **Then condense.** A 22x speedup from changing the formulation, in the same
   language. For comparison, rewriting the *same* dense algorithm in C++ would
   buy roughly 2x — see the discussion below on language choice.
3. **The third option, if you go nonlinear.** Condensing makes the Hessian
   dense and conditions badly over long horizons, and `MPC_solver.md` Part 3.3
   argues multiple shooting wins for nonlinear problems. The way out is to
   keep the sparse form but exploit its block-banded structure with a
   Riccati-style factorization: `O(N · nx³)` instead of `O((N·nx)³)`. At
   `N = 20, nx = 6` that is ~4×10³ flops against ~10⁷. This is what `acados`
   does.

> While you are here: note that bounding `v_0` is what makes the whole problem
> infeasible if the *measured* velocity ever exceeds `max_xy_vel`. It is the
> known limitation in the README. Since you are writing the solver, you now
> also own the fix — soft constraints with slack variables and an L1 penalty in
> the cost, which is the standard answer and cannot go infeasible.

---

## Stage 6-7: the SQP support pieces

**`numerical_jacobian` / `numerical_hessian`** — central differences,
`(f(z + εe) - f(z - εe)) / 2ε`. Central costs 2n evaluations but is O(ε²);
forward differences are O(ε) and will visibly cap how far your SQP
convergence test can be driven, which is a confusing thing to debug. These
exist so you can define a problem before writing analytic derivatives, and to
validate the analytic ones once you do.

**`nlp_kkt_residuals`** is `kkt_residuals` with the constant matrices replaced
by evaluated Jacobians. That is the whole difference, and it is the point:
KKT does not care whether the problem is linear.

**`merit_function`** — `φ(z; μ) = f(z) + μ·θ(z)` where
`θ(z) = ‖c_eq‖₁ + ‖max(c_ineq, 0)‖₁`. It exists because the objective alone
cannot judge a step: lowering `f` while breaking constraints is not progress.
`μ` is the exchange rate.

**`merit_penalty`** — for a step `p` that solves the QP subproblem, the
directional derivative of the L1 merit is exactly

$$D(\varphi; p) = \nabla f^T p - \mu\,\theta$$

so descent needs `μ > ∇fᵀp / θ`. Use the standard rule with curvature margin,

```
mu_required = (grad_f @ p + 0.5 * p @ B @ p) / ((1 - rho) * theta)    # rho = 0.5
mu = max(mu_prev, mu_required)
```

and **never decrease `μ` within a solve** — a ratcheting-down penalty can
cycle. If `θ == 0` you are already feasible; leave `μ` alone.

---

## Stage 8: `solve_sqp`

```
for iteration in range(max_iter):
    evaluate f, grad_f, c_eq, c_ineq, J_eq, J_ineq, and B = grad^2 L(z, lam, nu)
    if max(nlp_kkt_residuals(...)) < tol: converged
    p, lam_new, nu_new = solve_qp(
        H = B + hess_reg*I,     g = grad_f,
        A = J_eq,               b = -c_eq,
        C = J_ineq,             d = -c_ineq,
    )
    mu = merit_penalty(...)
    alpha = 1.0
    while merit(z + alpha*p) > merit(z) + armijo * alpha * D:
        alpha *= 0.5
    z += alpha * p;  lam, nu = lam_new, nu_new
```

Note the QP right-hand sides: `b = -c_eq` and `d = -c_ineq`, because the
subproblem constrains the *step* (`J p = -c`, i.e. "linearly, this step
removes the current violation"). Getting the sign wrong here produces a
solver that moves confidently in the wrong direction, which looks like a
line-search failure and is not.

**The two things that separate a working SQP from a toy:**

1. **`B` is the Hessian of the Lagrangian, not of `f`.** The QP already
   models the constraints as flat, so their curvature has to be carried
   somewhere, and the multiplier-weighted constraint Hessians are where. Using
   `∇²f` alone gives a method that crawls on curved constraints. If `f` is a
   sum of squares, the Gauss-Newton approximation is a legitimate and better
   behaved choice — it is positive semidefinite by construction, so the QP
   subproblem stays convex and `solve_qp`'s assumptions hold.
2. **The QP subproblem can be infeasible** even when the NLP is not —
   linearized constraints can contradict each other. Detect it via
   `solve_qp`'s status and report it. The real fix is elastic mode (relax the
   linearized constraints with penalized slacks), which is worth knowing
   exists before you need it.

**Test 1 — the one that must give exactly one iteration.** Feed `solve_sqp`
your point-mass MPC. The dynamics are linear and the cost quadratic, so the
linearization is *exact*: the first QP subproblem **is** the original problem.
It must converge in **one iteration** with `alpha = 1`. Two iterations means a
bug — most likely in the multiplier handoff or the constraint signs.

**Test 2 — a genuinely nonlinear problem.** Hock-Schittkowski 71, the standard
IPOPT tutorial problem:

```
min  x1*x4*(x1 + x2 + x3) + x3
s.t. x1*x2*x3*x4 >= 25
     x1^2 + x2^2 + x3^2 + x4^2 = 40
     1 <= xi <= 5,     x0 = (1, 5, 5, 1)
```

with known solution `x* = (1.0, 4.7430, 3.8211, 1.3794)`, `f* = 17.0140`.
Define it with finite-difference derivatives first. Expect convergence in
roughly 6-10 iterations; if it takes 30, check that `B` is the Lagrangian
Hessian and not `∇²f`.

---

## Where this leaves you

With Stages 1-5 done you can delete the CasADi dependency from the flight
path — `solve_qp` is a complete replacement for what IPOPT is doing, and a
faster one, because IPOPT is a general nonlinear solver being pointed at a
convex QP (README, Known limitations).

Stages 6-8 buy you nothing on the point-mass model. They are what you need for
the 12-state nonlinear MPC that `MPC_explanation_my_version.md` and
`MPC_solver.md` derive but deliberately do not implement.
