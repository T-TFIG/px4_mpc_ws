# Quadrotor Dynamics via Euler-Lagrange (and why our MPC doesn't use them)

This derives the *full* 6-DOF rigid-body quadrotor dynamics from first principles.
It is **not** what `mpc_solver.py` actually implements -- see the final section for
why, and what we use instead.

The goal is to arrive at $\dot x = f(x, u)$ starting from the Lagrangian
$L(q, \dot q) = T - V$, where $T$ is kinetic energy, $V$ is potential energy, and
$q$ is the vector of generalized coordinates.

*Note on frames:* this section uses the conventional Z-up world frame most
dynamics textbooks use (so $V = mgz$ below is positive altitude). Our actual
implementation uses PX4's NED convention instead (Z-down) -- see the last
section for how that reconciles.

## 1. Generalized coordinates

For a quadrotor, $q = [\xi^T, \eta^T]^T$, where:

- **Position** (world frame): $\xi = [x, y, z]^T$
- **Orientation** (Euler angles): $\eta = [\phi, \theta, \psi]^T$ (roll, pitch, yaw)

So $q = [x, y, z, \phi, \theta, \psi]^T$ -- six generalized coordinates.

## 2. The Euler-Lagrange equation

$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot q}\right) - \frac{\partial L}{\partial q} = Q$$

where $Q$ is the generalized force -- what the four motors actually produce (a
thrust and three torques), mapped into the $q$ coordinates.

## 3. Kinetic energy

$T = T_t + T_r$ (translational + rotational).

### 3.1 Translational

$$T_t = \frac{1}{2} m \dot\xi^T \dot\xi = \frac{1}{2} m(\dot x^2 + \dot y^2 + \dot z^2)$$

### 3.2 Rotational

$$T_r = \frac{1}{2}\,\omega^T J \omega, \qquad J = \mathrm{diag}(J_x, J_y, J_z)$$

**The key subtlety** (the part most people get wrong first try): the Euler angle
rate $\dot\eta$ is **not** the body angular velocity $\omega = [p, q, r]^T$.
$p, q, r$ live in the body frame; $\dot\phi, \dot\theta, \dot\psi$ live in
Euler-angle space -- different coordinate systems entirely. The correct
relation needs a transformation matrix:

$$\omega = W(\eta)\,\dot\eta$$

**Deriving $W(\eta)$:** using the ZYX aerospace convention,
$R = R_z(\psi)\,R_y(\theta)\,R_x(\phi)$, and computing $\hat\omega = R^T\dot R$
gives:

$$\begin{bmatrix}p\\q\\r\end{bmatrix} =
\begin{bmatrix}
1 & 0 & -\sin\theta \\
0 & \cos\phi & \sin\phi\cos\theta \\
0 & -\sin\phi & \cos\phi\cos\theta
\end{bmatrix}
\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix}
= W(\eta)\,\dot\eta$$

Substituting into $T_r$:

$$T_r = \frac{1}{2}\dot\eta^T W^T J W \dot\eta$$

So total kinetic energy:

$$T = \frac{1}{2} m \dot\xi^T \dot\xi + \frac{1}{2}\dot\eta^T W^T J W \dot\eta$$

## 4. Potential energy

$$V = mgz$$

## 5. The Lagrangian

$$L = T - V = \frac{1}{2} m \dot\xi^T \dot\xi + \frac{1}{2}\dot\eta^T W^T J W \dot\eta - mgz$$

## 6. Equations of motion

### 6.1 Translational

$T_t$ depends only on $\dot\xi$, and $V$ depends only on $z$, so:

$$\frac{\partial L}{\partial \dot\xi} = m\dot\xi \quad\Rightarrow\quad
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot\xi}\right) = m\ddot\xi,
\qquad
\frac{\partial L}{\partial \xi} = \begin{bmatrix}0\\0\\-mg\end{bmatrix}$$

Giving:

$$m\ddot\xi = Q_\xi - \begin{bmatrix}0\\0\\mg\end{bmatrix}$$

where $Q_\xi = R\,[0, 0, T]^T$ -- the single thrust $T$ acts along the body
z-axis and gets rotated into the world frame by $R$. Written out:

$$m\ddot x = T(\sin\phi\sin\psi + \cos\phi\sin\theta\cos\psi)$$
$$m\ddot y = T(-\sin\phi\cos\psi + \cos\phi\sin\theta\sin\psi)$$
$$m\ddot z = T\cos\phi\cos\theta - mg$$

This is the standard quadrotor translational model, and it's the one part of
this whole derivation that survives directly into our actual code (see below).

### 6.2 Rotational

This is where it gets ugly. Differentiating $T_r = \frac12\dot\eta^T W^T J W
\dot\eta$ with respect to $\eta$ and $\dot\eta$ produces extra terms *because
$W$ itself depends on $\eta$* -- these are Coriolis/centrifugal-like coupling
terms, exactly analogous to what shows up in manipulator-arm dynamics. The
result has the general form:

$$M(\eta)\,\ddot\eta + C(\eta, \dot\eta)\,\dot\eta = Q_\eta, \qquad M(\eta) = W^TJW$$

**A correction worth being explicit about:** $Q_\eta$ here is the generalized
force *conjugate to $\eta$*, not the real physical torque the motors produce
-- the motors act in the **body frame** ($\tau_{body}$), while our generalized
coordinates are Euler angles. Using the virtual-work principle (real power
delivered must match generalized power: $\tau_{body}\cdot\omega =
Q_\eta\cdot\dot\eta$, and $\omega = W(\eta)\dot\eta$), the correct relation is

$$Q_\eta = W(\eta)^T \tau_{body}$$

so the full rotational equation of motion, in terms of the torque the motors
actually produce, is

$$M(\eta)\,\ddot\eta + C(\eta,\dot\eta)\,\dot\eta = W(\eta)^T \tau_{body}$$

**Deriving $M(\eta)$:** carrying out $W^TJW$ explicitly gives

$$M(\eta) = \begin{bmatrix}
J_x & 0 & -J_x\sin\theta \\
0 & J_y\cos^2\phi + J_z\sin^2\phi & (J_y-J_z)\sin\phi\cos\phi\cos\theta \\
-J_x\sin\theta & (J_y-J_z)\sin\phi\cos\phi\cos\theta & J_x\sin^2\theta + J_y\sin^2\phi\cos^2\theta + J_z\cos^2\phi\cos^2\theta
\end{bmatrix}$$

**Deriving $C(\eta,\dot\eta)$:** rather than differentiate $L$ term-by-term by
hand (extremely easy to drop a sign across dozens of trigonometric
cross-terms), $C$ is computed the standard, systematic way -- via the
Christoffel symbols of $M(\eta)$ (this is exactly how manipulator-arm dynamics
are derived in, e.g., Spong, Hutchinson & Vidyasagar's *Robot Modeling and
Control*):

$$c_{ijk} = \frac{1}{2}\left(\frac{\partial M_{kj}}{\partial \eta_i} + \frac{\partial M_{ki}}{\partial \eta_j} - \frac{\partial M_{ij}}{\partial \eta_k}\right), \qquad
C_{kj}(\eta,\dot\eta) = \sum_{i=1}^{3} c_{ijk}\,\dot\eta_i$$

This was carried out symbolically (see `docs/scripts/derive_dynamics.py`,
which reproduces every result here) rather than by hand, specifically to
avoid transcription errors in dense trig algebra. As a correctness check,
$\dot M(\eta) - 2C(\eta,\dot\eta)$ was confirmed symbolically to be
skew-symmetric -- a well-known necessary property of any correctly-derived
Coriolis matrix. The full result, with $s_\phi=\sin\phi$, $c_\phi=\cos\phi$,
etc.:

$$C(\eta,\dot\eta) = \begin{bmatrix}
0 & \dot\theta(J_y-J_z)s_\phi c_\phi - \frac{\dot\psi c_\theta}{2}\big[J_x+(J_y-J_z)c_{2\phi}\big] &
  -\frac{\dot\theta c_\theta}{2}\big[J_x+(J_y-J_z)c_{2\phi}\big] - \dot\psi(J_y-J_z)s_\phi c_\phi c_\theta^2 \\[4pt]
\frac{\dot\psi c_\theta}{2}\big[J_x+(J_y-J_z)c_{2\phi}\big] - \dot\theta(J_y-J_z)s_\phi c_\phi &
  \dot\phi(J_z-J_y)s_\phi c_\phi &
  \frac{\dot\phi c_\theta}{2}\big[J_x+(J_y-J_z)c_{2\phi}\big] - \dot\psi\big[(J_x-J_z)-(J_y-J_z)s_\phi^2\big]s_\theta c_\theta \\[4pt]
\dot\psi(J_y-J_z)s_\phi c_\phi c_\theta^2 - \frac{\dot\theta c_\theta}{2}\big[J_x-(J_y-J_z)c_{2\phi}\big] &
  -\frac{\dot\phi c_\theta}{2}\big[J_x-(J_y-J_z)c_{2\phi}\big] + \dot\psi\big[(J_x-J_z)-(J_y-J_z)s_\phi^2\big]s_\theta c_\theta - \dot\theta(J_y-J_z)s_\phi c_\phi s_\theta &
  \dot\phi(J_y-J_z)s_\phi c_\phi c_\theta^2 + \dot\theta\big[(J_x-J_z)-(J_y-J_z)s_\phi^2\big]s_\theta c_\theta
\end{bmatrix}$$

where $c_{2\phi} = \cos 2\phi = \cos^2\phi - \sin^2\phi$.

So the full rotational equations of motion are $M(\eta)\ddot\eta +
C(\eta,\dot\eta)\dot\eta = W(\eta)^T\tau_{body}$, with $M$, $C$, and $W$ as
derived above.

**A practical note:** in real implementations, this Euler-angle form is rarely
used directly. It's algebraically correct, but $W(\eta)$ is singular at
$\theta=\pm90°$ (gimbal lock), and $C$'s trig density makes it expensive to
evaluate every control cycle. The far more common choice -- and what **PX4
itself actually uses internally** -- is Newton-Euler in body rates instead:

$$J\dot\omega + \omega \times (J\omega) = \tau$$

algebraically much simpler, with no $C(\eta,\dot\eta)$ coupling term at all,
and (combined with a quaternion attitude representation instead of Euler
angles, which is what PX4's estimator/rate controller actually use) no
gimbal-lock singularity either.

## Connecting this to what we actually built

`mpc_solver.py` uses neither of the above. It models the drone as a **point
mass**: state $[p_x,p_y,p_z,v_x,v_y,v_z]$, input = acceleration, dynamics =
plain double integrator. Every bit of rotational dynamics derived above --
$\phi,\theta,\psi$, $J$, $W(\eta)$, the whole $M\ddot\eta + C\dot\eta = \tau$
mess -- is completely absent from our model.

That's a deliberate simplification, not an oversight, and it's the same
architectural choice we made back when scoping this project: our MPC operates
at the **guidance/trajectory layer**, commanding position/velocity/acceleration
setpoints over Offboard mode, while PX4's own `mc_pos_control` -> attitude
controller -> rate controller cascade (Section 6.2's territory, already solved
and flight-tested inside PX4) handles turning "go this way" into actual body
torques. Section 6.1's translational equation is, not coincidentally, almost
exactly the model our point mass approximates (mass times acceleration equals
the horizontal/vertical force PX4's inner loop is able to produce) -- we're
just trusting PX4 to handle the tilt-angle/torque part of the equation for us.

The honest cost of this simplification: our MPC's acceleration commands are a
*request*, not a guarantee -- if PX4's inner loop can't actually achieve the
tilt needed to produce that acceleration fast enough (motor/attitude bandwidth
limits, which our point mass has no concept of), real tracking will lag behind
what the optimization predicted. That's very likely a contributor to the
tracking jitter/shake observed during circle-tracking testing, alongside the
solver re-solving independently each tick and reacting to raw sensor noise
(see the state-estimation stretch goal).

If this project ever extended to Stage 4 (replacing PX4's inner loop with a
full attitude/rate-level MPC, rather than riding on top of it), Section 6.2's
$M(\eta)\ddot\eta + C(\eta,\dot\eta)\dot\eta = W(\eta)^T\tau_{body}$ form --
or, more likely, its body-rate Newton-Euler equivalent -- is exactly the model
that would need to go into the solver instead of the double integrator.

## What actually changes if we build that Stage-4 MPC

Worth being explicit about the real scope jump, since it's substantially more
than swapping the dynamics function in `mpc_solver.py`:

- **State grows from 6 to 12** (position, velocity, orientation, angular
  velocity), and becomes genuinely nonlinear -- $M(\eta)$ and $C(\eta,\dot\eta)$
  above are configuration-dependent, unlike the point mass's constant,
  linear double integrator.
- **Representation choice matters.** This document derives everything in
  Euler angles because that's the natural Euler-Lagrange route, but
  $W(\eta)$'s gimbal-lock singularity at $\theta=\pm90°$ makes Euler angles a
  real liability in an optimizer that might explore that region mid-solve.
  The practical choice (and what PX4 itself uses) is quaternions + body rates
  with the Newton-Euler form instead -- meaning a second derivation, not a
  reuse of this one, if we want to match PX4's own internal representation.
- **Solve time becomes the binding constraint.** IPOPT re-solving a nonlinear
  program from scratch each tick was already the likely dominant cost in the
  6-state point-mass version; a 12-state nonlinear model over the same
  horizon is a much harder NLP. Real-time nonlinear MPC at attitude-loop rates
  typically means code-generated solvers (e.g. `acados`, which was flagged as
  a future option back when we first scoped this project) rather than
  IPOPT-in-a-Python-loop.
- **The PX4 interface changes entirely.** We'd no longer send
  `TrajectorySetpoint` over a position-mode `OffboardControlMode` -- we'd be
  commanding attitude or body-rate setpoints directly
  (`OffboardControlMode.attitude`/`body_rate = True`, with
  `VehicleAttitudeSetpoint`/`VehicleRatesSetpoint` instead), since we're now
  replacing the inner loop rather than sitting above it.

None of that is a reason not to do it -- just the honest reason it's a
separate project phase, not an afternoon's edit.

## 7. Assembling the full nonlinear state-space model

Sections 6.1 and 6.2 give two second-order equations. An MPC solver needs a
single first-order ODE, $\dot x = f(x, u)$. Define the state as

$$x = \begin{bmatrix}\xi\\\eta\\\dot\xi\\\dot\eta\end{bmatrix} \in \mathbb{R}^{12}
= [\,x,y,z,\ \phi,\theta,\psi,\ \dot x,\dot y,\dot z,\ \dot\phi,\dot\theta,\dot\psi\,]^T$$

Then, solving each second-order equation for its highest derivative:

$$\dot x = f(x, u) = \begin{bmatrix}
\dot\xi \\
\dot\eta \\
\dfrac{1}{m}\Big(R(\eta)\,[0,0,T]^T - [0,0,mg]^T\Big) \\
M(\eta)^{-1}\Big(W(\eta)^T\tau_{body} - C(\eta,\dot\eta)\dot\eta\Big)
\end{bmatrix}$$

The first two rows are trivial (velocity states carry straight through); the
last two are Sections 6.1 and 6.2 solved for $\ddot\xi$ and $\ddot\eta$
respectively. This is a genuinely nonlinear ODE -- $R(\eta)$, $M(\eta)$,
$M(\eta)^{-1}$, and $C(\eta,\dot\eta)$ are all trigonometric functions of the
state itself, unlike the point mass's constant, linear $\dot v = u$.

## 8. What the control input actually is

$T$ and $\tau_{body}$ above aren't the real control input either -- a
quadrotor has exactly four actuators (four rotor thrusts $f_1,f_2,f_3,f_4$),
and $T,\tau_{body}$ are a convenient intermediate quantity produced by all
four together. For a simple "+" configuration (front/back/left/right rotors),
with arm length $l$ and rotor drag-torque coefficient $c$:

$$\begin{bmatrix}T\\\tau_\phi\\\tau_\theta\\\tau_\psi\end{bmatrix} =
\begin{bmatrix}
1 & 1 & 1 & 1\\
0 & l & 0 & -l\\
-l & 0 & l & 0\\
c & -c & c & -c
\end{bmatrix}
\begin{bmatrix}f_1\\f_2\\f_3\\f_4\end{bmatrix}$$

This is the "mixer" or "control allocation" matrix -- PX4 has its own version
of exactly this (rotated 45° for an X-configuration airframe like the x500 we
fly), implemented in its `control_allocator` module. So the *true* control
input for a real-dynamics MPC is $u = [f_1,f_2,f_3,f_4]$, each individually
bounded ($0 \le f_i \le f_{max}$ -- a rotor can only push, never pull, and has
a maximum RPM) -- constraints that don't have a clean equivalent in $T,
\tau_{body}$ space, which is why serious implementations formulate the NLP
directly in terms of $f_i$.

## 9. The actual nonlinear MPC problem

With $\dot x = f(x,u)$ and $u=[f_1,f_2,f_3,f_4]$ in hand, the NMPC problem
(direct multiple-shooting, the standard formulation) is:

$$\min_{x_{0:N},\,u_{0:N-1}} \sum_{k=0}^{N-1}\Big[(x_k - x_k^{ref})^T Q (x_k - x_k^{ref}) + (u_k - u_{hover})^T R\, u_k\Big] + (x_N-x_N^{ref})^T Q_f (x_N-x_N^{ref})$$

subject to:

- $x_0 = x(t)$ -- the current measured/estimated state
- $x_{k+1} = \text{RK4}(x_k, u_k, \Delta t)$ for $k=0,\dots,N-1$ -- the
  nonlinear dynamics above, integrated with a proper Runge-Kutta step (forward
  Euler, good enough for the point mass, is generally too inaccurate for the
  faster rotational dynamics)
- $0 \le f_i \le f_{max}$ for each rotor -- actuator limits
- optionally, state bounds like $|\phi|,|\theta| \le \theta_{max}$ (cap tilt
  angle) and/or obstacle constraints, same idea as discussed earlier for the
  point-mass version

**The qualitative difference from our point-mass MPC**, concretely: in
`mpc_solver.py`, the solver directly chooses an acceleration and trusts it's
achievable. Here, the solver must reason about the *entire causal chain*
itself -- to accelerate horizontally, it first has to command rotor thrusts
that produce a torque, that (through $M(\eta)^{-1}$ and $C(\eta,\dot\eta)$)
produces an angular acceleration, that changes $\eta$, that (through $R(\eta)$
in the translational equation) finally redirects the thrust vector to produce
the acceleration it wants. That whole chain is exactly what our point mass
abstracted away and simply assumed PX4 would handle -- a real-dynamics MPC
has to solve it explicitly, every horizon step, which is precisely why it's a
much harder NLP and why solve time becomes the binding real-time constraint.
