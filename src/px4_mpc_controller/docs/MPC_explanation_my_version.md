# Quadrotor Dynamics and MPC, Derived from Newton

## Scope

From here on I want to explain the whole chain -- from the dynamics of the
drone all the way to the solver inside the MPC -- and go deep into every step
rather than quoting results.

I start with the dynamics, and I derive them the **Newtonian** way: draw the
forces, draw the torques, apply $F = ma$ and its rotational twin, and
assemble. This is a deliberate choice. The other route to the same equations
is Euler-Lagrange -- write down kinetic and potential energy, turn the crank
-- which is more mechanical and harder to get wrong. Newton is the opposite
trade: you have to be careful about what acts on what and in which frame, but
nothing is hidden behind a recipe. Every term that shows up in the final
model shows up here because I put a physical arrow on the drone and followed
it.

I derived it both ways before settling on this one, and both routes land on
the same $\dot x = f(x,u)$. Deriving it twice, by two independent methods,
was the cheapest correctness check available.

### Roadmap

| part | what it produces |
|---|---|
| **I -- Dynamics** | the continuous nonlinear model $\dot x = f_c(x,u)$ |
| **II -- Discretization** | the same model as $x_{k+1} = f_d(x_k,u_k)$ via RK4, which is what an optimizer can actually hold |
| **III -- The cost function** | a single number $J$ that ranks one predicted trajectory against another |
| **IV -- Constraints** | the rules a plan must obey, separating "better" from "allowed" |
| **V -- The solver** | how the resulting problem is actually solved, fast enough to fly on |
| **VI -- The receding horizon loop** | what turns a solved optimization problem into a feedback controller |

Each part ends where the next one begins, so the chain is unbroken.

---

# Part I -- Dynamics

*Goal: start from forces and torques, end with a single function
$\dot x = f_c(x,u)$ that says how the drone moves.*

---

### Step 1: The state variable

The state is everything I would need to know at one instant to predict the
future. For a rigid body in three dimensions that is four blocks:

$$x = \begin{bmatrix}\xi \\ v \\ \eta \\ \omega\end{bmatrix}$$

**Position** -- where the center of mass is, in the world frame:

$$\xi = \begin{bmatrix}x \\ y \\ z\end{bmatrix}$$

**Velocity** -- how fast it is moving, also in the world frame:

$$v = \begin{bmatrix}v_x \\ v_y \\ v_z\end{bmatrix}$$

**Orientation** -- which way it is pointing. I use **Euler angles**: roll
$\phi$, pitch $\theta$, yaw $\psi$.

$$\eta = \begin{bmatrix}\phi \\ \theta \\ \psi\end{bmatrix}$$

**Angular velocity** -- how fast it is rotating, measured in the *body*
frame, which is what a gyroscope bolted to the airframe actually reports:

$$\omega = \begin{bmatrix}p \\ q \\ r\end{bmatrix}$$

Stacked out in full:

$$x = \begin{bmatrix}x & y & z & v_x & v_y & v_z & \phi & \theta & \psi & p & q & r\end{bmatrix}^{\!\top}$$

A **12-state model**.

> **A notation warning.** $p$ is doing double duty in the usual literature:
> $p$ for the position vector, and $p$ for the roll rate. I write position as
> $\xi$ throughout to keep them apart. When you see $p$ in this document it
> is always the roll rate.

Note already that the state splits two-by-two: $\xi$ and $\eta$ are
*configuration* (where I am, how I am oriented), while $v$ and $\omega$ are
their *rates*. That structure is why the final model has the shape it does --
two of the four rows will be pure kinematics, and only two will contain any
physics.

---

### Step 2: The control input

A quadrotor has four independent inputs -- one per motor. But raw motor
speeds are an awkward thing to optimize over, because each motor contributes
to *everything* at once. Instead I use the equivalent force and moment, which
is the standard change of variables:

$$u = \begin{bmatrix}T \\ \tau_x \\ \tau_y \\ \tau_z\end{bmatrix}$$

| symbol | meaning |
|---|---|
| $T$ | total thrust, along the body $z$ axis |
| $\tau_x$ | roll torque |
| $\tau_y$ | pitch torque |
| $\tau_z$ | yaw torque |

Four inputs, and they map to the four motor speeds through a fixed, invertible
mixer matrix determined by the airframe geometry. So nothing is lost by
working in $(T, \tau)$ -- it is the same four numbers in a basis where each
one has a clean physical job.

This is also where the quadrotor's fundamental limitation becomes visible.
There are **four inputs but six degrees of freedom** (three translational,
three rotational). The drone is *underactuated*: it cannot produce a sideways
force directly. To move sideways it must first tilt, so that the single
thrust vector acquires a horizontal component. That coupling -- translation
being reachable only *through* rotation -- is the entire reason the
rotational half of this model matters to a position controller at all.

---

### Step 3: Translational dynamics (Newton's second law)

Now the actual physics. Newton's second law, for the center of mass:

$$F = ma \qquad \Longleftrightarrow \qquad m\dot v = F$$

so all I have to do is account for every force acting on the drone. There are
two:

- gravity
- thrust

I am **ignoring aerodynamic drag for now**. That is a real assumption, and it
is the one that fails first: drag grows with the square of airspeed, so the
model degrades as the drone gets faster. At the speeds this controller
targets it is a reasonable omission, and I note where it would enter if I
added it back.

#### Gravity

Gravity is easy because it lives naturally in the world frame and never
changes direction:

$$F_g = \begin{bmatrix}0 \\ 0 \\ -mg\end{bmatrix}$$

#### Thrust

Thrust is the interesting one. The propellers can only push along the drone's
own body $z$ axis -- straight "up" as far as the airframe is concerned. In
the **body frame** that is trivially:

$$F_T^{\mathcal B} = \begin{bmatrix}0 \\ 0 \\ T\end{bmatrix}$$

But Newton's law is a statement about an inertial frame, and the body frame
is not one -- it tumbles with the drone. So the thrust has to be rotated into
the world frame first. Let $R(\phi,\theta,\psi)$ be the rotation matrix from
body frame to world frame. Then:

$$F_T^{\mathcal W} = R(\phi,\theta,\psi)\begin{bmatrix}0 \\ 0 \\ T\end{bmatrix}$$

This single line is where the underactuation from Step 2 becomes concrete.
The magnitude $T$ is a scalar I control directly; the *direction* it points is
set entirely by the attitude. To push the drone north, I cannot ask for a
northward force -- I can only tilt north and let $R$ redirect the thrust I
already have.

#### Total force

Adding the two:

$$F = R(\phi,\theta,\psi)\begin{bmatrix}0 \\ 0 \\ T\end{bmatrix} + \begin{bmatrix}0 \\ 0 \\ -mg\end{bmatrix}$$

$$m\dot v = R(\phi,\theta,\psi)\begin{bmatrix}0 \\ 0 \\ T\end{bmatrix} + \begin{bmatrix}0 \\ 0 \\ -mg\end{bmatrix}$$

and dividing through by the mass:

$$\dot v = \frac{1}{m}R(\phi,\theta,\psi)\begin{bmatrix}0 \\ 0 \\ T\end{bmatrix} + \begin{bmatrix}0 \\ 0 \\ -g\end{bmatrix}$$

This equation tells us how the drone accelerates.

#### What we achieved

Half of the continuous dynamics is already done:

$$\boxed{\;\dot \xi = v\;}$$

$$\boxed{\;\dot v = \frac{1}{m}R(\phi,\theta,\psi)\begin{bmatrix}0 \\ 0 \\ T\end{bmatrix} + \begin{bmatrix}0 \\ 0 \\ -g\end{bmatrix}\;}$$

The first is not physics at all -- it is the definition of velocity, and it
is exact and free. The second is the whole of Newton's second law for this
vehicle. Notice that the only thing standing between the input $T$ and the
acceleration $\dot v$ is the rotation matrix $R$. That is why the next three
steps are entirely about orientation: **the attitude is not a side quest, it
is the steering.**

---

### Step 4: Attitude kinematics

Before touching rotational physics I have to be precise about something that
is easy to get wrong, and that costs you a working controller if you do.

There are **two different things**:

1. **Orientation** -- how the drone is rotated. That is $\eta = [\phi, \theta, \psi]^\top$.
2. **Angular velocity** -- how fast it is rotating. That is $\omega = [p, q, r]^\top$.

The tempting move is to assume they are related by a plain derivative:

$$\omega = \dot\eta$$

**which is not true.** The gyroscope reports rates about the drone's *own*
axes right now. The Euler angles are defined by a *sequence* of rotations
about three *different* axes -- roll about one, pitch about an axis roll has
already moved, yaw about an axis both have moved. So $\dot\phi, \dot\theta,
\dot\psi$ are rates about axes that are not the body axes and are not even
mutually perpendicular. Going between the two needs a change of basis that
depends on the current attitude:

$$\omega = T(\phi,\theta)\,\dot\eta$$

and inverting it, the form I actually need for the state-space model:

$$\dot\eta = E(\phi,\theta)\,\omega, \qquad E = T^{-1}$$

#### The two matrices

$T$ comes from expressing each of the three angle rates in the body frame and
summing them -- angular velocities add as vectors once they share a frame:

$$T(\phi,\theta) = \begin{bmatrix}
1 & 0 & -\sin\theta \\
0 & \cos\phi & \sin\phi\cos\theta \\
0 & -\sin\phi & \cos\phi\cos\theta
\end{bmatrix}$$

and inverting it gives the one the model uses:

$$\boxed{\;E(\phi,\theta) = \begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \dfrac{\sin\phi}{\cos\theta} & \dfrac{\cos\phi}{\cos\theta}
\end{bmatrix}\;}$$

That is the reason the transformation has to happen at all: the gyroscope and
the optimizer are speaking about rotation in two different languages, and $E$
is the translation.

> Both matrices are derived from scratch in
> [`euler_angle_rates.md`](euler_angle_rates.md) -- the three contributions,
> the inversion by hand, and a numerical check that $ET = \mathbb{I}$. Worth
> reading once, because the entries are easy to get subtly wrong and the
> resulting error looks like noise rather than a bug.

> **Where this breaks.** $\det T = \cos\theta$, so $E$ blows up at
> $\theta = \pm 90^\circ$ -- **gimbal lock**, where roll and yaw collapse onto
> the same physical axis. It is a defect of the *coordinates*, not the drone,
> and it is the strongest argument for quaternions. I accept it because the
> singularity sits far outside any attitude this controller will command.
> Discussed properly in
> [`euler_angle_rates.md`](euler_angle_rates.md#gimbal-lock), including why
> $E$ is already ill-conditioned well before it is singular.

---

### Step 5: Rotational dynamics

Now the rotational counterpart of Newton's second law. The naive guess would
be $\tau = I\dot\omega$ -- the exact analogue of $F = ma$ -- but that is
wrong for a rotating body. The correct statement is **Euler's rigid-body
equation**:

$$\tau = I\dot\omega + \omega \times (I\omega)$$

with

$$\tau = \begin{bmatrix}\tau_x \\ \tau_y \\ \tau_z\end{bmatrix},
\qquad
I = \begin{bmatrix}I_x & 0 & 0 \\ 0 & I_y & 0 \\ 0 & 0 & I_z\end{bmatrix}$$

The diagonal inertia matrix is not an approximation for a quadrotor -- the
airframe is symmetric enough that the body axes really are the principal
axes, so the off-diagonal products of inertia genuinely vanish.

#### Where the extra term comes from

$I\omega$ is the **angular momentum**. The extra $\omega \times (I\omega)$
term exists because I am writing this equation in the *body* frame, which is
rotating. Angular momentum is conserved in the world frame; expressed in a
frame that is itself spinning, that conservation picks up a correction. The
physical content is that **rotating bodies influence their own motion** --
spin a body about one axis while it already has momentum about another, and
it will generate torque about the third all by itself, with nothing applied.
That is the gyroscopic effect, and it is why a thrown object can tumble in
ways that look like something is pushing it.

Note it is *quadratic* in $\omega$. At low rotation rates it is negligible;
at aggressive rates it is not. This is the term that makes the rotational
dynamics genuinely nonlinear, and it is the reason a linear attitude
controller degrades when you fly hard.

#### Solving for angular acceleration

$$I\dot\omega = \tau - \omega \times (I\omega)$$

$$\dot\omega = I^{-1}\big(\tau - \omega \times (I\omega)\big)$$

$I$ is diagonal and positive definite, so $I^{-1} = \mathrm{diag}(1/I_x, 1/I_y, 1/I_z)$
always exists -- no numerical care needed.

This equation tells us: **given the applied torques, how does the angular
velocity change?**

---

### Step 6: Put everything together

Four blocks, in state order:

| | equation | what it is |
|---|---|---|
| Position | $\dot\xi = v$ | kinematics -- exact, free |
| Velocity | $\dot v = \frac{1}{m}R(\phi,\theta,\psi)\begin{bmatrix}0\\0\\T\end{bmatrix} + \begin{bmatrix}0\\0\\-g\end{bmatrix}$ | Newton's second law |
| Attitude | $\dot\eta = E(\phi,\theta)\,\omega$ | kinematics -- Step 4 |
| Angular velocity | $\dot\omega = I^{-1}\big(\tau - \omega \times (I\omega)\big)$ | Euler's rigid-body equation |

Two rows of physics, two rows of bookkeeping. The bookkeeping rows are not
optional and not trivial -- $E$ took the longest derivation in the document
-- but they contain no forces. All the force lives in rows two and four.

---

### Step 7: The continuous dynamics function

The state:

$$x = \begin{bmatrix}\xi \\ v \\ \eta \\ \omega\end{bmatrix}$$

The input:

$$u = \begin{bmatrix}T \\ \tau_x \\ \tau_y \\ \tau_z\end{bmatrix}$$

Therefore:

$$\dot x = f_c(x,u) = \begin{bmatrix}
v \\[4pt]
\dfrac{1}{m}R(\phi,\theta,\psi)\begin{bmatrix}0\\0\\T\end{bmatrix} + \begin{bmatrix}0\\0\\-g\end{bmatrix} \\[10pt]
E(\phi,\theta)\,\omega \\[4pt]
I^{-1}\big(\tau - \omega \times (I\omega)\big)
\end{bmatrix}$$

Twelve equations, four inputs, no approximations beyond the two I declared:
**no aerodynamic drag**, and **Euler angles instead of quaternions**.

#### What this is, and what it is not

Notice what makes this system hard to control, now visible in one place: the
input $T$ enters the velocity row *only through* $R(\phi,\theta,\psi)$, and
$\phi,\theta,\psi$ are two integrations away from the torques. To move
sideways I must torque, which changes $\omega$, which changes $\eta$, which
rotates $R$, which finally redirects the thrust. **Four inputs, twelve
states, and the path from input to position runs through the entire chain.**
That is what the controller has to plan through.

> **Corrections to the draft.** Three arithmetic slips in the Step 4
> derivation -- in $\omega_{\text{yaw}}$, in entry $(3,2)$ of $E$, and a sign
> in $p$ -- are recorded in
> [`euler_angle_rates.md`](euler_angle_rates.md#corrections-to-the-original-draft).

### End of Part I

What Part I produced:

$$\dot x = f_c(x,u), \qquad x \in \mathbb{R}^{12},\ u \in \mathbb{R}^{4}$$

a continuous-time, nonlinear description of the drone, with two declared
approximations (no aerodynamic drag, Euler angles rather than quaternions).

**Why that is not yet usable.** $f_c$ is a *differential equation*. An
optimizer cannot hold one -- it works on a finite list of decision variables
and a finite list of algebraic constraints. Before this model can appear
inside a cost function it has to be turned into a **difference equation**

$$x_{k+1} = f_d(x_k, u_k)$$

for a fixed step $\Delta t$: a rule that maps one sample to the next, which
*can* be written as a constraint row. That conversion is Part II.

Two things carry forward and are worth naming now, because they are what
Part II has to respect:

- $f_c$ is **nonlinear**, so $f_d$ will be too. The equality constraint
  $x_{k+1} - f_d(x_k,u_k) = 0$ is therefore a *curved* surface, which is
  precisely why the solver has to be an SQP that repeatedly linearizes rather
  than a single QP solve -- exactly the picture in Part 5 of
  [`optimization_visualized.md`](optimization_visualized.md).
- The four rows have very different character. Two are exact kinematics, two
  carry the physics. Discretization will not treat them equally well, and the
  choice of integrator is where that shows up.

---

# Part II -- Discretization

*Goal: turn $\dot x = f_c(x,u)$ into $x_{k+1} = f_d(x_k,u_k)$ -- a rule the
optimizer can hold as a constraint.*

The reason I have to discretize comes straight out of the MPC methodology:
the controller works by **simulating a horizon**. It asks where the drone
will be one $\Delta t$ from now, and the step after that, $N$ times over,
before it commits to anything. A horizon is a chain of predictions, and a
chain needs a rule that steps from one link to the next.

Which raises the question: why not ordinary integration -- forward Euler, or
backward Euler? They are one line each.

Because neither is honest about energy. Forward Euler **gains** it: a system
that should oscillate forever instead creeps upward until it diverges.
Backward Euler **loses** it: the same system quietly decays to zero. Over one
step neither error is large. The damage is that the error is *biased* -- it
points the same way every step, so it accumulates along the horizon instead
of averaging out, and the far end of the prediction is systematically wrong.

So here comes the beautiful method: **Runge-Kutta 4th order (RK4)**. Higher
and lower orders exist, but 4th is the sweet spot and the most renowned of
them all -- accurate enough that discretization stops being the limiting
error, cheap enough to run inside a control loop.

> Both claims above are measured rather than asserted, in
> [`why_rk4.md`](why_rk4.md): the exact factor by which each Euler method
> gains or loses energy, and RK4 vs forward Euler on the real 12-state model.
> Skip it if you already believe me.

---

### Step 1: The drone state as a continuous system

Real drone physics arrives as a differential equation -- exactly what Part I
produced:

$$\dot x = f_c(x, u)$$

where $f_c$ is the **continuous** version.

But a computer does not speak continuous, it speaks in samples. Say the MPC
runs every $\Delta t$ seconds -- for example $\Delta t = 0.01\,$s. The
optimizer does not want

$$\dot x = f_c(x,u)$$

it wants

$$x_{k+1} = f_d(x_k, u_k)$$

where $f_d$ is the **discrete** version.

The difference is not cosmetic. $\dot x = f_c(x,u)$ is a statement about an
instantaneous rate; it has no notion of "next", so it cannot be written as a
row in a constraint matrix. $x_{k+1} = f_d(x_k,u_k)$ is an algebraic relation
between two variables the optimizer already holds, so it can be enforced
directly:

$$x_{k+1} - f_d(x_k, u_k) = 0, \qquad k = 0,\dots,N-1$$

That is the entire purpose of Part II.

---

### Step 2: The Runge-Kutta 4th order (RK4)

**RK4 is not the optimizer. It is the translator between continuous and
discrete.**

$$\dot x = f_c(x,u) \quad \Longrightarrow \quad x_{k+1} = f_d(x_k, u_k)$$

Given $x_k$ and $u_k$, compute four slope estimates:

$$k_1 = f_c(x_k,\; u_k)$$

then

$$k_2 = f_c\!\left(x_k + \tfrac{\Delta t}{2}k_1,\; u_k\right)$$

then

$$k_3 = f_c\!\left(x_k + \tfrac{\Delta t}{2}k_2,\; u_k\right)$$

then

$$k_4 = f_c\!\left(x_k + \Delta t\, k_3,\; u_k\right)$$

Finally:

$$\boxed{\;x_{k+1} = x_k + \frac{\Delta t}{6}\big(k_1 + 2k_2 + 2k_3 + k_4\big)\;}$$

Each $k_i$ is a slope probed at a different point across the interval:
$k_1$ at the start, $k_2$ and $k_3$ at the midpoint (guess, then corrected),
$k_4$ at the **end**. The result is their weighted average, with the
midpoints getting double weight.

Two details that are easy to get wrong and cost you the method:

- **$k_4$ takes the full step** $\Delta t\,k_3$, not $\Delta t/2$. The
  fourth-order accuracy comes from the stage errors cancelling in the
  weighted sum, and that cancellation needs $k_4$ at the far side of the
  interval. Get it wrong and RK4 silently degrades to *first* order -- still
  flies, still looks plausible, quantified in
  [`why_rk4.md`](why_rk4.md#the-k_4-trap).
- **$u_k$ is the same in all four stages.** The input is held on a zero-order
  hold across the interval, which is what the hardware actually does.

---

### End of Part II

What Part II produced:

$$x_{k+1} = f_d(x_k, u_k)
= x_k + \frac{\Delta t}{6}\big(k_1 + 2k_2 + 2k_3 + k_4\big)$$

an algebraic map from one sample to the next, accurate to $O(\Delta t^4)$,
ready to be written as an optimizer constraint.

The cost is that $f_d$ is now a *composition* of four evaluations of $f_c$.
The solver needs $\partial f_d/\partial x_k$ and $\partial f_d/\partial u_k$
to linearize the constraint at each SQP iteration, and the chain rule has to
run through all four stages -- which is the main argument for letting
automatic differentiation build $f_d$ rather than writing the derivatives by
hand. Worked through in [`why_rk4.md`](why_rk4.md#what-rk4-costs-the-solver).

What is still missing: $f_d$ says how the drone moves, not where it *should*
move or what it is not allowed to do. That is the cost function and the
constraints, next.

> **Correction to the draft.** I first wrote that the risk grows the longer
> the drone flies. It does not, in closed loop -- MPC re-measures $x_0$ every
> cycle, so integration error is discarded and restarted 100 times a second.
> It accumulates over the *horizon*, not over flight time, and the real
> damage is a systematically wrong plan rather than drift. See
> [`why_rk4.md`](why_rk4.md#which-error-actually-accumulates).

---

# Part III -- The cost function

*Goal: turn "fly well" into a single number $J$ that the optimizer can
minimize.*

Traditional MPC mostly splits the cost function into **three parts**, and you
can add more if you want. But first, what a cost function even is: the name
explains itself. It is a *cost* the solver must answer to -- the thing that
tells the computer whether it is doing well or badly. The solver's entire job
is to minimize it. That is the whole purpose of having one.

At the current state of this project I use only the general form that most
MPC implementations have. Things like battery optimization or other
objectives can be added on top later.

Now the mathematics:

$$J(X, U) = \sum_{k=0}^{N-1} (x_k - x_{\text{ref},k})^\top Q\, (x_k - x_{\text{ref},k})
\;+\; \sum_{k=0}^{N-1} (u_k - u_{\text{hover}})^\top R\, (u_k - u_{\text{hover}})
\;+\; (x_N - x_{\text{ref},N})^\top Q_f\, (x_N - x_{\text{ref},N})$$

Three terms: **where you are**, **what it costs to get there**, and **how you
finish**. Each of the steps below builds one of them.

Note what $J$ takes as arguments -- $X$ and $U$, the *entire* horizon, not a
single point:

$$X = \{x_0, x_1, \dots, x_N\}, \qquad U = \{u_0, u_1, \dots, u_{N-1}\}$$

This is where the receding-horizon idea earns its keep. A controller that
scores only the next instant cannot see that braking now is worth it because
of where it leaves you half a second later. Scoring the whole plan is what
lets an action look bad immediately and still win.

> Everything here is a **choice**, and that is the difference between Part III
> and the two parts before it. The dynamics are physics -- getting $f_c$ wrong
> is a bug. There is no correct $Q$ and $R$ waiting to be discovered; there is
> only the trade-off I decide to make, written in a form the solver can chew
> on. This is where the engineering judgement in MPC actually lives.

---

### Step 1: State tracking cost

Suppose the desired position is $x_{\text{ref}}$. At prediction step $k$ the
error is

$$e_k = x_k - x_{\text{ref},k}$$

If the drone is exactly at the reference, $e_k = 0$ -- no penalty. If it is
far away, the penalty should increase. The simplest choice is the squared
Euclidean distance:

$$\lVert e_k \rVert^2$$

But that treats all twelve states as equally important, which they are not --
a metre of position error and a radian of roll error are not the same kind of
mistake, and they are not even in the same units. So introduce a weight
matrix $Q$:

$$e_k^\top Q\, e_k = (x_k - x_{\text{ref},k})^\top Q\, (x_k - x_{\text{ref},k})$$

This is the **state tracking cost**. $Q$ is diagonal in practice, and each
entry is the price of one unit of error in that state. Raising $Q_{zz}$
relative to the rest says "I care about altitude more than anything else",
and the optimizer will trade the others away to protect it.

> Why *squared*, rather than $\lvert e \rvert$ or anything else? It is not
> arbitrary -- the choice is what makes the problem solvable at 100 Hz. See
> [`why_quadratic_cost.md`](why_quadratic_cost.md), which also covers how to
> pick $Q$ entries when the states have different units.

### Step 2: Control effort cost

Suppose there were no penalty on the control input. The optimizer might
discover that full thrust gets to the goal fastest -- and it would be right,
by the only measure it was given.

Physically that could mean:

- wasted battery
- excessive motor wear
- aggressive motion

So penalize large inputs. The right thing to penalize is not $u$ itself but
the *deviation from what it costs to do nothing*. A quadrotor must spend
thrust just to stay in the air, and charging it for that would be charging it
for existing. With $u_{\text{hover}} = [mg,\ 0,\ 0,\ 0]^\top$ the deviation is

$$u_k - u_{\text{hover}}$$

and again a quadratic penalty:

$$(u_k - u_{\text{hover}})^\top R\, (u_k - u_{\text{hover}})$$

where $R$ is another weight matrix.

$Q$ and $R$ are in direct tension, and that tension is the controller's
personality. Large $Q$ relative to $R$ buys aggression -- chase the reference,
spend whatever it takes. Large $R$ relative to $Q$ buys smoothness -- accept
tracking error rather than thrash the motors. Only the **ratio** matters:
scaling both by the same factor scales $J$ and leaves the minimizer exactly
where it was.

### Step 3: Terminal cost

#### The MPC optimization problem

MPC solves this problem at every control step:

$$\min_{X,\,U} \; J$$

where

$$X = \{x_0, x_1, \dots, x_N\}, \qquad U = \{u_0, u_1, \dots, u_{N-1}\}$$

and the cost is

$$J = \sum_{k=0}^{N-1}\Big[(x_k - x_{\text{ref},k})^\top Q\,(x_k - x_{\text{ref},k})
+ (u_k - u_{\text{ref},k})^\top R\,(u_k - u_{\text{ref},k})\Big]
\; + \; V_f(x_N)$$

There are two parts, and they ask different questions:

| part | | the question it asks |
|---|---|---|
| **running cost** | $\displaystyle\sum_{k=0}^{N-1}\ell(x_k, u_k)$ | "During these $N$ steps, how well did I follow the desired behaviour?" |
| **terminal cost** | $V_f(x_N)$ | "After my prediction stops, what kind of state am I leaving the system in?" |

The running cost is Steps 1 and 2, already built. Everything below is about
the second one.

#### Why MPC needs a terminal cost

The key problem: **MPC has limited vision.**

Suppose $N = 10$. The optimizer sees

$$x_0 \to x_1 \to \dots \to x_{10}$$

but the real system continues

$$x_{10} \to x_{11} \to x_{12} \to \dots$$

The optimizer cannot see this, because the horizon ends. So without a
terminal cost,

$$J = \sum_{k=0}^{N-1}\ell(x_k, u_k)$$

the optimizer only cares about what happens *before* $x_N$. It has no
mathematical information about the consequences of ending at $x_N$ -- and
what a solver is not told about, it will not protect.

#### What the terminal cost actually does

It adds a penalty on the final predicted state:

$$V_f(x_N)$$

meaning: *assign a value to the final predicted state.* For tracking, the
natural choice has the same quadratic shape as everything else:

$$\boxed{\;V_f(x_N) = (x_N - x_{\text{ref},N})^\top Q_f\, (x_N - x_{\text{ref},N})\;}$$

Now MPC evaluates

$$\text{trajectory quality} \;+\; \text{final state quality}$$

#### Example: what goes wrong without it

Imagine the drone has a prediction horizon of 5 seconds:

```
current
x0 ---- x1 ---- x2 ---- x3 ---- x4 ---- x5
                                         |
                                       stop
```

Two possible solutions, in a toy state $[\text{position},\ \text{velocity}]^\top$
with the target at position 10:

**Trajectory A** -- smoothly approaches the target:

$$x_5 = \begin{bmatrix}10 \\ 0\end{bmatrix}$$

**Trajectory B** -- rushes toward the target:

$$x_5 = \begin{bmatrix}10 \\ 8\end{bmatrix}$$

Both have the same position. But B has high velocity. Without a terminal
cost, MPC might not care -- the position tracking inside the horizon was
good, and position is all it was asked about.

With a terminal cost,

$$V_f(x_5) = (x_5 - x_{\text{ref}})^\top Q_f\,(x_5 - x_{\text{ref}})$$

Trajectory B receives a larger penalty. Why? Because it ends the prediction
in a state that **requires more correction afterward**. B arrives at the
target already overshooting it; the cost of that overshoot falls outside the
horizon, which is exactly the blind spot $V_f$ exists to cover.

#### Terminal cost is not the final destination

This is the most important misunderstanding, so it is worth stating flatly.
The terminal state

$$x_N \;\ne\; x_{\text{mission end}}$$

It is only

$$x_N = \text{end of this MPC prediction window}$$

Example. The drone's mission lasts 20 minutes. The MPC horizon is $N = 20$
steps. At each update:

```
time 0:
x0 ---------------- x20
                     ^
                     terminal state

time 0.01s:
x1 ---------------- x21
                     ^
                     new terminal state
```

The terminal point **moves every iteration**. It is a sliding window, not a
destination -- which is why $V_f$ is not "the cost of finishing the mission"
but "the cost of everything I cannot see from here". The drone never actually
arrives at $x_N$ in any meaningful sense; by the time it gets there, the
horizon has moved on 100 times.

#### How do we choose $Q_f$?

Two common approaches.

**Method 1: choose $Q_f$ manually.** You decide, based on what you care
about. For a hovering drone in the toy 2-state example:

$$Q_f = \begin{bmatrix}100 & 0 \\ 0 & 50\end{bmatrix}$$

meaning that at the horizon end, position error matters a lot and velocity
error also matters. Simple, and often enough.

**Method 2: use LQR / Riccati.** This is where Riccati appears. For a
linearized system

$$x_{k+1} = A x_k + B u_k$$

solve the infinite-horizon LQR problem. The Riccati equation gives a matrix
$P$ which represents the **future cost after the horizon**. Then set

$$\boxed{\;Q_f = P\;}$$

Meaning: *the terminal cost is chosen to approximate the infinite future.*

#### The MPC interpretation of Riccati's $P$

Do not think "Riccati controls MPC". Think of it as a question and an answer.

MPC asks: *what happens after $x_N$?*

Riccati answers:

$$V_f(x_N) = x_N^\top P x_N$$

So the relationship is:

```
MPC prediction:
x0 ---- x1 ---- x2 ---- ... ---- xN
                                  |
                                  |
                          terminal cost
                                  |
Riccati:  "what is the cost after xN?"
```

$P$ is the entire infinite tail of the trajectory, collapsed into one matrix.
That is a strong claim and it deserves a proof rather than an assertion.

> The reason a single matrix can stand in for an infinite trajectory is
> genuinely deep, and this is the one place in Part III where the choice
> stops being a matter of taste and becomes provable. The proof is the
> infinite-horizon split, then backward induction to the Riccati difference
> equation, and it shows something stronger than "close enough": with
> $Q_f = P$ the finite-horizon problem is *identical* to the infinite-horizon
> one, not merely an approximation of it. Written out in full in the retired
> `MPC_explanation.md` §8.3--8.5, still readable with
> `git show 4a43e21:src/px4_mpc_controller/docs/MPC_explanation.md`.

#### The complete MPC cost

The final form:

$$\boxed{\;J = \sum_{k=0}^{N-1}\Big[(x_k - x_{\text{ref}})^\top Q\,(x_k - x_{\text{ref}})
+ (u_k - u_{\text{ref}})^\top R\,(u_k - u_{\text{ref}})\Big]
+ (x_N - x_{\text{ref}})^\top Q_f\,(x_N - x_{\text{ref}})\;}$$

where

| matrix | controls |
|---|---|
| $Q$ | trajectory tracking |
| $R$ | input effort |
| $Q_f$ | the importance of the horizon's ending state |

> **A note on $u_{\text{ref}}$ versus $u_{\text{hover}}$.** Step 2 wrote the
> input penalty against $u_{\text{hover}}$, and here it is written against the
> general $u_{\text{ref},k}$. They are the same thing: $u_{\text{ref}}$ is the
> general form, and $u_{\text{hover}} = [mg, 0, 0, 0]^\top$ is the specific
> choice for a drone that should default to holding station. For an
> aggressive trajectory you would let $u_{\text{ref},k}$ vary along the
> horizon instead.

---

### End of Part III

What Part III produced: a single scalar $J$ that ranks any complete plan
$(X, U)$ against any other, built from three quadratic terms and two weight
matrices you choose plus one you can derive.

What is still missing: $J$ says which plans are *better*, not which are
**allowed**. Nothing so far stops the optimizer from commanding negative
thrust, tilting past vertical, or flying through the floor -- all of which
score beautifully if they happen to reduce $J$. Those are the constraints,
and they are next.

---

# Part IV -- Constraints

*Goal: state the rules a plan must obey, so that "better" and "allowed"
become two different things.*

A constraint regulates the behaviour of the system. Our system cannot command
the motors beyond their specification, and some parts of it must never be
violated at all -- the drone cannot go below the ground. Now let me define
the constraints. They divide into three kinds.

---

### 1. Dynamic constraint

This makes the system obey its own dynamics, rather than letting the
optimizer adjust the state arbitrarily. It is an **equality** constraint:

$$x_{k+1} = f(x_k, u_k), \qquad k = 0, 1, 2, \dots, N-1$$

### 2. Input constraint

This limits the input to the motors. The input is thrust and torque, and in
the real world the effort available from the system is limited, so it has to
be limited in the plan too:

$$u_{\min} \le u_k \le u_{\max}$$

Converted into inequality form:

$$u_k - u_{\max} \le 0$$
$$u_{\min} - u_k \le 0$$

### 3. State constraint

This limits the system state -- position, velocity, or whatever else must not
be allowed to go where it should not.

A drone cannot go below the ground:

$$z_k \ge 0$$

General form:

$$g(x_k) \le 0$$

---

### The complete MPC optimization problem

**Decision variables:**

$$X = \{x_0, x_1, \dots, x_N\}, \qquad U = \{u_0, u_1, \dots, u_{N-1}\}$$

**Objective:**

$$\min_{X,\,U} \quad
\sum_{k=0}^{N-1}\Big[(x_k - x_{\text{ref},k})^\top Q\,(x_k - x_{\text{ref},k})
+ (u_k - u_{\text{ref},k})^\top R\,(u_k - u_{\text{ref},k})\Big]
+ (x_N - x_{\text{ref},N})^\top Q_f\,(x_N - x_{\text{ref},N})$$

**Subject to:**

$$\begin{aligned}
x_{k+1} &= f_d(x_k, u_k), && k = 0,\dots,N-1 && \text{(dynamics)}\\
u_k - u_{\max} &\le 0, && k = 0,\dots,N-1 && \text{(input upper limit)}\\
u_{\min} - u_k &\le 0, && k = 0,\dots,N-1 && \text{(input lower limit)}\\
g(x_k) &\le 0, && k = 0,\dots,N && \text{(state limits)}
\end{aligned}$$

---

# Part V -- The solver

*Goal: take the problem Part IV stated and actually solve it, by turning one
hard nonlinear problem into a sequence of easy quadratic ones.*

Now that the objective and the constraints are known, we can move on.

---

### Step 1: Connect this to SQP (Sequential Quadratic Programming)

**From the Lagrangian.** Introduce one Lagrange multiplier for each dynamic
constraint -- call them $\lambda_0, \lambda_1, \dots, \lambda_{N-1}$ -- and
one for each inequality:

$$\begin{aligned}
\mathcal{L}(X, U, \lambda, \mu) = J(X,U)
&+ \sum_{k=0}^{N-1} \lambda_k^\top\big(x_{k+1} - f(x_k,u_k)\big) \\
&+ \sum_{k=0}^{N-1} (\mu_k^{\max})^\top (u_k - u_{\max})
 + \sum_{k=0}^{N-1} (\mu_k^{\min})^\top (u_{\min} - u_k) \\
&+ \sum_{k=0}^{N-1} (\mu_k^{g})^\top\, g(x_k)
\end{aligned}$$

Now the KKT conditions on every constraint.

**1. Stationarity.** $\nabla\mathcal{L} = 0$, meaning
$\partial\mathcal{L}/\partial x_k = 0$ for every state and
$\partial\mathcal{L}/\partial u_k = 0$ for every input, stacked into one
vector.

**2. Primal feasibility.**

$$x_{k+1} - f(x_k, u_k) = 0, \qquad
u_k - u_{\max} \le 0, \qquad
u_{\min} - u_k \le 0, \qquad
g(x_k) \le 0$$

**3. Dual feasibility.**

$$\mu_k^{\max} \ge 0, \qquad \mu_k^{\min} \ge 0, \qquad \mu_k^{g} \ge 0$$

**4. Complementary slackness.**

$$\mu_k^{\max}\,(u_k - u_{\max}) = 0, \qquad
\mu_k^{\min}\,(u_{\min} - u_k) = 0, \qquad
\mu_k^{g}\, g(x_k) = 0$$

### Step 2: Build the SQP QP subproblem

Approximate the objective around the current iterate $z^j$:

$$J(z^j + d) \approx J(z^j) + \nabla J(z^j)^\top d + \tfrac{1}{2}\, d^\top B_j\, d$$

$J(z^j)$ is a constant, so drop it -- it shifts every candidate step equally.
The QP objective becomes:

$$\min_d \; \tfrac{1}{2}\, d^\top B_j\, d + \nabla J(z^j)^\top d$$

> **$B_j$ is the Hessian of the *Lagrangian*, not of $J$.**
> $$B_j = \nabla^2 J + \sum_k \lambda_k \nabla^2 c_k + \sum_k \mu_k \nabla^2 g_k$$
> Steps 3 and 4 are about to replace the curved constraints with straight
> lines, and the curvature those lines throw away has to be carried somewhere.
> Why it matters here specifically, and what it costs to get wrong, is in
> [`sqp_details.md`](sqp_details.md#which-hessian).

### Step 3: Linearize the equality constraint

The physical constraint is $x_{k+1} = f(x_k, u_k)$. Move everything to one
side and stack all $N$ of them into one vector:

$$c(z) = \begin{bmatrix}
x_1 - f(x_0, u_0) \\
x_2 - f(x_1, u_1) \\
\vdots \\
x_N - f(x_{N-1}, u_{N-1})
\end{bmatrix}, \qquad c(z) = 0$$

Linearizing, $c(z^j + d) \approx c(z^j) + \nabla c(z^j)\, d$, so the SQP
constraint becomes

$$c(z^j) + \nabla c(z^j)\, d = 0$$

### Step 4: Linearize the inequality constraint

Let $g(z)$ stand for all the inequalities together -- input and state:

$$g(z^j + d) \approx g(z^j) + \nabla g(z^j)\, d \;\le\; 0$$

### Step 5: The complete SQP QP subproblem

$$\min_d \; \tfrac{1}{2}\, d^\top B_j\, d + \nabla J(z^j)^\top d$$

subject to

$$\begin{aligned}
\text{dynamics:} \quad & c(z^j) + \nabla c(z^j)\, d = 0\\
\text{inequalities:} \quad & g(z^j) + \nabla g(z^j)\, d \le 0
\end{aligned}$$

### Step 6: The QP Lagrangian

$$\mathcal{L}_{QP} = \tfrac{1}{2}d^\top B_j d + \nabla J(z^j)^\top d
+ (\lambda^{QP})^\top\big(c(z^j) + \nabla c(z^j)\,d\big)
+ (\mu^{QP})^\top\big(g(z^j) + \nabla g(z^j)\,d\big)$$

### Step 7: KKT conditions for the QP

Now apply KKT again, this time to the QP.

**1. Stationarity**, $\partial\mathcal{L}_{QP}/\partial d = 0$:

$$B_j\, d + \nabla J(z^j) + \nabla c(z^j)^\top \lambda^{QP}
+ \nabla g(z^j)^\top \mu^{QP} = 0$$

**2. Primal feasibility.** $c(z^j) + \nabla c(z^j)d = 0$ and
$g(z^j) + \nabla g(z^j)d \le 0$.

**3. Dual feasibility.** $\mu^{QP} \ge 0$.

**4. Complementary slackness.**
$\mu^{QP}\big(g(z^j) + \nabla g(z^j)\,d\big) = 0$.

### Step 8: Finding the solution of the QP

**Case 1: inactive.** With $\mu^{QP} = 0$, stationarity and the equality
constraint together are one linear system:

$$\begin{bmatrix}
B_j & \nabla c(z^j)^\top \\
\nabla c(z^j) & 0
\end{bmatrix}
\begin{bmatrix} d \\ \lambda^{QP}\end{bmatrix}
= -\begin{bmatrix}\nabla J(z^j) \\ c(z^j)\end{bmatrix}$$

**Case 2: active.** The active inequalities hold as equalities,
$g_{\mathcal A}(z^j) + \nabla g_{\mathcal A}(z^j)\,d = 0$, and join the same
system:

$$\begin{bmatrix}
B_j & \nabla c(z^j)^\top & \nabla g_{\mathcal A}(z^j)^\top \\
\nabla c(z^j) & 0 & 0 \\
\nabla g_{\mathcal A}(z^j) & 0 & 0
\end{bmatrix}
\begin{bmatrix} d \\ \lambda^{QP} \\ \mu^{QP}\end{bmatrix}
= -\begin{bmatrix}\nabla J(z^j) \\ c(z^j) \\ g_{\mathcal A}(z^j)\end{bmatrix}$$

Note the right-hand side is the constraint **values**, not their gradients,
and that both matrices are symmetric with zero diagonal blocks.

> Two things this leaves open: how the system is factorized (symmetric but
> **indefinite**, so LU or $LDL^\top$ -- never Cholesky), and how you know
> which inequalities are in $\mathcal A$ in the first place. The second is the
> whole difficulty: the active set is not known in advance and cannot be
> enumerated. Both in [`sqp_details.md`](sqp_details.md).

---

# Part VI -- The receding horizon loop

*Goal: turn the solved optimization problem into a feedback controller.*

Parts I through V produce an answer to one question: *given where the drone
is right now, what is the best sequence of inputs over the next $N$ steps?*

That is not yet a controller. It is a plan, computed once, for a model that
is wrong. This part is what turns it into a controller, and it comes down to
a decision that looks wasteful and is not: **compute the whole sequence, use
only the first element, and throw the rest away.**

---

### Step 1: The control loop

Every $\Delta t$ seconds:

```
1.  measure          x_0 <- x_measured
2.  solve            the NLP of Part IV, using Part V
                       ->  U* = {u_0*, u_1*, ..., u_{N-1}*}
                           X* = {x_0,  x_1*, ..., x_N*}
3.  apply            u_0*   -- and only u_0*
4.  discard          u_1*, ..., u_{N-1}*
5.  wait             until the next cycle
6.  repeat           from 1
```

Step 1 is the constraint Part IV did not write down:

$$\boxed{\;x_0 = x_{\text{measured}}\;}$$

It looks trivial, and it is the most important line in the loop. Everything
else in this document is open-loop: given $x_0$, the optimizer computes a
sequence assuming the model is exact. There is no feedback anywhere inside
the optimization. **All of the feedback in MPC enters through this one
equality**, once per cycle, when the horizon is re-anchored to where the drone
actually is rather than where the last plan said it would be.

Note also what this makes the horizon do. At cycle $j$ the plan runs from
step $j$ to step $j+N$; at cycle $j+1$ it runs from $j+1$ to $j+N+1$. The far
end moves forward by exactly one step, every cycle, forever. It stays $N$
steps away and is never reached -- it **recedes**, the way the horizon at sea
recedes as you sail toward it.

```
cycle 1:   [====== plan N steps ======]
           ^ apply
cycle 2:    [====== plan N steps ======]
            ^ apply
cycle 3:     [====== plan N steps ======]
             ^ apply
```

### Step 2: Why throw away 97% of the answer

With $N = 30$, steps 3 and 4 above use one input and discard twenty-nine.
That is not a compromise forced by anything -- it is deliberate, and it is
the point.

**Because the plan is only as good as the model.** $f_d$ ignores drag, gets
the mass slightly wrong, and knows nothing about wind. Over one step those
errors are negligible. By step 20 the predicted state and the real state have
visibly parted company. So $u_{19}^*$ was computed for a situation that will
not occur, and executing it would be acting on a stale belief.

$u_0^*$ is the one input in the sequence computed for a state that is
*measured* rather than predicted. It is the only one worth trusting, so it is
the only one used.

**The rest of the sequence still does real work,** even though it is
discarded. It is what makes $u_0^*$ correct in the first place. A controller
that optimized only the next step would brake too late, because braking looks
bad immediately and only pays off later. The other 29 steps exist so that
$u_0^*$ knows what is coming -- they are the lookahead, not the output.

That gives the honest summary of what MPC is:

> **Plan far, commit near, and replan constantly.**

### Step 3: Warm starting

Solving from scratch every cycle would be wasteful for the opposite reason
that discarding the plan is not: consecutive problems are nearly identical.
The horizon has moved one step, and the drone has moved roughly where it was
predicted to.

So the previous solution is an excellent initial guess -- just shifted one
step forward:

$$U_{\text{guess}} = \{u_1^*,\ u_2^*,\ \dots,\ u_{N-1}^*,\ u_{N-1}^*\}$$
$$X_{\text{guess}} = \{x_1^*,\ x_2^*,\ \dots,\ x_N^*,\ f_d(x_N^*, u_{N-1}^*)\}$$

Everything shifts left by one, and the gap at the end is filled by repeating
the last input and rolling the model forward one more step. The multipliers
$\lambda$ and $\mu$ shift the same way.

This matters more than it sounds. SQP converges quadratically **near** the
solution and unreliably far from it, so where you start determines whether
you need 3 iterations or 30. A cold start solves a hard problem; a warm start
solves an easy one. In practice it is the difference between MPC running at
100 Hz and MPC not running at all.

### Step 4: The real-time iteration

There is one more idea, and it is the one that looks wrong at first.

Part V's SQP is a loop: build a QP, solve it, step, check convergence, repeat
until the KKT residual is small. That loop has no bound you can put in a
control budget -- it might take 3 iterations or 15, and a hard deadline
cannot accommodate "might".

The **real-time iteration** scheme resolves this by refusing to converge:

> Do **one** SQP iteration per control cycle. Apply the resulting $u_0$.
> Move on.

The answer is suboptimal, and it does not matter. Because of warm starting,
the single iteration starts from last cycle's answer, which was itself one
iteration past the cycle before. The solver is not converging on a fixed
problem -- it is *tracking* a problem that moves slightly every cycle, and it
stays close enough. Over time the iterates chase the true solution without
ever sitting on it, which is the same bargain the receding horizon already
made with the plan.

What it buys is a solve time that is **constant and known**: exactly one
Jacobian evaluation and one QP solve per cycle, every cycle. For flight code
that predictability is worth more than optimality -- a controller that is
usually fast and occasionally slow misses deadlines, and a missed deadline is
a failsafe.

---

### End of Part VI

The complete controller, in one place:

| part | produces | used by |
|---|---|---|
| I | $\dot x = f_c(x,u)$ | II |
| II | $x_{k+1} = f_d(x_k,u_k)$ via RK4 | IV's dynamics constraint |
| III | the cost $J$ with $Q$, $R$, $Q_f$ | V's objective |
| IV | the full NLP: minimize $J$ subject to dynamics and limits | V |
| V | SQP: solve it by repeated QP | VI's step 2 |
| VI | measure, solve, apply $u_0$, discard, repeat | the drone |

Everything in Parts I--V exists to make one number correct: $u_0^*$, computed
one hundred times a second, from a plan that is thrown away as soon as it is
made.

#### What this document does not cover

Three things a production controller needs that are not derived here, listed
so the gaps are known rather than hidden:

- **Terminal set.** Part III gives the terminal *cost*. Classical stability
  proofs also require a terminal *constraint*, $x_N \in \mathcal{X}_f$ -- an
  invariant set the local controller can hold. Cost alone does not prove the
  closed loop is stable.
- **Infeasibility.** If the drone is already outside a state constraint, the
  problem has no solution and the solver returns nothing -- leaving the
  controller with no output. Real systems soften state constraints with slack
  variables, and never soften input constraints.
- **Where $x_{\text{ref}}$ comes from.** Part III assumes a reference
  trajectory exists. Generating one that is smooth, dynamically feasible, and
  consistent with the horizon is its own problem.

---

# Appendix

## Related documents

Side documents, for readers who want a claim checked rather than taken on
trust. Nothing in them is needed to follow the main line.

| document | what it covers |
|---|---|
| [`euler_angle_rates.md`](euler_angle_rates.md) | Step 4 in full: deriving $T$ and $E$ from scratch, inverting by hand, the numerical check, and gimbal lock |
| [`why_rk4.md`](why_rk4.md) | Part II's claims, measured: Euler's energy bias, RK4 vs Euler on the real model, the $k_4$ trap, the Jacobian cost |
| [`why_quadratic_cost.md`](why_quadratic_cost.md) | Part III: why the cost is squared rather than $\lvert e\rvert$ or $e^4$, and how to pick $Q$ and $R$ |
| [`sqp_details.md`](sqp_details.md) | Part V's open ends: which Hessian $B_j$ is, factorizing the indefinite KKT system, and finding the active set |

Related work elsewhere in the repo:

| document | what it covers |
|---|---|
| [`MPC_solver.md`](MPC_solver.md) | How the resulting optimization is actually solved, fast enough to fly on |
| [`optimization_visualized.md`](optimization_visualized.md) | Lagrange multipliers, KKT, gradient descent, Newton and SQP, drawn |
| [`writing_the_solver.md`](writing_the_solver.md) | Build order for writing the QP/SQP solver by hand |
