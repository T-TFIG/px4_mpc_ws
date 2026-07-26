# From a Point Mass to a Real Quadrotor: Deriving the Dynamics Behind Our MPC

## Why this document exists

Right now, `mpc_solver.py` controls the drone by pretending it's a point
mass -- a dot with position and velocity, pushed around by an acceleration
we get to choose freely. That's a real simplification: a real quadrotor
can't just accelerate sideways, it has to *tilt* first, and tilting takes
time and is itself governed by physics we haven't modeled at all yet.

This document builds the *real* model, one honest step at a time, so that
the next version of our MPC can reason about the actual vehicle instead of
an idealized dot. Nothing here is dropped in as a finished formula -- every
equation is derived from the one before it, with an explanation of *why*
that step happens, so you can always trace an equation back to where it came
from instead of having to take it on faith.

### Roadmap -- where we're going, in one paragraph

We'll describe the drone with six numbers: three for position, three for
orientation. We'll write down its kinetic and potential energy in terms of
those six numbers and their rates of change, because there's a systematic
recipe (Euler-Lagrange) that turns "energy as a function of coordinates"
directly into "equations of motion" -- no need to draw every force by hand.
Applying that recipe will split cleanly into two halves: a straightforward
one for position, and a genuinely messy one for orientation, because the
drone's orientation-related "effective mass" changes as it tilts. We'll
derive that messiness properly (it's called the Coriolis term), and *then*
make a deliberate, justified decision to drop it for our actual controller,
because for the way our drone actually flies, it turns out to be small
enough not to matter. Finally we'll connect the physical torques a real
quadrotor produces to what its four motors actually do, and assemble
everything into the nonlinear optimization problem an MPC needs to solve.

If at any point you feel lost, jump back to this paragraph -- it's the map.

---

## Part 1: Describing the drone with numbers

### 1.1 What are we even trying to describe?

A quadrotor's *state* -- everything you'd need to know to say exactly what
it's doing at one instant -- is two things:

1. **Where its center of mass is**, in the world: three numbers, $x,y,z$.
2. **Which way it's pointing**: three more numbers, some parameterization of
   orientation.

For orientation, we'll use the most common choice for aircraft: **Euler
angles** roll ($\phi$), pitch ($\theta$), and yaw ($\psi$) -- the same three
angles you'd use to describe "tilted this much sideways, tilted this much
forward/back, pointed this compass direction." (We'll see later that this
choice has a real downside -- gimbal lock -- but it's the natural one to
start with, and it's what lets us use the energy-based method below cleanly.)

Put together, that's six numbers describing the drone completely (ignoring,
for now, how fast anything is changing):

$$q = \begin{bmatrix}\xi\\\eta\end{bmatrix}, \qquad
\xi = \begin{bmatrix}x\\y\\z\end{bmatrix} \text{ (position)}, \qquad
\eta = \begin{bmatrix}\phi\\\theta\\\psi\end{bmatrix} \text{ (orientation)}$$

In the language of classical mechanics, $q$ is called the vector of
**generalized coordinates** -- "generalized" just meaning "whatever set of
numbers we've chosen to describe the configuration," as opposed to some
specific physical meaning like Cartesian position alone.

### 1.2 Why bother with energy instead of just F = ma?

You *could* derive the drone's equations of motion by drawing every force
and torque by hand and applying Newton's laws directly. For a single rigid
body that's very doable. The reason we won't do that here is that this
system has an awkward complication baked in: our orientation coordinates
($\phi,\theta,\psi$) are not the same thing as the angular velocity a
gyroscope on the drone would actually measure (we'll make this precise in a
moment) -- and once that distinction exists, force-diagram bookkeeping gets
error-prone fast.

**Lagrangian mechanics** sidesteps this. Instead of forces, you write down
one scalar number -- kinetic energy minus potential energy, called the
Lagrangian $L$ -- purely as a function of your coordinates $q$ and their
rates $\dot q$:

$$L(q,\dot q) = T(q,\dot q) - V(q)$$

Then a fixed, mechanical recipe (the **Euler-Lagrange equation**, introduced
properly in Part 3) turns $L$ directly into the equations of motion. All we
have to get right is the energy -- the recipe handles everything else,
including all the messy coupling terms we'd otherwise have to spot by eye in
a force diagram. That's the whole reason for the detour through energy: it's
mechanical and hard to get wrong, rather than clever and easy to get wrong.

So our job for the rest of Part 1 and Part 2 is just: **write down $T$ and
$V$ correctly.** The recipe in Part 3 does the rest.

---

## Part 2: Kinetic and potential energy

### 2.1 Splitting kinetic energy in two

The drone's kinetic energy has two independent contributions: energy from
moving through space (translational), and energy from spinning (rotational).
They add:

$$T = T_t + T_r$$

### 2.2 Translational kinetic energy -- the easy half

This one is exactly the familiar $\frac12 mv^2$ from introductory physics,
just written for a 3D velocity vector $\dot\xi = [\dot x,\dot y,\dot z]^T$:

$$T_t = \frac{1}{2} m\,\dot\xi^T\dot\xi = \frac{1}{2}m(\dot x^2+\dot y^2+\dot z^2)$$

Nothing subtle here -- moving faster costs more kinetic energy, in any
direction, equally.

### 2.3 Rotational kinetic energy -- and the subtlety that changes everything

For a spinning rigid body, the rotational analog of $\frac12 mv^2$ is

$$T_r = \frac{1}{2}\,\omega^T J\, \omega, \qquad J = \begin{bmatrix}J_x&0&0\\0&J_y&0\\0&0&J_z\end{bmatrix}$$

where $\omega = [p,q,r]^T$ is the **body angular velocity** -- the three
numbers a gyroscope physically bolted to the drone's frame would read off
right now -- and $J$ is the moment-of-inertia matrix (how "hard to spin" the
drone is about each of its own axes).

Here's the subtlety, and it's worth slowing down for because it's the single
most common mistake in this whole derivation: **$\omega$ is not $\dot\eta$.**
It's tempting to write $[p,q,r]^T = [\dot\phi,\dot\theta,\dot\psi]^T$ directly
-- but that's wrong, and here's the intuition for why.

Imagine the drone is pitched fully sideways ($\theta = 90°$), and then you
increase yaw ($\dot\psi > 0$, the compass-direction number is changing).
Because the drone is already tipped over, "changing yaw" from that
orientation doesn't correspond to spinning about the drone's own vertical
axis at all anymore -- it's partially rolling it instead, from the body's
point of view. $\dot\phi,\dot\theta,\dot\psi$ describe how fast each *Euler
angle number* is changing; $p,q,r$ describe how fast the *body itself* is
physically spinning about its own three axes right now. Those coincide only
in special cases, not in general. This is exactly the same distinction
recurring elsewhere in this project: it's the rotational analog of "NED
position isn't the same as ENU position" -- two different, related, but
distinct coordinate systems, and you need an explicit conversion between
them.

That conversion is a matrix, call it $W(\eta)$, satisfying:

$$\omega = W(\eta)\,\dot\eta$$

### 2.4 Deriving $W(\eta)$

We build the drone's orientation from three successive rotations (the
aerospace-standard "ZYX" order: yaw, then pitch, then roll):
$R(\eta) = R_z(\psi)R_y(\theta)R_x(\phi)$. The body angular velocity can be
recovered from this rotation matrix and its time-derivative via
$\hat\omega = R^T\dot R$ (a standard identity relating a rotation matrix's
rate of change to the angular velocity it represents). Carrying that out
gives, after simplification:

$$\begin{bmatrix}p\\q\\r\end{bmatrix} =
\underbrace{\begin{bmatrix}
1 & 0 & -\sin\theta \\
0 & \cos\phi & \sin\phi\cos\theta \\
0 & -\sin\phi & \cos\phi\cos\theta
\end{bmatrix}}_{W(\eta)}
\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix}$$

Sanity-check it against the story above: at $\theta=0,\phi=0$ (drone level),
$W$ reduces to the identity matrix, i.e. $\omega = \dot\eta$ exactly -- which
matches intuition, since at that orientation the two coordinate systems
briefly line up. Away from level flight, they diverge, exactly as the
sideways-yaw example predicted.

### 2.5 Putting rotational kinetic energy in terms of $\eta$

Substituting $\omega = W(\eta)\dot\eta$ into $T_r = \frac12\omega^TJ\omega$:

$$T_r = \frac{1}{2}\dot\eta^T \underbrace{W(\eta)^TJW(\eta)}_{M(\eta)}\dot\eta$$

Call this new combined matrix $M(\eta) = W^TJW$. Working through the matrix
multiplication by hand (it's mechanical, just tedious -- each entry is a sum
$M_{ij}=\sum_k J_k W_{ki}W_{kj}$) gives the explicit result:

$$M(\eta) = \begin{bmatrix}
J_x & 0 & -J_x\sin\theta \\
0 & J_y\cos^2\phi + J_z\sin^2\phi & (J_y-J_z)\sin\phi\cos\phi\cos\theta \\
-J_x\sin\theta & (J_y-J_z)\sin\phi\cos\phi\cos\theta & J_x\sin^2\theta + J_y\sin^2\phi\cos^2\theta + J_z\cos^2\phi\cos^2\theta
\end{bmatrix}$$

Notice something important for later: **$M(\eta)$ depends on the current
orientation** ($\phi,\theta$ appear inside it). Physically: how "hard to
rotate" the drone feels, expressed in terms of Euler-angle coordinates,
changes depending on which way it's currently pointed. That's the seed of
all the complexity in Part 4 -- an ordinary constant mass, like in
$T_t=\frac12 m\dot\xi^T\dot\xi$, doesn't do this; $M(\eta)$ does.

So, total kinetic energy:

$$T = \frac{1}{2}m\dot\xi^T\dot\xi + \frac{1}{2}\dot\eta^TM(\eta)\dot\eta$$

### 2.6 Potential energy

Just gravity, and only position matters (orientation doesn't change height):

$$V = mgz$$

### 2.7 The Lagrangian

$$L = T - V = \frac{1}{2}m\dot\xi^T\dot\xi + \frac{1}{2}\dot\eta^TM(\eta)\dot\eta - mgz$$

We now have the one scalar function the whole rest of the derivation runs
on. Everything from here is mechanical application of the recipe.

---

## Part 3: From energy to equations of motion

### 3.1 The recipe

The Euler-Lagrange equation says: for each coordinate $q_i$ in our vector
$q$, the equation of motion is

$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot q_i}\right) - \frac{\partial L}{\partial q_i} = Q_i$$

**Where $Q_i$ comes from, and why it's needed at all:** $L=T-V$ only
captures energy the system already accounts for on its own -- kinetic
energy, and *conservative* forces like gravity that come from a potential
(that's what $V=mgz$ already is). $L$ has no way to know about forces being
actively applied from outside. $Q_i$ is exactly that missing piece -- the
**generalized force**: whatever external push is doing work on coordinate
$q_i$ that isn't already baked into $V$. For our drone, gravity's already
inside $L$; the only other force is the rotors' thrust, so that's all $Q_i$
needs to represent. We apply this once for the $\xi$ block
(Section 3.2) and once for the $\eta$ block (Section 3.3) -- they turn out
to decouple into two separate, independently-solvable equations, which is
convenient, because the two halves are very different in difficulty.

### 3.2 Translational equation of motion

$L$'s only $\xi$-dependence is through $T_t=\frac12m\dot\xi^T\dot\xi$ (for
velocity) and $V=mgz$ (for position). Plugging into the recipe:

$$\frac{\partial L}{\partial \dot\xi} = m\dot\xi \implies \frac{d}{dt}\Big(\cdot\Big) = m\ddot\xi,
\qquad \frac{\partial L}{\partial \xi} = \begin{bmatrix}0\\0\\-mg\end{bmatrix}$$

$$\implies m\ddot\xi = Q_\xi - \begin{bmatrix}0\\0\\mg\end{bmatrix}$$

**Where $Q_\xi$ comes from:** it's the one force a quadrotor produces
directly -- a single thrust $T$, from the four rotors combined, always
pushing along the drone's *own* body z-axis. In body-frame coordinates
that vector is simply $[0,0,T]^T$ ("straight up," relative to the body,
by definition of the body frame). But $\xi=[x,y,z]$ is a **world-frame**
coordinate, so the generalized force conjugate to it has to be that same
physical thrust, described in world-frame coordinates instead. $R(\eta)$ is
exactly the tool for that -- by definition, it takes a vector expressed in
body-frame components and re-expresses that same physical vector in
world-frame components. So:

$$Q_\xi = R(\eta)\,[0,0,T]^T$$

is nothing more exotic than "take the thrust vector as the body sees it, and
rotate it into how the world sees it." (Contrast this with $Q_\eta$ in
Section 3.5, which needs a genuinely extra derivation step -- $\eta$ is a
set of angles, not a Cartesian position, so there's no single rotation
matrix that directly converts a torque into it the way $R(\eta)$ converts a
force into $Q_\xi$ here.) Written out fully:

$$m\ddot x = T(\sin\phi\sin\psi + \cos\phi\sin\theta\cos\psi)$$
$$m\ddot y = T(-\sin\phi\cos\psi + \cos\phi\sin\theta\sin\psi)$$
$$m\ddot z = T\cos\phi\cos\theta - mg$$

This should match intuition: when level ($\phi=\theta=0$), thrust points
straight up ($m\ddot z = T - mg$, $\ddot x=\ddot y=0$) -- exactly a hovering
drone. Tilt it, and a component of that same thrust redirects horizontally.
**This equation is the one piece of the whole derivation that survives,
almost unchanged, into our actual point-mass MPC** -- our simplified model
essentially trusts PX4 to handle the tilting part, and just uses this
equation's *consequence* (mass times acceleration equals a controllable
force) directly.

### 3.3 Rotational equation of motion -- where it gets hard

$L$'s $\eta$-dependence is only through $T_r=\frac12\dot\eta^TM(\eta)\dot\eta$
($V$ doesn't involve orientation at all). Applying the same recipe:

$$\frac{d}{dt}\left(\frac{\partial L}{\partial\dot\eta}\right) - \frac{\partial L}{\partial\eta} = Q_\eta$$

Here's exactly where the difficulty from Section 2.5 shows up. If $M$ were a
constant matrix, $\frac{\partial L}{\partial\dot\eta}=M\dot\eta$ and its time
derivative would just be $M\ddot\eta$ -- done. But $M$ depends on $\eta$, so
differentiating $\frac12\dot\eta^TM(\eta)\dot\eta$ with respect to $\eta$
picks up *extra terms*, because the "mass" itself is changing as the
coordinate it multiplies changes. Working through both differentiations, the
result takes the general form

$$M(\eta)\ddot\eta + C(\eta,\dot\eta)\dot\eta = Q_\eta$$

$C(\eta,\dot\eta)\dot\eta$ collects exactly those extra terms. This is
directly analogous to the Coriolis and centrifugal terms you may have seen
in rotating-reference-frame problems (a spinning-frame effect), or to the
identical-looking term in robot-arm dynamics -- it's a genuinely general
phenomenon whenever a system's "effective mass" depends on its own
configuration, not something specific to drones.

**Building intuition for what $C$ represents physically:** think of a figure
skater pulling their arms in mid-spin and speeding up without any external
torque -- their moment of inertia changed *because of their own motion*, and
that changing-inertia effect shows up as an apparent extra "force" in the
equations even though no new torque was applied. $C(\eta,\dot\eta)\dot\eta$
is the quadrotor's version of that same bookkeeping: it's what has to be
added to account for $M(\eta)$ itself changing as the drone rotates.

### 3.4 Deriving $C(\eta,\dot\eta)$ properly

You could get $C$ by grinding through
$\frac{d}{dt}\left(\frac{\partial L}{\partial\dot\eta}\right) -
\frac{\partial L}{\partial\eta}$ term by term, but there's a systematic
shortcut used throughout robotics for exactly this situation (any $T=\frac12
\dot q^T M(q)\dot q$): the **Christoffel symbols** of $M$,

$$c_{ijk} = \frac{1}{2}\left(\frac{\partial M_{kj}}{\partial \eta_i} + \frac{\partial M_{ki}}{\partial \eta_j} - \frac{\partial M_{ij}}{\partial \eta_k}\right),
\qquad C_{kj}(\eta,\dot\eta) = \sum_{i=1}^{3} c_{ijk}\,\dot\eta_i$$

This isn't a different, simpler physics -- it's the exact same result as
brute-force differentiating $L$, just organized so you differentiate $M$
(which we already have in closed form) rather than $L$ directly.

Given how many trigonometric cross-terms are involved, this was carried out
**symbolically** rather than by hand (see `docs/scripts/derive_dynamics.py`,
which reproduces every result here exactly) -- specifically to avoid a
transcription error in dense algebra. As a correctness check, a
well-known necessary property of any correctly-derived $C$ -- that $\dot
M(\eta) - 2C(\eta,\dot\eta)$ must be skew-symmetric -- was verified to hold
exactly. The full result, with $s_\phi=\sin\phi,\ c_\phi=\cos\phi$, etc., and
$c_{2\phi}=\cos2\phi$:

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

Notice, as a preview of Part 5's decision: **every single entry is scaled by
either $(J_y-J_z)$ or $(J_x-J_z)$** -- differences between principal
inertias -- and every entry is a *product of two angular rates*. Hold onto
both of those observations; they're exactly the justification for what we do
next.

### 3.5 The real torque isn't quite $Q_\eta$

One more piece before we can call this equation finished. A real quadrotor's
motors produce torque in the **body frame** -- call it $\tau_{body}$ -- but
our recipe produces $Q_\eta$, the generalized force conjugate to *Euler
angles*. These aren't automatically the same thing, for the identical reason
$\omega\neq\dot\eta$ back in Section 2.3: body-frame quantities and
Euler-angle-frame quantities are related, not identical.

The connection comes from the **virtual work principle**: the real physical
power a torque delivers must equal the power computed in whatever
coordinates you're using, i.e. $\tau_{body}\cdot\omega = Q_\eta\cdot\dot\eta$
for every possible $\dot\eta$. Since $\omega=W(\eta)\dot\eta$ (Section 2.3
again), this forces:

$$Q_\eta = W(\eta)^T\tau_{body}$$

So the complete rotational equation of motion, in terms of torque the motors
actually produce, is:

$$M(\eta)\ddot\eta + C(\eta,\dot\eta)\dot\eta = W(\eta)^T\tau_{body}$$

---

## Part 4: A practical note on representation

Before moving on: in real flight-control implementations, this Euler-angle
form is rarely used directly, for two concrete reasons visible in what we
just derived. First, $W(\eta)$ has $\cos\theta$ in denominators once you
invert it to solve for $\dot\eta$ -- at $\theta=\pm90°$ it becomes singular
(**gimbal lock**). Second, $C$'s trigonometric density (Section 3.4) is
expensive to evaluate every control cycle.

The standard alternative -- and what **PX4 itself uses internally** -- is
Newton-Euler dynamics written directly in body rates, with orientation
tracked as a quaternion instead of Euler angles:

$$J\dot\omega + \omega\times(J\omega) = \tau_{body}$$

Algebraically simpler (no $C(\eta,\dot\eta)$ coupling matrix at all), and
quaternions have no orientation at which the representation itself breaks
down. We're deriving the Euler-angle version here because it's the natural
route from Lagrangian mechanics and because gimbal lock isn't a practical
concern for the gentle, non-aerobatic flight this project targets -- but if
this were ever headed toward real hardware or aggressive maneuvers, this is
the representation change that would come first.

---

## Part 5: The decision -- do we actually need $C(\eta,\dot\eta)$?

Recall the two observations flagged at the end of Section 3.4:

1. Every term in $C$ is a **product of two angular rates**
   ($\dot\phi\dot\theta$, $\dot\phi\dot\psi$, $\dot\theta^2$, etc.) --
   $C(\eta,\dot\eta)\dot\eta$ is therefore *quadratic* in how fast the drone
   is rotating. Compare that to $M(\eta)\ddot\eta$, which is linear in
   angular *acceleration* and present at any rotation speed, including zero.
   A quadratic-in-rate term shrinks fast as rates shrink -- and this project
   flies a gentle circle, not an aerobatic routine, so angular rates stay
   small throughout.
2. Every entry is scaled by an inertia *difference*, $(J_y-J_z)$ or
   $(J_x-J_z)$ -- not by the inertias themselves. A quadrotor with a roughly
   symmetric mass distribution (true of most common frames, including the
   x500 we're simulating) has $J_y\approx J_z$, which pushes several terms
   toward zero regardless of speed.

Both effects point the same direction, and for our situation, both apply.
**Decision: drop $C(\eta,\dot\eta)\dot\eta$.** The rotational equation of
motion we'll actually implement is the simplified

$$M(\eta)\ddot\eta = W(\eta)^T\tau_{body}$$

This is a legitimate, standard, and commonly-used simplification for
near-hover multicopter flight -- not a shortcut taken out of laziness. It's
also honestly bounded: it stops being valid for fast, aggressive rotation
(drone racing, aerobatics) or a deliberately asymmetric airframe, where
$C(\eta,\dot\eta)\dot\eta$ would no longer be small. If this project ever
extended toward that kind of flight, Section 3.4's full result is what would
need to go back in -- it isn't wasted work, it's the documented fallback.

---

## Part 6: Assembling the full model

### 6.1 The state-space form

An MPC solver needs a single first-order equation, $\dot x = f(x,u)$, not two
separate second-order ones. Define the state by stacking position,
orientation, and both their rates:

$$x = \begin{bmatrix}\xi\\\eta\\\dot\xi\\\dot\eta\end{bmatrix} \in\mathbb{R}^{12}$$

Then, solving Section 3.2's and Part 5's equations for the highest
derivative in each:

$$\dot x = f(x,u) = \begin{bmatrix}
\dot\xi\\
\dot\eta\\
\dfrac{1}{m}\Big(R(\eta)[0,0,T]^T - [0,0,mg]^T\Big)\\
M(\eta)^{-1}W(\eta)^T\tau_{body}
\end{bmatrix}$$

(the first two rows are trivial -- velocity states just carry through
unchanged). This is a genuinely nonlinear ODE: $R(\eta)$, $M(\eta)$, and
$M(\eta)^{-1}$ are all trigonometric functions of the state itself, unlike
the point mass's constant, linear $\dot v = u$.

### 6.2 What the controls actually are

$T$ and $\tau_{body}$ aren't the real control input either -- they're
produced *together* by the drone's four actual actuators, the individual
rotor thrusts $f_1,f_2,f_3,f_4$. For a "+"-configuration frame with arm
length $l$ and rotor drag-torque coefficient $c$:

$$\begin{bmatrix}T\\\tau_\phi\\\tau_\theta\\\tau_\psi\end{bmatrix} =
\begin{bmatrix}1&1&1&1\\0&l&0&-l\\-l&0&l&0\\c&-c&c&-c\end{bmatrix}
\begin{bmatrix}f_1\\f_2\\f_3\\f_4\end{bmatrix}$$

This is the **mixer** (or "control allocation") matrix -- PX4 has its own
version of exactly this (rotated 45° for our X-configuration x500), in its
`control_allocator` module. The true control input for a real-dynamics MPC
is $u=[f_1,f_2,f_3,f_4]$, each individually bounded ($0\le f_i\le f_{max}$ --
a rotor can push but never pull, and has a maximum RPM) -- a constraint that
doesn't translate cleanly into $T,\tau_{body}$ space, which is why serious
implementations formulate the optimization directly in terms of $f_i$.

---

## Part 7: The actual optimization problem

With $\dot x=f(x,u)$ and $u=[f_1,f_2,f_3,f_4]$ established, the nonlinear MPC
problem (using the standard "direct multiple shooting" formulation) is:

$$\min_{x_{0:N},\,u_{0:N-1}} \sum_{k=0}^{N-1}\Big[(x_k-x_k^{ref})^TQ(x_k-x_k^{ref}) + (u_k-u_{hover})^TR\,(u_k-u_{hover})\Big] + (x_N-x_N^{ref})^TQ_f(x_N-x_N^{ref})$$

(the control term is measured *about hover*, not about zero -- see
`MPC_solver.md` Part 4.3 for why that distinction is essential and what
$u_{hover}$ works out to.)

subject to:

- $x_0 = x(t)$ -- start from the current measured/estimated state
- $x_{k+1} = \text{RK4}(x_k,u_k,\Delta t)$ -- the dynamics above, integrated
  with a proper Runge-Kutta step (forward Euler, fine for the point mass, is
  generally too crude once rotational dynamics are involved)
- $0\le f_i\le f_{max}$ for each rotor
- optionally, tilt limits ($|\phi|,|\theta|\le\theta_{max}$) and/or obstacle
  constraints, same idea as discussed for the point-mass version

**The qualitative shift from our point-mass MPC, stated plainly:** right now,
`mpc_solver.py` picks an acceleration and simply trusts PX4 can deliver it.
The model derived here can't take that shortcut -- to accelerate
horizontally, the solver has to command rotor thrusts that produce a torque,
that (through $M(\eta)^{-1}$) produces an angular acceleration, that changes
$\eta$ over time, that (through $R(\eta)$ back in Section 3.2) finally
redirects the thrust vector to produce the acceleration it actually wanted.
That whole causal chain is exactly what the point mass abstracted away.
Reasoning through it explicitly, every horizon step, is what makes this a
genuinely harder optimization problem -- and is why solve time (not modeling
accuracy) becomes the binding real-time constraint, likely requiring a
code-generated solver like `acados` rather than IPOPT called fresh in a
Python loop.

---

## Summary, for when you come back to this later

- The drone's configuration is six numbers: position $\xi$ and Euler angles
  $\eta$.
- Kinetic energy splits into translational ($\frac12m\dot\xi^T\dot\xi$, easy)
  and rotational ($\frac12\dot\eta^TM(\eta)\dot\eta$, harder -- because
  $\omega\neq\dot\eta$, and $M(\eta)=W^TJW$ depends on orientation).
- Euler-Lagrange turns that energy into two equations: translational (Section
  3.2, survives into our current code) and rotational (Section 3.3,
  $M\ddot\eta+C\dot\eta=W^T\tau_{body}$).
- $C(\eta,\dot\eta)$ is real, correctly derived, and symbolically verified --
  but we deliberately drop it (Part 5), because it's quadratic in angular
  rate and scaled by inertia *differences*, both of which are small for our
  gentle flight on a roughly symmetric frame.
- The real control input is four rotor thrusts, related to thrust/torque by
  a fixed mixer matrix (Section 6.2).
- The full nonlinear MPC problem (Part 7) has to reason through the entire
  tilt-then-accelerate causal chain that our current point-mass model skips
  -- which is both the whole point of upgrading to it, and the reason it's a
  meaningfully harder problem to solve in real time.
