# From a Point Mass to a Real Quadrotor: The Dynamics and Control Behind Our MPC

## Why this document exists

Right now, `mpc_solver.py` controls the drone by pretending it's a point
mass -- a dot with position and velocity, pushed around by an acceleration
we get to choose freely. That's a real simplification: a real quadrotor
can't just accelerate sideways, it has to *tilt* first, and tilting takes
time and is itself governed by physics we haven't modeled at all yet.

This document builds the *real* model, one honest step at a time -- and then
builds the *control* on top of it: what MPC actually is, why it needs a
prediction horizon, why it throws away most of what it computes, and what it
means mathematically for its answer to be optimal.

Nothing here is dropped in as a finished formula. Every equation is derived
from the one before it, with an explanation of *why* that step happens, so
you can always trace an equation back to where it came from instead of having
to take it on faith. If an equation appears without its origin explained,
that's a bug in this document -- say so and it will be fixed.

### Roadmap -- where we're going

The document has two halves.

**Half one: the model (Parts 1-6).** We'll describe the drone with six
numbers: three for position, three for orientation. We'll write down its
kinetic and potential energy in terms of those six numbers and their rates of
change, because there's a systematic recipe (Euler-Lagrange) that turns
"energy as a function of coordinates" directly into "equations of motion" --
no need to draw every force by hand. Applying that recipe splits cleanly into
two halves: a straightforward one for position, and a genuinely messy one for
orientation, because the drone's orientation-related "effective mass" changes
as it tilts. We'll derive that messiness properly (it's called the Coriolis
term), and *then* make a deliberate, justified decision to drop it, because
for the way our drone actually flies it turns out to be small enough not to
matter. Finally we connect the physical torques a real quadrotor produces to
what its four motors actually do, and assemble everything into a single
nonlinear state-space model $\dot x = f(x,u)$.

**Half two: the control (Parts 7-12).** A model tells you what the drone
*will do*; it doesn't tell you what to *command*. We'll see why the obvious
greedy approach fails (some correct actions, like braking, look bad in the
short term), which forces us to optimize a whole *sequence* over a
**prediction horizon**. We'll cover why we then throw away all but the first
step and re-plan -- the **receding horizon** principle, and the thing that
makes MPC a feedback controller. Then we build the cost function and
constraints that define the optimization, and finally derive from scratch
what it even means for a solution to be optimal: **Lagrange multipliers** and
the **KKT conditions**, which are what every numerical solver is ultimately
trying to satisfy.

A companion document, `MPC_solver.md`, then covers how a computer actually
*finds* such a solution fast enough to fly on.

If at any point you feel lost, jump back to this section -- it's the map.

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

## Part 7: From a model to a decision

Everything so far has answered *"what will the drone do if I apply control
$u$?"* That is a **model**. It does not tell us what $u$ to actually choose.

Parts 7 through 12 are about that second question -- which is the whole of
MPC. This is where the document stops being physics and becomes control.

### 7.1 The obvious approach, and why it fails

The instinctive idea: at each instant, pick the $u$ that most reduces the
error *right now*. This is called a **greedy** or **myopic** policy.

It fails, and it's worth seeing exactly how, because the failure is what
motivates everything that follows.

**Failure 1: it cannot plan to brake.** Suppose the drone is 10 m from its
target and must *stop* there. A greedy controller asks "which control most
reduces my position error over the next 0.1 s?" -- and the answer is always
"accelerate toward the target as hard as possible." It does this the entire
way, arrives at the target travelling at maximum speed, and sails straight
past. To stop on target you must start decelerating *while still far away*,
which briefly makes your position error decrease more slowly than it could.
A greedy controller will never choose that, because it looks worse right now.

**Failure 2: it cannot plan to tilt.** This one is specific to our
underactuated drone. To accelerate right, the drone must first roll right --
and rolling, by itself, does nothing whatsoever to reduce position error. It
costs control effort and buys no immediate improvement. A purely greedy
controller sees no reason to do it.

The lesson generalizes:

> **Some correct actions look bad in the short term. Their value only appears
> when you look far enough ahead.**

Braking, tilting, and swerving around an obstacle are all in this category.
Any controller that evaluates only the present instant is structurally blind
to them.

### 7.2 Optimizing a sequence instead of an instant

The fix follows directly: don't optimize a single control, optimize a
**sequence** of controls covering a window of future time.

Choose a window of $N$ steps, each of duration $\Delta t$. The decision is
now the whole sequence

$$u_0,\; u_1,\; \dots,\; u_{N-1}$$

and we judge it by what the *entire predicted trajectory* looks like, not by
what happens in the next instant. Braking early now scores well, because the
plan that brakes early is the plan that ends up stopped on target. Tilting
now scores well, because the plan that tilts is the plan that gets there.

That window is called the **prediction horizon**, and its length is

$$T_{horizon} = N\,\Delta t$$

This single idea -- judge a sequence, not an instant -- is the core of MPC.
Everything else is machinery for doing it well.

### 7.3 Choosing the horizon length

Two opposing pressures.

**Too short and you're still myopic.** The horizon has to be long enough to
contain the consequences of your decisions. The natural yardstick is the
system's own settling time. For our drone, the relevant question is: *how
long does it take to arrest a full-speed motion?* With
$v^{max} = 5\ \mathrm{m/s}$ and $a^{max} = 3\ \mathrm{m/s^2}$:

$$t_{stop} = \frac{v^{max}}{a^{max}} = \frac{5}{3} \approx 1.7\ \mathrm{s}$$

A horizon shorter than that literally cannot contain a stop manoeuvre -- the
controller would be unable to *see* the consequence of not braking.

**Too long and you pay twice.** The number of decision variables grows
linearly with $N$, so solve time grows with it. Worse, the extra prediction
is *fiction*: our model is a point-mass-plus-rigid-body approximation with no
aerodynamics, so a 10-second prediction is confidently wrong. Optimizing
hard against a bad prediction is worse than useless.

The practical rule: **make the horizon a bit longer than the dominant time
constant, and no longer.** Our configuration lands exactly there:

$$N = 20,\quad \Delta t = 0.1\ \mathrm{s} \quad\Longrightarrow\quad T_{horizon} = 2.0\ \mathrm{s}$$

which comfortably exceeds the 1.7 s stopping time, and stays inside the
window where our model is credible.

### 7.4 The receding horizon principle

Here is the part that surprises everyone the first time.

We compute a full 2-second, 20-step plan. Then we **execute only the first
step and throw the other nineteen away**. We take a fresh measurement,
and re-solve the entire problem from scratch, 0.1 seconds later.

```
   measure x(t)  ->  solve for u_0 ... u_19  ->  apply ONLY u_0  ->  wait dt
        ^                                                              |
        +--------------------------------------------------------------+
```

**Why discard 95% of the work?** Because the plan was computed from a model,
and the model is wrong. It ignores wind, unmodelled rotor dynamics, and every
approximation catalogued in Parts 4 and 5. If we computed one plan and
executed it open-loop to completion, those errors would compound without
limit and the drone would end up nowhere near the reference.

By re-solving from the **measured** state every tick, reality is injected
back into the problem 10 times a second. The plan is a *prediction*; only its
first step is a *commitment*.

This is precisely what converts open-loop planning into **closed-loop
feedback**, and it is the reason MPC works at all despite being built on an
imperfect model. The name says it: the horizon *recedes* -- it keeps sliding
forward, always 2 seconds ahead, never actually arrived at.

An everyday analogy: driving. You continuously plan the next several seconds
of your route, but you only commit to the next small steering input, then
look again and re-plan. You never execute a 10-second steering plan blind.

### 7.5 So how do we "find the optimal position to go to"?

This deserves stating explicitly, because it's a natural question and the
answer is slightly counter-intuitive.

**We never pick a target position directly.** There is no line of reasoning
in MPC that says "go to this point next." Instead:

1. The **decision variables** are the entire trajectory -- every state
   $x_0\dots x_N$ *and* every control $u_0\dots u_{N-1}$.
2. The **constraints** define which trajectories are physically possible
   (they must obey $\dot x = f(x,u)$, respect rotor limits, start from where
   we actually are).
3. The **cost function** scores every such trajectory with a single number.
4. The optimizer searches the space of all feasible trajectories and returns
   the one with the lowest score.

The "optimal next position and velocity" is simply $x_1$ of the winning
trajectory. It is an **output** of the optimization, not an input to it. It
emerges from asking a much larger question -- "what is the best whole future
from here?" -- and then reading off its first step.

That is the conceptual heart of the method. The remaining parts make each of
those four bullets precise.

---

## Part 8: The cost function -- encoding what "good" means

The cost function is where we tell the optimizer what we actually want. Every
behaviour the controller exhibits is, ultimately, a consequence of what we
wrote here.

### 8.1 The three competing desires

$$J = \underbrace{\sum_{k=0}^{N-1}(x_k-x_k^{ref})^TQ(x_k-x_k^{ref})}_{\text{(1) track the reference}}
+ \underbrace{\sum_{k=0}^{N-1}(u_k-u_{hover})^TR(u_k-u_{hover})}_{\text{(2) don't work too hard}}
+ \underbrace{(x_N-x_N^{ref})^TQ_f(x_N-x_N^{ref})}_{\text{(3) end up somewhere sensible}}$$

**(1) Tracking.** Penalizes being in the wrong state. $Q\succeq0$ is a weight
matrix determining how much each state error matters.

**(2) Control effort.** Penalizes working hard. Without it, the optimizer
would use arbitrarily violent inputs to shave off microscopic tracking
errors, producing commands that are jerky, energy-hungry, and outside the
range where our model is accurate.

Note carefully that this is measured **relative to hover**, not relative to
zero. A naive $\|u_k\|^2$ would call "all motors off" the cheapest possible
control, and the optimizer would happily trade altitude for control savings.
$u_{hover} = \frac{mg}{4}[1,1,1,1]^T$ is derived in `MPC_solver.md` Part 4.3;
the point is that hovering must cost *nothing*, so only deviation from it is
penalized.

**(3) Terminal cost.** Discussed in 8.3 -- it handles the horizon's edge.

### 8.2 Why squared errors

Three reasons, in increasing order of depth:

- **Smooth.** $|e|$ has a kink at zero that gradient-based optimizers dislike;
  $e^2$ is differentiable everywhere.
- **Large errors dominate.** Doubling an error quadruples its penalty, so the
  optimizer prioritizes fixing big deviations over polishing small ones --
  usually what you want.
- **It preserves structure.** A quadratic cost with *linear* dynamics gives a
  Quadratic Program, the best-behaved class in optimization (Part 12). With
  nonlinear dynamics we lose that, but the quadratic cost still makes the
  problem far more tractable than a general one -- and it is what enables the
  Gauss-Newton shortcut described in `MPC_solver.md` Part 8.3.

### 8.3 The terminal cost, and the edge-of-horizon problem

The horizon has to stop somewhere, and the optimizer knows exactly when it
stops. This creates a genuine pathology: **the optimizer has no reason to
care what happens at step $N+1$, because it cannot see it.**

Left unchecked, this produces plans that look excellent for 2 seconds and
then fall apart. The classic failure is arriving at the target position at
step $N$ with a large velocity -- perfect position score, and a catastrophe
0.1 seconds later. For our drone it can be worse: a plan can end at the right
position while inverted and tumbling, and score beautifully.

The terminal cost $Q_f$ is the fix: it is the proxy for "everything after the
horizon." A large $Q_f$ tells the optimizer that ending in a bad state is
expensive even though the consequences lie beyond what it can see.

The principled choice is the **infinite-horizon LQR cost-to-go** -- linearize
about hover, solve the algebraic Riccati equation for $P$, and set
$Q_f = P$. Then the finite-horizon cost approximates the infinite-horizon
one, and (with a terminal constraint set) this is the standard route to a
*provable* closed-loop stability guarantee. See `MPC_solver.md` Part 4.5.

### 8.4 What the weights actually control

Only the **ratios** of $Q$, $R$, $Q_f$ matter -- scaling all three by the
same factor leaves the minimizer unchanged.

$$\frac{Q}{R}\ \text{large} \;\Rightarrow\; \text{aggressive, tight tracking, twitchy commands}$$
$$\frac{Q}{R}\ \text{small} \;\Rightarrow\; \text{smooth, gentle commands, looser tracking}$$

This is the fundamental tuning trade-off, and it is genuinely a trade-off:
you cannot have arbitrarily tight tracking and arbitrarily smooth commands at
once.

One trap specific to this model: $Q$ weights **metres against radians**.
A "1" in the position block and a "1" in the attitude block are not
comparable -- 1 m of position error and 1 radian (57°!) of attitude error are
wildly different in severity. `MPC_solver.md` Part 4.4 covers Bryson's rule
for setting these sensibly rather than by guesswork.

### 8.5 Where the reference comes from

$x_k^{ref}$ is the desired state at each step of the horizon -- and note it is
indexed by $k$, i.e. it is a *trajectory*, not a fixed point. This matters:
when tracking a moving reference (our circle), the controller is given not
just where to be now, but where it should be at every step of the plan. That
is what allows it to anticipate the curve rather than perpetually chase a
target that has already moved on.

In our code the reference is generated analytically from the circle
parameterization (`reference_trajectory.py`), evaluated at each of the $N+1$
horizon times.

---

## Part 9: Constraints -- encoding what is possible

The cost says what we *want*. The constraints say what is *allowed*. This
separation is MPC's signature advantage over classical control, where limits
can usually only be imposed by clipping the output after the fact -- which
silently invalidates whatever reasoning produced that output.

### 9.1 Initial condition

$$x_0 = \hat x(t)$$

The plan must start from where the drone actually is. As established in 7.4,
this one line is what makes the whole scheme feedback rather than open-loop
planning.

### 9.2 Dynamics

$$x_{k+1} = F(x_k, u_k), \qquad k = 0,\dots,N-1$$

where $F$ denotes integrating $\dot x = f(x,u)$ over one step (numerically --
see `MPC_solver.md` Part 2, which derives RK4 and explains why forward Euler
is inadequate here).

**These constraints are what make the plan physically real.** Without them
the optimizer would "solve" the problem instantly by teleporting the drone
onto the reference at step 1 -- cost zero, completely useless. The dynamics
constraints are what force every candidate trajectory to be one the drone
could actually fly.

### 9.3 Actuator limits

$$0 \;\le\; f_{i,k} \;\le\; f_{max}, \qquad i=1,\dots,4$$

The lower bound is genuinely physical: a fixed-pitch rotor can push but
**cannot pull**. The upper bound is the motor's maximum.

Because these are *inside* the optimization, the resulting plan is one the
drone can actually execute. Contrast with clipping: if you compute a
trajectory assuming unlimited thrust and then clip, the executed trajectory
is not the one you optimized, and any guarantee you had is void.

### 9.4 State limits

$$|\phi_k| \le \phi^{max}, \qquad |\theta_k| \le \theta^{max}$$

Tilt limits serve two independent purposes, worth separating:

1. **Physical.** Past roughly 60° of tilt, the vertical thrust component
   $T\cos\phi\cos\theta$ can no longer support the vehicle's weight, and it
   descends regardless of throttle.
2. **Mathematical.** Recall from Part 4 that $W(\eta)$ becomes singular at
   $\theta = \pm90°$ -- gimbal lock -- which would make $M(\eta)^{-1}$ blow up
   *inside our own dynamics function*. Constraining tilt keeps the
   optimizer's iterates away from a region where our chosen representation
   breaks down, quite apart from whether the physical drone could survive it.

Reason 2 is easy to miss: the constraint is partly protecting the solver from
a modelling choice we made, not just protecting the aircraft.

---

## Part 10: The complete optimization problem

Assembling Parts 8 and 9:

$$\begin{aligned}
\min_{x_{0:N},\;u_{0:N-1}}\quad
& \sum_{k=0}^{N-1}\Big[(x_k-x_k^{ref})^TQ(x_k-x_k^{ref}) + (u_k-u_{hover})^TR(u_k-u_{hover})\Big] \\
&\qquad\qquad + (x_N-x_N^{ref})^TQ_f(x_N-x_N^{ref})\\[6pt]
\text{subject to}\quad
& x_0 = \hat x(t) && \text{(where we are)}\\
& x_{k+1} = F(x_k,u_k), && \text{(physics)}\\
& 0 \le f_{i,k} \le f_{max}, && \text{(actuator limits)}\\
& |\phi_k| \le \phi^{max},\ |\theta_k| \le \theta^{max} && \text{(state limits)}
\end{aligned}$$

Solve this. Take $u_0$. Discard the rest. Re-measure. Repeat at 10 Hz.

That is the complete algorithm. What remains is understanding what "solve
this" means -- which is Part 11.

---

## Part 11: What makes a solution optimal -- Lagrange multipliers and KKT

We can now state the problem. But how do we *recognize* an optimum when we
have one? Without an answer, "solve this" is not a well-defined instruction.

This part builds the answer from scratch: unconstrained optimality first,
then equality constraints (Lagrange multipliers), then inequality constraints
(KKT). These conditions are what every numerical method in `MPC_solver.md` is
actually trying to satisfy.

### 11.1 The easy case: no constraints

Minimize $f(z)$ over all $z$, no restrictions. The condition is familiar:

$$\nabla f(z^*) = 0$$

The reasoning: if $\nabla f \ne 0$, then moving a small step in the direction
$-\nabla f$ decreases $f$. So we weren't at a minimum. Only when the gradient
vanishes is there no improving direction left.

Note what this argument really used: **there is no improving direction that
we are allowed to move in.** That phrasing is what generalizes.

### 11.2 One equality constraint: the geometric argument

Now minimize $f(z)$ subject to $g(z) = 0$.

The constraint $g(z)=0$ carves out a surface, and we may only move *along*
it. So $\nabla f(z^*) = 0$ is no longer required -- the gradient can be
nonzero, provided every direction that would exploit it is forbidden.

Make that precise. A direction $d$ keeps us on the surface (to first order)
exactly when

$$\nabla g(z)^T d = 0$$

and moving in direction $d$ changes the cost at rate

$$\nabla f(z)^T d$$

**Optimality means: for every direction $d$ that stays on the surface, the
cost does not decrease.** Since $d$ and $-d$ are both allowed, "does not
decrease" forces "does not change":

$$\nabla g(z^*)^Td = 0 \;\;\Longrightarrow\;\; \nabla f(z^*)^Td = 0$$

Read that as a statement about vectors: $\nabla f$ is orthogonal to every
vector that $\nabla g$ is orthogonal to. In other words, $\nabla f$ has no
component along the surface at all -- it points entirely *across* it,
parallel to $\nabla g$. So there exists a scalar $\lambda$ with

$$\nabla f(z^*) = -\lambda\,\nabla g(z^*)
\qquad\Longleftrightarrow\qquad
\nabla f(z^*) + \lambda\,\nabla g(z^*) = 0$$

That scalar $\lambda$ is the **Lagrange multiplier**.

The physical picture: at the optimum, the cost gradient is pulling you in
some direction, and the constraint is pushing back with exactly equal and
opposite force. The system is in equilibrium. If the two didn't balance,
you'd slide along the surface and improve.

### 11.3 The Lagrangian

The condition above is elegantly repackaged by defining the **Lagrangian**:

$$\mathcal{L}(z,\lambda) = f(z) + \lambda\,g(z)$$

Because then:

$$\nabla_z\mathcal{L} = \nabla f + \lambda\nabla g = 0 \quad\text{(the condition we just derived)}$$
$$\nabla_\lambda\mathcal{L} = g(z) = 0 \quad\text{(the original constraint, recovered for free)}$$

So a constrained problem in $z$ becomes an **unconstrained** stationarity
problem in $(z,\lambda)$ jointly. That is the whole trick, and it generalizes
directly to many constraints -- one multiplier each, $\lambda$ becomes a
vector:

$$\mathcal{L}(z,\lambda) = f(z) + \lambda^Tg(z)$$

### 11.4 Inequality constraints: two cases

Now add $h(z) \le 0$. The new feature is that an inequality can be either
"in play" or not, and the two cases behave completely differently.

**Case A -- inactive ($h(z^*) < 0$).** We are strictly inside the allowed
region. Small moves in *any* direction remain feasible, so the constraint
exerts no influence whatsoever; locally the problem behaves as if it weren't
there. Its multiplier is zero:

$$\nu = 0$$

**Case B -- active ($h(z^*) = 0$).** We are sitting exactly on the boundary,
and it *is* restraining us. This resembles the equality case -- but with a
crucial asymmetry. A boundary can only push **one way**.

Precisely: feasible directions are those that don't take us outside, i.e.
$\nabla h^Td \le 0$. Optimality requires $\nabla f^Td \ge 0$ for all such
$d$ (no feasible direction improves). Unlike 11.2 we cannot conclude
equality, because $d$ and $-d$ are *not* both feasible here. Working through
it gives

$$\nabla f(z^*) = -\nu\,\nabla h(z^*), \qquad \nu \ge 0$$

**The sign condition $\nu\ge0$ is the essential difference from equality
constraints,** and it has a clean meaning: the constraint can push you *away
from* violation, never *toward* it. A negative $\nu$ would describe a
constraint dragging you out of the feasible region, which is nonsense.

### 11.5 Complementary slackness

The two cases combine into one elegant statement. In Case A, $\nu = 0$; in
Case B, $h(z^*) = 0$. Either way the product vanishes:

$$\nu_i\,h_i(z^*) = 0 \qquad \text{for every } i$$

This is **complementary slackness**: for each inequality, *either* it is
slack and exerts no force, *or* it is tight and may exert force. Never both
nonzero.

It is also the source of real computational difficulty. With $m$
inequalities there are in principle $2^m$ possible active/inactive
combinations, and identifying the right one is combinatorial. The two great
families of algorithms are essentially two different escapes from this
combinatorial explosion -- active-set methods search the combinations
cleverly, interior-point methods smooth the either/or into a single equation.
`MPC_solver.md` Part 9 covers both.

### 11.6 The KKT conditions

Putting everything together, for the general problem

$$\min_z f(z) \quad\text{s.t.}\quad g(z)=0,\quad h(z)\le0$$

with Lagrangian $\mathcal{L}(z,\lambda,\nu) = f(z)+\lambda^Tg(z)+\nu^Th(z)$,
a point $z^*$ is optimal only if there exist multipliers $\lambda,\nu$ such
that:

$$\boxed{
\begin{aligned}
&\textbf{Stationarity:} && \nabla f(z^*) + \nabla g(z^*)^T\lambda + \nabla h(z^*)^T\nu = 0\\
&\textbf{Primal feasibility:} && g(z^*)=0,\qquad h(z^*)\le 0\\
&\textbf{Dual feasibility:} && \nu \ge 0\\
&\textbf{Complementary slackness:} && \nu_i h_i(z^*) = 0 \quad\forall i
\end{aligned}}$$

These are the **Karush-Kuhn-Tucker (KKT) conditions**, and they are the
central object of constrained optimization. Every solver in `MPC_solver.md`
is, underneath, an algorithm for finding a point that satisfies them.

In plain words:
1. **Stationarity** -- cost gradient exactly balanced by constraint forces.
2. **Primal feasibility** -- the answer actually satisfies the constraints.
3. **Dual feasibility** -- constraints only push outward, never inward.
4. **Complementary slackness** -- only constraints you're touching can push.

### 11.7 What the multipliers mean physically

The multipliers are not bookkeeping artifacts -- they carry real information.

$\lambda_i$ and $\nu_i$ are **shadow prices**: each measures how much the
optimal cost would improve if that constraint were relaxed by one unit. (Up
to a sign convention, $\partial J^*/\partial(\text{constraint level}) = \mp\lambda$.)

This is directly useful diagnostically. If a rotor-thrust bound comes back
with a large $\nu$, that constraint is expensive -- the controller is
straining against the motors, and a more powerful vehicle would meaningfully
improve performance. If $\nu = 0$, that limit isn't binding at all and could
be tightened for free. Reading multipliers tells you *which* physical
limitation is actually holding your controller back.

### 11.8 Necessary versus sufficient -- and where nonlinearity bites

A crucial caveat, and it is exactly where our nonlinear model differs from
the simple case.

For a **convex** problem, KKT is both necessary **and sufficient**: find a
KKT point and you have provably found the *global* optimum.

For our **nonconvex** problem (nonlinear dynamics constraints), KKT is only
**necessary**. A KKT point may be a local minimum, a saddle point, or even a
local maximum. To confirm a local minimum you need the second-order
condition -- the Hessian of the Lagrangian must be positive definite on the
subspace of directions that stay feasible:

$$d^T\nabla^2_{zz}\mathcal{L}\,d > 0 \quad\text{for all } d\ne0 \text{ with } \nabla g\,d = 0,\ \nabla h_{\mathcal{A}}\,d = 0$$

("moving in any feasible direction increases the cost"). And even this
certifies only a *local* minimum. Global optimality is, in general, not
verifiable.

There is also a technical prerequisite: KKT is valid only under a
**constraint qualification** such as LICQ (the active constraint gradients
must be linearly independent). It usually holds, but degenerate or redundant
constraints can break it -- one more reason to formulate constraints cleanly.

---

## Part 12: Convex versus nonconvex -- what nonlinearity costs

It is worth being blunt about what the better model costs us, because it is
easy to assume the price is just "more algebra." It isn't.

If the dynamics were **linear** ($x_{k+1}=Ax_k+Bu_k$) with our quadratic cost
and linear bounds, the problem would be a **convex Quadratic Program** -- and
convexity is worth an enormous amount:

| Property | Convex (linear model) | Nonconvex (our model) |
|---|---|---|
| Local optimum is global | **Yes** | **No** |
| Solution unique | Yes | No |
| KKT sufficient | Yes | Only necessary |
| Solve time | Predictable | Variable |
| Initial guess matters | No | **Critically** |
| Convergence | Reliable | Can fail |

Every guarantee in the middle column is lost the moment $R(\eta)$,
$M(\eta)$, and $M(\eta)^{-1}$ enter the constraints. **That, not the extra
trigonometry, is the real cost of the full model.**

**What a local minimum looks like physically.** This isn't abstract. Suppose
the drone must yaw 180°. Turning left and turning right are both locally
optimal, separated by a cost barrier. A solver started with a leftward guess
converges leftward and never discovers that rightward was equally good -- or,
with an obstacle on the left, *better*. Which answer you get depends entirely
on where the search began.

**A direct consequence:** warm-starting the optimizer from the previous
tick's solution stops being a mere efficiency trick and becomes a
**correctness** requirement. It keeps consecutive solves in the same basin of
attraction, preventing the controller from jumping between distinct local
optima and producing violently discontinuous commands -- even though each
individual solve is "optimal." See `MPC_solver.md` Part 6.3.

---

## Part 13: Where to go next

This document has built the complete picture from physics to a stated
optimization problem, and characterized what its solution must satisfy
(KKT). What it has *not* done is explain how a computer actually finds such
a point in a few milliseconds.

That is `MPC_solver.md`, which picks up exactly here and covers:

- **Discretization** -- deriving RK4, and why forward Euler is unsafe on our
  fast rotational modes (its Part 2)
- **Transcription** -- multiple vs single shooting, and why single shooting
  fails on nonlinear dynamics (Part 3)
- **Cost and constraint traps** specific to this model -- yaw wrapping,
  $u_{hover}$, incommensurable units (Parts 4-5)
- **Newton-type methods** -- solving the KKT system, indefinite Hessians,
  Gauss-Newton (Part 8)
- **SQP versus interior-point** -- including the log-barrier method derived in
  full, and why interior-point methods warm-start poorly (Part 9)
- **Automatic differentiation** -- where all the required gradients come from
  (Part 10)
- **Real-time feasibility** -- the RTI scheme and `acados` (Part 11)
- **A build order** for implementing this safely (Part 12)

---

## Summary, for when you come back to this later

**The model (Parts 1-6)**

- The drone's configuration is six numbers: position $\xi$ and Euler angles
  $\eta$.
- Kinetic energy splits into translational ($\frac12m\dot\xi^T\dot\xi$, easy)
  and rotational ($\frac12\dot\eta^TM(\eta)\dot\eta$, harder -- because
  $\omega\ne\dot\eta$, and $M(\eta)=W^TJW$ depends on orientation).
- Euler-Lagrange turns that energy into two equations: translational (Section
  3.2, the one that survives into our current point-mass code) and rotational
  (Section 3.3, $M\ddot\eta + C\dot\eta = W^T\tau_{body}$).
- $C(\eta,\dot\eta)$ is real, correctly derived, and symbolically verified --
  but deliberately dropped (Part 5), because it is quadratic in angular rate
  and scaled by inertia *differences*, both small for gentle flight on a
  roughly symmetric frame.
- The real control input is four rotor thrusts, mapped to thrust and torque
  by a fixed mixer matrix (Section 6.2).

**The control problem (Parts 7-10)**

- Greedy control fails because **some correct actions look bad in the short
  term** -- braking and tilting both do. The fix is to optimize a *sequence*
  over a **prediction horizon** rather than an instant (7.1-7.2).
- Horizon length is bounded below by the system's settling time ($\approx1.7$ s
  to stop, hence our 2.0 s horizon) and above by model credibility and solve
  cost (7.3).
- **Receding horizon:** plan $N$ steps, apply only the first, re-measure,
  re-plan. Discarding the rest is what makes MPC *feedback* rather than
  open-loop planning (7.4).
- We never choose a target position directly -- the optimal next state
  **emerges** as $x_1$ of the best whole trajectory (7.5).
- The **cost** trades tracking against control effort, with a terminal cost
  handling the edge-of-horizon problem; the **constraints** encode physics
  and actuator/state limits (Parts 8-9).

**Optimality (Parts 11-12)**

- Unconstrained optimality is $\nabla f = 0$; with constraints it becomes
  "no *allowed* direction improves."
- **Lagrange multipliers** arise because at an optimum $\nabla f$ must be
  parallel to $\nabla g$ -- cost gradient exactly balanced by constraint
  force (11.2).
- **KKT** = stationarity + primal feasibility + dual feasibility ($\nu\ge0$)
  + complementary slackness ($\nu_ih_i=0$). Every solver is an algorithm for
  finding a KKT point (11.6).
- Multipliers are **shadow prices** -- they tell you which physical limit is
  actually constraining your controller (11.7).
- For our nonconvex problem KKT is only **necessary**, not sufficient. Local
  minima are real and physical, and warm starting becomes a correctness
  concern rather than an optimization (11.8, Part 12).
