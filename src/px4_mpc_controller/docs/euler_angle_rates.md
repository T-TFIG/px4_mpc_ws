# Deriving $E(\phi,\theta)$, and Why It Exists

A companion to Step 4 of [`MPC_explanation_my_version.md`](MPC_explanation_my_version.md).

Step 4 states two matrices without deriving them:

$$\omega = T(\phi,\theta)\,\dot\eta, \qquad \dot\eta = E(\phi,\theta)\,\omega, \qquad E = T^{-1}$$

This document builds both from scratch, inverts $T$ by hand, checks the
result numerically, and works through the singularity. Nothing here is needed
to follow the main derivation -- Step 4 gives you the matrices and moves on.

---

## Why the two are not the same

The gyroscope measures $\omega = [p,q,r]^\top$. These are **body** angular
velocities: rotation rates about the drone's *own* axes, as they are right
now.

The optimizer, on the other hand, stores $\phi, \theta, \psi$ -- and those are
defined by a *sequence* of rotations about *different* axes. Roll happens
about one axis, then pitch about an axis that roll has already moved, then yaw
about an axis that both have moved. Three rotations, three different frames.
So $\dot\phi, \dot\theta, \dot\psi$ are rates about three axes that are not
the body axes, and are not even mutually perpendicular in general.

That is the whole reason $\omega \ne \dot\eta$. They are rates about
different sets of axes, and going between them needs a change of basis that
depends on the current attitude.

The principle that gets us there: **angular velocities add as vectors**, so
long as they are all expressed in the same frame. So express each of the
three contributions in the body frame and sum:

$$\omega = \omega_{\text{roll}} + \omega_{\text{pitch}} + \omega_{\text{yaw}}$$

Take them one at a time.

### Contribution from roll

Roll is about the body $x$ axis -- the last rotation in the sequence, so it
needs no correction at all. It is already in the body frame:

$$\omega_{\text{roll}} = \begin{bmatrix}\dot\phi \\ 0 \\ 0\end{bmatrix}$$

### Contribution from pitch

Pitch occurs *before* roll in the sequence, so $\dot\theta$ lives in a frame
that roll has since rotated away from the body frame. To express it in the
body frame, undo the roll:

$$\omega_{\text{pitch}} = R_x(-\phi)\begin{bmatrix}0 \\ \dot\theta \\ 0\end{bmatrix},
\qquad
R_x(-\phi) = \begin{bmatrix}1 & 0 & 0\\ 0 & \cos\phi & \sin\phi \\ 0 & -\sin\phi & \cos\phi\end{bmatrix}$$

$$\omega_{\text{pitch}} = \begin{bmatrix}0 \\ \cos\phi\,\dot\theta \\ -\sin\phi\,\dot\theta\end{bmatrix}$$

Read that physically: a pure pitch rate does not show up purely on the
gyroscope's $q$ channel. If the drone is rolled, part of it bleeds into $r$.
That bleed is exactly what $\omega = \dot\eta$ would have thrown away.

### Contribution from yaw

Yaw occurs first of all, so it has to be brought through *both* subsequent
rotations:

$$\omega_{\text{yaw}} = R_x(-\phi)R_y(-\theta)\begin{bmatrix}0 \\ 0 \\ \dot\psi\end{bmatrix}$$

Working inside out, with
$R_y(-\theta) = \begin{bmatrix}\cos\theta & 0 & -\sin\theta\\ 0 & 1 & 0\\ \sin\theta & 0 & \cos\theta\end{bmatrix}$:

$$R_y(-\theta)\begin{bmatrix}0\\0\\\dot\psi\end{bmatrix}
= \begin{bmatrix}-\sin\theta\,\dot\psi \\ 0 \\ \cos\theta\,\dot\psi\end{bmatrix}$$

and applying $R_x(-\phi)$ to that:

$$\omega_{\text{yaw}} = \begin{bmatrix}-\sin\theta\,\dot\psi \\ \sin\phi\cos\theta\,\dot\psi \\ \cos\phi\cos\theta\,\dot\psi\end{bmatrix}$$

### Adding them together

Summing component by component:

$$p = \dot\phi - \sin\theta\,\dot\psi$$
$$q = \cos\phi\,\dot\theta + \sin\phi\cos\theta\,\dot\psi$$
$$r = -\sin\phi\,\dot\theta + \cos\phi\cos\theta\,\dot\psi$$

In matrix form:

$$\begin{bmatrix}p \\ q \\ r\end{bmatrix} =
\underbrace{\begin{bmatrix}
1 & 0 & -\sin\theta \\
0 & \cos\phi & \sin\phi\cos\theta \\
0 & -\sin\phi & \cos\phi\cos\theta
\end{bmatrix}}_{T(\phi,\theta)}
\begin{bmatrix}\dot\phi \\ \dot\theta \\ \dot\psi\end{bmatrix}$$

Two structural things worth noticing. $T$ depends on $\phi$ and $\theta$ but
**not on $\psi$** -- yaw is the outermost rotation, so rotating the whole
picture about the world $z$ axis does not change how the contributions stack
up. And the first column is $[1,0,0]^\top$ exactly, because roll needs no
correction: it is already a body-axis rate.

---

## Inverting $T$ by hand

The model needs the other direction -- given what the gyroscope says, how do
the stored angles change:

$$\begin{bmatrix}\dot\phi \\ \dot\theta \\ \dot\psi\end{bmatrix}
= T(\phi,\theta)^{-1}\begin{bmatrix}p \\ q \\ r\end{bmatrix},
\qquad E(\phi,\theta) = T(\phi,\theta)^{-1}$$

Rather than quote the inverse, solve the three equations. The structure makes
this easy: the second and third rows involve only $\dot\theta$ and $\dot\psi$,
so they form a self-contained $2\times2$ system:

$$\cos\phi\,\dot\theta + \sin\phi\cos\theta\,\dot\psi = q$$
$$-\sin\phi\,\dot\theta + \cos\phi\cos\theta\,\dot\psi = r$$

**Eliminate $\dot\psi$.** Multiply the first by $\cos\phi$, the second by
$-\sin\phi$, and add. The $\dot\psi$ terms cancel
($\sin\phi\cos\phi\cos\theta - \sin\phi\cos\phi\cos\theta = 0$) and
$\cos^2\phi + \sin^2\phi = 1$ leaves:

$$\dot\theta = \cos\phi\,q - \sin\phi\,r$$

**Eliminate $\dot\theta$.** Multiply the first by $\sin\phi$, the second by
$\cos\phi$, and add. Now the $\dot\theta$ terms cancel:

$$\cos\theta\,\dot\psi = \sin\phi\,q + \cos\phi\,r
\qquad\Longrightarrow\qquad
\dot\psi = \frac{\sin\phi}{\cos\theta}q + \frac{\cos\phi}{\cos\theta}r$$

**Back-substitute** into the first row, $\dot\phi - \sin\theta\,\dot\psi = p$:

$$\dot\phi = p + \sin\theta\,\dot\psi
= p + \sin\phi\tan\theta\,q + \cos\phi\tan\theta\,r$$

Collecting all three:

$$\boxed{\;E(\phi,\theta) = \begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \dfrac{\sin\phi}{\cos\theta} & \dfrac{\cos\phi}{\cos\theta}
\end{bmatrix}\;}$$

Notice the middle row has no $\theta$ in it at all: $\dot\theta$ is recovered
from $q$ and $r$ by a plain planar rotation through $\phi$. That row is the
only one that stays finite everywhere, which is the first hint at what
follows.

### Numerical check

The derivation above is short enough to check directly. At
$\phi = 0.37$, $\theta = -0.61$ rad:

$$\lVert E\,T - \mathbb{I}\rVert_\infty = 2.2\times10^{-16}$$

-- machine epsilon, so the two matrices really are inverses. This check is
worth running whenever you touch either matrix; it catches sign errors and
swapped trig functions immediately. For instance, the common slip of writing
$\sin\phi/\cos\phi$ instead of $\sin\phi/\cos\theta$ in entry $(3,2)$ gives
$\lVert E\,T - \mathbb{I}\rVert_\infty = 4.97\times10^{-2}$ at the same
angles -- small enough to look like a rounding artefact if you are not
looking for it, large enough to corrupt every attitude prediction.

---

## Gimbal lock

$$\det T = \cos\theta$$

so $T$ is singular, and $E$ blows up, at $\theta = \pm 90^\circ$. Both the
first and third rows of $E$ carry $1/\cos\theta$.

This is **gimbal lock**. Pitched straight up or straight down, the roll axis
and the yaw axis have become the same physical axis, so a rotation about
either produces the identical motion -- and the parameterization can no
longer tell you which one caused it. The map from $(\phi,\theta,\psi)$ to
orientation stops being locally invertible, which is exactly what a
determinant hitting zero means.

Three things worth being clear about:

**It is a defect of the coordinates, not the drone.** Nothing physical
happens at $\theta = 90^\circ$. A real quadrotor pitched vertical is
perfectly well behaved; it is the three-number description that fails. Any
three-parameter representation of orientation has this problem somewhere --
it is a topological fact about $SO(3)$, not a bad choice of angles.

**The failure is not just at the singularity.** $1/\cos\theta$ is already 2 at
$\theta = 60^\circ$ and 5.76 at $\theta = 80^\circ$. Well before the blow-up,
$E$ becomes ill-conditioned, meaning small errors in $\omega$ get amplified
into large errors in $\dot\eta$. If the MPC is planning through steep
attitudes, the model degrades gradually rather than failing suddenly.

**The alternative is quaternions.** Four numbers with one norm constraint,
no singularity anywhere, and a kinematic relation
$\dot{\mathbf q} = \frac{1}{2}\mathbf q \otimes \begin{bmatrix}0\\\omega\end{bmatrix}$
that is bilinear rather than trigonometric -- which also makes it cheaper and
better-behaved to differentiate for the solver. The costs are the extra state,
the unit-norm constraint the optimizer has to respect, and the double cover
($\mathbf q$ and $-\mathbf q$ are the same rotation, which a cost function
must not be allowed to care about).

Step 4 accepts Euler angles anyway, because the singularity sits far outside
any attitude this controller will command. That is a defensible choice for a
position controller and an indefensible one for anything acrobatic.

---

## Corrections to the original draft

Three arithmetic slips from the first version of Step 4, recorded so the
fixes are traceable:

1. **$\omega_{\text{yaw}}$, second component.** The draft had
   $\sin\phi\sin\theta\,\dot\psi$; the correct value is
   $\sin\phi\cos\theta\,\dot\psi$. This propagated into the expression for
   $q$. The draft's $T$ matrix already had the correct entry, so the vector
   and the matrix disagreed -- the matrix was right.
2. **$E$, entry $(3,2)$.** The draft had $\sin\phi/\cos\phi$; it should be
   $\sin\phi/\cos\theta$. Caught by the numerical check above.
3. **Sign in $p$.** The draft had a double negative,
   $p = \dot\phi - -\sin\theta\,\dot\psi$. It is
   $p = \dot\phi - \sin\theta\,\dot\psi$, consistent with the $-\sin\theta$
   in $T$.

---

| back to | |
|---|---|
| [`MPC_explanation_my_version.md`](MPC_explanation_my_version.md) | Part I dynamics, Part II discretization |
| [`why_rk4.md`](why_rk4.md) | Part II's claims, measured |
