# Why RK4, Measured

A companion to Part II of [`MPC_explanation_my_version.md`](MPC_explanation_my_version.md).

Part II claims that forward Euler gains energy, backward Euler loses it, and
RK4 does neither -- and that RK4 is worth four function evaluations per step.
This document measures all of it, on the actual 12-state quadrotor model.
Nothing here is needed to follow the main derivation. It is here so the
claims are checkable rather than taken on trust.

---

## Why Euler is biased, exactly

"Euler is inaccurate" is not the real problem. *Biased* is.

Take the cleanest possible test: a system whose exact solution neither grows
nor decays, so any growth or decay in the numerical answer is pure artefact.
A harmonic oscillator $\ddot y = -\omega^2 y$ has eigenvalues $\pm i\omega$
-- purely imaginary, energy conserved exactly, forever.

Apply each scheme to $\dot x = \lambda x$ with $\lambda = i\omega$ and read
off the factor the state is multiplied by each step:

| scheme | amplification $R(z)$, $z = i\omega\Delta t$ | $\lvert R\rvert$ |
|---|---|---|
| forward Euler | $1 + z$ | $\sqrt{1 + (\omega\Delta t)^2} > 1$ |
| backward Euler | $\dfrac{1}{1-z}$ | $\dfrac{1}{\sqrt{1+(\omega\Delta t)^2}} < 1$ |
| RK4 | $1 + z + \frac{z^2}{2} + \frac{z^3}{6} + \frac{z^4}{24}$ | $1 - \frac{(\omega\Delta t)^6}{72} + \dots \approx 1$ |

The inequalities hold for **any** $\Delta t > 0$. Forward Euler does not
merely go unstable if you push the step too far -- on an oscillatory mode it
grows at every step size there is. Backward Euler is unconditionally stable,
but pays by damping something that should not be damped.

With a 1 Hz oscillation sampled at 100 Hz ($\omega\Delta t = 0.0628$):

| scheme | per step | after 1 second (100 steps) |
|---|---|---|
| forward Euler | $\times 1.001972$ | $\times 1.2177$ -- 22% energy invented |
| backward Euler | $\times 0.998032$ | $\times 0.8212$ -- 18% energy destroyed |
| RK4 | $\times 0.99999999957$ | $\times 1.0000000$ |

Both Euler methods are wrong by roughly $\pm 20\%$ per second in a quantity
that should be exactly constant. RK4 is wrong in the tenth digit.

---

## The same thing on the real model

The oscillator demonstrates the mechanism, not the drone -- the open-loop
quadrotor is a chain of integrators, so it does not oscillate on its own. But
it does as soon as the MPC closes a loop around it, and in the meantime the
plain truncation error is bad enough by itself.

Integrating the full 12-state $f_c$ from a tilted, spinning initial condition
over a 0.3 s horizon (30 steps at 100 Hz), against a reference computed with
RK4 at $\Delta t = 1.25\times10^{-6}$:

| $\Delta t$ | forward Euler | RK4 | RK4 improvement per halving |
|---|---|---|---|
| 0.0100 | $2.25\times10^{-2}$ | $3.44\times10^{-10}$ | -- |
| 0.0050 | $1.13\times10^{-2}$ | $2.17\times10^{-11}$ | $\times 15.8$ |
| 0.0025 | $5.64\times10^{-3}$ | $2.74\times10^{-12}$ | $\times 7.9$ |

Two things to read off:

**Size.** At the actual control rate, RK4 is **seven orders of magnitude**
more accurate than forward Euler, for four function evaluations instead of
one.

**Slope.** Halving $\Delta t$ halves the Euler error (first order) but
divides the RK4 error by about 16 (fourth order, $2^4 = 16$). The last row's
$\times 7.9$ is not RK4 failing -- it is the reference's own noise floor at
$\sim 10^{-12}$, which the method has already reached.

### Where the fourth order comes from

Four evaluations buy fourth-order accuracy rather than merely better
first-order accuracy because the stages are arranged so their individual
errors *cancel* in the weighted sum. Each $k_i$ is wrong; the Taylor
expansion of the combination matches the true solution's expansion through
$\Delta t^4$ and only disagrees at $\Delta t^5$:

$$\text{local error per step} = O(\Delta t^5),
\qquad \text{global error over a fixed time} = O(\Delta t^4)$$

(one power is lost because a fixed interval takes $\propto 1/\Delta t$ steps).

The weights $\frac{1}{6}, \frac{2}{6}, \frac{2}{6}, \frac{1}{6}$ are
Simpson's weights, and that is not a coincidence. RK4 is Simpson's rule
applied to

$$x(t_k + \Delta t) = x(t_k) + \int_{t_k}^{t_k+\Delta t} f_c\big(x(s), u(s)\big)\,ds$$

-- an integral whose integrand it has to discover as it goes, since the
integrand depends on the $x(s)$ being solved for. Every integration scheme in
existence is a different guess at that integral from information available at
the start of the step. Forward Euler guesses the rate at $t_k$ holds for the
whole interval; RK4 probes four times and averages.

---

## The $k_4$ trap

$k_4$ must use the **full step**:

$$k_4 = f_c(x_k + \Delta t\, k_3,\; u_k) \qquad
\text{not} \qquad k_4 = f_c(x_k + \tfrac{\Delta t}{2} k_3,\; u_k)$$

The cancellation above depends on $k_4$ being a probe at the *end* of the
interval. Move it to the midpoint and the cancellation collapses. Measured on
the same 12-state model, same horizon:

| $\Delta t$ | correct RK4 | $k_4$ at half step |
|---|---|---|
| 0.0100 | $3.44\times10^{-10}$ | $3.75\times10^{-3}$ |
| 0.0050 | $2.17\times10^{-11}$ | $1.88\times10^{-3}$ |
| 0.0025 | $2.74\times10^{-12}$ | $9.41\times10^{-4}$ |

The broken column **halves** rather than dividing by 16. The scheme silently
degrades from fourth order to *first* -- the same order as forward Euler --
while still paying for four function evaluations.

This is worth watching for because of how it fails. There is no crash, no
NaN, no obviously wrong number. The drone still flies and the trajectories
still look reasonable; you have simply bought a first-order integrator at
four times the price. The only symptom is that refining $\Delta t$ does not
help as much as it should, which is exactly the test above.

---

## Which error actually accumulates

An intuition worth correcting, because it changes what you should worry
about.

It is natural to reason that integration error builds up the longer the drone
flies, so a long flight is riskier than a short one. In closed loop that is
not how it works. **MPC re-measures the true state at the start of every
cycle.** $x_0$ is a measurement, not an integration -- so integration error
is thrown away and restarted 100 times a second. It cannot accumulate over
minutes of flight.

What it accumulates over is the **horizon**: 30 steps of compounding error
inside a single prediction, discarded and rebuilt on the next cycle.

And the consequence is worse than drift, because it is not random. A biased
integrator means the plan is systematically wrong about where the drone ends
up, so the controller commits to an input chosen for a future that will not
happen -- and then does it again next cycle, the same way. Persistent bias in
the model becomes persistent bias in the control. The failure mode is not
slow drift; it is a controller confidently steering toward the wrong place.

---

## What RK4 costs the solver

$f_d$ is a *composition* of four evaluations of $f_c$, each feeding the next.
The solver does not only evaluate $f_d$ -- it needs the Jacobians
$\partial f_d/\partial x_k$ and $\partial f_d/\partial u_k$ to build the
linearized constraint at each SQP iteration. Differentiating through a
four-stage composition means the chain rule runs through all four stages:

$$\frac{\partial x_{k+1}}{\partial x_k}
= \mathbb{I} + \frac{\Delta t}{6}\left(\frac{\partial k_1}{\partial x_k}
+ 2\frac{\partial k_2}{\partial x_k} + 2\frac{\partial k_3}{\partial x_k}
+ \frac{\partial k_4}{\partial x_k}\right)$$

where, writing $A(x) = \partial f_c/\partial x$ evaluated at the stage point,

$$\frac{\partial k_1}{\partial x_k} = A_1, \qquad
\frac{\partial k_2}{\partial x_k} = A_2\left(\mathbb{I} + \tfrac{\Delta t}{2}\frac{\partial k_1}{\partial x_k}\right), \qquad
\frac{\partial k_3}{\partial x_k} = A_3\left(\mathbb{I} + \tfrac{\Delta t}{2}\frac{\partial k_2}{\partial x_k}\right)$$

and so on -- each stage's derivative depends on the previous stage's. Four
Jacobian evaluations of $f_c$ plus four matrix products, per horizon step,
per SQP iteration.

This is the price of the accuracy, and it is the main argument for letting an
automatic-differentiation tool (CasADi, JAX) build $f_d$ and its derivatives
rather than writing them by hand. Hand-derived RK4 Jacobians are correct
right up until someone changes a term in $f_c$ and forgets to update them.

---

## Reproducing these numbers

Everything above comes from one script. The model is the $f_c$ of Part I with
$m = 1.5$ kg, $I = \mathrm{diag}(0.029, 0.029, 0.055)$, started from a
tilted, spinning state with a hovering-plus-8% thrust and small torques.

```python
def step_rk4(x, u, h):
    k1 = fc(x, u)
    k2 = fc(x + h/2*k1, u)
    k3 = fc(x + h/2*k2, u)
    k4 = fc(x + h*k3,   u)      # FULL step -- this is the line that matters
    return x + h/6*(k1 + 2*k2 + 2*k3 + k4)

def step_fe(x, u, h):
    return x + h*fc(x, u)
```

The order test is the whole experiment in three lines: integrate a fixed
interval at $\Delta t$ and at $\Delta t/2$, take the ratio of the errors
against a high-accuracy reference, and check it against $2^{\text{order}}$.
It catches the $k_4$ bug, and any other silent order loss, immediately.

---

| back to | |
|---|---|
| [`MPC_explanation_my_version.md`](MPC_explanation_my_version.md) | Part I dynamics, Part II discretization |
