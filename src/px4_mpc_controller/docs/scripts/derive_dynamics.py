"""Symbolic derivation of the full 6-DOF quadrotor rotational dynamics
(M(eta), C(eta, etadot)) via Euler-Lagrange + Christoffel symbols.

Run with: python3 derive_dynamics.py
(requires sympy: pip install sympy)

Produces the Euler-Lagrange form of the model used in ../MPC_solver.md
Part 1 -- M(eta), C(eta, etadot), W(eta) -- plus a
skew-symmetry check (Mdot - 2C must be skew-symmetric) as a correctness
sanity check on C.
"""
import sympy as sp

phi, theta, psi = sp.symbols('phi theta psi', real=True)
dphi, dtheta, dpsi = sp.symbols('dphi dtheta dpsi', real=True)
Jx, Jy, Jz = sp.symbols('J_x J_y J_z', positive=True)

W = sp.Matrix([
    [1, 0, -sp.sin(theta)],
    [0, sp.cos(phi), sp.sin(phi) * sp.cos(theta)],
    [0, -sp.sin(phi), sp.cos(phi) * sp.cos(theta)],
])
J = sp.diag(Jx, Jy, Jz)

M = sp.simplify(W.T * J * W)

print("=== M(eta) = W^T J W ===")
for i in range(3):
    for j in range(3):
        print(f"M[{i+1}][{j+1}] =", sp.simplify(M[i, j]))

# Christoffel symbols (Spong's convention):
#   c_ijk = 1/2 (dM_kj/dq_i + dM_ki/dq_j - dM_ij/dq_k)
#   C_kj  = sum_i c_ijk * qdot_i
q = [phi, theta, psi]
qd = [dphi, dtheta, dpsi]

C = sp.zeros(3, 3)
for k in range(3):
    for j in range(3):
        entry = 0
        for i in range(3):
            c_ijk = sp.Rational(1, 2) * (
                sp.diff(M[k, j], q[i]) + sp.diff(M[k, i], q[j]) - sp.diff(M[i, j], q[k])
            )
            entry += c_ijk * qd[i]
        C[k, j] = sp.simplify(sp.trigsimp(entry))

print()
print("=== C(eta, etadot) ===")
for i in range(3):
    for j in range(3):
        print(f"C[{i+1}][{j+1}] =", C[i, j])

# Correctness check: Mdot - 2C must be skew-symmetric (standard result for any
# C derived this way from a symmetric M(q)).
Mdot = sp.zeros(3, 3)
for i in range(3):
    for j in range(3):
        Mdot[i, j] = sum(sp.diff(M[i, j], q[k]) * qd[k] for k in range(3))

skew_check = sp.simplify(Mdot - 2 * C + (Mdot - 2 * C).T)
print()
print("=== skew-symmetry check (Mdot - 2C + (Mdot-2C)^T, should be all zeros) ===")
sp.pprint(skew_check)
