"""Hand-written optimization solvers, replacing the CasADi/IPOPT dependency.

    qp.py   convex QP -- finds a KKT point directly (interior-point)
    sqp.py  general NLP -- reduces to a sequence of the above

Both are skeletons at present; see docs/writing_the_solver.md.
"""
