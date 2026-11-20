# Implementation and reproducibility

## Sources and scope

The mathematical reference is Y. Shoukry, P. Nuzzo, A. Balkan, I. Saha, A. L. Sangiovanni-Vincentelli, S. A. Seshia, G. J. Pappas, and P. Tabuada, *Linear Temporal Logic Motion Planning for Teams of Underactuated Robots Using Satisfiability Modulo Convex Programming*, IEEE CDC 2017, [DOI 10.1109/CDC.2017.8263808](https://doi.org/10.1109/CDC.2017.8263808). The source archive is [SatEX.zip](https://yshoukry.bitbucket.io/SatEX/SatEX.zip). The local archive was inspected without running its old Python 2 code or bundled Z3 binaries.

The archive's `examples/LTL Motion Planning/ScalabilityLTLMotionPlanning.py` implements single-robot 3D reach-avoid planning with CPLEX and an old Z3 distribution. It does not contain the paper's generic LTL, team-level propositions, collision avoidance, or loop construction. This project is a new implementation of that motion-planning method and its extensions, not a drop-in replacement for every SatEX API or every application in the ZIP. In particular, generic quadratic convex clauses, secure-state-estimation experiments, and SAT benchmark datasets are outside this implementation.

## Boolean and continuous planning

At each labeled sample, each robot occupies exactly one allowed region. The discrete search enforces adjacency and the mission. For each robot pair and sample, a selected signed coordinate separation implements the infinity-norm requirement `max(abs(p_i-p_j)) >= epsilon`. State evolution, bounds, region membership, selected separation constraints, and any loop closure are affine constraints.

The `smc` backend repeatedly asks Z3 for a discrete candidate and checks it using HiGHS through SciPy's linear-programming API. Confirmed infeasible enabled constraint groups produce a learned Boolean clause. `full` rejects all enabled groups together; `prefix` searches for a shorter infeasible prefix; `deletion` removes groups while infeasibility remains. Deletion filtering targets an irreducible conflict, not necessarily the smallest possible conflict; resource limits may leave a larger valid conflict. Timeouts, numerical errors, and unknown results never justify new infeasibility clauses.

The `smt` backend encodes the bounded Boolean and linear-real problem directly in Z3 and provides a useful correctness/performance comparison on small instances. It is not the original paper's full nonlinear Z3/dReal experiment. Neither backend promises identical runtime, memory usage, trajectory shape, or discrete-plan ordering across dependency versions and machines.

The SMC continuous solve minimizes the sum of absolute control inputs for the selected Boolean plan. It does not compare all possible Boolean plans to certify globally minimum control effort. Feasibility and satisfaction are the key results; different valid trajectories may have different costs.

## Time and LTL semantics

A horizon `H` has labeled samples `0,...,H-1`. Finite mode has `H-1` input intervals and uses finite-trace semantics, including strong next: `X p` is false at the last sample. For example, a finite `G F p` can be satisfied by a final visit; it does not imply perpetual visits after the result ends.

Lasso mode adds a physical endpoint `x[H]` and one more input interval. It chooses a loop-start index `l` and enforces `x[H] == x[l]` for every coordinate of every robot's state. The resulting infinite trace is `0,...,l-1,(l,...,H-1)^omega`, replaying the corresponding controls. Matching position alone would be invalid for robots with nonzero velocity or higher derivatives. Until and release must use their correct least/greatest fixed-point semantics on the cycle; recurrence equations alone are insufficient.

The result includes `loop_start`, region labels for the `H` unique samples, full state trajectories, and controls. The validator recomputes dynamics, membership, bounds, separation, loop equality, and LTL satisfaction independently of the encoded solver model.

Lasso closure is a sufficient representation for periodic solutions. It is not complete for every infinite dynamically feasible mission. A horizon search only investigates the configured finite range, and exhaustion of that range cannot establish unbounded infeasibility. An unknown result is distinct from bounded unsatisfiability.

## Dynamics and numerical interpretation

User-supplied matrices define a discrete-time linear model exactly as provided. The integrator helper uses exact zero-order-hold discretization: a chain of order `q` has `A[i,j] = dt**(j-i)/(j-i)!` for `j >= i` and `B[i] = dt**(q-i)/(q-i)!`. Bounds and initial/final conditions are explicit in the saved problem, making each result reproducible.

The optional `Robot.terminal_state` mask constrains individual endpoint coordinates; `None` (JSON `null`) leaves a coordinate unconstrained. The reach-avoid, crossing, and 3D examples stop their derivative states at the endpoint. In lasso mode the endpoint is `x[H]`, so a terminal constraint also applies to `x[loop_start]` through full-state closure.

The original archive instead combines an exact matrix exponential for A with `B=[0,...,dt]`, which is a different discrete model from exact zero-order-hold integration. Its code also contains inconsistent final-derivative dimensions, an omitted final-sample obstacle prohibition, incomplete dwell-time support, and a Boolean-vector/region-index output mismatch. The new implementation validates dimensions and treats all labeled samples consistently rather than preserving those defects.

HiGHS computes in floating point. Constraint feasibility is checked again with the validator's stated tolerance, which is recorded with the result. A numerically validated trajectory is not an exact rational certificate or a formal proof under all hardware/model uncertainty. Z3's exact linear-real search still needs numerical conversion before plotting/export and independent numerical checks.

## Collision and obstacle guarantees

All collision constraints apply at discrete sample instants. The infinity-norm clearance uses point positions. Region membership is defined by closed halfspaces, so shared boundaries may belong to multiple regions; the selected region label supplies the discrete interpretation. Finite-size robots need inflated obstacle geometry and a suitable separation margin.

Adjacency and sampled collision separation do not prove that paths are safe between samples. In particular, linear interpolation in the viewer is only a display convention and is generally not the continuous integrator trajectory. No continuous-time collision certificate, robust feedback controller, sensor uncertainty model, or physical-robot execution stack is supplied.

## Examples and benchmark interpretation

The supplied scenarios cover reach-avoid planning, two robots crossing a shared workspace, repeated independent patrols, synchronized repeated team visits using a counting predicate, 3D planning, and an adaptation of the legacy unit-grid scenario. They are intentionally small runnable examples of the method. They should not be described as exact reproductions of the paper's Figure 2/3 geometry.

From the paper text, the single-robot experiments use coordinate-wise integrator chains, `dt=0.5 s`, input bound `0.2`, initial position `(0.5,0.5)`, target `(5.5,2.0)`, and zero initial/final higher derivatives. The visit mission is a conjunction of eventual region visits. The multi-robot crossing examples use separation `epsilon=0.2 m`. The recurring mission requires all robots to visit region 1 together infinitely often, the first half to visit region 2 together infinitely often, and the second half to visit region 3 together infinitely often. The full polygon coordinates and experimental scripts are absent from the shared ZIP; published table values cannot be reproduced from that archive alone.

The original 3D scalability example varies x tick count from 6 through 20, with y/z tick counts 4. Unit cells at even x-cell indices and z-cell index 1 or 3 are obstacles. It has fourth-order translation chains and a second-order yaw chain, sample time 1 s, acceleration bound 0.5 per coordinate, and a bound of 1 on consecutive jerk differences. This project's legacy scenario keeps the obstacle-layout idea and explicit modern dynamics; consult its saved JSON for the precise runnable configuration.

The CLI benchmark compares the two implemented backends on the same saved small problems. JSON/CSV record statuses, horizons, elapsed times, and validation results. These are local measurements, not evidence for the original paper's claimed speedups over Syclop RRT/EST. A fair replication would require the original geometry, full planner configurations, matching time budgets, multiple trials, and preserved machine/dependency metadata.

## Dependencies

Runtime dependencies are NumPy, SciPy/HiGHS, Z3, and Matplotlib. `uv.lock` pins the tested dependency resolution. Development checks use pytest and Ruff. The HTML viewer embeds its data, scripts, and styling directly, makes no network requests, and requires no web server. See [SciPy linear programming](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.linprog.html), [Z3](https://github.com/Z3Prover/z3), and [HiGHS](https://highs.dev/) for the solvers' upstream documentation.
