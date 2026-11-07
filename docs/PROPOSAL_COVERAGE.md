# Proposal coverage (status is based on code/test, not physical demonstration)

| Proposal item | Implemented here | Current validation |
|---|---|---|
| Ordered pickup / drop / return | Grid × deterministic task automaton synthesis | 8 unit tests, simulator |
| Collision-free navigation | 4-neighbor obstacle avoidance | Grid-world simulation |
| Multiple priority classes | Max-priority queue | Simulation; no live interrupts during carrying |
| Concurrent obstacle safety | Discrete invariant, STL G monitor | Unit tests |
| Bounded eventuality | Discrete step deadline and STL F monitor | Unit tests |
| Go-to-goal control | Differential-drive P controller | Kinematic unit test |
| Encoder odometry | `integrate_odometry` for measured wheel distances | Unit test, hardware input not wired |
| ArUco | Optional OpenCV detector | Not tested on camera hardware |
| Romi arm | Validated pulse envelope, abstract pick/drop 3-servo sequence | Mock only, joint poses not calibrated |
| TI-RSLK MAX motor interface | Guarded GPIO PWM motor diagnostic | Compiles only once copied into existing SDK; not tested on board |
| True general LTL synthesis | **NOT implemented** | Restricted ordered-event automaton only |
| Full continuous STL synthesis | **NOT implemented** | Monitors and bounded discrete constraint only |
| Real encoder+IMU fused navigation | **NOT implemented** | Requires hardware mapping/calibration |
| Physical safe delivery | **NOT demonstrated** | Requires hardware and integration |

The project is a research prototype. Do not present simulation success as proof of hardware readiness.
