# Navigation

Following the person (phase 1), guiding them back (phase 2) and obstacle avoidance. Plain Python (no ROS imports).

| File | Contents |
|------|----------|
| `navigator.py` | `Navigator.step(Observation) -> Command`: the whole controller (tracker, obstacles, corners, state machine, controllers, VFH). Called by the ROS node and by the simulation. `Params` holds every tunable value. |
| `state_machine.py` | FOLLOW, LOST, SIGNAL, TURN_AROUND, GUIDE, WAIT_PERSON, DONE and the alert played when each is entered. |
| `controllers.py` | `FollowController` (direction-following, PD + feed-forward of the person's speed) and `GoToGoal` (Aicardi et al. 1995). |
| `vfh.py` | `VFH`: Vector Field Histogram with the VFH+ refinements. |
| `diff_drive.py` | `DiffDrive`: (v, ω) ↔ wheel speeds, dead reckoning. |

Tests: `test/test_navigation.py`.
