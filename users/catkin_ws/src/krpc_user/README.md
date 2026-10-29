# krpc_user: Team Vyoma-Niti-Mandala Mission Code (Kibo-RPC 7)

This folder contains **all of our team's code**. It flies the Int-Ball2 drone inside the "Kibo" module of the ISS, visits every checkpoint, and finishes at the goal. The aim is the highest possible score.

This guide is written for new team members. You need basic Python, but you do not need to know ROS or 3D math in advance.

---

## 1. The game in 60 seconds

1. The robot starts near the docking station.
2. The game gives us a list of **checkpoints**. Each checkpoint is a spot 0.5 m in front of an AR marker on the wall.
3. At each checkpoint the robot must:
   - be **close** to the checkpoint position,
   - **look at** the marker, so the marker is in the middle of the camera image,
   - have the right **roll**, meaning the right rotation around the line of sight.
4. When all three are good, we **report** the checkpoint. The judge then scores it Lv3 (best), Lv2, Lv1, or fail.
5. The **goal** is always checkpoint **12**. It is visited last, and then we report "mission finished".
6. **Faster = more points** (time bonus). The order of checkpoints therefore matters a lot.

The number of checkpoints depends on the difficulty level:

| Level | Checkpoints (goal included) |
|---|---|
| easy | 6 |
| normal | 8 |
| hard | 10 |
| veryhard | 12 |

---

## 2. How the judge really scores (read this before you change anything)

We read the simulator's own judge source code (`rootfs/krpc/judge_module/judge.py`). Four facts drive our whole design:

| Fact | What it means for our code |
|---|---|
| The judge takes **one snapshot** of the robot pose (`/sensor_fusion/navigation`) when the report arrives. | We read the same topic, so we can compute the score **before** reporting. |
| Each checkpoint is judged **only once**. A second report is ignored. | Never report too early. A bad report loses that checkpoint forever. |
| Final level = the **lowest** of distance / centering / roll. | All three must be good at the same time. |
| The judge reads the pose only ~5 times per second. | The robot must be **still** when we report, and stay still for a moment. |

The three conditions, with the usual limits:

| Condition | What is measured | Lv3 | Lv2 | Lv1 |
|---|---|---|---|---|
| Distance | robot center ↔ checkpoint position | ≤ 0.10 m | ≤ 0.20 m | ≤ 0.30 m |
| Centering | angle between camera direction and direction to the marker | ≤ 10° | ≤ 20° | ≤ 30° |
| Roll | rotation around the line of sight vs. the target attitude | ≤ 10° | ≤ 20° | ≤ 30° |

The real limits always come from the API (`score_bands`, `centering_bands`, `roll_bands`). The code never hard-codes them.

The attitude (`attitude.quaternion`) that the API gives for each checkpoint already points the camera at the marker with the correct roll. If we fly exactly to `position` + `attitude.quaternion`, all three conditions are perfect.

---

## 3. Which files can I edit?

### ✅ You MAY edit (our code)

```
users/catkin_ws/src/krpc_user/
├── README.md                     ← this guide
├── scripts/
│   ├── user_template.py          ← MAIN program (ROS node). Mission flow.
│   ├── krpc_config.py            ← ALL tunable numbers. Start here when tuning.
│   ├── krpc_judge_replica.py     ← our copy of the judge's scoring rules
│   ├── krpc_route_planner.py     ← chooses the fastest checkpoint order
│   ├── krpc_geometry.py          ← 3D math (vectors, quaternions)
│   └── krpc_vision.py            ← optional camera/ArUco debug tool (off by default)
├── test/                         ← offline tests (run without ROS / simulator)
└── src/user_template_node.cpp    ← official C++ example (NOT used, left untouched)
```

### ❌ Do NOT edit

| Path | Why |
|---|---|
| `users/catkin_ws/src/krpc_api_server/` | Official game API. The real competition uses its own copy. |
| `users/catkin_ws/src/krpc_ib2_msgs/` | Official message definitions |
| `users/bootstrap/`, `docker/`, `rootfs/`, `rviz/` | Simulator itself |
| File **names** `user_template.py`, `krpc_user.launch`, `CMakeLists.txt`, `package.xml`, package name `krpc_user` | The simulator looks for exactly these names |

---

## 4. Rules for our code (keep these or lose points)

1. **Three mandatory interfaces** in `user_template.py` (lines marked `DO NOT REMOVE`):
   - subscribe to `/ib2_user/start`,
   - provide the `/ib2_user/stop` service, which cancels all movement and answers `SUCCESS`,
   - set `rospy.set_param('/ib2_user/ready', True)` at the **end** of `__init__`.
2. **Never wait forever.** Every `wait_for_server`, `wait_for_service`, and result wait has a time limit from `krpc_config.py`.
3. **Never read `cheat_sheet.json` or any "answer" file/topic.** The simulator has an `Illegal` (cheating) detector. Checkpoint data must come only from `/mission/get_checkpoints`.
4. **Report a checkpoint only after `krpc_judge_replica` says it is good.** Remember that each checkpoint is judged only once.
5. **Quaternion order is `[x, y, z, w]`** everywhere, the same as ROS and the judge.
6. The movement result is `result.type == CtlCommandResult.TERMINATE_SUCCESS`. The field is called `type`, not `status`.
7. Our helper files start with `krpc_` so their names never clash with other Python modules.
8. The robot runs Python **3.6** (ROS Melodic). Do not use newer Python features such as dataclasses or the walrus operator `:=`. Only use `numpy`/`cv2` inside `krpc_vision.py`.

---

## 5. Architecture

### 5.1 Mission flow

```
 /ib2_user/start
        │
        ▼
 ┌─────────────── PREPARE (timer not running yet) ───────────────┐
 │ 1. wait for /ctl/command and /event/checkpoint_passed servers │
 │ 2. /mission/get_checkpoints  → krpc_judge_replica.parse_...   │
 │ 3. first /sensor_fusion/navigation message (start pose)       │
 │ 4. krpc_route_planner.plan_route → fastest order, goal last   │
 └───────────────────────────────────────────────────────────────┘
        │
        ▼
 /event/mission_start   ← the clock starts here
        │
        ▼
 for each checkpoint in the order:
   ├─ over time budget? → skip (never skip the goal)
   ├─ MOVE   /ctl/command MOVE_TO_ABSOLUTE_TARGET (position + attitude)
   ├─ VERIFY judge replica says Lv3 with safety margin
   │         + robot still + held for HOLD_SEC
   │      ├─ OK  → /event/checkpoint_passed
   │      └─ not OK → retry the move (aim camera from the real position)
   │                  after all retries: report only if level ≥ MIN_NOTIFY_LEVEL
        │
        ▼
 /event/mission_finished  ← the clock stops here
```

### 5.2 Who does what

| File | Job | Uses ROS? |
|---|---|---|
| `user_template.py` | Talks to ROS, runs the mission flow, handles stop/timeouts, logs everything | yes |
| `krpc_config.py` | Every tunable number | no |
| `krpc_judge_replica.py` | Parses the checkpoint list and computes distance/centering/roll levels exactly like `judge.py` | no |
| `krpc_route_planner.py` | Held-Karp algorithm: the exact fastest order (start → all → goal) | no |
| `krpc_geometry.py` | Vector and quaternion math (look-at, roll/twist, angles) | no |
| `krpc_vision.py` | Optional: finds the ArUco marker in the camera image (debug only) | yes |

Only `user_template.py` (and the optional vision module) depend on ROS. Everything else is plain Python, which is why it can be tested on any laptop.

### 5.3 Ideas used to get a high score

- **Planning before the clock starts.** All slow preparation happens before `mission_start`.
- **Exact best order.** Held-Karp finds the optimal route, not just a "good" one.
- **Judge replica.** We report only when we already know the result is Lv3, so no checkpoint is wasted.
- **Safety margin + hold time.** This covers the judge's delayed snapshot.
- **Smart retry.** If the robot stops slightly off target, we re-aim the camera from where it really is and keep the target roll (`look_at_with_roll`).
- **Time guard.** If time runs out, we skip remaining checkpoints and secure the goal.

---

## 6. How to work on the code

### 6.1 Run the offline tests (any computer with Python 3, no ROS)

From the `users/catkin_ws/src/krpc_user` folder:

```bash
python3 -m unittest discover -s test -v
```

The tests load the simulator's **real** `judge.py` and checkpoint generator from `rootfs/` and check that our numbers match them exactly. Run them after every change to the `krpc_*.py` files.

### 6.2 Run the local simulator (needs Docker)

From the repository root:

```bash
bash ./build.sh                       # only the first time (slow: it compiles OpenCV)
LEVELS=veryhard bash ./simulation.sh  # LEVELS = easy | normal | hard | veryhard
```

Watch it live at <http://localhost>. When the run ends, read:

| File | What to look at |
|---|---|
| `share/logs/latest/result.json` | `Illegal` must be `false`; every checkpoint `success: true`, `level: 3`; `goal.success: true`; `mission.duration_sec` (lower = better) |
| `share/logs/latest/user_programs_logs/` | Our log: route, `leg done ...`, `verify ...`, `summary ...` lines |

### 6.3 Package for the Web simulator

```bash
cd users
bash ./packing.sh vyoma_niti_mandala   # creates users/vyoma_niti_mandala.tar.gz
```

Upload the `.tar.gz` on the [Kibo-RPC web simulator](https://jaxa.krpc.jp/user-auth). The Web simulator runs on ARM64, like the real robot, so always confirm the final version there.

---

## 7. Parameter tuning (krpc_config.py)

Change **one** value at a time, run `veryhard` and `normal` a few times, and keep the change only if every checkpoint is still Lv3 and `duration_sec` went down.

| Parameter | What it does | Raise it → | Lower it → |
|---|---|---|---|
| `V_EST`, `W_EST`, `LEG_OVERHEAD` | Estimated speed used to choose the route order | — | — (set them to the measured values from `leg done` log lines) |
| `HOLD_SEC` | How long the good pose must last before reporting | safer | faster |
| `POS_MARGIN_RATIO`, `ANG_MARGIN_RATIO` | How much stricter than Lv3 we are | faster, riskier | safer, maybe slower |
| `V_STILL`, `W_STILL` | Speed that counts as "still" | faster, riskier | safer |
| `SETTLE_TIMEOUT` | Max wait for a perfect pose before retrying | more patient | faster retries |
| `MOVE_RETRIES` | Extra attempts per checkpoint | more chances | less time lost |
| `MISSION_TIME_BUDGET` | When to give up remaining checkpoints and go to the goal | more checkpoints | safer goal |
| `EARLY_NOTIFY` | Report before the movement command fully finishes | — | — (experimental, test carefully) |
| `ENABLE_VISION` | Log the marker position in the camera image | — | — (debug only) |

### Reading the log

```
route [3, 1, 7, ...], estimated 182.4s          ← chosen order
leg done id=3 attempt=0 result=0 move=14.20s dist=1.234m rot=95.0deg
verify id=3 ok=True settle=0.85s level=3 dist=0.004m(L3) center=0.3deg(L3) roll=0.2deg(L3)
notify checkpoint_id=3 passed (t=16.1s)
...
summary checkpoint 3: level=3 16.1s
mission time 190.2s
```

- `result=0` means `TERMINATE_SUCCESS`.
- `move` is the flight time and `settle` is the waiting time after the flight. Use them to tune.

---

## 8. Glossary

| Word | Meaning |
|---|---|
| ROS | Robot Operating System. Programs ("nodes") talk to each other through it. |
| Topic | A continuous stream of messages (e.g. the robot position 10×/s). You *subscribe* to it. |
| Service | A question → answer call (e.g. "give me the checkpoints"). |
| Action | A long task with a result at the end (e.g. "fly there"). It can be cancelled. |
| Callback | A function ROS calls automatically when a new message arrives. |
| Quaternion | 4 numbers `[x, y, z, w]` that describe an orientation. See `krpc_geometry.py`. |
| Pose | Position + orientation. |
| Leg | One flight from one checkpoint to the next. |
| Held-Karp | Dynamic-programming algorithm that solves the fastest-order problem exactly. |
