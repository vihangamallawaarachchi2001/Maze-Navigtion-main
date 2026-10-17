# Audit progress — 17 October 2026

- Read user master task and source/assets; preserve original Git commit 1952a2e.
- User supplied image.png; inspected 0.5 m six-by-six layout, visually matches walls.
- Corrected portable environment, map override, paths, manifests, logging timeout.
- User installed native Jazzy/Harmonic via scripts/setup.sh; initial unavailable
  python3-torch apt attempt fixed to a local pip CPU torch virtual environment.
- Both packages build; eleven tests pass (evidence/logs/build.txt, tests.txt/xml).
- Fresh saved-policy grid evaluation: 1071/1071 train and 189/189 held-out successes,
  all shortest grid paths. results/current/dqn_eval; no new convergence claim.
- Default OGRE2 failed with Mesa EGL segmentation fault. OGRE fallback works on
  desktop Xwayland/llvmpipe. Live scan, odometry, map and Nav2 active verified.
- Three-goal baseline/DQN trial batch running; results/current/trials.csv.
- Real screenshot capture works using XComposite. Qt screen grabs were black;
  desktop GNOME screenshot DBus API denied access. Do not count black captures.
- Completed all six comparison trials: 3 A*, 3 DQN, all successful, no chassis contacts.
- Fresh SLAM tour completed 55/55 waypoints in 346 wall seconds; saved maps/current.
  First map save missed data; transient-local delivery and 10-second save timeout fixed it.
  Wall median/p95 error 0.012/0.056 m; reference free-space coverage 78.9%.
- Independent clean build passed. Explicit colcon pytest selection ran 8 package tests;
  root suite 11 passed. Initial colcon default discovered zero tests, not counted as pass.
- Fresh 300-episode smoke training: 4371 steps, 181 s; held-out 21/189 successes,
  168 loops. This weak checkpoint stays separate; supplied policy used for navigation.
- Actual RViz mouse tests: XTest ignored by Wayland; direct X11 mouse events through
  SetGoal worked. Three GUI scenarios all succeeded; CSV includes extra initial G1.
- Genuine XComposite screenshots include Gazebo, fresh SLAM, far DQN path and three
  clicked-goal navigating/final states. CSV/logs supply action-result evidence.
- Completed six-page report/PDF, VIVA notes, final verification, beginner summary.
- Remaining user actions: names/IDs, GitHub URL/upload, original PDF check, VIVA rehearsal.

