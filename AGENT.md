# FlexivTDKCollect Agent Workflow

## Scope

- Target host: `XenseShuCaiLaptop`.
- Repository: `/home/xense/haptic_exo_teleop_ws/shihao/FlexivTDKCollect`.
- Modify only this repository when working on Flexiv TDK collection.
- Treat all existing dirty and untracked files as user work. Read the relevant
  complete diff before editing and never revert unrelated changes.
- Do not commit or push automatically. Ask the user before either operation.
- Never recursively or batch delete files. Delete only one explicit file path
  after confirming that it is safe.

## Environment

- SSH entrypoint:

  ```bash
  ssh -o BatchMode=yes XenseShuCaiLaptop
  ```

- Python interpreter:

  ```text
  /home/xense/miniconda3/envs/haptic_exo_env/bin/python
  ```

- Run scripts from the repository root. The source tree is imported directly;
  editable installation is not required.
- The remote host does not provide `rg` or `apply_patch`. Use bounded
  `find`, `grep`, and `sed` for inspection. Apply exact patches with
  `git apply --check` followed by `git apply` over SSH standard input.
- Send complex remote commands to `bash -s` over SSH standard input. Avoid
  nested PowerShell, SSH, Bash, and Python quoting.
- Keep transmitted patches and temporary command scripts ASCII. PTY transport
  can corrupt non-ASCII text.
- Do not place temporary scripts, patches, or test output in the repository.

## Baseline

Perform one concise baseline check at the start of a task:

```bash
cd /home/xense/haptic_exo_teleop_ws/shihao/FlexivTDKCollect
git branch --show-current
git rev-parse HEAD
git status --short
```

Also inspect the complete diff for files relevant to the requested change.
Do not repeat stable environment probes during the same task.

## Software Validation

Validate from cheap to expensive without importing or opening hardware unless
the user explicitly authorizes it:

1. Parse changed Python files with `ast.parse`.
2. Run `git diff --check`.
3. Run the mock test suite:

   ```bash
   PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
     /home/xense/miniconda3/envs/haptic_exo_env/bin/python \
     -m unittest discover -s scripts/tests -v
   ```

Report software validation separately from real-hardware validation.

## Hardware Safety

- Do not run collection, homing, force-zeroing, teleoperation, camera, gripper,
  or tactile scripts without explicit user authorization and workspace safety
  confirmation.
- Do not stop or alter another user's process or device connection.
- RDK and TDK robot connections are exclusive. Close both temporary RDK robot
  wrappers and release their underlying pybind robot objects before creating
  the TDK teleoperation object.
- Robot preparation owns fault clearing, enabling, waiting for operational
  state, switching the configured Tool, and zeroing the force/torque sensor.
  Every wait must be bounded and faults must abort the operation.
- `TransparentCartesianTeleopLAN.Start()` starts the TDK control loop but does
  not permit leader-follower motion. Motion requires
