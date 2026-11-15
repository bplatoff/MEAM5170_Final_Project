# Setup

How to get the project running on each of our machines. The repo holds the code; each machine builds its own
environment. Nothing compiled is committed.

## Repo layout

```
final_project/
├── Quadruped-PyMPC/   submodule: upstream iit-DLSLab/Quadruped-PyMPC, pinned, never edited (see below)
├── controllers/       our controller code (tube MPC etc.), subclassing the upstream controller
├── planning/          global collocation planner and the tube error model (planning/robust_control/)
├── trackC/            course generator, friction check, contact logger (see trackC/README.md)
├── acados/            Mac only, built from source, ignored by git
├── data/              run logs and datasets, ignored by git
├── SETUP.md
└── .gitignore
```

Pinned versions (the combination known to work):

| Component | Version |
|---|---|
| Quadruped-PyMPC | `025eab40ea78599be0e3004c63f895d66e1b8b9a` (upstream main, 10 Nov 2026) |
| acados (Mac source build) | `26ef6fee90d641b5ef45b8c4fb4648c848a7e27c` |
| Python (Mac) | 3.11 |

## Quadruped-PyMPC is read-only

We never edit files inside `Quadruped-PyMPC/`. Upstream changes often (its install method changed on
10 Nov 2026), and keeping it pristine means the pin always describes exactly what we run.

- **Settings** (scene, friction, gait, MPC type) are set from our own scripts, on the config module, before calling
  `run_simulation`:

  ```python
  from quadruped_pympc import config as cfg
  cfg.simulation_params['scene'] = 'debug_v03'
  cfg.mpc_params['mu'] = 0.5
  ```

- **Controller changes** (per-foot friction, tube tightening, obstacle constraints) go in `controllers/` as
  subclasses of the upstream classes, e.g. `class TubeNMPC(Acados_NMPC_Nominal)`, swapped in after the wrapper is
  built: `wrapper.srbd_controller_interface.controller = TubeNMPC()`.

- `git status` in the main repo shows `modified content` next to `Quadruped-PyMPC` if someone edited it. Undo with
  `git -C Quadruped-PyMPC checkout -- .`

- Moving the pin to a newer upstream commit is a team decision. Rerun the checks in sections 3 and 4 afterwards, then
  commit the new pointer: `git -C Quadruped-PyMPC checkout <commit> && git add Quadruped-PyMPC && git commit`.

## 1. Clone (everyone)

```bash
git clone --recurse-submodules https://github.com/bplatoff/MEAM5170_Final_Project.git final_project
cd final_project
```

Already cloned without `--recurse-submodules`? Run `git submodule update --init`. After every `git pull`, run
`git submodule update` so the submodule sits at the pinned commit.

Use a path with no spaces. acados builds and generates C code, and spaces in the path break it.

## 2a. Linux or Windows (WSL): pixi

Quadruped-PyMPC's pixi environment only supports `linux-64`, which covers native Linux and WSL.

```bash
curl -fsSL https://pixi.sh/install.sh | sh          # once; then open a new terminal
cd Quadruped-PyMPC
pixi shell -e humble-no-cuda                         # first run downloads everything, including prebuilt acados
pip install shapely pyyaml                           # trackC extras (inside the pixi shell)
```

WSL notes:

- Clone inside the Linux home folder (`~/...`), not under `/mnt/c/`. That path is slow and causes permission
  problems.
- Windows 11 shows the MuJoCo window through WSLg with no extra setup.

No NVIDIA GPU is needed. We use the gradient-based MPC (acados, CPU). Only the sampling-based MPC uses CUDA.

## 2b. macOS (Apple Silicon): conda plus acados from source

pixi does not work on macOS for this project, because no macOS build of acados is packaged. Build it yourself.

```bash
# environment
conda create -n qpympc -c conda-forge python=3.11 numpy scipy matplotlib casadi pinocchio cmake make pkg-config tqdm gymnasium
conda activate qpympc

# acados, pinned, inside final_project/ (ignored by git)
git clone --recurse-submodules https://github.com/acados/acados.git
cd acados
git checkout 26ef6fee90d641b5ef45b8c4fb4648c848a7e27c && git submodule update --recursive
mkdir -p build && cd build
cmake .. && make install -j8
pip install -e ../interfaces/acados_template
cd ../..
```

Add these lines to `~/.zshrc` (adjust the path to where your `final_project` lives), then open a new terminal:

```bash
export ACADOS_SOURCE_DIR="$HOME/path/to/final_project/acados"
export DYLD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:$DYLD_LIBRARY_PATH"
```

Check: `ls $ACADOS_SOURCE_DIR/lib/link_libs.json` should print the file.

Then the remaining packages:

```bash
conda activate qpympc
pip install mujoco gym-quadruped liecasadi readchar "adam-robotics[casadi]" jax meshcat shapely pyyaml
pip install -e Quadruped-PyMPC
```

On the first simulation run acados asks to install `tera_renderer`. Accept.

macOS only: any script that opens the MuJoCo viewer must be started with `mjpython` instead of `python`
(macOS requires windows to run on the main thread). Scripts that don't render, such as the trackC checks, run with
plain `python`.

## 3. Run the demo

From `Quadruped-PyMPC/`, inside your environment (`pixi shell -e humble-no-cuda`, or `conda activate qpympc`):

```bash
python simulation/simulation.py      # Linux / WSL
mjpython simulation/simulation.py    # macOS
```

Arrow keys change the forward speed and turn rate, Space pauses. This runs upstream's defaults
(`random_boxes` scene). To try other settings, do it from a script of ours as shown above, not by editing
`config.py`.

## 4. Running our code

The same four rules apply to every Python file in this repo (`trackC/`, `planning/`, `controllers/`), whether it
is a test, a check or a full simulation.

**Rule 1: use the project environment, never the system Python.** The environment is not stored in the repo. It
lives on your machine and is built once in section 2. Activate it in every new terminal:

```bash
# macOS
conda activate qpympc

# Linux / WSL: pixi opens a new shell, so type these one at a time
cd Quadruped-PyMPC
pixi shell -e humble-no-cuda
cd ..
```

Check you are in it: `python -c "import casadi, mujoco, gym_quadruped, acados_template; print('ok')"` prints `ok`.

**Rule 2: run from the repo root, with `python -m`.** Give the module by its dotted path, without `.py`:

```bash
cd final_project
python -m planning.test_planner --scenario multi
```

Do not `cd` into a subfolder and run `python some_file.py`. That only works for files with no imports from the
rest of the repo.

**Rule 3: imports are package style,** written as the path from the repo root:

```python
from planning.direct_collocation import plan_trajectory
from planning.robust_control.tube_error_model import design_tube_error_model
from trackC.terrain_gen import build
from trackC.contact_logger import ContactLogger
```

This lets one script use the planner, the tube model, the course generator and the logger together. Every code
folder has an (empty) `__init__.py`; add one when you create a new folder. File paths given as arguments are also
relative to the repo root (`trackC/courses/debug_v03.yaml`).

**Rule 4 (macOS only): scripts that open the MuJoCo viewer start with `mjpython` instead of `python`,** with the
same arguments: `mjpython -m <module>`.

### Checks and tests

```bash
# Course and logger (trackC). Expected values are in trackC/README.md.
python -m trackC.visualize_course trackC/courses/debug_v03.yaml     # writes course.png
python -m trackC.terrain_gen trackC/courses/debug_v03.yaml          # installs the scene into gym_quadruped
python -m trackC.friction_check trackC/courses/debug_v03.yaml       # every surface within about 0.01 of its set friction
python -m trackC.smoke_test trackC/courses/debug_v03.yaml           # four physics checks of the logger

# Planner (planning). Add --no-plot to skip the figures.
python -m planning.test_planner --scenario empty
python -m planning.test_planner --scenario single
python -m planning.test_planner --scenario multi

# Tube error model
python -m unittest discover -s planning/robust_control -p "test_*.py" -v
```

New tests go next to the code they test, in files named `test_*.py`, and must run from the repo root as above.

## Reference machine

Results can differ slightly between platforms (different acados builds, floating point). Numbers we report come
from the Linux desktop with the pixi environment. Long batch runs (many seeded courses for learning data) also
run there, headless, over SSH. Use the JetKVM only when the screen is needed.
