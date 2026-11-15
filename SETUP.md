# Setup

How to get the project running on each of our machines. The repo holds the code; each machine builds its own
environment. Nothing compiled is committed.

## Repo layout

```
final_project/
├── Quadruped-PyMPC/   submodule: team fork of iit-DLSLab/Quadruped-PyMPC, branch meam5170
├── trackC/            course generator, friction check, contact logger (see trackC/README.md)
├── acados/            Mac only, built from source, ignored by git
├── data/              run logs and datasets, ignored by git
├── SETUP.md
└── .gitignore
```

Pinned versions (the combination known to work):

| Component | Version |
|---|---|
| Quadruped-PyMPC | `025eab40ea78599be0e3004c63f895d66e1b8b9a` (upstream main, 10 Nov 2026), on our fork's `meam5170` branch |
| acados (Mac source build) | `26ef6fee90d641b5ef45b8c4fb4648c848a7e27c` |
| Python (Mac) | 3.11 |

Upstream Quadruped-PyMPC changes often (its install method changed on 10 Nov 2026). Do not pull upstream into
`meam5170` without telling the team and rerunning the checks at the bottom of this file.

## 1. Clone (everyone)

```bash
git clone --recurse-submodules https://github.com/bplatoff/MEAM5170_Final_Project.git final_project
cd final_project
```

Already cloned without `--recurse-submodules`? Run `git submodule update --init`.

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

Arrow keys change the forward speed and turn rate, Space pauses. Robot, gait, MPC type and scene are set in
`quadruped_pympc/config.py`.

## 4. Check the trackC tools

From `trackC/`:

```bash
python visualize_course.py courses/debug_v03.yaml     # writes course.png
python terrain_gen.py courses/debug_v03.yaml          # installs the scene into gym_quadruped
python friction_check.py courses/debug_v03.yaml       # every surface within about 0.01 of its set friction
python smoke_test.py courses/debug_v03.yaml           # expected values are in trackC/README.md
```

If all four match the README, your setup is good. To walk on the course, set `'scene': 'debug_v03'` in
`Quadruped-PyMPC/quadruped_pympc/config.py`.

## Reference machine

Results can differ slightly between platforms (different acados builds, floating point). Numbers we report come
from the Linux desktop with the pixi environment. Long batch runs (many seeded courses for learning data) also
run there, headless, over SSH. Use the JetKVM only when the screen is needed.

## Working on the Quadruped-PyMPC fork

Changes to the controller (tube tightening, friction inputs) are committed inside the submodule, on `meam5170`,
and pushed to the fork. Then commit the new submodule pointer in the main repo:

```bash
cd Quadruped-PyMPC
git checkout meam5170
# edit, then
git commit -am "..." && git push
cd ..
git add Quadruped-PyMPC && git commit -m "Bump Quadruped-PyMPC" && git push
```

After pulling the main repo, run `git submodule update` so your submodule moves to the commit the team recorded.
