# xgo_localization_demo

> Developed as part of the course **Master Project: Distributed Systems** at
> [DOS](https://www.tu.berlin/dos), TU Berlin.

A runnable demo for [`xgo_localization`](https://github.com/luckyTamme/xgo_localization):
drive the XGO2 and watch it localise, record a session, and replay that session
through either SLAM backend — on the robot or on a laptop.

Nothing is built. Both images are published, so every command below is a pull.

- **The node itself lives in the other repository.** This one is deployment:
  Docker, compose, drivers, and a viewer layout.
- Viewer is [Lichtblick](https://github.com/lichtblick-suite/lichtblick).

---

## Quickstart

```bash
git clone https://github.com/luckyTamme/xgo_localization_demo
cd xgo_localization_demo
```

**On the robot:**

```bash
docker compose --profile live up
```

**On any machine, replaying a bag:**

```bash
BAG=./bags/run docker compose --profile replay up
```

Then open Lichtblick, connect to `ws://<robot>:8765` (or `ws://localhost:8765`
for replay), and load [`lichtblick/localization.json`](lichtblick/localization.json).

---

## The workflow this is built around

Record once on the robot, replay many times at a desk. A recording holds **raw
sensor inputs only** — never the SLAM output — so the same bag can be pushed
through either backend afterwards and the results compared.

```bash
# 1. on the robot: drive and record
RECORD=true docker compose --profile live up
#    Ctrl-C when done  ->  ./bags/run/

# 2. copy it to your machine
rsync -avP <user>@<robot>:~/xgo_localization_demo/bags/run ./bags/

# 3. replay it, either backend
BAG=./bags/run                  docker compose --profile replay up
BAG=./bags/run BACKEND=rtabmap  docker compose --profile replay up
```

### Looping a replay

`LOOP=true` restarts the bag when it ends, which is handy for an unattended
display. It does distort one thing, so know it before you read anything into it:
on each restart the odometry detects the backward time jump and resets to the
origin, while the SLAM backend keeps its accumulated map. `map -> odom` then has
to absorb the whole discrepancy in a single step, so the `odom` frame visibly
snaps across the map. Localisation is fine; the correction is not, and anyone
watching TF will reasonably think something is broken.

Use a single pass at a slower `RATE` when you actually want to judge the output.

### Bag size

The camera dominates a recording by roughly an order of magnitude — on a
seven-minute run it is about 90 % of the bytes — and contributes nothing to
localisation. So it streams to the viewer but is **excluded from recordings by
default**:

| | ~7 min run |
|---|---|
| `RECORD_CAMERA=false` (default) | ~25 MB |
| `RECORD_CAMERA=true` | ~575 MB |

---

## Settings

All are environment variables read by `compose.yaml`.

| variable | default | meaning |
|---|---|---|
| `BACKEND` | `cartographer` | `cartographer` or `rtabmap` |
| `BAG` | `./bags/run` | bag to replay (replay profile) |
| `RATE` | `1.0` | replay speed |
| `LOOP` | `false` | restart the bag when it ends — see the caveat below |
| `RECORD` | `false` | record raw inputs while running live |
| `RECORD_CAMERA` | `false` | include the camera in recordings |
| `CAMERA` | `true` | run the camera at all |
| `ROS_DOMAIN_ID` | `42` | DDS domain |
| `IMAGE` | published image | override to run a locally built one |

---

## Images

| tag | platforms | contents |
|---|---|---|
| `ghcr.io/luckytamme/xgo-localization-demo:latest` | amd64 + arm64 | node, both backends, bag playback, viewer bridge |
| `ghcr.io/luckytamme/xgo-localization-demo:robot` | arm64 | the above plus XGO driver, LiDAR driver, Pi camera |

Two tags rather than one multi-arch tag because the robot half needs a Raspberry
Pi libcamera build and the XGO drivers, which have no amd64 equivalent. A single
tag whose contents differ per platform is far harder to reason about than two
honest ones.

Everything upstream is pinned by commit SHA — the node, the XGO driver fork, the
LiDAR driver, `cxx_serial`, and the base image. Tags move; SHAs do not. To take a
newer node, bump `NODE_SHA` in the [`Dockerfile`](Dockerfile); the change shows up
in the diff and CI republishes.

---

## First-time robot setup

Once per robot, on the Pi host — not in a container.

**1. Free the UART from Bluetooth.** The XGO mainboard sits on GPIO 14/15, which
the PL011 UART drives. Add to `/boot/firmware/config.txt`:

```
dtoverlay=disable-bt
```

**2. Enable the serial hardware, disable the login shell on it:**
`raspi-config` → Interface Options → Serial Port → login shell **no**, hardware
**yes**.

**3. Enable the camera.** In `/boot/firmware/config.txt`:

```
camera_auto_detect=1
dtoverlay=ov5647
```

**4. Install Docker and let your user run it:**

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"   # log out and back in
```

Reboot after the `config.txt` edits, then check the devices exist:

```bash
ls -l /dev/ttyAMA0 /dev/ttyUSB0 /dev/video0
```

`docker compose up` fails outright if a device listed in `compose.yaml` is
missing. Without the camera fitted, remove the `/dev/video*`, `/dev/media*` and
`/dev/dma_heap/*` entries and run with `CAMERA=false`.

---

## Verifying

```bash
docker compose --profile replay exec replay bash -lc 'ros2 topic list'
docker compose --profile replay exec replay bash -lc 'ros2 topic echo /diagnostics --once'
```

You want `level: 0` and a message like `map->odom fresh (0.03 s old)`. `level: 1`
for the first few seconds is normal while the backend converges; `level: 2` means
no IMU is arriving and the pose is frozen.

### The Lichtblick layout

[`lichtblick/localization.json`](lichtblick/localization.json) has four panels: a
3D view (occupancy grid, laser scan, and the `map` and `base_laser` frames), the
diagnostics summary, the raw pose, and the camera. Import it after connecting.

The `odom` frame is hidden by default because when everything is healthy it sits
on top of `map` and adds nothing. Switch it on when a backend misbehaves — the
gap between `map` and `odom` *is* the correction, so it is the first thing worth
looking at.

See the [node's README](https://github.com/luckyTamme/xgo_localization#health-and-diagnostics)
for the full diagnostics schema and a troubleshooting table.

---

## Gotchas

- **`ipc: host` is not decoration.** Without it FastDDS' shared-memory transport
  fails silently: `ros2 topic info` shows publisher and subscriber matched while
  nothing is ever delivered.
- **Host networking is robot-only.** Docker Desktop on macOS does not support it
  and it is unreliable on Windows, which is why the replay profile publishes port
  8765 instead.
- **Replay sets `use_sim_time` for you**, and the bag is played with `--clock`.
  Mixing those up produces a pose that is wrong without erroring.
- **rtabmap keeps a database between runs.** The launch passes
  `--delete_db_on_start`, so each run starts fresh.

---

## Developing against it

Iterate on the node without rebuilding — it is pure Python, so mount your
checkout over the installed copy in a `docker-compose.override.yml` (gitignored).
Note the path: the workspace is built with `--merge-install`, so packages live in
a shared `lib/`, not a per-package one. Mounting the wrong path silently creates
an unused directory and the container keeps running the baked-in code.

```yaml
services:
  replay:
    volumes:
      - ../xgo_localization/xgo_localization:/ws/install/lib/python3.12/site-packages/xgo_localization:ro
```

Changing deployment instead? Build locally and point compose at it:

```bash
docker build --target core -t xgo-localization-demo:dev .
IMAGE=xgo-localization-demo:dev BAG=./bags/run docker compose --profile replay up
```

When a node change is ready: push `xgo_localization`, bump `NODE_SHA` here, and
CI republishes both images.
