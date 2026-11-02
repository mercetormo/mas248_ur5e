# MAS248 Lecture 9 — Image targets and mock drawing

This package converts an image into TCP targets and supplies `run_picture` to
plan the drawing and optionally execute it on the MAS248 mock controller.

The converter extracts OpenCV contour **boundaries**, not pen-stroke
centrelines. Scaling uses the **full raster width**, including its margins;
height follows the pixel aspect ratio. No A4 or physical paper model is assumed.
The image-centred frame has X right, Y up and Z normal to the image.

## Measured workcell and bundled example

The workcell model is supplied by the `mas248_ur5e` package. Its drawing plane
is at **world Z = 0 m**, and its `tcp` frame is at the measured pen tip.
Pass `--center-z 0` explicitly when generating targets for this model.

The bundled `examples/pumpkin/targets.json` contains 12 closed contours and
1,615 poses. The full image width is 0.18 m, its world centre is `(0.55, 0, 0)` m,
and its TCP targets are at Z = 0.003 m: a 3 mm clearance above the drawing plane.
TCP +Z points down towards the plane. The preview is stored beside the JSON.
These coordinates are a mock drawing example, not a physical contact calibration.

## Start the mock simulation

Build from the workspace root:

```bash
cd ~/mas248_ws
colcon build --symlink-install --packages-select \
  mas248_project mas248_ur5e mas248_ur5e_demos mas248_ur5e_moveit_config
```

Prepare **each ROS terminal** with the same domain ID (group 19):

```bash
source /opt/ros/humble/setup.bash
source ~/mas248_ws/install/setup.bash
export ROS_DOMAIN_ID=19
```

Terminal 1: start the mock hardware and controllers, and leave it running.

```bash
ros2 launch mas248_ur5e_moveit_config ros2_control.launch.py
```

Terminal 2: start MoveIt and RViz, and leave it running. On the tested WSL
setup, software rendering avoids the D3D12 RViz crash:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch mas248_ur5e_moveit_config moveit.launch.py
```

Terminal 3: plan the complete bundled drawing. The default mode is **PLAN ONLY**.

```bash
ros2 run mas248_ur5e_demos run_picture \
  --targets ~/mas248_ws/src/mas248_ur5e/mas248_ur5e_demos/examples/pumpkin/targets.json
```

Add `--max-strokes 1` for a single-contour trial. To execute the complete drawing
on the mock controller:

```bash
ros2 run mas248_ur5e_demos run_picture \
  --targets ~/mas248_ws/src/mas248_ur5e/mas248_ur5e_demos/examples/pumpkin/targets.json \
  --execute
```

`run_picture` plans the approach and every selected contour before execution.
Each Cartesian segment transfers above the next start, descends, follows the
contour and retracts vertically. The default lift is 0.03 m. Planned endpoints
are propagated as the start states of subsequent segments.

Execution requires every Cartesian fraction to be complete, valid trajectory
timing, and the named `MAS248MockHardware` system using
`mock_components/GenericSystem`. Collision checking remains enabled. An absolute
0.5 rad joint-step check is applied to the returned Cartesian trajectories;
the relative, average-based jump filter is disabled for these densely spaced
drawing points. Velocity and acceleration scaling are both 10%.

The full 12-contour example has been planned and executed successfully on the
mock controller. This does not validate a physical robot or pen contact; real
execution uses the lab's real-driver setup and supervised validation.

## Generate targets for the measured model

```bash
ros2 run mas248_ur5e_demos image_to_targets pumpkin_outline.jpg \
  --width-m 0.18 \
  --center-x 0.55 --center-y 0 --center-z 0 \
  --tcp-offset-mm 3 \
  --max-contours 12 --max-points 200 \
  --output /tmp/mas248_targets_measured.json \
  --preview /tmp/mas248_preview_measured.png \
  --stages-dir /tmp/mas248_stages_measured \
  --no-popup
```

Use the generated JSON path with `run_picture --targets`. The stage viewer is
`/tmp/mas248_stages_measured/index.html`; `--open-stages` opens the HTML viewer
instead of an OpenCV popup.

The JSON stores `strokes[].poses` in world coordinates and
`strokes[].poses_image` in image-centred coordinates. To display target paths,
image bounds and TCP orientation arrows alongside the robot:

```bash
ros2 run mas248_ur5e_demos show_targets_rviz \
  --json /tmp/mas248_targets_measured.json
```

Markers use `/mas248/drawing_targets`, with the static transform
`world -> image_center`. The displayed rectangle represents the full image.

## Offline checks

From this package's source directory:

```bash
cd ~/mas248_ws/src/mas248_ur5e/mas248_ur5e_demos
python3 -m unittest discover -s tests -v
python3 scripts/run_picture.py \
  --targets examples/pumpkin/targets.json --validate-only
```

Offline checks cover image mapping, TCP orientations, target validation,
planning-state chaining, partial-path rejection and the mock execution guard.
Reachability and simulated execution are checked separately with MoveIt.
