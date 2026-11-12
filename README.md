# Autonav Ackermann Rover (v4.8)

A 3D-printed robot for the Autonomous Navigation Challenge. It steers like a car (Ackermann, MG996R servo) and drives the rear wheels from one JGA25-370 encoder motor through a printed differential. The RPLIDAR C1 sits low in the nose. **It uses no screws:** printed snap pins hold it together, and the electronics are glued or taped on.

## Start here

1. **See it in 3D:** download [`out/robot_viewer.html`](out/robot_viewer.html) (on that page, click the "Download raw file" button) and open it in a browser. It has a 14-step assembly walkthrough, a steering demo and the rulebook checks. It needs an internet connection to load the 3D library.
2. **Print:** open [`out/autonav_plates.3mf`](out/autonav_plates.3mf) in Bambu Studio. It holds 8 plates, already laid out flat. Print order, filaments and orientation notes are in [`PRINT_PLATES.md`](PRINT_PLATES.md). Using another slicer? Every part is in [`out/stl/`](out/stl).
   - **Print plate 0 (the fit coupon) first.** Push its pins into its holes. If they're too tight or too loose, change `FIT` in `robot.py`, rebuild and reprint before printing anything else.
3. **Dimensions:** [`out/handoff/autonav_robot.md`](out/handoff/autonav_robot.md) has the key measurements and every part's position and size in mm. [`out/handoff/autonav_robot.obj`](out/handoff/autonav_robot.obj) is the full assembly as a plain-text 3D model. Both are readable by people and by AI tools.
4. **Assemble:** follow the steps in the viewer. Have ready: superglue, double-sided tape, hot glue, a rubber band and a little grease.

Frame used everywhere: X right, Y forward, Z up, mm, with the origin on the floor under the rear axle.

## Key numbers

| | |
|---|---|
| Size (wheels straight) | 295.5 x 171 x 174.3 mm; 193.6 mm wide at full steering lock (limit: 300 mm cube) |
| Mass | about 343 g printed + 867 g bought = 1210 g (limit: 1500 g) |
| Wheelbase / track | 150 mm / 140 mm rear, 144 mm front |
| Lidar scan plane | 41.8 mm above the floor |
| Minimum turn radius | 452 mm |

## Changing the design

Everything is generated from `robot.py`. It needs Python 3 with `manifold3d`, `trimesh` and `numpy`:

```
pip install manifold3d trimesh numpy
python robot.py        # parts -> out/stl, checks -> out/report.json (fails if parts collide, gears don't mesh, etc.)
python make_plates.py  # Bambu Studio project -> out/autonav_plates.3mf
python make_handoff.py # dimension sheet + OBJ -> out/handoff/
python make_viewer.py  # 3D viewer -> out/robot_viewer.html
```

The knobs that matter after a test print are at the top of `robot.py`: `FIT` (pin/hole clearance), `SPLINE_FIT` (servo horn grip) and `BACKLASH` (gear play).

**Not verified yet:** none of this has been printed or assembled. The motor RPM, the MG996R flange height and the Brio camera clip are unmeasured. Current-limit the IBT-2 motor driver, because a stalled motor can strip the printed gears.
