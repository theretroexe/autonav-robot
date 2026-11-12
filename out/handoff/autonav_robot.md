# Autonav Ackermann robot v4.8: dimensions for teammates and AI tools

Model file: `autonav_robot.obj` (plain-text Wavefront OBJ, same folder). Every part is its own `o <name>` object, already in its assembled position, so a bounding box of an object's `v` lines gives that part's position directly.

**Coordinate frame:** X right, Y forward, Z up, millimetres. Origin = floor directly under the rear-axle centre.  
**Units:** all numbers in mm, angles in degrees.

Front of the robot = +Y (lidar end). Right = +X. Parts ending `_L` are on the left (−X), `_R` on the right (+X).

## Key dimensions

| What | Value |
|---|---|
| Overall size, wheels straight (as modelled) L x W x H | 295.5 x 171.0 x 174.3 mm; x -85.5..85.5, y -34.0..261.5, z 0.0..174.3 |
| Envelope incl. full steering lock L x W x H | 295.5 x 193.6 x 174.3 mm (rule: 300 mm cube) |
| Mass printed / bought / total | 343 / 867 / 1210 g (rule: 1500 g) |
| Wheelbase | 150.0 mm (rear axle y=0, front axle y=150.0) |
| Track (wheel centre to centre) | rear 140.0 mm, front 144.0 mm |
| Wheel | Ø68.0 mm incl. 3.0 mm TPU tyre, 20.0 mm wide; axle height z=34.0 |
| Ground clearance (min) | 7.4 mm |
| Steering | Ackermann, MG996R; lock inner/outer 31.0° / 21.5°; min turn radius 452 mm; kingpins at x=±63.0, scrub 9.0 mm |
| Steering servo output shaft | x=0.0, y=146.4 (shaft points down) |
| Drive | JGA25-370 encoder motor -> printed crown/bevel diff, ratio 36:11 = 3.27; RWD |
| LIDAR (RPLIDAR C1) | centre x=0, y=223.0; base z=12.0; scan plane z=41.8 mm |
| Camera pan servo (SG90) shaft | x=0.0, y=212.0 on the mid deck |
| Bottom plate | z 17.5–21.5 mm |
| Mid deck | z 86.0–89.0 mm |
| Rear top deck (half length) | z 130.0–133.0 mm |
| Lower standoffs (plate -> mid deck), x,y | (-44, -22); (44, -22); (-46, 75); (46, 75); (-40, 172); (40, 172) |
| Upper standoffs (mid -> top deck), x,y | (-46, -8); (46, -8); (-46, 60); (46, 60) |
| IR line sensors (under plate), centre x,y | IR_front (40.0, 237.0); IR_rear (32.0, -14.0) |
| Raspberry Pi 4B on mid deck (taped) | board corner (-28.0, 35.0), 56 x 85 board, USB/Ethernet to the rear; its hole centres (reference only, no screws) (-24.5, 38.5); (-24.5, 96.5); (24.5, 38.5); (24.5, 96.5) |
| Fasteners | none bought: printed snap pins/pegs; electronics glued or taped |

## Printed parts (76)

Bounding box in the assembly frame (min corner, max corner) and size along X, Y, Z.

| Part | Subassembly | Min x, y, z | Max x, y, z | Size X x Y x Z | Mass g |
|---|---|---|---|---|---|
| cam_cradle | cam | -30.0, 199.0, 96.3 | 30.0, 225.0, 140.3 | 60.0 x 26.0 x 44.0 | 11.4 |
| axle_cap_L | chassis | -52.0, -11.0, 34.0 | -19.0, 11.0, 42.0 | 33.0 x 22.0 x 8.0 | 2.5 |
| axle_cap_R | chassis | 19.0, -11.0, 34.0 | 52.0, 11.0, 42.0 | 33.0 x 22.0 x 8.0 | 2.5 |
| axle_clip_front_L | chassis | -83.5, 143.5, 27.5 | -82.3, 156.0, 40.5 | 1.2 x 12.5 x 13.0 | 0.1 |
| axle_clip_front_R | chassis | 82.3, 143.5, 27.5 | 83.5, 156.0, 40.5 | 1.2 x 12.5 x 13.0 | 0.1 |
| axle_clip_rear_L | chassis | -81.5, -6.5, 27.5 | -80.3, 6.0, 40.5 | 1.2 x 12.5 x 13.0 | 0.1 |
| axle_clip_rear_R | chassis | 80.3, -6.5, 27.5 | 81.5, 6.0, 40.5 | 1.2 x 12.5 x 13.0 | 0.1 |
| bottom_plate_front | chassis | -48.0, 140.0, 17.5 | 48.0, 260.0, 21.5 | 96.0 x 120.0 x 4.0 | 15.5 |
| bottom_plate_rear | chassis | -52.0, -32.0, 17.5 | 52.0, 146.0, 21.5 | 104.0 x 178.0 x 4.0 | 33.4 |
| diff_case_crown | chassis | -3.0, -25.5, 8.5 | 25.5, 25.5, 59.5 | 28.5 x 51.0 x 51.0 | 12.8 |
| diff_cover | chassis | -25.5, -20.0, 14.0 | 0.0, 20.0, 54.0 | 25.5 x 40.0 x 40.0 | 13.0 |
| front_axle | chassis | -69.0, 130.0, 15.0 | 69.0, 162.0, 55.0 | 138.0 x 32.0 x 40.0 | 13.0 |
| half_shaft_L | chassis | -83.5, -5.0, 29.0 | -7.5, 5.0, 39.0 | 76.0 x 10.0 x 10.0 | 4.3 |
| half_shaft_R | chassis | 7.5, -5.0, 29.0 | 83.5, 5.0, 39.0 | 76.0 x 10.0 x 10.0 | 4.3 |
| horn_adapter | chassis | -9.0, 126.4, 30.6 | 9.0, 152.9, 39.0 | 18.0 x 26.5 x 8.4 | 1.6 |
| kingpin_L | chassis | -68.0, 147.9, 11.0 | -58.0, 155.0, 52.9 | 10.0 x 7.1 x 41.9 | 0.8 |
| kingpin_R | chassis | 58.0, 147.9, 11.0 | 68.0, 155.0, 52.9 | 10.0 x 7.1 x 41.9 | 0.8 |
| knuckle_L | chassis | -85.5, 126.4, 24.6 | -54.9, 156.0, 46.4 | 30.6 x 29.6 x 21.8 | 3.2 |
| knuckle_R | chassis | 54.9, 126.4, 24.6 | 85.5, 156.0, 46.4 | 30.6 x 29.6 x 21.8 | 3.2 |
| lidar_cradle | chassis | -33.0, 184.2, 9.6 | 33.0, 261.5, 36.7 | 66.0 x 77.3 x 27.1 | 12.0 |
| motor_mount | chassis | 0.9, 28.0, 21.5 | 45.9, 90.5, 47.0 | 45.0 x 62.5 x 25.5 | 7.9 |
| motor_pinion | chassis | 15.6, 18.5, 25.9 | 31.6, 26.5, 42.1 | 16.0 x 8.0 x 16.2 | 1.0 |
| pivot_pin_arm_L | chassis | -62.9, 128.8, 22.4 | -54.9, 134.4, 34.6 | 8.0 x 5.6 x 12.2 | 0.2 |
| pivot_pin_arm_R | chassis | 54.9, 128.8, 22.4 | 62.9, 134.4, 34.6 | 8.0 x 5.6 x 12.2 | 0.2 |
| pivot_pin_horn_L | chassis | -8.5, 128.8, 25.4 | -1.5, 133.9, 37.2 | 7.0 x 5.1 x 11.8 | 0.1 |
| pivot_pin_horn_R | chassis | 1.5, 128.8, 25.4 | 8.5, 133.9, 37.2 | 7.0 x 5.1 x 11.8 | 0.1 |
| rear_housing | chassis | -54.0, -14.0, 21.5 | 54.0, 14.0, 34.0 | 108.0 x 28.0 x 12.5 | 9.2 |
| side_gear_L | chassis | -11.2, -8.7, 25.3 | -2.5, 8.7, 42.7 | 8.7 x 17.5 x 17.5 | 0.7 |
| side_gear_R | chassis | 2.5, -8.7, 25.3 | 11.2, 8.7, 42.7 | 8.7 x 17.5 x 17.5 | 0.7 |
| snap_pin_00 | chassis | -25.5, -10.1, 13.5 | -18.5, -5.0, 44.0 | 7.0 x 5.1 x 30.5 | 0.4 |
| snap_pin_01 | chassis | -25.5, 6.9, 13.5 | -18.5, 12.0, 44.0 | 7.0 x 5.1 x 30.5 | 0.4 |
| snap_pin_02 | chassis | -51.5, -9.1, 13.5 | -44.5, -4.0, 44.0 | 7.0 x 5.1 x 30.5 | 0.4 |
| snap_pin_03 | chassis | -51.5, 5.9, 13.5 | -44.5, 11.0, 44.0 | 7.0 x 5.1 x 30.5 | 0.4 |
| snap_pin_04 | chassis | 18.5, -10.1, 13.5 | 25.5, -5.0, 44.0 | 7.0 x 5.1 x 30.5 | 0.4 |
| snap_pin_05 | chassis | 18.5, 6.9, 13.5 | 25.5, 12.0, 44.0 | 7.0 x 5.1 x 30.5 | 0.4 |
| snap_pin_06 | chassis | 44.5, -9.1, 13.5 | 51.5, -4.0, 44.0 | 7.0 x 5.1 x 30.5 | 0.4 |
| snap_pin_07 | chassis | 44.5, 5.9, 13.5 | 51.5, 11.0, 44.0 | 7.0 x 5.1 x 30.5 | 0.4 |
| snap_pin_08 | chassis | 1.9, 42.4, 13.5 | 8.9, 47.5, 30.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| snap_pin_09 | chassis | 37.9, 42.4, 13.5 | 44.9, 47.5, 30.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| snap_pin_10 | chassis | 1.9, 84.4, 13.5 | 8.9, 89.5, 30.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| snap_pin_11 | chassis | 37.9, 84.4, 13.5 | 44.9, 89.5, 30.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| snap_pin_12 | chassis | -30.5, 187.1, 13.5 | -23.5, 192.2, 30.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| snap_pin_13 | chassis | 23.5, 187.1, 13.5 | 30.5, 192.2, 30.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| snap_pin_14 | chassis | -30.5, 255.4, 13.5 | -23.5, 260.5, 30.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| snap_pin_15 | chassis | 23.5, 255.4, 13.5 | 30.5, 260.5, 30.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| snap_pin_16 | chassis | -39.0, 157.9, 13.5 | -32.0, 163.0, 52.9 | 7.0 x 5.1 x 39.4 | 0.5 |
| snap_pin_17 | chassis | 32.0, 157.9, 13.5 | 39.0, 163.0, 52.9 | 7.0 x 5.1 x 39.4 | 0.5 |
| snap_pin_18 | chassis | -9.5, 156.4, 15.5 | -2.5, 161.5, 32.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| snap_pin_19 | chassis | 2.5, 156.4, 15.5 | 9.5, 161.5, 32.0 | 7.0 x 5.1 x 16.5 | 0.2 |
| spider_cross | chassis | -1.6, -17.0, 23.0 | 2.0, 17.0, 52.5 | 3.6 x 34.0 x 29.5 | 0.7 |
| spider_gear_A | chassis | -8.5, -8.5, 36.5 | 8.5, 8.5, 45.2 | 17.0 x 17.0 x 8.7 | 0.5 |
| spider_gear_B | chassis | -8.5, -11.7, 22.9 | 8.5, -0.7, 37.6 | 17.0 x 11.0 x 14.8 | 0.5 |
| spider_gear_C | chassis | -8.5, 0.7, 22.9 | 8.5, 11.7, 37.6 | 17.0 x 11.0 x 14.8 | 0.5 |
| standoff_low_0 | chassis | -48.6, -26.0, 13.5 | -39.4, -18.0, 93.0 | 9.2 x 8.0 x 79.5 | 3.8 |
| standoff_low_1 | chassis | 39.4, -26.0, 13.5 | 48.6, -18.0, 93.0 | 9.2 x 8.0 x 79.5 | 3.8 |
| standoff_low_2 | chassis | -50.6, 71.0, 13.5 | -41.4, 79.0, 93.0 | 9.2 x 8.0 x 79.5 | 3.8 |
| standoff_low_3 | chassis | 41.4, 71.0, 13.5 | 50.6, 79.0, 93.0 | 9.2 x 8.0 x 79.5 | 3.8 |
| standoff_low_4 | chassis | -44.6, 168.0, 13.5 | -35.4, 176.0, 93.0 | 9.2 x 8.0 x 79.5 | 3.8 |
| standoff_low_5 | chassis | 35.4, 168.0, 13.5 | 44.6, 176.0, 93.0 | 9.2 x 8.0 x 79.5 | 3.8 |
| tie_rod_L | chassis | -62.9, 126.4, 27.6 | -1.0, 134.4, 30.4 | 61.9 x 8.0 x 2.8 | 1.3 |
| tie_rod_R | chassis | 1.0, 126.4, 27.6 | 62.9, 134.4, 30.4 | 61.9 x 8.0 x 2.8 | 1.3 |
| tire_front_L | chassis | -82.0, 116.0, 0.0 | -62.0, 184.0, 68.0 | 20.0 x 68.0 x 68.0 | 11.2 |
| tire_front_R | chassis | 62.0, 116.0, 0.0 | 82.0, 184.0, 68.0 | 20.0 x 68.0 x 68.0 | 11.2 |
| tire_rear_L | chassis | -80.0, -34.0, 0.0 | -60.0, 34.0, 68.0 | 20.0 x 68.0 x 68.0 | 11.2 |
| tire_rear_R | chassis | 60.0, -34.0, 0.0 | 80.0, 34.0, 68.0 | 20.0 x 68.0 x 68.0 | 11.2 |
| wheel_front_L | chassis | -82.0, 117.8, 1.8 | -62.0, 182.2, 66.2 | 20.0 x 64.4 x 64.4 | 9.9 |
| wheel_front_R | chassis | 62.0, 117.8, 1.8 | 82.0, 182.2, 66.2 | 20.0 x 64.4 x 64.4 | 9.9 |
| wheel_rear_L | chassis | -80.0, -32.2, 1.8 | -60.0, 32.2, 66.2 | 20.0 x 64.4 x 64.4 | 10.0 |
| wheel_rear_R | chassis | 60.0, -32.2, 1.8 | 80.0, 32.2, 66.2 | 20.0 x 64.4 x 64.4 | 10.0 |
| mid_deck_front | mid | -55.0, 128.0, 86.0 | 55.0, 236.0, 93.1 | 110.0 x 108.0 x 7.1 | 15.0 |
| mid_deck_rear | mid | -55.0, -31.0, 86.0 | 55.0, 134.0, 89.0 | 110.0 x 165.0 x 3.0 | 28.3 |
| standoff_high_0 | mid | -50.6, -12.0, 82.0 | -41.4, -4.0, 137.0 | 9.2 x 8.0 x 55.0 | 2.4 |
| standoff_high_1 | mid | 41.4, -12.0, 82.0 | 50.6, -4.0, 137.0 | 9.2 x 8.0 x 55.0 | 2.4 |
| standoff_high_2 | mid | -50.6, 56.0, 82.0 | -41.4, 64.0, 137.0 | 9.2 x 8.0 x 55.0 | 2.4 |
| standoff_high_3 | mid | 41.4, 56.0, 82.0 | 50.6, 64.0, 137.0 | 9.2 x 8.0 x 55.0 | 2.4 |
| top_deck | top | -50.0, -31.0, 130.0 | 50.0, 100.0, 133.0 | 100.0 x 131.0 x 3.0 | 16.6 |

## Bought parts (15)

Bounding box in the assembly frame (min corner, max corner) and size along X, Y, Z. Bought parts are simplified stand-ins (outer envelope only).

| Part | Subassembly | Min x, y, z | Max x, y, z | Size X x Y x Z | 
|---|---|---|---|---|
| Logitech_Brio_100 | cam | -36.5, 198.0, 120.3 | 36.5, 231.0, 174.3 | 73.0 x 33.0 x 54.0 |
| IR_front | chassis | 33.0, 221.0, 7.4 | 47.0, 253.0, 17.5 | 14.0 x 32.0 x 10.1 |
| IR_rear | chassis | 25.0, -30.0, 7.4 | 39.0, 2.0, 17.5 | 14.0 x 32.0 x 10.1 |
| JGA25_370_encoder_motor | chassis | 10.9, 21.5, 21.5 | 35.9, 97.0, 46.5 | 25.0 x 75.5 x 25.0 |
| LiPo_3S_2200 | chassis | -38.5, 16.0, 21.5 | -13.5, 122.0, 56.5 | 25.0 x 106.0 x 35.0 |
| MG996R | chassis | -16.6, 136.6, 35.5 | 37.3, 156.3, 82.4 | 54.0 x 19.7 x 46.9 |
| RPLIDAR_C1 | chassis | -27.8, 195.2, 12.0 | 27.8, 250.8, 53.3 | 55.6 x 55.6 x 41.3 |
| ESP32_S3_DevKitC | mid | -35.0, 6.5, 97.5 | 34.0, 31.9, 102.1 | 69.0 x 25.4 x 4.6 |
| IBT2_BTS7960 | mid | -33.0, 121.0, 92.0 | 17.0, 171.0, 134.0 | 50.0 x 50.0 x 42.0 |
| IMU_BNO085 | mid | -36.0, -28.0, 94.0 | -16.0, -3.0, 97.6 | 20.0 x 25.0 x 3.6 |
| Raspberry_Pi_4B | mid | -28.0, 33.0, 90.2 | 28.0, 120.0, 107.2 | 56.0 x 87.0 x 17.0 |
| SG90 | mid | -6.1, 190.4, 73.1 | 6.1, 222.6, 99.8 | 12.2 x 32.2 x 26.7 |
| buck_5V_Pi | mid | 29.0, 80.0, 94.0 | 55.0, 131.0, 105.6 | 26.0 x 51.0 x 11.6 |
| buck_6V_servo | mid | -10.0, -27.0, 94.0 | 33.0, -6.0, 105.6 | 43.0 x 21.0 x 11.6 |
| lidar_usb_adapter | mid | -55.0, 80.0, 94.0 | -31.0, 98.0, 99.6 | 24.0 x 18.0 x 5.6 |

## Notes

- Source of truth is `robot.py` (parametric). Regenerate: `python robot.py && python make_handoff.py`.
- Per-part print files: `out/stl/<part>.stl` (assembly pose; lay flat in the slicer) or `out/autonav_plates.3mf`.
- Printed clearances: FIT 0.25 mm radial on pins/bushings; tune after printing `00_FIT_TEST_COUPON_print_first.stl`.
- Not modelled: wiring, switches, glue/tape. Bought-part shapes are approximate envelopes; check real datasheets before cutting anything.
