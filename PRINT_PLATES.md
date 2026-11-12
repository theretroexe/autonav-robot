# Print plates — Bambu P2S (256 mm bed), v4.8

The STLs are in `out/stl/`. The grams are estimates from the CAD volumes, so trust the slicer's number.
The robot needs about **340 g**: 216 g PLA, 79 g PETG, 45 g TPU. The fit-test coupon adds 15 g of PETG.

| # | Filament | Plate | Parts (STL names) | Est. g |
|---|---|---|---|---|
| 0 | PETG | **Fit-test coupon, print this first** | `00_FIT_TEST_COUPON_print_first` | 15 |
| 1 | PLA | Bottom plate | `bottom_plate_rear`, `bottom_plate_front`, `tie_rod_L/R`, `axle_cap_L/R` | 57 |
| 2 | PLA | Mid deck | `mid_deck_rear`, `mid_deck_front`, `knuckle_L/R` | 50 |
| 3 | PLA | Brackets | `top_deck`, `front_axle`, `lidar_cradle`, `rear_housing`, `motor_mount`, `cam_cradle` | 70 |
| 4 | PLA | Wheels | `wheel_front_L/R`, `wheel_rear_L/R` | 40 |
| 5 | PETG | Gears + horn (0.12 mm layers) | `diff_case_crown`, `diff_cover`, `side_gear_L/R`, `spider_gear_A/B/C`, `motor_pinion`, `horn_adapter` | 30 |
| 6 | PETG | Pins, standoffs, shafts | `standoff_low_0..5`, `standoff_high_0..3`, `snap_pin_00..19`, `kingpin_L/R`, `pivot_pin_*` (x4), `spider_cross` (3-arm, lay flat side down), `axle_clip_*` (x4), `half_shaft_L/R` | 51 |
| 7 | TPU 95A | Tyres | `tire_front_L/R`, `tire_rear_L/R` | 45 |

## Orientation notes
- **Decks and plates:** print flat. The dovetail tabs fit with a 0.1 mm gap; superglue them.
- **Snap pins, kingpins and pivot pins:** lay each on its flat side, so the split prongs flex parallel to the bed.
- **Standoffs:** lay each on a hex flat. The end pegs may need small tree supports.
- **Half shafts:** stand them upright with a brim. They are round in the middle, so they can't lie flat.
- **Wheels:** spoke face down. Print the tyres flat in TPU.
- **Crown gear:** put the plain side (the side that joins the cover) down, so the teeth face up.
- **Front axle, knuckles and cam cradle:** these will probably need tree supports. Check the slicer preview.
- **Gears:** use 0.12 mm layers for accurate teeth. 0.16–0.20 mm is fine for everything else.
