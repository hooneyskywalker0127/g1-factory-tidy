# g1-factory-tidy

One Unitree G1 picking things up off the floor of a factory cell and putting them
on a rack. Structured only -- known objects, known places. The unstructured case
is not in this repository.

The robot is never handed the object's coordinates. Where the object is comes out
of the RGB-D camera in its head; the true position is read only to score how far
off the estimate was.

## What works

- Building the cell and viewing it
- Placing the things to be picked up as rigid bodies, on the shelves and on the floor
- Estimating a floor object's position and size from the head camera, scored against truth

## What does not

- Reaching for it. Neither the upper-body IK nor the lower-body balance controller is attached
- The robot does not pick anything up yet

## The cell

A 6 x 4 m floor with walls on the north and west sides only; south and east are
left open. The west wall has a 1.20 m doorway. G1's shoulders are about 0.45 m
across and a carried crate is 0.60 m on its long side, so that width clears both
at once. The opening is cut full height -- nothing to duck under.

Two racks stand against the north wall
(`Environments/Hospital/Props/SM_MedShelf_01d`).

The static parts -- floor, walls, racks -- are baked once into `map/cell.usd`.
The things to be picked up are not: a rigid body baked into a static shell is
either frozen as scenery or already falling on load. `map/props.py` places them
fresh on every run.

## Where the dimensions come from

The floor box is 0.30 x 0.30 x 0.32 m at 1.5 kg. Those numbers were measured off
the robot (`map/measure_g1_hands.py`): palms 0.326 m apart, 0.21 m in front of
the body. The grip is two hands pressing the side faces, so the box has to be
slightly narrower than that separation for the hands to press inward rather than
reach. At 0.32 m tall the grip point stands 0.16 m off the floor, clear of the
backs of the hands. The same box still fits a shelf (0.374 m between decks,
0.369 m of board depth). Anything bigger is easier to pick up and has nowhere to
go.

## The camera

The real G1 carries an Intel RealSense D435i in its head. Its geometry is kept as
specified -- 86 degree depth field of view, no depth closer than 0.40 m. Only the
resolution is dropped, to 424 x 240. That is not a spec violation: a policy gets
a downsampled image on the real robot too, and it is the one place a one-person
project can buy back render time.

The camera is pitched 30 degrees down. G1 has no neck joint -- head_link is fixed
to the torso -- and mounted level it cannot see its own feet: the floor only
enters the frame 1.5 m out. A box 0.9 m away sits on the bottom edge with its
near face cut off, which pushed the estimated centre 76 mm out. Pitched down, the
floor starts at 0.49 m, just outside the depth sensor's own near limit.

That 0.40 m limit is not implemented as a clipping plane. Clipping would delete
the colour image with it, and the real sensor still sees what is close -- it just
returns no depth. The frame is rendered normally and depth below 0.40 m is
discarded afterwards.

## How good the estimate is

Measured from five robot poses. The box was in the same place all five times.

| Robot stands at | Distance | Centre error | Height error |
|---|---|---|---|
| (-2.00, -0.40) facing it | 0.80 m | 6 mm | +2 mm |
| (-2.00, +0.30) facing it | 1.50 m | 8 mm | +3 mm |
| (-1.20, -1.20) from the side | 0.80 m | 1 mm | +2 mm |
| (-2.60, -0.55) diagonally | 0.86 m | 2 mm | +2 mm |
| (-2.00, -0.20) facing it | 1.00 m | 7 mm | +2 mm |

Three reasons not to read too much into that. Only the viewpoint changed, so
nothing here says the estimate survives the object moving. The floor is taken as
z = 0 rather than fitted, which a real robot would have to earn. And the walls
and racks are cut away using the cell's own coordinates -- the object's position
was never given, but the room's layout was.

## Files

| File | What it does |
|---|---|
| `map/build_cell.py` | Builds the static shell, saves `map/cell.usd` |
| `map/cell_layout.py` | Dimensions and placements, with the reasoning in the comments |
| `map/props.py` | Places the things to be picked up, as rigid bodies |
| `map/view_cell.py` | Viewer |
| `map/probe_assets.py` | Measures candidate assets instead of guessing from their names |
| `map/measure_g1_hands.py` | Measures G1's hands and shoulders |
| `map/look_from_g1.py` | One frame from the head camera |
| `map/find_box.py` | Estimates the floor object from depth, scores it against truth |
| `map/vision.py` | The estimation, on its own |
| `common/` | Rack, box and tray components, taken from humanoid-swarm-sim |

## Running it

Needs IsaacLab 2.3.2 / Isaac Sim 5.1.

```
conda activate env_isaaclab
python map/build_cell.py
python map/view_cell.py
```

## Traps

Things that actually caught us here.

- A shelf deck is not one surface. The rack is double-sided: each deck is a front
  board and a back board with a ~53 mm gap, and a vertical wire partition standing
  in that gap. Anything centred on a deck starts inside that partition and is
  launched on the first step. Put things on the front board alone, with their rear
  face 10 mm short of its rear edge.
- The crate asset has a 90 degree turn baked in, so its long side runs along y.
  It has to be turned to go on a shelf.
- G1 falls over in about a second without a balance controller. Measure and test
  perception with gravity off or the root fixed -- numbers taken off a robot lying
  on the floor mean nothing.
- `cam.data.pos_w` and `quat_w_ros` read all zeros on a cpu device, because Fabric
  is disabled there. Unprojecting depth with a zero quaternion turns every point
  into NaN, which empties the cloud silently instead of raising. Read the camera
  pose off the stage.
