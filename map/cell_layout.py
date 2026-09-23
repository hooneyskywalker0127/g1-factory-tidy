# The cell -- one 6 x 4 m corner of a plant, not a whole factory. Metres, world
# frame, floor top at z = 0, origin at the floor's centre.
#
# Two walls only (north and west). South and east are left open so a camera can
# look into the cell from outside without a wall in the way, and so the map
# stays the minimum that still reads as "indoors".
#
# The doorway is the one thing here that is sized off the robot rather than off
# the room: G1's shoulders are ~0.45 m across and a carried crate is 0.601 m on
# its long side, so 1.20 m clears both with room to turn. It is cut full height
# -- no header beam -- so there is nothing to duck under either.
FLOOR = (6.0, 4.0)          # x, y
WALL_H = 2.5
WALL_T = 0.10
DOOR_W = 1.20               # in the west wall
DOOR_CY = 0.0               # its centre along y

# Rack centres in x. Their y is NOT set here: the asset's origin is not
# documented, so build_cell.py measures each rack's bounding box and slides it
# back until its rear face sits RACK_GAP off the north wall's inner face.
RACK_X = (-0.55, 0.55)
RACK_GAP = 0.03


# --- the clutter ----------------------------------------------------------
# These are NOT in cell.usd. They are the things the robot moves, so they have
# to be rigid bodies, and a rigid body baked into the static shell is either
# frozen or dropped on load. props.py spawns them instead, off this table.
#
# (x, y, rotz) -- centre on the floor, degrees about z.
#
# One box, on purpose. The job is picking a thing off the floor and putting it
# on a shelf, and that is not easier to solve with eight things on the floor --
# it is the same problem plus a choice of which one. More get added back once
# one works.
#
# Its size comes off the hand that has to grasp it. The grasp poses are
# synthesised by Dexonomy, whose Dex3-1 templates are one-handed -- a large
# diameter wrap, a medium wrap, a four-finger prismatic pinch. Measuring the
# objects those actually succeeded on, the dimension the fingers close across
# ran 7 to 117 mm, while the long axis reached 339 mm: the type wraps one span,
# it does not enclose the whole object. 0.10 m across sits inside that.
#
# Mass follows the same source: Dex3-1's rated load is 500 g.
#
# An earlier version of this box was 0.30 x 0.30 x 0.32 m at 1.5 kg, sized to
# G1's 0.326 m palm separation for a two-handed press. That grip is not what the
# hand does, and neither the span nor the mass is within what it can hold.
FLOOR_BOX_SIZE = (0.20, 0.14, 0.10)      # w, d, h
FLOOR_BOX_MASS = 0.4
FLOOR_BOXES = (
    (-2.00, -1.20, 15.0),
)

FLOOR_CRATES = ()

# Two already on the shelves, so the goal state is visible in the start state.
# (rack index, which usable deck -- decks are read off the asset at spawn time,
# not hardcoded, because only the stage knows which ones have the headroom.)
DECK_CRATES = ((0, 0), (1, 1))


# Boxes stocked on the shelves.
#
# A deck is NOT one surface. The rack is double-sided: every deck is a front
# board and a back board with a ~53 mm gap between them, and a vertical wire
# partition standing in that gap. Anything centred on the deck is centred on
# that partition and starts the run inside it. Everything here sits on the
# FRONT board alone -- props.py lines each item's rear face up 10 mm short of
# the front board's rear edge (+1.441), which is also the side the robot
# reaches from.
#
# The front board is only 0.369 m deep, which rules the carton out by depth,
# not by height. Measured with map/probe_assets.py:
#   D1  0.380 x 0.250 x 0.149   two to a deck
#   D3  0.396 x 0.258 x 0.266   one to a deck
# CardBoxC (0.500 x 0.500) is dropped: on a 0.369 m board it hangs 134 mm off
# the front of the rack.
SHELF_BOX_FOOT = {"D1": (0.380, 0.250), "D3": (0.396, 0.258)}

# Pairs sit square (rotz 0) at +-0.23: 80 mm between them and 25 mm to each end
# of the 0.890 m board. Turning a pair even a few degrees eats that gap -- the
# one pair that was launched had 44 mm.
SHELF_BOXES = (
    (0, 1, -0.23,  0.0, "D1"),
    (0, 1,  0.23,  0.0, "D1"),
    (0, 2, -0.18,  5.0, "D3"),
    (0, 3,  0.18, -5.0, "D3"),
    (1, 0, -0.23,  0.0, "D1"),
    (1, 0,  0.23,  0.0, "D1"),
    (1, 2,  0.18,  5.0, "D3"),
    (1, 3, -0.18, -5.0, "D3"),
)

# The crate has a 90 degree turn baked into the asset, so it arrives 0.400 wide
# and 0.600 deep -- which no 0.369 m board takes. Shelved crates are turned back
# to put the long side across the rack. This is the footprint AS THE ASSET
# ARRIVES; the rotation below is what turns it.
CRATE_FOOT_SHELVED = (0.400, 0.600)
CRATE_SHELVED_ROTZ = 90.0
SHELF_CLEAR = 0.010          # gap left to the partition
