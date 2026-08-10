# A localized bump: where the vacuum physically touched something solid, and how
# it got there. Built by the bump_map node from the two bumper switches + TF.
#
# The bump map is the tactile keep-out layer: LiDAR/cameras see couch skirts, bed
# valances and curtains as solid, but only a physical contact proves something is
# truly solid, so a coverage planner can still clean under/through the rest.

std_msgs/Header header        # header.frame_id = map (or odom); stamp = bump time

geometry_msgs/Point contact   # estimated contact point on the obstacle surface
geometry_msgs/Point robot     # robot base_footprint at the bump
geometry_msgs/Point approach  # robot base_footprint a moment before the bump
float64 heading               # robot heading at the bump (rad, in header.frame_id)

uint8 side                    # which bumper switch(es) fired (see constants)
uint8 SIDE_BOTH=0
uint8 SIDE_LEFT=1
uint8 SIDE_RIGHT=2

# Richer signal from optical-interrupter bumpers; all 0.0 when only the binary
# left/right switches are available (current sim + simplest hardware). *_depress
# is 0..1 (how far that side is pressed); *_rate is the press rate (1/s). The
# left/right depress RATIO estimates the contact angle within the arc, replacing
# the geometric guess bump_map makes from binary switches.
float32 left_depress
float32 right_depress
float32 left_rate
float32 right_rate
