# Internal state of the precision-land state machine (navigator/precland.cpp).
# Published every cycle while AUTO_PRECLAND is the active navigator mode, so
# it shows up in flight logs and can be watched live -- precland.cpp itself
# has no other logged/published trace of which phase it's in.

uint64 timestamp		# time since system start (microseconds)

uint8 STATE_START = 0			# just activated, checking for an immediately visible target
uint8 STATE_HORIZONTAL_APPROACH = 1	# centering at PLD_HOV_ALT above the target
uint8 STATE_DESCEND_ABOVE_TARGET = 2	# position-mode descent ratchet, hold, or re-approach
uint8 STATE_FINAL_APPROACH = 3		# close to the ground, descending regardless of target visibility
uint8 STATE_SEARCH = 4			# target not visible, climbing to PLD_SRCH_ALT and waiting for it
uint8 STATE_FALLBACK = 5		# gave up, landing normally at the current position
uint8 STATE_DONE = 6			# landed

uint8 state

uint8 MODE_OPPORTUNISTIC = 1	# only precision-land if the target was visible when activated
uint8 MODE_REQUIRED = 2	# search for the target if not immediately visible

uint8 mode

bool target_pose_valid		# whether a fresh, usable landing_target_pose is currently available
int32 search_count		# number of target-search/recovery attempts during this landing
