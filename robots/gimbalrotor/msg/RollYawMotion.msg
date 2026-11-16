# One replaceable motion request in an enabled contact session. Units: rad, rad/s.
# epoch comes from motion_epoch; STOP revokes it. source is unique per client run.
uint64 session
uint64 epoch
string source
uint64 sequence
uint8 ALPHA=0
uint8 ROLL=1
uint8 YAW=2
uint8 BODY_PITCH=3
uint8 VELOCITY=4
uint8 STOP=5
uint8 JOG=6
uint8 mode
float64 value
# BODY_PITCH frame identity, chosen by the source on each entry to target mode.
uint64 frame
# Positive speed limit for BODY_PITCH. Its value is an absolute offset from the
# body's local-Y frame captured when this source enters body-pitch target mode.
float64 rate_limit
