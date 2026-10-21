# Request sent by the receiver to the passer indicating that the receiver
# has controlled the ball. The passer uses this to transition out of
# PASSING_FINISHED and back to DEFAULT (so it does not chase the ball).
uint32 request_uid
uint8 from_robot_id
