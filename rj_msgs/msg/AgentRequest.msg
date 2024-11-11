# 0 -> RESERVED FOR ACKNOWLEDGE ON RESPONSE
# 1 -> TestRequest
# 2 -> PositionResponse
# 3 -> PassResponse
# ACKNOWLEDGE REQUEST DOES NOT (AND SHOULD NOT) EXIST
IncomingBallRequest[<=1] incoming_ball_request
PassRequest[<=1] pass_request
KickerRequest[<=1] kicker_request
ScorerRequest[<=1] scorer_request
JoinWallRequest[<=1] join_wall_request
LeaveWallRequest[<=1] leave_wall_request
ResetScorerRequest[<=1] reset_scorer_request
BallInTransitRequest[<=1] ball_in_transit_request
TestRequest[<=1] test_request
PositionRequest[<=1] position_request
SeekerRequest[<=1] seeker_request
JoinMarkingRequest[<=1] join_marking_request
LeaveMarkingRequest[<=1] leave_marking_request