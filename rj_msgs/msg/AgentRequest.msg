# 0 -> RESERVED FOR ACKNOWLEDGE ON RESPONSE
# 1 -> TestRequest
# 2 -> PositionResponse
# 3 -> PassResponse
# ACKNOWLEDGE REQUEST DOES NOT (AND SHOULD NOT) EXIST
ResetScorerRequest[<=1] reset_scorer_request
PositionRequest[<=1] position_request
TestRequest[<=1] test_request
BallInTransitRequest[<=1] ball_in_transit_request
ScorerRequest[<=1] scorer_request
IncomingBallRequest[<=1] incoming_ball_request
PassRequest[<=1] pass_request
LeaveWallRequest[<=1] leave_wall_request
JoinWallRequest[<=1] join_wall_request