# 0 -> RESERVED FOR ACKNOWLEDGE ON RESPONSE
# 1 -> TestRequest
# 2 -> PositionResponse
# 3 -> PassResponse
# ACKNOWLEDGE REQUEST DOES NOT (AND SHOULD NOT) EXIST
BallInTransitRequest[<=1] ball_in_transit_request
ScorerRequest[<=1] scorer_request
TestRequest[<=1] test_request
PassRequest[<=1] pass_request
IncomingBallRequest[<=1] incoming_ball_request
ScorerCancelRequest[<=1] scorer_cancel_request
ResetScorerRequest[<=1] reset_scorer_request
JoinWallRequest[<=1] join_wall_request
LeaveWallRequest[<=1] leave_wall_request
PositionRequest[<=1] position_request