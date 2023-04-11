# 0 -> RESERVED FOR ACKNOWLEDGE ON RESPONSE
# 1 -> TestRequest
# 2 -> PositionResponse
# 3 -> PassResponse
# ACKNOWLEDGE REQUEST DOES NOT (AND SHOULD NOT) EXIST
TestRequest[<=1] test_request
PositionRequest[<=1] position_request
PassRequest[<=1] pass_request
IncomingBallRequest[<=1] incoming_ball_request
BallInTransitRequest[<=1] ball_in_transit_request
ScorerRequest[<=1] scorer_request