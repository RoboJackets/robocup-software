# 0 -> RESERVED FOR ACKNOWLEDGE ON RESPONSE
# 1 -> TestRequest
# 2 -> PositionResponse
# 3 -> PassResponse
uint8 request_type # 0 -> TestRequest; 1 -> PassRequest; 2 -> PositionRequest; ...
# ACKNOWLEDGE REQUEST DOES (AND SHOULD NOT) EXIST
TestRequest[<=1] test_request
PositionRequest[<=1] position_request
PassRequest[<=1] pass_request