# 0 -> AcknowledgeResponse
# 1 -> TestResponse
# 2 -> PositionResponse
# 3 -> PassResponse
uint8 response_type
Acknowledge[<=1] acknowledge_response
TestResponse[<=1] test_response
PositionResponse[<=1] position_response
PassResponse[<=1] pass_response