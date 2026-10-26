# 
#   @brief request sent by an agent to a specific agent indicating that
#   this agent is going to kick the ball to the other agent iff the other
#   agent acknowledges this message.
#  
#   agents will send an incoming ball request to the agent they will be passing
#   to.
#  
#  /
uint32 request_uid
uint8 from_robot_id