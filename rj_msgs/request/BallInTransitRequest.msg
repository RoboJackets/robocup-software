# /**
#  * @brief request sent by an agent to a specific agent indicating the
#  * ball has left this agent's possession and is currently headed for the
#  * other agent.
#  *
#  * an agent will send a BallInTransitRequest to a robot after they have
#  * kicked the ball so the other robot knows to begin receiving the ball.
#  *
#  */
uint32 request_uid
uint8 from_robot_id