# 
#   @brief request sent by an agent to one or many agents indicating
#   that it would like to know the current position the agent is playing.
#  
#   position requests were originally used for testing, however they may be
#   useful for coordinating complex behaviors.  For example, agent a may need
#   to know that agent b is currently a waller before attempting to ask agent b
#   for help.
#  /
uint32 request_uid