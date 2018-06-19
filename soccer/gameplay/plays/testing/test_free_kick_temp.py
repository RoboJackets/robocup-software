import play
import behavior
import tactics.stopped.circle_near_ball
import robocup
import main
import math
import constants

class TestFreeKickTemp(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def execute_running(self):
        # Find the hole in the defenders to kick at
        # The limit is 20 cm so any point past it should be defenders right there
        win_eval = robocup.WindowEvaluator(main.system_state())

        # 500 cm min circle distance plus the robot width
        test_distance = .5 + constants.Robot.Radius 

        # +- 60 degrees max as max offset to dodge ball
        max_angle = 50 * constants.DegreesToRadians

        # How much left and right of a robot to give
        # Dont make this too big or it will always go far to the right or left of the robots
        robot_angle_offset = 8 * constants.DegreesToRadians

        zero_point = robocup.Point(0, 0)
    
        # Limit the angle so as we get closer, we dont miss the goal completely as much
        goal_vector = constants.Field.TheirGoalSegment.center() - main.ball().pos
        max_length_vector = robocup.Point(constants.Field.Length, constants.Field.Width)
        goal_limit = (goal_vector.mag() / max_length_vector.mag()) * max_angle

        # Limit on one side so we dont directly kick out of bounds
        # Add in the angle from the sideline to the target
        field_limit = (1 - abs(main.ball().pos.x) / (constants.Field.Width / 2)) * max_angle
        field_limit = field_limit + goal_vector.angle_between(robocup.Point(0, 1))

        # Limit the angle based on the opponent robots to try and always minimize the
        left_robot_limit = 0
        right_robot_limit = 0

        for robot in main.their_robots():
            ball_to_bot = robot.pos - main.ball().pos
            
            # Add an extra radius as wiggle room
            # kick eval already deals with the wiggle room so it isn't needed there
            if (ball_to_bot.mag() <= test_distance + constants.Robot.Radius):
                angle = goal_vector.angle_between(ball_to_bot)
                
                print(ball_to_bot)
                print(angle)
                # Try and rotate onto the goal vector
                # if we actually do, then the robot is to the right of the ball vector
                ball_to_bot.rotate(zero_point, angle)
                if (ball_to_bot.angle_between(goal_vector) < 0.01):
                    right_robot_limit = max(right_robot_limit, angle + robot_angle_offset)
                    #left_robot_limit = max(left_robot_limit, -angle + robot_angle_offset)
                else:
                    #right_robot_limit = max(right_robot_limit, -angle + robot_angle_offset)
                    left_robot_limit = max(left_robot_limit, angle + robot_angle_offset)
            else:
                win_eval.add_excluded_robot(robot)
                    

        # Angle limit on each side of the bot->goal vector
        left_angle = max_angle
        right_angle = max_angle
        
        # Make sure we limit the correct side due to the field
        if main.ball().pos.x < 0:
            left_angle = min(left_angle, field_limit)
        else:
            right_angle = min(right_angle, field_limit)

        # Limit due to goal
        left_angle = min(left_angle, goal_limit)
        right_angle = min(right_angle, goal_limit)

        # Limit to just over the robots
        if (left_robot_limit is not 0):
            left_angle = min(left_angle, left_robot_limit)
        if (right_robot_limit is not 0):
            right_angle = min(right_angle, right_robot_limit)

        # Get the angle thatwe need to rotate the target angle behind the defenders
        # since kick eval doesn't support a nonsymmetric angle around a target
        rotate_target_angle = (left_angle + -right_angle)/2
        target_width = (left_angle + right_angle)

        target_point = goal_vector.normalized() * test_distance
        target_point.rotate(zero_point, rotate_target_angle)

        point, chance = win_eval.eval_pt_to_pt(main.ball().pos, target_point + main.ball().pos, target_width)

        main.system_state().draw_line(robocup.Line(main.ball().pos, target_point + main.ball().pos), (0, 255, 0), "Target Point")

        # Test draw points
        target_point.rotate(zero_point, target_width/2)
        p1 = target_point + main.ball().pos
        target_point.rotate(zero_point, -target_width)
        p2 = target_point + main.ball().pos
        p3 = main.ball().pos
        main.system_state().draw_polygon([p1, p2, p3], (0, 0, 255), "Free Kick search zone")

        if (len(point) > 0):
            main.system_state().draw_line(robocup.Line(main.ball().pos, point[0].segment.center()), (255, 255, 0), "Target Shot")
        