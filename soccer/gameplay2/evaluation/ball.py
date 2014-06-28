

def is_moving_towards_our_goal():
    ball_path = robocup.Segment(main.ball().pos, (main.ball().pos + main.ball().vel.normalized()))
    return main.ball().vel.magsq() > 0.02 and ball_path.intersects(constants.Field.OurGoalSegment)


def is_in_our_goalie_zone():
    raise NotImplementedError()


# The ball's motion follows the equation X(t) = X_i + V_i*t - 0.5*(c*g)*t^2
def predict(X_i, V_i, t):
    raise NotImplementedError("The value of the coefficient of rolling friction between the ball and the field hasn't been set")
    c = 1       # The coefficient of rolling for a golf ball on the field's felt surface
    g = 9.81    # gravitational coefficient (m/s^2)
    m = 0.04593 # mass of golf ball (kg)

    return X_i + (V_i * t) - (0.5 * c * g * t**2)



# returns a Robot or None indicating which opponent has the ball
def opponent_with_ball():
    def angle_btw_three_pts(a, b, vertex):
        VA = math.sqrt( (vertex.x - a.x)**2 + (vertex.y - a.y)**2 )
        VB = math.sqrt( (vertex.x - b.x)**2 + (vertex.y - b.y)**2 )
        return math.acosf((VA*VA + VB*VB - AB*AB)/(2*VA*VB))

    closest_bot = None
    closest_dist = float("inf")
    for bot in main.their_robots():
        if bot.visible:
            dist = (bot.pos - main.ball().pos).mag()
            if dist < closest_dist:
                closest_bot, closest_dist = bot, dist

    angle = closest_bot.angle * DegreesToRadians
    theta = angle_btw_three_pts(closest_bot.pos + robocup.Point(math.cos(angle), math.sin(angle)),
                                main.ball().pos,
                                closest_bot.pos)
    max_radius = constants.Robot.Radius * (2.0 + 2.0*math.cos(theta))

    if closest and closest.pos.near_point(ball().pos, max_radius):
        return closest
    else:
        return None
