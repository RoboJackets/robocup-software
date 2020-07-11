import main
import robocup
import constants


## Returns list of all the robocup points to evaluate a function at
#
# @param num_width Number of bins width-wise
# @param num_length Number of bins length-wise
# @return List of 2d array of points to evaluate at
#         Outer list is the X moving from left to right
#         Inner list is the Y moving from bottom to top
def get_visualization_points(num_width=20, num_length=40):
    x_half = 0.5 * constants.Field.Width / num_width
    y_half = 0.5 * constants.Field.Length / num_length

    output = []

    for x in range(-1 * round(num_width / 2), round(num_width / 2)):
        sublist = []
        for y in range(0, num_length):
            x_cent = x * constants.Field.Width / num_width + x_half
            y_cent = y * constants.Field.Length / num_length + y_half

            sublist.append(robocup.Point(x_cent, y_cent))
        output.append(sublist)

    return output


## Draws the given values onto the field as an overlay
#
# @param values 2D List of values taken from the points returned in the `get_visualization_points` function
#               Outer list is the X moving from left to right
#               Inner list is the Y moving from bottom to top
# @param show_max Shows the max value as a white rectangle when true
#
# @note values is trucated to the range of (0, 1)
# @note values are evenly distributed over the entire field
# @note If append is used when creating the values list, make sure the order is correct (See `test_adaptive_formation_weights.py`)
def display_visualization_points(values, show_max=True):
    num_width = len(values)
    num_length = len(values[0])

    x_half = 0.5 * constants.Field.Width / num_width
    y_half = 0.5 * constants.Field.Length / num_length

    max_pt = robocup.Point(0, 0)
    max_val = 0

    for x in range(-1 * round(num_width / 2), round(num_width / 2)):
        for y in range(0, num_length):
            x_cent = x * constants.Field.Width / num_width + x_half
            y_cent = y * constants.Field.Length / num_length + y_half

            val = values[0].pop(0)
            val = min(val, 1)
            val = max(val, 0)

            if (val > max_val):
                max_pt = robocup.Point(x_cent, y_cent)
                max_val = val

            rect = [
                robocup.Point(x_cent - x_half, y_cent - y_half),
                robocup.Point(x_cent + x_half, y_cent - y_half),
                robocup.Point(x_cent + x_half, y_cent + y_half),
                robocup.Point(x_cent - x_half, y_cent + y_half)
            ]
            # Linear interpolation between Red and Blue
            val_color = (round(val * 255), 0, round((1 - val) * 255))

            # Draw onto the Overlay layer
            main.debug_drawer().draw_polygon(rect, val_color, "Overlay")
        values.pop(0)

    rect = [
        robocup.Point(max_pt.x - x_half, max_pt.y - y_half),
        robocup.Point(max_pt.x + x_half, max_pt.y - y_half),
        robocup.Point(max_pt.x + x_half, max_pt.y + y_half),
        robocup.Point(max_pt.x - x_half, max_pt.y + y_half)
    ]
    # Make a white rect at the max value
    val_color = (255, 255, 255)

    # Draw onto the Max layer
    main.debug_drawer().draw_polygon(rect, val_color, "Max")
