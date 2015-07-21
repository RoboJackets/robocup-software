import unittest
import main
import robocup
import constants
import root_play


class TestWindowEvaluator(unittest.TestCase):

    def test_eval_pt_to_seg(self):
        # NOTE: setting the root play like this is really hacky and should be changed
        # without doing this though, set_our_robots() fails
        main._root_play = root_play.RootPlay()

        # add an opponent sitting right in front of their goal
        bot = robocup.OpponentRobot(11)
        bot.pos = robocup.Point(0, constants.Field.Length - constants.Robot.Radius * 1.5)
        bot.visible = True
        main.set_their_robots([bot])
        main.set_our_robots([])

        shot_from = robocup.Point(0, constants.Field.Length / 2.0)

        win_eval = robocup.WindowEvaluator(main.system_state())
        windows, best = win_eval.eval_pt_to_opp_goal(shot_from)

        # the obstacle should split our shot into two windows
        self.assertEqual(len(windows), 2)
        self.assertTrue(best != None)
