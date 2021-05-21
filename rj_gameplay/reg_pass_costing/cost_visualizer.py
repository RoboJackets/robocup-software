import matplotlib
from matplotlib.offsetbox import TextArea, DrawingArea, OffsetImage, AnnotationBbox
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
matplotlib.use('TkAgg')

import numpy as np
import cost_heuristic
from numpy import arange, sin, pi
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle

import sys
import random
import stp.rc as rc
import stp.testing as testing

from tkinter import *
from tkinter import font
import tkinter as Tk


#screen coordinates
width = 3000
height = 1600
font_size = 24
root = Tk.Tk()
textBox = Text()

def main():
	root.geometry(str(width) + 'x' + str(height))
	default_font = font.nametofont("TkDefaultFont")
	default_font.configure(size=font_size)
	root.wm_title("Pass Cost Visualizer")
	textBox.pack(side=Tk.RIGHT)
	button = Tk.Button(master=root, text='Refresh field', command= lambda: draw_plot())
	button.pack(side=Tk.RIGHT)
	draw_plot()
	root.protocol("WM_DELETE_WINDOW", exit)
	Tk.mainloop()

def get_our_bots():
	our_bots = []
	lines = open("visualizer_data.txt").readlines()
	ball_robot_num = int(lines[13].strip())
	our_position_data = lines[slice(1, 7)]
	for datum in our_position_data:
		datum = datum.strip()
		x, y = datum.split(", ")
		if len(our_bots) == ball_robot_num:
			our_bots.append(make_robot(float(x), float(y), ours=True, has_ball=True))
		else:
			our_bots.append(make_robot(float(x), float(y), ours=True, has_ball=False))
	return our_bots

def get_their_bots():
	their_bots = []
	lines = open("visualizer_data.txt").readlines()
	their_position_data = lines[slice(7, 13)]
	for datum in their_position_data:
		datum = datum.strip()
		x, y = datum.split(", ")
		their_bots.append(make_robot(float(x), float(y), True))
	return their_bots

def make_robot(x, y, ours, has_ball=False):
	 return rc.Robot(robot_id=random.randint(0, 1000000000),
        is_ours=ours,
        pose=np.array([x,y,0]),
        twist=np.array([0,0,0]),
        visible=True,
        has_ball_sense=has_ball,
        kicker_charged=True,
        kicker_healthy=True,
        lethal_fault=False)


def plot_field():
	f = plt.figure()
	a = f.add_subplot(111)
	ax = f.axes[0]
	ax.set_xlim(-3, 3)
	ax.set_ylim(0, 9)

	field = mpimg.imread('field.jpeg')
	plt.imshow(field, alpha=0.5, aspect='auto', extent=[-3, 3, 0, 9])

	our_bots = get_our_bots()
	count = 0
	their_bots = get_their_bots()
	for bot in our_bots:
		ax.add_patch(Circle((bot.pose[0], bot.pose[1]), 0.18, zorder=10, color='#5eeb34'))
		ax.annotate(str(count), xy=(bot.pose[0], bot.pose[1]), fontsize=14, zorder=20)
		count += 1
	for bot in their_bots:
		ax.add_patch(Circle((bot.pose[0], bot.pose[1]), 0.18, zorder=10, color='#d94025'))
	world_state = rc.WorldState(our_bots, their_bots, None, None, field=testing.generate_divB_field())
	costs = []
	i = 0
	for bot in our_bots:
		costs.append((len(costs), cost_heuristic.cost_heuristic(world_state, bot)))
	def descending_costs(val):
		if val[1] == 99999:
			return -1
		else:
			return val[1]
	costs.sort(key=descending_costs)
	text = ""
	for cost in costs:
		if cost[1] == -1:
			text += "Robot " + str(cost[0]) + " has the ball.\n"
		else:
			text += "Robot " + str(cost[0]) + " has cost " + str(cost[1]) + "\n"
	textBox.delete(1.0,"end")
	textBox.insert(1.0, text)
	plt.grid()
	plt.draw()
	return f

# a tk.DrawingArea
def draw_plot():
	for child in root.winfo_children():
		if isinstance(child, Tk.Canvas):
			child.destroy()
	canvas = FigureCanvasTkAgg(plot_field(), master=root)
	canvas.draw()
	canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
	canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

if __name__ == "__main__":
    main()