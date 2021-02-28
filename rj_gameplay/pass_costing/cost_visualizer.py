import matplotlib
matplotlib.use('TkAgg')

from numpy import arange, sin, pi
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import sys
import random
import tkinter as Tk


root = Tk.Tk()
root.wm_title("Embedding in TK")


def plot_field(s):
	f = Figure(figsize=(5, 4), dpi=100)
	a = f.add_subplot(111)
	t = arange(0.0, 3.0, 0.01)
	s = arange(0, s, s/300)

	a.plot(t, s)
	a.set_title('Tk embedding')
	a.set_xlabel('X axis label')
	a.set_ylabel('Y label')
	return f

# a tk.DrawingArea
def draw_plot():
	for child in root.winfo_children():
		if isinstance(child, Tk.Canvas):
			child.destroy()
	canvas = FigureCanvasTkAgg(plot_field(random.randint(0,100)), master=root)
	canvas.draw()
	canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
	canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)


draw_plot()
button = Tk.Button(master=root, text='Quit', command=draw_plot)
button.pack(side=Tk.BOTTOM)

Tk.mainloop()