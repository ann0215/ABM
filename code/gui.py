'''
Implements a GUI for an escape simulation using Tkinter,
with pedestrians as ovals, obstacles as rectangles, and group dynamics with connecting lines. 

Features:
1. Initializes a canvas with a gray background and draws a grid.
2. Obstacles and boundaries placement, represent agents with ovals,
    connect group members with lines to represent their interactions.
3. Updates timeto track the simulation's progress in seconds.
'''

import tkinter as tk
from tkinter import Label

class GUI:
    def __init__(self):
        """
        Initializes the main window and canvas for the escape simulation.
        """
        self.top = tk.Tk()
        self.top.title("Escape Simulation")
        self.top.geometry("1080x710")
        self.top.resizable(width=False, height=False)

        self.c = tk.Canvas(self.top, width=1080, height=680, bg="#A9A9A9")
        self.c.pack()
        self.label = Label(self.top, text="Time = 0.0 s")
        self.label.pack()
        
        self.draw_grid()
        
    def draw_grid(self, grid_size=40):
        """
        Draws a grid on the canvas to represent the simulation area.
        """
        for x in range(0, 1080, grid_size):
            self.c.create_line(x, 0, x, 680, fill="#D3D3D3")
        for y in range(0, 680, grid_size):
            self.c.create_line(0, y, 1080, y, fill="#D3D3D3")
       
    def add_barrier(self, barrier_set, scale=40):
        """
        Draws barriers and room boundaries on the canvas based on the given coordinates.
        """
        # Draw boundaries
        (left_bottom0, left_bottom1), (right_top0, right_top1) = barrier_set[:2]
        self.c.create_rectangle(left_bottom0*scale, left_bottom1*scale, right_top0*scale, right_top1*scale, fill="#696969", outline="#696969")
        # Draw additional barriers
        for t in range(3, len(barrier_set), 2):
            (left_bottom0, left_bottom1), (right_top0, right_top1) = barrier_set[t], barrier_set[t+1]
            self.c.create_rectangle(left_bottom0*scale, left_bottom1*scale, right_top0*scale, right_top1*scale, fill="#696969", outline="#696969")

    def update_time(self, _time):
        """
        Updates the time label with the current simulation time.
        """
        self.label['text'] = f"Time = {_time} s"
        
    def add_oval(self, x1, y1, x2, y2, oval_tag):
        """
        Draws an oval (representing a pedestrian) and a label inside it on the canvas.
        """
        self.c.create_oval(x1, y1, x2, y2, fill="#FFE4B5", tag=oval_tag)
        centerX = (x1 + x2) / 2
        centerY = (y1 + y2) / 2
        self.c.create_text(centerX, centerY, text=str(oval_tag), font=("Arial", 12), fill="black", tag=oval_tag)

    def add_line(self, ped, Peoplelist, line_tag):
        """
        Draws lines between group members to represent connections.
        """
        if ped.group_size > 1:
            for i in Peoplelist:
                if i.group_id == ped.group_id:
                    self.c.create_line(ped.loc[0], ped.loc[1], i.loc[0], i.loc[1], dash=(4, 2), tag=f"line-{line_tag}")

    def del_oval(self, oval_tag):
        """
        Deletes an oval from the canvas.
        """
        self.c.delete(oval_tag)

    def del_line(self, line_tag):
        """
        Deletes a line from the canvas.
        """
        self.c.delete(f"line-{line_tag}")

    def update_gui(self):
        """
        Refreshes the GUI to reflect any changes.
        """
        self.top.update()
        self.c.update()

    def start(self):
        """
        Starts the main loop of the GUI.
        """
        self.top.mainloop()
