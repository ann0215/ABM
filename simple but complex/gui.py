import tkinter as tk
from tkinter import Label
import math

class Barrier:
    def __init__(self):
        self.barrier_list = []
        self.barrier_type = None
        self.radius = None
        self.barrier_rec_par = None
        self.center_x = None
        self.center_y = None
        self.start_angle = None
        self.end_angle = None

    def rectangle_barrier(self, start_x, start_y, end_x, end_y): 
        """start_* = end_* -> straight line"""
        self.barrier_type = "rectangle"
        self.barrier_rec_par = (start_x, start_y, end_x, end_y)
        for i in range(start_x, end_x + 1):
            for j in range(start_y, end_y + 1):
                self.barrier_list.append((i, j))
                
    # def circular_barrier(self, center_x, center_y, radius):
    #     self.barrier_type = "circle"
    #     self.radius = radius
    #     for i in range(center_x - radius, center_x + radius + 1):
    #         for j in range(center_y - radius, center_y + radius + 1):
    #             if math.sqrt((i - center_x) ** 2 + (j - center_x) ** 2) <= radius:
    #                 self.barrier_list.append((i, j))
    
    # def arc_barrier(self, center_x, center_y, radius, start_angle, end_angle):
    #     self.barrier_type = "arc"
    #     self.radius = radius
    #     self.center_x = center_x
    #     self.center_y = center_y
    #     self.start_angle = start_angle
    #     self.end_angle = end_angle
    #     for i in range(center_x - radius, center_x + radius + 1):
    #         for j in range(center_y - radius, center_y + radius + 1):
    #             angle = math.degrees(math.atan2(j - center_y, i - center_x))
    #             if angle < 0:
    #                 angle += 360
    #             if radius - 1 < math.sqrt((i - center_x) ** 2 + (j - center_y) ** 2) <= radius and start_angle <= angle <= end_angle:
    #                 self.barrier_list.append((i, j))

class GUI:
    def __init__(self):
        self.barrier=None
        """Build Window"""
        self.top = tk.Tk()
        self.top.title("Escape Panic Simulation")
        self.top.geometry("1080x710")
        self.top.resizable(width=False, height=False)
        """Set Canvas"""
        self.c = tk.Canvas(self.top, width=1080, height=680, bg="#A9A9A9")
        self.c.pack()
        self.label = Label(self.top, text="Time = 0.0 s")
        self.label.pack()
        """Set Colors for Group"""
        self.colors = {
            1: "#FFE4B5",  # yellow
            2: "#ADD8E6",  # bluw
            3: "#90EE90",  # green
            4: "#FFB6C1"   # pink
        }

    # def add_roomborder(self):
    #     """Room Borders"""
    #     self.c.create_rectangle(0, 0, 1080, 40, fill="#696969", outline="#696969")
    #     self.c.create_rectangle(0, 640, 1080, 680, fill="#696969", outline="#696969")
    #     self.c.create_rectangle(0, 0, 40, 680, fill="#696969", outline="#696969")
    #     self.c.create_rectangle(1040, 0, 1080, 140, fill="#696969", outline="#696969")
    #     self.c.create_rectangle(1040, 220, 1080, 680, fill="#696969", outline="#696969")
        
    def add_obstacles(self, Barrier):
        scale = 40
        self.barrier = Barrier
        for (x, y) in self.barrier.barrier_list:
            if self.barrier.barrier_type == 'rectangle':
                (start_x, start_y, end_x, end_y) =  self.barrier.barrier_rec_par
                self.c.create_rectangle(start_x * scale, start_y * scale, end_x * scale, end_y * scale, fill="#696969", outline="#696969")
                # self.c.create_rectangle(x * scale, y * scale, x * scale + scale, y * scale + scale, fill="#696969", outline="#696969")
            if self.barrier.barrier_type == 'circle':
                radius = Barrier.radius
                self.c.create_oval((x - radius) * scale, (y - radius) * scale, (x + radius) * scale, (y + radius) * scale, fill="#696969", outline="#696969")
            if self.barrier.barrier_type == 'arc':
                center_x = Barrier.center_x
                center_y = Barrier.center_y
                radius = Barrier.radius
                start_angle = Barrier.start_angle
                end_angle = Barrier.end_angle
                self.c.create_arc((center_x - radius) * scale, (center_y - radius) * scale, (center_x + radius) * scale, (center_y + radius) * scale, start=start_angle, extent=end_angle - start_angle, fill="#696969", outline="#696969")
 
    # def add_barrier(self):
    #     """Room Borders"""
    #     self.c.create_rectangle(0, 0, 1080, 40, fill="#696969", outline="#696969")
    #     self.c.create_rectangle(0, 640, 1080, 680, fill="#696969", outline="#696969")
    #     self.c.create_rectangle(0, 0, 40, 680, fill="#696969", outline="#696969")
    #     self.c.create_rectangle(1040, 0, 1080, 140, fill="#696969", outline="#696969")
    #     self.c.create_rectangle(1040, 220, 1080, 680, fill="#696969", outline="#696969")
    #     """Room Barriers"""
    #     self.c.create_rectangle(400, 160, 720, 280, fill="#696969", outline="#696969")
    #     self.c.create_rectangle(400, 400, 720, 520, fill="#696969", outline="#696969")

    def update_time(self, _time):
        self.label['text'] = "Time = "+_time + " s"
    """Circle Plot"""
    def add_oval(self, x1, y1, x2, y2, oval_tag):
        # fill_color = self.colors.get(group_size, "#FFFFFF")
        self.c.create_oval(x1, y1, x2, y2, fill="#FFE4B5", tag=oval_tag)
    """Circle Delete"""
    def del_oval(self, oval_tag):
        self.c.delete(oval_tag)
    """Update GUI"""
    def update_gui(self):
        self.top.update()
        self.c.update()
    """Start GUI"""
    def start(self):
        self.top.mainloop()
