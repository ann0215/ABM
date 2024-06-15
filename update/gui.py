"""Gui Plot"""

import tkinter as tk
from tkinter import Label

class GUI:
    def __init__(self):
        """Framework Setting"""
        self.top = tk.Tk()
        self.top.title("Escape Simulation")
        self.top.geometry("1080x710")
        self.top.resizable(width=False, height=False)
        """Canvas Setting"""
        self.c = tk.Canvas(self.top, width=1080, height=680, bg="#A9A9A9")
        self.c.pack()
        self.label = Label(self.top, text="Time = 0.0 s")
        self.label.pack()

    def add_barrier(self,barrier_set,scale=40):
        # Room boundray
        (left_bottom0, left_bottom1) = barrier_set[0]
        (right_top0, right_top1) = barrier_set[1]
        self.c.create_rectangle(left_bottom0*scale, left_bottom1*scale, right_top0*scale, scale, fill="#696969", outline="#696969")
        self.c.create_rectangle(left_bottom0*scale, (right_top1-1)*scale, right_top0*scale, right_top1*scale, fill="#696969", outline="#696969")
        self.c.create_rectangle(left_bottom0*scale, left_bottom1*scale, scale, right_top1*scale, fill="#696969", outline="#696969")
        self.c.create_rectangle((right_top0-1)*scale, left_bottom0*scale, right_top0*scale, 300, fill="#696969", outline="#696969")
        self.c.create_rectangle((right_top0-1)*scale, 380, right_top0*scale, right_top1*scale, fill="#696969", outline="#696969")
        
        # Obstacles
        t=3
        while t<len(barrier_set):
            (left_bottom0, left_bottom1) = barrier_set[t]
            (right_top0, right_top1) = barrier_set[t+1]
            self.c.create_rectangle(left_bottom0*scale, left_bottom1*scale, right_top0*scale, right_top1*scale, fill="#696969", outline="#696969")
            t+=2
            
    def update_time(self, _time):
        self.label['text'] = "Time = "+_time + " s"
    """Oval(ped) Plotting"""
    def add_oval(self, x1, y1, x2, y2, oval_tag):
        self.c.create_oval(x1, y1, x2, y2, fill="#FFE4B5", tag=oval_tag) 
    """Group Connection Plotting"""
    def add_line(self,ped, Peoplelist, line_tag):
        if ped.group_size>1:
            for i in range(len(Peoplelist)):
                if Peoplelist[i].group_id == ped.group_id:
                    self.c.create_line(ped.loc[0], ped.loc[1], Peoplelist[i].loc[0], Peoplelist[i].loc[1], dash=(4, 2),tag=f"line-{line_tag}")    
    """Delete Ovals"""
    def del_oval(self, oval_tag):
        self.c.delete(oval_tag)
    """Delete Lines"""
    def del_line(self, line_tag):
        self.c.delete(f"line-{line_tag}")
        self.c.tag_lower(f"line-{line_tag}")
    '''Undate GUI'''
    def update_gui(self):
        self.top.update()
        self.c.update()
    '''Sart GUI'''
    def start(self):
        self.top.mainloop()

