# coding: utf-8

import sys

import random

# from PySide2 import QtCore, QtWidgets, QtGui
# import tkinter

import tkinter as tk
from lib_visualization.vectorField_visualization import * 

obs_def = []
obs_def.append(Obstacle(
    x0 = [-1,1],
    a = [0.5,0.5],
    p = [1,1],
    th_r = 0,
    sf=0.1))

obs_def.append(Obstacle(
    x0 = [-1,1],
    a = [0.5,0.5],
    p = [1,1],
    th_r = 0,
    sf=0.1))

obs_def.append(Obstacle(
    x0 = [-1,1],
    a = [0.5,0.5],
    p = [1,1],
    th_r = 0,
    sf=0.1))


class takeInput(object):
    def __init__(self,requestMessage):
        self.n_obstacles_max = 5
        self.str_start = "GUI - "
        self.obs = obs_def

        self.root = tk.Tk()
        self.root.title(self.str_start + "Obstacle Avoidance")
        # self.root.geometry("500x500")
    
        self.string = ""
        self.frame = tk.Frame(self.root)
        self.frame.pack()

        # self.acceptInput(requestMessage)
        # self.waitForInput()
        
        self.obstacleDefinition()
        self.waitForInput()

    def acceptInput(self,requestMessage):
        r = self.frame

        k = tk.Label(r,text=requestMessage)
        k.pack(side='left')
        self.e = tk.Entry(r,text='Name')
        self.e.pack(side='left')
        self.e.focus_set()
        b = tk.Button(r,text='Static',command=self.visualize_animation)
        b.pack(side='left')

        b_ani = tk.Button(r,text='Animated',command=self.visualize_animation)
        b_ani.pack(side='right')
    
    def obstacleDefinition(self, visualization="vectorfield"):
        r = self.frame

        # v = tk.IntVar()
        # v.pack(side="left")
        # e1 = Entry(r, text="x0[0]")
        # v.set(13)
        width_e = 3
        col_start = 1
        rowStep_obs = 3
        self.n_obstacles = 3
        self.k = tk.Label(r,text="Position").grid(row=0,column=0+col_start)
        self.k = tk.Label(r,text="Axis length").grid(row=0,column=1+col_start)
        self.k = tk.Label(r,text="Orientation").grid(row=0,column=2+col_start)
        self.k = tk.Label(r,text="Curvature").grid(row=0,column=3+col_start)
        self.k = tk.Label(r,text="Margin").grid(row=0,column=4+col_start)

        self.entry_center = []
        self.entry_axis = []
        self.entry_orientation = []
        self.entry_curvature = []
        self.entry_margin = []
        
        for oo in range(self.n_obstacles):
            self.Obs_1 = tk.Label(r,text="Obstacle {}".format(oo+1)).grid(row=1+rowStep_obs*oo, column=col_start-1, rowspan=2)

            entry_center = [0,0]
            for ii in range(2):
                entry_center[ii] = tk.Entry(r, width=width_e)
                entry_center[ii].grid(row=ii+1+rowStep_obs*oo,column=0+col_start)
                entry_center[ii].insert(0, self.obs[oo].x0[0])
            self.entry_center.append(entry_center)

            entry_axis = [0, 0]
            for ii in range(2):
                entry_axis[ii] = tk.Entry(r, width=width_e)
                entry_axis[ii].grid(row=ii+1+rowStep_obs*oo, column=1+col_start)
                entry_axis[ii].insert(0, self.obs[oo].a[0])
            self.entry_axis.append(entry_axis)
            
            self.entry_orientation.append(tk.Entry(r, width=width_e))
            self.entry_orientation[oo].grid(row=1+rowStep_obs*oo, column=2+col_start, rowspan=2)
            self.entry_orientation[oo].insert(0, self.obs[oo].th_r)

            self.entry_curvature.append(tk.Entry(r, width=width_e))
            self.entry_curvature[oo].grid(row=1+rowStep_obs*oo, column=3+col_start, rowspan=2)
            self.entry_curvature[oo].insert(0, self.obs[oo].p[0])

            self.entry_margin.append(tk.Entry(r, width=width_e))
            self.entry_margin[oo].grid(row=1+rowStep_obs*oo, column=4+col_start, rowspan=2)
            self.entry_margin[oo].insert(0, self.obs[oo].sf)
            
            self.space = tk.Label(r,text=" ").grid(row=rowStep_obs*oo+3)

        button_run = tk.Button(r,text='Evaluate Vectorfield',command=self.visualize_vectorfield)
        button_run.grid(row=rowStep_obs*self.n_obstacles+1, column=2, columnspan=2)

        self.space = tk.Label(r,text=" ").grid(row=rowStep_obs*self.n_obstacles+2)

        
    def visualize_animation(self):
        self.getInt()

        
    def visualize_vectorfield(self):
        for oo in range(self.n_obstacles):
            for ii in range(2):
                print(self.entry_center[oo][ii].get())
                self.obs[oo].x0[ii] = self.checkType(self.entry_center[oo][ii], "float")
                self.obs[oo].a[ii] = self.checkType(self.entry_axis[oo][ii], "float", minVal=0)
                
            self.obs[oo].th_r[ii] = self.checkType(entry_orientation[oo], "float")
            self.obs[oo].p[ii] = self.checkType(entry_curvature[oo], "integer", minVal=1, maxVal=20)
            self.obs[oo].sf[ii] = self.checkType(entry_margin[oo], "float", minVal=0)

        

    def checkType(self, entry_field, varType="float", minVal=None, maxVal=None):
        value_str = entry_field.get()
        try:
            if varType=="integer":
                value_varType = int(value_str)
            elif varType=="float":
                value_varType = float(value_str)
            else:
                print("Unknown variable type")
                return
        except:
            self.text_warning = tk.Label(self.frame,
                                         text="Please enter an {} variable.".format(varType))
            self.text_warning.pack(side='bottom')
            print("WARNING -- No {} input.".format(varType), value_str)
            return

        
        if not minVal == None:
            if (value_varType< minVal)
                self.text_warning = tk.Label(self.frame,
                                             text="Please enter a value greater than {}.".format(minVal))
                self.text_warning.pack(side='bottom')
                
                value_varType = minVal

        if not maxVal == None:
            if (value_varType>minVal)
                self.text_warning = tk.Label(self.frame,
                                             text="Please enter a value smaller than {}.".format(minVal))
                self.text_warning.pack(side='bottom')
                
                value_varType = maxVal
                
        return value_varType

    
    def getInt(self):
        self.string = self.e.get()
        
        try:
            self.n_obstacles = int(self.string)
        except:
            self.text_warning = Label(self.frame,
                                      text="Please enter an integer.")
            self.text_warning.pack(side='bottom')
            print("WARNING -- No integer input.", self.string)
            return
        
        if (self.n_obstacles<0 or self.n_obstacles>self.n_obstacles_max):
            self.text_warning = Label(self.frame,
                                      text="Please enter an obstacle number between 0 and {}.".format(self.n_obstacles_max))
            self.text_warning.pack(side='bottom')
            print("WARNING -- Wrong obstacle number input: {}.".format(self.n_obstacles), self.string)
            return
        self.root.destroy()

    def waitForInput(self):
        self.root.mainloop()

def getText(requestMessage):
    msgBox = takeInput(requestMessage)
    #loop until the user makes a decision and the window is destroyed
    # return msgBox.getString()

    
if (__name__=="__main__"):
    var = getText('Number of Obstacles')
    print("Var:", var)

    
