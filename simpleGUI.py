# -*- coding: utf-8 -*-
"""
Created on Tue Mar 12 09:32:31 2019

@author: Sramana Dan


implementing a simple GUI
"""

import gripper_movements_rpi as gmr
import main 
from tkinter import *
from tkinter import filedialog
from tkinter import tkMessageBox



class MyFirstGUI:
    def __init__(self, master):
        self.master = master
        master.title("iCurve")

        self.label = Label(master, text="Choose the specifications for your catheter")
        self.label.pack()

        self.select_catheter = Button(master, text="Select the catheter file", command=self.select_catheter)
        self.select_catheter.pack()
        
        self.increment = Label(master,text='Enter the increment length (in mm) ')
        self.increment.pack()
        
        self.enterLength = Entry(master,bd = 5)
        self.enterLength.pack()
        
    
        self.okayIncrement = Button(master,text='okay',command=self.increment_length)
        self.okayIncrement.pack()
        
        self.fudge = Label(master,text = 'Enter the fudge factor')
        self.fudge.pack()
        
        self.enterFudge = Entry(master, bd = 5)
        self.enterFudge.pack()
        
        self.okayFudge = Button(master,text='okay',command=self.get_fudge)
        self.okayFudge.pack()
        
        self.start_bending = Button(master, text="Bend", command=self.start)
        self.start_bending.pack()

        self.close_button = Button(master, text="Close", command=master.quit)
        self.close_button.pack(side = BOTTOM)
        

    def select_catheter(self):
        print("Choosing Catheter!")
        global filename
        filename = filedialog.askopenfilename(filetypes =(("CSV Files","*.csv"),("Excel files",".xls"))) 
        #folder_path.set(filename)
        print("file selected is  " + filename)
        
    def returnfilename(self):
        return filename
    
    def increment_length(self):
        global enterLength
        print('the increment length is ' + self.enterLength.get())
    
    def returnincrement(self):
        return enterLength
    
    def get_fudge(self):
        global enterFudge
        print('the entered fudge factor is ' + self.enterFudge.get())
        
    def returnfudge(self):
        return enterFudge 
    
    def start(self):
        startWindow = Toplevel()
        
        zero = Button(startWindow,text='Zero the pins',command = gmr.zero_position() )
        zero.pack()
        
        home = Button(startWindow,text='Set to home position (partially open)',command = gmr.home_position())
        home.pack()
        
        fullyOpen = Button(startWindow,text='Open the pins fully')
        fullyOpen.pack()
        
        heatingControl = Button(startWindow,text='Heating Control', command =self.heating)
        heatingControl.pack()
        
        start_bending = Button(startWindow, text="Start Bending", command = self.bending)
        start_bending.pack()
        
        close_button = Button(startWindow, text="Close", command=startWindow.quit)
        close_button.pack(side = BOTTOM)
        
    def bending(self):
        angles = list(main.directions[0:12,1])
        tkMessageBox.showinfo("Info!","The first few angles are : " + angles)
        
    def heating(self):
        heatWindow = Toplevel()
        
        heatTime = Button(heatWindow,text='Set the heating time')
        heatTime.pack()
        
        heatPower = Button(heatWindow,text='Set the heating power')
        heatPower.pack()
        
        close_button = Button(heatWindow, text="Close", command=heatWindow.quit)
        close_button.pack(side = BOTTOM)
        
        

root = Tk()
my_gui = MyFirstGUI(root)
root.mainloop()
filename = my_gui.returnfilename()
increment= my_gui.returnincrement()
fudge_factor = my_gui.returnfudge()