# Last updated 20251126 1430
# Added consideration for different voltage divider networks in different engines for measuring battery voltage
# Engines are now sent an engine specific 8 bit value to use for determining low voltage. (typ 168 or 180,
#  168 is for a 15k upstream resistor in the divider network, and 143 is for 18k.)
#   Added provisions for detecting engines that have stopped sending status reports when they are still
# registered.  They get sent a reset, and are unregistered locally.
#  Added provisions for handling engine reports of low battery; and for this code to tell an engine to
# disconnect from its battery.
# Added processing for additional engineModule error reports.

import tkinter as tk
from tkinter import *
from tkinter import messagebox
from tkinter import scrolledtext
import tkinter.font as font
from datetime import datetime
from PIL import Image, ImageDraw, ImageFont
import time
import sys
import os
import shutil
import socket
import threading
import serial




protectSerial = threading.Lock()
protectInternals = threading.Lock()   # Protect internal data when updating internally and with extern data
DEBUG = True

# # # # # # # # # # # # # # #  E n g i n e M a s t e r   c o m m s   s e t u p       # # # # # # # # # # # # # # #
# #

ENGCom = 'com14'
#ENGCom = '/dev/ttyACM0'  # Expected port on Ubuntu

try:
    ENGPort = serial.Serial(ENGCom,115200)
except:
    ENGCom = ''
    print("No serial connection made.")


registerMe = "50"
statusRpt = "51"
engineModuleInformation = "53"
batteryProtectionMode = "54"
schedulingIssue = "55"
unhandledCommand = "56"
speedSettingIssue = "57"

engineConduitInformation = "102"
eModBadCommand = "103"
radioFailure = "104"
unknownCommand = "105"

ACKRegistration = "151"
engineReset = "152"
resetAll = "153"
setUpPercent = "155"
setUpCruise = "156"
startUpEngine = "157"
shutDownEngine = "158"
goForward = "159"
goRearward = "160"
motorControl = "161"
updateCruiseValues = "162" 
emergencyStop = "163"
setPowerToOff = "164"

noEngine = 255
# # # # # # # # # # # # # # # # # # # #   E n g i n e M a s t e r   l i s t e n e r       # # # # # # # # # # # # # # # #
# #


class ListenToENG (threading.Thread):
  # Receive packets from engineConduit communicating through common ENGPort port.

    def run(Self):
        time.sleep(0.2)  # Let other threads run
        print('engineConduit listening thread started.', flush = True)
        while True:
            protectSerial.acquire()
            if (ENGPort.in_waiting > 0):  # There's something at the com port connected to Engine Feather to read
                fromENG = (ENGPort.readline().decode("utf-8").rstrip('\n\r')).lstrip('\0')
                protectSerial.release()
#                print('\n' + str(datetime.now())[11:-3] + ' Received from EngineConduit: ' + '\'' + fromENG +'\'')

#        Process and respond as needed to any inputs
                splitInput = fromENG.split()
                if splitInput[0] == registerMe:
                    engineSerial = int(splitInput[1])
    #registration
                    print('\n Engine ' + hex(engineSerial) + " registering")
                    index = tc.getIndexFromSerial(engineSerial)
                    if index != noEngine:
                        protectInternals.acquire()
                        tc.refreshEngineData(index, index+1)  # Resets engine data to original state
                        tc.myEngines[index][tc.theObsEngineStateIdx] = tc.REG   # mark engine registered
                        tc.myEngines[index][tc.theSetEngineStateIdx] = tc.REG
                        tc.myEngines[index][tc.timeOutIdx] = time.time()  # Set last status report time
                        protectInternals.release()
                        protectSerial.acquire()
                        ENGPort.write((ACKRegistration + ' ' + str((engineSerial & 0xFF00) >> 8) + " " + str(engineSerial & 0xFF) + " " + str(index) + \
                                        " " +  str(tc.myEngines[index][tc.battCutoffIdx]) + '\n').encode())
                        protectSerial.release()                                        
                        print("Just sent ACKregistration back to engine conduit for " + tc.myEngines[index][tc.theEngineNameIdx])
                        print("Sent: " + ACKRegistration + ' ' + str((engineSerial & 0xFF00) >> 8) + " " + str(engineSerial & 0xFF) + " " + str(index) + \
                                        " " +  str(tc.myEngines[index][tc.battCutoffIdx]) + '\n')
                    else:
                        tc.importantMessage = str(datetime.now())[11:-7] +  " Unknown engine trying to register: " + splitInput[1]
    # Protecting battery and other messages from engine module
                elif splitInput[0] == engineModuleInformation:
                    try:
                        index = int(splitInput[1])
                        engineName = tc.myEngines[index][tc.theEngineNameIdx]
                    except:
                        engineName = "unknown"
                    if engineName != "unknown":
                        print(str(datetime.now())[11:-7] +  " " + fromENG[2:])
                        if splitInput[2] == batteryProtectionMode:
                            tc.importantMessage = str(datetime.now())[11:-7] +  " " + engineName + " Protected battery!"
                            tc.refreshEngineData(index, index+1)  # Resets engine data to original state
                            tc.myEngines[index][tc.batteryBeingProtectedIdx] = True
                        elif splitInput[2] == schedulingIssue:
                            tc.importantMessage = str(datetime.now())[11:-7] +  " " + engineName + " Emod scheduling issue"
                        elif splitInput[2] == speedSettingIssue:
                            tc.importantMessage = str(datetime.now())[11:-7] +  " " + engineName + " Emod speed setting issue"
                        elif splitInput[2] == unhandledCommand:
                            tc.importantMessage = str(datetime.now())[11:-7] +  " " + engineName + " Emod REJECTED COMMAND"    
                        else:
                            tc.importantMessage = str(datetime.now())[11:-7] +  " " + fromENG[2:] + "...UNK mess from eng"
                    else:
                        tc.importantMessage = str(datetime.now())[11:-7] +  " " + fromENG[2:] + "...UNK engine"
    # Reports from engineConduit
                elif splitInput[0] == engineConduitInformation:
                    try:
                        index = int(splitInput[1])
                        engineName = tc.myEngines[index][tc.theEngineNameIdx]
                    except:
                        engineName = "unknown"
                    if engineName != "unknown":                        
                        print(str(datetime.now())[11:-7] +  " " + fromENG[2:])
                        if splitInput[2] == eModBadCommand:
                            tc.importantMessage = str(datetime.now())[11:-7] +  " " + engineName + " EngMod bad command"
                        elif splitInput[2] == radioFailure:
                            tc.importantMessage = str(datetime.now())[11:-7] +  " " + engineName + " EngConduit no radio"
                        elif splitInput[2] == unknownCommand:
                            tc.importantMessage = str(datetime.now())[11:-7] +  " " + engineName + " Unknown command"
                        else:
                            tc.importantMessage = str(datetime.now())[11:-7] +  " " + fromENG[2:] + "...UNK mess from conduit"
                    else:
                        tc.importantMessage = str(datetime.now())[11:-7] +  " " + fromENG[2:]
    # Status report from presumably active engine                   
                elif splitInput[0] == statusRpt:   # splitInput is: command,  index#, dir, spdCtl, speed, volts, [desRPS, obsRPS]
                    try:
                        index = int(splitInput[1])
                        engineName = tc.myEngines[index][tc.theEngineNameIdx]
                    except:
                        engineName = "unknown"
                    if engineName != "unknown": 
                        print(" Status: " + str(datetime.now())[11:-7] + str(" ") + engineName + " " + fromENG[2:])
                        if tc.myEngines[index][tc.theSetEngineStateIdx] != tc.UNREG:
                            tc.myEngines[index][tc.timeOutIdx] = time.time()
                            tc.myEngines[index][tc.theObsDirIdx] = int(splitInput[2])
                            if tc.myEngines[index][tc.theObsDirIdx] == tc.FORWARD and tc.myEngines[index][tc.theSetDirIdx] == tc.CHANGINGTOF:
                                tc.myEngines[index][tc.theSetDirIdx] = tc.FORWARD
                            elif tc.myEngines[index][tc.theObsDirIdx] == tc.REVERSE and tc.myEngines[index][tc.theSetDirIdx] == tc.CHANGINGTOR:
                                tc.myEngines[index][tc.theSetDirIdx] = tc.REVERSE     
                            tc.myEngines[index][tc.theObsMotorCtlIdx] = int(splitInput[3])
                            tc.myEngines[index][tc.theObsSpeedIdx] =   int(splitInput[4])
                            tc.myEngines[index][tc.digitalVoltsIdx] = int(splitInput[5])
                            tc.myEngines[index][tc.theOBSdesiredRPSIdx] = int(splitInput[6]) + 256* int(splitInput[7])
                            tc.myEngines[index][tc.theOBSsmoothedRPSIdx] = int(splitInput[8]) + 256* int(splitInput[9])
                            
                            if tc.myEngines[index][tc.theObsDirIdx] != tc.UNK:             # engine has known dir => its started up
                                tc.myEngines[index][tc.theObsEngineStateIdx] = tc.STARTEDUP
                            elif tc.myEngines[index][tc.theObsMotorCtlIdx] != tc.UNK:
                                tc.myEngines[index][tc.theObsEngineStateIdx] = tc.SHUTDOWN
                                tc.myEngines[index][tc.theSetEngineStateIdx] = tc.SHUTDOWN
                        else:
                            if not tc.myEngines[index][tc.batteryBeingProtectedIdx]:
                                tc.importantMessage = str(datetime.now())[11:-7] +  " Status report from known but unregistered engine: " + splitInput[1]
                                tc.refreshEngineData(index, index+1)  # Resets engine data to original state
                                protectSerial.acquire()
                                ENGPort.write((engineReset + ' ' + str(index) + '\n').encode())
                                protectSerial.release()
                    else:
                        tc.importantMessage = str(datetime.now())[11:-7] +  " Status report from uncatalogged engine: " + splitInput[1]
                                
                else:
                    print(" Unk EngineConduit input: " + fromENG[0:])


            else:  # No input
                protectSerial.release()

            for j in range(len(tc.myEngines)):
                if (tc.myEngines[j][tc.theSetEngineStateIdx] != tc.UNREG) and (time.time() - tc.myEngines[j][tc.timeOutIdx] > 30.0):  #  reg'd engine has timed out
                    tc.refreshEngineData(j, j+1)  # Resets engine data to original state
                    protectSerial.acquire()
                    ENGPort.write((engineReset + ' ' + str(j) + '\n').encode())
                    protectSerial.release()
                    tc.importantMessage = str(datetime.now())[11:-7] +  " Engine " + tc.myEngines[j][tc.theEngineNameIdx] + " reset - timed out." 
            time.sleep(0.2)



##********************************************************************************************************
##                                                 C l a s s   S A E
##********************************************************************************************************
##
##     Frame to allow user to SHOW ALL ENGINES
##                            ================
###########
class SAE(tk.Frame):
      def __init__(self, master):
            super().__init__(master)
            self.configure(bg="black")
 ##########
            # Set up grid
            for i in range(12):
                 self.SetGridSAE = Canvas(self, width=128, height=72, bg="black", highlightthickness=0).grid(row=0, column=i, sticky='ne')
                 self.SetGridSAE = Canvas(self, width=128, height=72, bg="black", highlightthickness=0).grid(row=i, column=0, sticky='ne')
      ## Set up frame for displaying known engines and for listing ones that are registered.

            self.listboxSAEStatic = scrolledtext.ScrolledText(self, width=25, height=8, font=('Arial Baltic', 24), fg="blue")
#            self.listboxSAEStatic = Listbox(self,width=25, height=6, font=('Arial Baltic', 26), bd = 4, relief = RAISED, activestyle="none")
            self.listboxSAESelect = Listbox(self,width=25, height=8, font=('Arial Baltic', 24), bd = 4, relief = RAISED, activestyle="none")
            self.listboxSAESelect.bind("<<ListboxSelect>>", self.LBActivity)
            self.listboxSAESelect.selection_clear(0, 'end')
            self.listboxSAESelect.config(selectbackground = "cyan", bg="pale green")

            self.textw = scrolledtext.ScrolledText(self, width=40, height=2, font=('Arial Baltic', 22), fg="yellow", bg="grey")
            self.textw.grid(row=10, column=1, rowspan = 2, columnspan=9,sticky='nesw')
            self.textw.insert(END, "")
            
            self.known = Label(self, text="All engines", font=('times', 30), fg="green2", bg="black")
            self.registered = Label(self, text="Registered", font=('times', 30), fg="green2", bg="black")
            self.StopAllBtn = Button(self, text='Stop All', font=('times', 40), fg="white",\
                                     bd = 6, relief = RAISED, bg="deep pink", command=self.selectedStopAllBtn)
            self.ResetAllBtn = Button(self, text='Reset All', font=('times', 40), fg="white",\
                                     bd = 6, relief = RAISED, bg="red", command=self.selectedResetAllBtn)            
            
            # Place listbox and buttons

            self.listboxSAEStatic.grid(row=1, column=1, rowspan = 7, columnspan=5, sticky='nw') 
            self.listboxSAESelect.grid(row=1, column=7, rowspan = 7, columnspan=4, sticky='nw')
            self.known.grid(row=6, column=2, rowspan = 1, columnspan=4, sticky='nw') 
            self.registered.grid(row=6, column=8, rowspan = 1, columnspan=3, sticky='nw')
            self.StopAllBtn.grid(row=8, column=3, rowspan = 2, columnspan=3, sticky='nw')
            self.ResetAllBtn.grid(row=8, column=7, rowspan = 2, columnspan=4, sticky='nw')            
##            self.btnGoBackSAE.grid(row=10, rowspan = 2, column = 8, columnspan=4, sticky = 'se')

            

      def reset(self):
            self.listboxSAEStatic.delete('1.0', END)
            self.listboxSAESelect.delete(0, END)
            self.listboxSAESelect.selection_clear(0, 'end')
            self.IndicesOfListed = []

### PROTECTED
            protectInternals.acquire()
            for idx in range(len(tc.myEngines)):                
                  self.listboxSAEStatic.insert(END, tc.myEngines[idx][tc.theEngineNameIdx] + '\n')
                  self.listboxSAEStatic.yview(END)
                  if tc.myEngines[idx][tc.theObsEngineStateIdx] != tc.UNREG:
                        self.addInfo = tc.myEngines[idx][tc.theEngineNameIdx] + \
                        "[" + \
                        tc.DIRS[tc.myEngines[idx][tc.theSetDirIdx]][0:1] + \
                        " " + str(tc.myEngines[idx][tc.theObsSpeedIdx]) + "]"
                        self.listboxSAESelect.insert(END, self.addInfo)
                        self.IndicesOfListed.append(idx)
            protectInternals.release()
### PROTECTION RELEASED
                                   
            if not self.IndicesOfListed:
                 # list is empty!
                  self.listboxSAESelect.insert(END, "No engines to show.")
                  self.listboxSAESelect.configure(state = tk.DISABLED)
            else: self.listboxSAESelect.configure(state = tk.NORMAL)

            if tc.importantMessage != "":      #  Needs to be protected from race condition with serial reader
                if len(tc.textwMessage) > 250:
                    tc.textwMessage = tc.textwMessage[tc.textwMessage.find("\n")+1:] + tc.importantMessage + "\n"
                else:
                    tc.textwMessage += tc.importantMessage + "\n"
                self.textw.delete('1.0', END)
                self.textw.insert(END, tc.textwMessage)
                self.textw.yview(END)
                tc.importantMessage = ""



      def LBActivity(self,event):
            self.temp = self.IndicesOfListed[self.listboxSAESelect.curselection()[0] ]
            tc.setSelectedIndex(self.temp)
            tc.setState(tc.enterManageEngine)            


      def selectedResetAllBtn(Self):
        if messagebox.askyesno('Make it so?...........'):
          tc.refreshEngineData(0, len(tc.myEngines))   # reset all engines
          protectSerial.acquire()
          ENGPort.write((resetAll + '\n').encode())
          protectSerial.release()
          tc.setState(tc.enterSAE)          


      def selectedStopAllBtn(self):
        protectSerial.acquire()
        ENGPort.write((emergencyStop + '\n').encode())
        protectSerial.release()
        for index in range(len(tc.myEngines)):
          if tc.myEngines[index][tc.theSetEngineStateIdx] != tc.UNREG:
            tc.myEngines[index][tc.theSetSpeedIdx] = 0  # Emergency stop implies set speed is zero




##*********************************************************************************************************
##                                        C l a s s   M E
##*********************************************************************************************************
##
##    Frame for managing individual engine control parameters
##      
class ME(tk.Frame):
      def __init__(self, master):
        super().__init__(master)
        self.configure(bg="black")
        for i in range(24):
              self.SetGridCSP = Canvas(self, width=64, height=36, bg="black", highlightthickness=0).grid(row=0, column=i, sticky='ne')
              self.SetGridCSP = Canvas(self, width=64, height=36, bg="black", highlightthickness=0).grid(row=i, column=0, sticky='ne')
              self.SetGridCSP = Canvas(self, width=64, height=36, bg="black", highlightthickness=0).grid(row=23, column=i, sticky='ne')
              self.SetGridCSP = Canvas(self, width=64, height=36, bg="black", highlightthickness=0).grid(row=i, column=23, sticky='ne')
              
        self.MCOptionsEntry = StringVar()
        
      # Fixed titles and labels        
        self.labelEngine=Label(self,text="" ,font=("Times",42),fg="green2",bg="black")
        self.labelEngine.grid(row=0, column=2, rowspan = 2, columnspan=20, sticky='new')
        self.current=Label(self,text="Set                    Observed\n===                   =======" ,font=("Times",36),fg="cyan",bg="black")
        self.current.grid(row=2, column=6, rowspan = 2, columnspan=10, sticky='nw')
        self.controls=Label(self,text="Controls\n========" ,font=("Times",36),fg="cyan",bg="black")
        self.controls.grid(row=2, column=17, rowspan = 2, columnspan=5, sticky='nw')

      # Row labels
        self.state=Label(self,text="Engine State" ,font=("Times",34),fg="grey",bg="black")
        self.state.grid(row=5, column=1, rowspan = 2, columnspan=5, sticky='nw')
        self.direction=Label(self,text="Direction" ,font=("Times",34),fg="grey",bg="black")
        self.direction.grid(row=7, column=1, rowspan = 2, columnspan=5, sticky='nw')
        self.motorControlSetting=Label(self,text="Motor Control" ,font=("Times",34),fg="grey",bg="black")
        self.motorControlSetting.grid(row=10, column=1, rowspan = 2, columnspan=5, sticky='nw')

        self.RPS=Label(self,text="RPS des / obs" ,font=("Times",34),fg="grey",bg="black")
        self.RPS.grid(row=12, column=1, rowspan = 2, columnspan=5, sticky='nw')
        
        self.spds=Label(self,text="Speed / %" ,font=("Times",34),fg="grey",bg="black")
        self.spds.grid(row=15, column=1, rowspan=2, columnspan=5, sticky='nw')
        self.batteryLevel=Label(self,text="Battery Level" ,font=("Times",34),fg="grey",bg="black")
        self.batteryLevel.grid(row=18, column=1, rowspan = 2, columnspan=5, sticky='nw')

     # Battery gauge

        self.gauge = tk.Canvas(self, bg="black", width=345, height=70)
        self.gauge.config(highlightthickness=0, bd=0)
        self.gauge.grid(row=18, column=9, rowspan = 3, columnspan = 6, sticky='nw')
        self.gauge.create_rectangle(0, 0, 345, 10, fill="black", tag="upperBlock")
        self.gauge.create_rectangle(0, 0, 5, 10, fill="black", tag="needle")
        self.gauge.create_rectangle(0, 10, 40, 70, fill="red", outline="red")
        self.gauge.create_rectangle(40, 10, 90, 70, fill="yellow", outline="yellow", tag="yellowBlock")
        self.gauge.create_rectangle(90, 10, 345, 70, fill="green", outline="green", tag="greenBlock")
        
     # Current values
     
        self.DisplayStateSet=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.DisplayStateSet.grid(row=5, column=6, rowspan = 2, columnspan=7, sticky='nw')

        self.DisplayStateObs=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.DisplayStateObs.grid(row=5, column=11, rowspan = 2, columnspan=7, sticky='nw')
                           
        self.DisplayDirSet=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.DisplayDirSet.grid(row=7, column=6, rowspan = 2, columnspan=7, sticky='nw')

        self.DisplayDirObs=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.DisplayDirObs.grid(row=7, column=11, rowspan = 2, columnspan=7, sticky='nw')

        self.DisplayMCSet=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.DisplayMCSet.grid(row=10, column=6, rowspan = 2, columnspan=7, sticky='nw')
        
        self.DisplayMCObs=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.DisplayMCObs.grid(row=10, column=11, rowspan = 2, columnspan=7, sticky='nw')

        self.RPSvaluesSet=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.RPSvaluesSet.grid(row=12, column=6, rowspan = 2, columnspan=7, sticky='nw')

        self.RPSvaluesObs=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.RPSvaluesObs.grid(row=12, column=11, rowspan = 2, columnspan=7, sticky='nw')
        
        self.SpeedSet=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.SpeedSet.grid(row=15, column=6, rowspan = 2, columnspan=7, sticky='nw')
        
        self.SpeedObs=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.SpeedObs.grid(row=15, column=11, rowspan = 2, columnspan=7, sticky='nw')
        
        self.Volts=Label(self,text="" ,font=("Times",34),fg="white",bg="black")
        self.Volts.grid(row=20, column=10, rowspan = 2, columnspan=3, sticky='news')
     
     # Selection widgets

        self.btnStartStop = Button(self, text=' Start up ', font=('times', 24), bg="green", bd = 8,\
                             relief = RAISED, command=self.activate)
        self.btnStartStop.grid(row=5, rowspan = 2, column = 17, columnspan=4, sticky = 'nw')

        self.btnReset = Button(self, text='Reset', font=('times', 32), fg="white",\
                                 bd = 6, relief = RAISED, bg="red", command=self.selectedResetBtn)
        self.btnReset.grid(row=6, column=21, rowspan = 2, columnspan=4, sticky='nw')

        self.btnDirection = Button(self, text="Change Dir", font=('times', 24), bg="green", bd = 8,\
                             relief = RAISED, command=self.changeDirection)
        self.btnDirection.grid(row=7, rowspan = 2, column = 17, columnspan=4, sticky = 'nsw')

        MCOptions = ["Percent", "Cruise"]
        self.MCOptionsMenu=OptionMenu(self,self.MCOptionsEntry,*MCOptions, command=self.chooseMotorControl)
        self.MCOptionsMenu.config(font=("", 24))
        menu = root.nametowidget(self.MCOptionsMenu.menuname)
        menu.config(font=("", 20))
        self.MCOptionsMenu.configure(bg="green", fg = "white")
        self.MCOptionsMenu.grid(row=10, column=17, rowspan = 2, columnspan=3, sticky='nw')

        self.btnUpdateCruiseValues = Button(self, text='Update\nCruise\nValues ', font=('times', 18), bg="green", bd = 8,\
                             relief = RAISED, command=self.updateCrsValues)
        self.btnUpdateCruiseValues.grid(row=10, rowspan = 3, column = 21, columnspan=4, sticky = 'nw')

        self.speedChoices = Listbox(self,width=4, height=5, font=('Noto Mono', 30), bd = 8, relief = RAISED, exportselection = False,\
                               activestyle="none", selectmode=SINGLE)
        # Insert elements into listbox
        for self.values in range(0,101):
                self.speedChoices.insert(END, self.values)
        self.speedChoices.config(selectbackground = "cyan")
        # Set up speedScrollbar
        self.speedScrollbar = Scrollbar(self, activebackground="cyan", bd = 8, relief = RAISED, bg="black")
        self.speedChoices.config(yscrollcommand = self.speedScrollbar.set, bg="pale green")
        self.speedScrollbar.config(width=56, command = self.speedChoices.yview)
        self.speedChoices.grid(row=15, column=16, rowspan = 5, columnspan = 3, sticky='nse')
        self.speedScrollbar.grid(row=15, column=19, rowspan = 5, columnspan = 2, sticky='nws')
       
        self.setSpeedPCNT = Button(self, text="Choose", font=('times', 24), bg="green", bd = 8,\
                             relief = RAISED, command=self.changeSpeedPCT)
        self.setSpeedPCNT.grid(row=17, rowspan = 2, column = 21, columnspan=3, sticky = 'nsw')

        self.btnPowerOff = Button(self, text="Power Off", font=('times', 28), bg="red", bd = 8,\
                             relief = RAISED, command=self.powerOff)
        self.btnPowerOff.grid(row=20, rowspan = 2, column = 1, columnspan=3, sticky = 'nsw')

        self.btnGoBackME = Button(self, text='<-Go back', font=('times', 30), bg="red", bd = 8,\
                                   relief = RAISED, command=self.goBackFromME)
        self.btnGoBackME.grid(row=22, rowspan = 2, column = 20, columnspan=4, sticky = 'sw')

            
      def reset(self):
        self.labelEngine.config(text=tc.myEngines[tc.selectedIdx][tc.theEngineNameIdx])
        
        self.DisplayStateSet.config(text=tc.STATES[tc.myEngines[tc.selectedIdx][tc.theSetEngineStateIdx]])
        self.DisplayStateObs.config(text=tc.STATES[tc.myEngines[tc.selectedIdx][tc.theObsEngineStateIdx]])
        
        self.DisplayDirSet.config(text=tc.DIRS[tc.myEngines[tc.selectedIdx][tc.theSetDirIdx]])
        self.DisplayDirObs.config(text=tc.DIRS[tc.myEngines[tc.selectedIdx][tc.theObsDirIdx]])

        self.MCOptionsEntry.set(tc.CONTROLS[tc.myEngines[tc.selectedIdx][tc.theSetMotorCtlIdx]])
        self.DisplayMCSet.config(text=tc.CONTROLS[tc.myEngines[tc.selectedIdx][tc.theSetMotorCtlIdx]])
        self.DisplayMCObs.config(text=tc.CONTROLS[tc.myEngines[tc.selectedIdx][tc.theObsMotorCtlIdx]])

        self.RPSvaluesSet.config(text=str(tc.myEngines[tc.selectedIdx][tc.theOBSdesiredRPSIdx]))            
        self.RPSvaluesObs.config(text=str(tc.myEngines[tc.selectedIdx][tc.theOBSsmoothedRPSIdx]))
        
        self.SpeedSet.config(text=str(tc.myEngines[tc.selectedIdx][tc.theSetSpeedIdx]))            
        self.SpeedObs.config(text=str(tc.myEngines[tc.selectedIdx][tc.theObsSpeedIdx]))

        # "Digital volts" is a digital (8 bit) representation of the analog voltage level of an engine's battery.
        # Each engine has an assigned value for determining if the voltage has dropped to 10.5 volts or
        # below.  This value is in the battCutoffIdx field of tc.myEngines for a given engine.
        # Typically this value is 143 or 180 depending on which resistors were used in a divider network
        # on the engine control board.
        #  To compute analog voltage observed one computes: 10.5 * "Digital volts" / 10.5 volt digital value (143 or 180)

        self.packVolts = 10.5 * tc.myEngines[tc.selectedIdx][tc.digitalVoltsIdx] / tc.myEngines[tc.selectedIdx][tc.battCutoffIdx]
#                self.packVolts = 11.1
        if (tc.myEngines[tc.selectedIdx][tc.batteryBeingProtectedIdx]):  ## Engine reported battery protection mode
            self.Volts.config(text="OFF!", fg="red")
            self.gauge.itemconfig("yellowBlock", fill="red")
            self.gauge.itemconfig("greenBlock", fill="red")
        elif self.packVolts < 10.5:
            self.Volts.config(text="DANGER!", fg="red")
            self.gauge.itemconfig("yellowBlock", fill="red")
            self.gauge.itemconfig("greenBlock", fill="red")            
        else:
            self.gauge.itemconfig("upperBlock", fill="black")
            self.gauge.delete("needle")
            self.upperCorner = 40 + ((self.packVolts-10.5) * 143)
            self.gauge.create_rectangle(self.upperCorner, 0, self.upperCorner+6, 10,  fill="white", outline="brown", tag ="needle")
            self.gauge.itemconfig("yellowBlock", fill="yellow")
            self.gauge.itemconfig("greenBlock", fill="green")
            if self.packVolts <= 10.8:
                self.Volts.config(text=f"{self.packVolts:.2f}" + " volts", fg="yellow")
            else: self.Volts.config(text=f"{self.packVolts:.2f}" + " volts", fg="green")

        if tc.myEngines[tc.selectedIdx][tc.theObsMotorCtlIdx] == tc.UNK:
              self.btnStartStop["state"] = "disabled"
              self.btnStartStop.config(bg="grey")
              self.btnDirection["state"] = "disabled"
              self.btnDirection.config(bg="grey")
              self.MCOptionsMenu.configure(state=tk.NORMAL)        
              self.MCOptionsMenu.config(bg="green")
              self.setSpeedPCNT["state"] = "disabled"
              self.setSpeedPCNT.config(bg="grey")
              self.btnUpdateCruiseValues["state"] = "disabled"
              self.btnUpdateCruiseValues.config(bg="grey")
              self.speedChoices["state"] = "disabled"
              self.speedChoices.config(bg="grey")

        elif tc.myEngines[tc.selectedIdx][tc.theObsEngineStateIdx] == tc.REG or\
                             tc.myEngines[tc.selectedIdx][tc.theObsEngineStateIdx] == tc.SHUTDOWN:
              self.btnStartStop.config(text = " Start up ", fg="black", bg="green")
              self.btnStartStop["state"] = "normal"
              self.btnDirection["state"] = "disabled"
              self.btnDirection.config(bg="grey")
              self.MCOptionsMenu.configure(state=tk.NORMAL)        
              self.MCOptionsMenu.config(bg="green")
              self.setSpeedPCNT["state"] = "disabled"
              self.setSpeedPCNT.config(bg="grey")
              self.btnUpdateCruiseValues["state"] = "disabled"
              self.btnUpdateCruiseValues.config(bg="grey")
              self.speedChoices["state"] = "disabled"
              self.speedChoices.config(bg="grey")

        else: # State is startedup
              self.btnStartStop.config(text = "Shut down", fg="black", bg="green")
              self.btnStartStop["state"] = "normal"
              if tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] != tc.CHANGINGTOR and \
                     tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] != tc.CHANGINGTOF:
                      self.btnDirection["state"] = "normal"
                      self.btnDirection.config(fg="black", bg="green")
                      self.setSpeedPCNT["state"] = "normal"
                      self.setSpeedPCNT.config(bg="green", fg="black")
              else:
                      self.btnDirection["state"] = "disabled"
                      self.btnDirection.config(bg="grey")
                      self.setSpeedPCNT["state"] = "disabled"
                      self.setSpeedPCNT.config(bg="grey")
              self.MCOptionsMenu.configure(state=tk.DISABLED)
              self.MCOptionsMenu.config(fg="black",bg="grey")
              if tc.myEngines[tc.selectedIdx][tc.theSetMotorCtlIdx] == tc.CRUISE:
                  self.btnUpdateCruiseValues["state"] = "normal"
                  self.btnUpdateCruiseValues.config(bg="green")
              else:
                  self.btnUpdateCruiseValues["state"] = "disabled"
                  self.btnUpdateCruiseValues.config(bg="grey")
              self.speedChoices["state"] = "normal"
              self.speedChoices.config(bg="pale green")

    
             

      def activate(self):
            self.btnStartStop.configure(bg="red", fg = "white")
            if tc.myEngines[tc.selectedIdx][tc.theObsEngineStateIdx] == tc.STARTEDUP:          # Shut down
                tc.myEngines[tc.selectedIdx][tc.theSetEngineStateIdx] = tc.SHUTDOWN
                tc.myEngines[tc.selectedIdx][tc.theSetSpeedIdx] = 0    #  Assume engine will go to zero as part of shutting down.
                tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] = tc.UNK
                protectSerial.acquire()
                ENGPort.write((shutDownEngine + ' ' + str(tc.selectedIdx) + '\n').encode())
                protectSerial.release()
            else:
                tc.myEngines[tc.selectedIdx][tc.theSetEngineStateIdx] = tc.STARTEDUP         # Start up
                tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] = tc.FORWARD
                # Send startup command and lights pins assignments
                protectSerial.acquire()
                ENGPort.write((startUpEngine + ' ' + str(tc.selectedIdx) + \
                    tc.myEngines[tc.selectedIdx][tc.engineParamsIDX] + '\n').encode())
                protectSerial.release()
                
                  

      def selectedResetBtn(self):
            if messagebox.askyesno('Make it so?...........'):
                tc.refreshEngineData(tc.selectedIdx, tc.selectedIdx+1)  # reset this engine only
                protectSerial.acquire()
                ENGPort.write((engineReset + ' ' + str(tc.selectedIdx) + '\n').encode())
                protectSerial.release()
                tc.setState(tc.enterSAE)


      def changeDirection(self):
        if tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] != tc.CHANGINGTOR and \
               tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] != tc.CHANGINGTOF:
            tc.myEngines[tc.selectedIdx][tc.theSetSpeedIdx] = 0   #  Direction change can only happen at zero; Engine will force it
            if tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] == tc.FORWARD:
                tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] = tc.CHANGINGTOR
                protectSerial.acquire()
                ENGPort.write((goRearward + ' ' + str(tc.selectedIdx) + '\n').encode())
                protectSerial.release()
                
            elif tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] == tc.REVERSE:
                tc.myEngines[tc.selectedIdx][tc.theSetDirIdx] = tc.CHANGINGTOF
                protectSerial.acquire()
                ENGPort.write((goForward + ' ' + str(tc.selectedIdx) + '\n').encode())
                protectSerial.release()


      def chooseMotorControl(self,chosenItem):   # chosen item is a value, not an index
            chosenItemIdx = 0
            if chosenItem == "Percent":
                chosenItemIdx = tc.PERCENT
                self.setSpeedPCNT.config(text="Choose\n PCNT")
            else:
                chosenItemIdx = tc.CRUISE
                self.setSpeedPCNT.config(text="Choose\nSpeed")
            if chosenItemIdx != tc.myEngines[tc.selectedIdx][tc.theSetMotorCtlIdx]:
                  tc.myEngines[tc.selectedIdx][tc.theSetMotorCtlIdx] = chosenItemIdx
                  if chosenItemIdx == tc.PERCENT:
                      protectSerial.acquire()
                      ENGPort.write((setUpPercent + ' ' + str(tc.selectedIdx) + '\n').encode())
                      protectSerial.release()
                  else:
                    with open(tc.myEngines[tc.selectedIdx][tc.theEngineNameIdx] + "cruiseValues.txt", "r") as self.valuesFile:
                      self.labels = self.valuesFile.readline()
                      self.values = self.valuesFile.readline()
                      self.values = self.values[:self.values.find("R")]  # Strip "RPS/10" off end.
#                      print(setUpCruise + ' ' + str(tc.selectedIdx) + " " + self.values + '\n')
                      protectSerial.acquire()
                      ENGPort.write((setUpCruise + ' ' + str(tc.selectedIdx) + " " + self.values + '\n').encode())
                      protectSerial.release()

      def updateCrsValues(self):
          # Update cruise values only when "cruise" has been selected as motor control method.
          if tc.myEngines[tc.selectedIdx][tc.theSetMotorCtlIdx] == tc.CRUISE:
              with open(tc.myEngines[tc.selectedIdx][tc.theEngineNameIdx] + "cruiseValues.txt", "r") as self.valuesFile:
                  self.labels = self.valuesFile.readline()
                  self.values = self.valuesFile.readline()
                  self.values = self.values[:self.values.find("R")]  # Strip "RPS/10" off end.
#                  print(updateCruiseValues + ' ' + str(tc.myEngines[tc.selectedIdx][tc.theEngineSerialIdx]) + " " + self.values + '\n')
                  protectSerial.acquire()
                  ENGPort.write((updateCruiseValues + ' ' + str(tc.selectedIdx) + " " +    self.values + '\n').encode())
                  protectSerial.release()

      def changeSpeedPCT(self):
          self.chosen = self.speedChoices.curselection()[0];
          tc.myEngines[tc.selectedIdx][tc.theSetSpeedIdx] = self.chosen
          protectSerial.acquire()
          ENGPort.write((motorControl + ' ' + str(tc.selectedIdx) + " " + str(self.chosen) + '\n').encode())
          protectSerial.release()


      def powerOff(self):
          tc.refreshEngineData(tc.selectedIdx, tc.selectedIdx+1)  # reset this engine only
          tc.myEngines[tc.selectedIdx][tc.batteryBeingProtectedIdx] = True
          protectSerial.acquire()
          ENGPort.write((setPowerToOff + ' ' + str(tc.selectedIdx) + '\n').encode())
          protectSerial.release()          
          tc.setState(tc.enterSAE)
          
      def goBackFromME(self):
      # Go back button clicked.
            tc.setState(tc.enterSAE)







##########################################################################################################            
##********************************************************************************************************
##                                            C l a s s   T r a i n C o n t r o l
##********************************************************************************************************
##########################################################################################################
class TrainControl(tk.Frame):
      # A frame to manage Train Control

      def __init__(self, master):
        super().__init__(master)
        #  Engine states (both startedUp and shutDown -> Registered)
        self.UNREG = 0
        self.REG = 1
        self.STARTEDUP = 2
        self.SHUTDOWN = 3
        
        self.UNK = 0
        self.REVERSE = 1
        self.FORWARD = 2

        self.CHANGINGTOF = 3
        self.CHANGINGTOR = 4
        

        self.PERCENT = 1
        self.CRUISE = 2
        

        self.STATES =["unregistered", "registered", "started up", "shut down"]
        self.DIRS = ["unknown", "reverse", "forward", "changingToF", "changingToR"]
        self.CONTROLS = ["unknown", "percent", "cruise"]
        


        self.theEngineNameIdx = 0           # constant
        self.theEngineSerialIdx = 1         # constant
        
        self.theSetEngineStateIdx = 2       # changed by user
        self.theObsEngineStateIdx = 3       # inferred from engine data
        
        self.theSetDirIdx = 4               # changed by user
        self.theObsDirIdx = 5               # read from engine

        self.theSetMotorCtlIdx = 6          # changed by user
        self.theObsMotorCtlIdx = 7          # read from engine
        
        self.theSetSpeedIdx = 8             # changed by user
        self.theObsSpeedIdx = 9             # read from engine
        
        self.digitalVoltsIdx = 10           # read from engine

        self.theOBSdesiredRPSIdx = 11       # read from engine
        self.theOBSsmoothedRPSIdx = 12      # read from engine
        self.batteryBeingProtectedIdx = 13  # read from engine
        self.engineParamsIDX = 14           # sent to engine to specify pin mappings for lights 
                                            #  pin A0 is 14,  A1 is 15,  A2 is 16,   A3 is 17 and A4 is 18
        self.battCutoffIdx = 15             # eight bit battery cutoff value
        self.timeOutIdx = 16                # last time status report was received

        self.enterSAE = 0
        self.inSAE = 1
        self.enterManageEngine = 2
        self.manageEngine = 3
        self.UIState = self.enterSAE
        self.selectedIdx = -1

        self.importantMessage = ""
        self.textwMessage = ""




##CPUs/engines
##  0x944B:      # engineConduit
##  0x2961:      # (1)Southern NW2 -- Proto V3.0 PCB; head/rear lights, cruise      updated 20250831
##  0x78BD:      # (2)Southern NW2 -- V3.1 PCB; head/rear lights, cruise            updated 20250831
##  0x7735:      # Chessie F3s -- V0 PCB; head light                                updated 20250831   could add cruise
##  0xe16c:      # Chicago and NW -- V0 PCB; head list                              updated 20250831   could add cruise
##  0x8462:      # SF_Dash9_607 --V3.0 PCB; head/rear/ditch lights, cruise
##  0x18ee:      # TEST                                                             updated 20250831
##  0xdffc:      # Placeholder
##  0x21EA:      # Placeholder  
      
        
  #            0                  1           2        3         4         5        6          7       8     9     10     11    12     13      14          15    16
  #          const              const      set        obs       set       obs      set        obs     set   obs    obs   obs    obs   prot    lights      batt  last
  #         Eng name           serial#    State      State      dir       dir       MC         MC     spd   spd   volts desRPS smRPS  batt     GPIO     cutoff  report
        self.myEngines = \
         [["(1)Southern nw2",  0x2961, self.UNREG, self.UNREG, self.UNK, self.UNK, self.UNK, self.UNK,  0,    0,    0,    0,    0,  False, " 20 21 5 6",   168,  0.0],\
          ["(2)Southern nw2",  0x78BD, self.UNREG, self.UNREG, self.UNK, self.UNK, self.UNK, self.UNK,  0,    0,    0,    0,    0,  False, " 20 21 5 6",   168,  0.0],\
           ["Chessie F3",      0x7735, self.UNREG, self.UNREG, self.UNK, self.UNK, self.UNK, self.UNK,  0,    0,    0,    0,    0,  False, " 14 15 16 17", 177,  0.0],\
           ["CNW SD40",        0xe16c, self.UNREG, self.UNREG, self.UNK, self.UNK, self.UNK, self.UNK,  0,    0,    0,    0,    0,  False, " 14 15 16 17", 176,  0.0],\
           ["SF_Dash9_607",    0x8462, self.UNREG, self.UNREG, self.UNK, self.UNK, self.UNK, self.UNK,  0,    0,    0,    0,    0,  False, " 20 21 5 6",   143,  0.0],\
           ["Mikes",           0x18EE, self.UNREG, self.UNREG, self.UNK, self.UNK, self.UNK, self.UNK,  0,    0,    0,    0,    0,  False, " 15 16 17 18", 168,  0.0],\
           ["CNW-NW2",         0xB877, self.UNREG, self.UNREG, self.UNK, self.UNK, self.UNK, self.UNK,  0,    0,    0,    0,    0,  False, " 15 16 17 18", 168,  0.0]\
          ]

        
        self.SAEprocess = SAE(self)
        self.MEprocess = ME(self)

        self.UIState = self.enterSAE
        
        self.after(1000, self.tickTock)


            
            
###################################################  E N D   TrainControl   Init  ####################################

      def setState(self, state):
            self.UIState = state

      def setSelectedIndex(self, idx):
            self.selectedIdx = idx

      def refreshEngineData(self, first, last):
          for j in range(first, last):
              self.myEngines[j][self.theSetEngineStateIdx] = self.UNREG              
              self.myEngines[j][self.theObsEngineStateIdx] = self.UNREG
              self.myEngines[j][self.theSetDirIdx] = self.UNK
              self.myEngines[j][self.theObsDirIdx] = self.UNK
              self.myEngines[j][self.theSetMotorCtlIdx] = self.UNK
              self.myEngines[j][self.theObsMotorCtlIdx] = self.UNK
              self.myEngines[j][self.theSetSpeedIdx] = 0
              self.myEngines[j][self.theObsSpeedIdx] = 0
              self.myEngines[j][self.digitalVoltsIdx] = 0
              self.myEngines[j][self.theOBSdesiredRPSIdx] = 0
              self.myEngines[j][self.theOBSsmoothedRPSIdx] = 0
              self.myEngines[j][self.batteryBeingProtectedIdx] = False
              self.myEngines[j][tc.timeOutIdx] = time.time()

      def getIndexFromSerial(self, serialNum):
          for i in range(len(self.myEngines)):
              if self.myEngines[i][self.theEngineSerialIdx] == serialNum:
                  return i
          return noEngine
        

      def tickTock(self):
            if self.UIState == self.enterSAE:
                  self.MEprocess.grid_forget()
                  self.SAEprocess.grid_forget()
                  self.SAEprocess.reset()
                  self.SAEprocess.grid(row=0, column=0)           # Display check-in prereg'd person screen
                  self.grid()
                  self.UIState = self.inSAE
                  
            if self.UIState == self.inSAE:                      # Keep messages box up to date
                  self.SAEprocess.reset()                  

            if self.UIState == self.enterManageEngine:
                  self.SAEprocess.grid_forget()
                  self.MEprocess.grid_forget()
                  self.MEprocess.reset()
                  self.MEprocess.grid(row=0, column=0)           # Display check-in prereg'd person screen
                  self.grid()
                  self.UIState = self.manageEngine
            if self.UIState == self.manageEngine:
                  self.MEprocess.reset()



                  
            self.after(1000, self.tickTock)

# ********************************   End Class TrainControl()  **********************************************





if __name__ == '__main__':
      root = tk.Tk()
      root.geometry('1536x1024')
      root.title("Train Control")
#      root.attributes('-fullscreen', True)

      if (ENGCom != ''):
            ENGListen = ListenToENG()
            ENGListen.start()
      else:
          print("No connection to engine conduit")

      tc = TrainControl(root)  # Start up train control GUI
      
      try:
            root.mainloop()
      except:
            shuttingDown = True
            root.destroy()
            sys.exit("All done")
