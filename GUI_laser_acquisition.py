import os
import configparser
from pypylon import genicam
from pypylon import pylon
import sys
import tkinter
from tkinter import ttk, Label, StringVar, Text, Scrollbar, RIGHT, Y, OptionMenu
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backend_bases import cursors
import matplotlib.backends.backend_tkagg as tkagg
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backend_bases import MouseEvent
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

import numpy as np
import math
import time
from scipy.optimize import curve_fit
from datetime import date
import matplotlib.patches as patches
from skimage.transform import rescale, resize
import matplotlib.patheffects as pe
from datetime import datetime
from save_img import saveImg

import inspect
# Import the .NET Common Language Runtime (CLR) to allow interaction with .NET
import clr

############################################################# general functions to be used below ##############################################################

def supergauss_2D(data_tuple, ux, uy, sigx, sigy, P, amp, off):
    (x, y) = data_tuple
    xarg = ((x-ux)**2)/(2*sigx**2)
    yarg = ((y-uy)**2)/(2*sigy**2)
    val = off + amp*np.exp(-(xarg + yarg)**P)
    return val.ravel()

def supergauss_1D(x, ux, sigx, P, amp, off):
    xarg = ((x-ux)**2)/(2*sigx**2)
    val = off + amp*np.exp(-xarg**P)
    return val

def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata
    global coords
    coords = [ix, iy]
#     if len(coords) == 2:
#         fig.canvas.mpl_disconnect(cid)
    return coords

def gauss(xpt, A, xoff, sig, off):
    return A*np.exp(-0.5*((xpt-xoff)/sig)**2)+off

class camAcq():
    def __init__(self, synthetic_camera=False, synth_shape=(1000, 1000)):
        self.synthetic = synthetic_camera
        if not self.synthetic:
            # Get the transport layer factory.
            self.tlFactory = pylon.TlFactory.GetInstance()
            # Get all attached devices and exit application if no device is found.
            self.devices = self.tlFactory.EnumerateDevices()
            if len(self.devices) == 0:
                raise pylon.RuntimeException("No camera present.")
            self.options = np.array([d.GetUserDefinedName() for d in self.devices])
            self.arr = np.array([[0,0]])
        else:
            self.options = np.array(["ayy", "lmao"])
            self.arr = np.zeros(synth_shape)
            self.shape = synth_shape
        self.bkgnd = np.zeros_like(self.arr)
    def open_camera(self, cam_name, exposure_time, trigger_mode):
        if not self.synthetic:
            self.cam_coord = np.where(self.options==cam_name)[0][0]
            try:
                self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(self.devices[self.cam_coord]))
                self.camera.Open()
                try:
                    self.camera.ExposureTime.Value = exposure_time
                except:
                    self.camera.ExposureTimeRaw.Value = exposure_time
                self.camera.TriggerMode.SetValue(trigger_mode.capitalize())
                self.camera.PixelFormat.SetValue("Mono12")
                self.camera.MaxNumBuffer.SetValue(10)
                self.camera.MaxNumQueuedBuffer.SetValue(10)
                try:
                    self.camera.StreamGrabber.MaxTransferSize.Value = 4 * 1024 * 1024
                except:
                    pass
                self.width = self.camera.Width.GetValue()
                self.height = self.camera.Height.GetValue()
                self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
                # self.camera.StartGrabbing(pylon.GrabStrategy_OneByOne)
            except genicam.GenericException as e:
            # Error handling
                print("An exception occurred.", e.GetDescription())
                exitCode = 1
        else:
            self.width = self.shape[1]
            self.height = self.shape[0]

# simulate laser beam
class simulateLaser():
    def __init__(self):
        pass
    def generateArr(self, h, w):
        self.arrcoords = np.array([[(i, j) for i in range(h)] for j in range(w)])
        self.gaussarr = supergauss_2D(np.transpose(self.arrcoords), h/2, w/2, 300, 300, 3, 500, 0)
        self.gaussarr = self.gaussarr.reshape(h, w)
        
#store energy/pixel sum time history information
class dataHistory():
    def __init__(self):
        self.arr = np.array([])
        self.std = np.array([])
        self.line = np.array([])
        self.lineLong = np.array([])
    def addData(self, data, shot_avg, reset, resetLong):
        if len(self.arr)>reset:
            self.arr = self.arr[1:]
        if len(self.line)>reset:
            self.line = self.line[1:]
            self.std = self.std[1:]
        if len(self.lineLong)>resetLong:
            self.lineLong = self.lineLong[1:]
        self.arr = np.append(self.arr, data)
        self.std = np.append(self.std, np.std(self.arr[-shot_avg:])/np.sqrt(shot_avg))
        self.line = np.append(self.line, np.mean(self.arr[-shot_avg:]))
        self.lineLong = np.append(self.lineLong, data)

class coordHistory():
    def __init__(self):
        self.arr = np.empty((0,2))
        self.std = np.empty((0,2))
        self.line = np.empty((0,2))
    def addData(self, data, shot_avg, reset):
        if len(self.arr)>reset:
            self.arr = self.arr[1:]
            self.std = self.std[1:]
            self.line = self.line[1:]
        self.arr = np.append(self.arr, [data], axis = 0)
        self.std = np.append(self.std, [np.std(self.arr[-shot_avg:], axis = 0)/np.sqrt(shot_avg)], axis=0)
        self.line = np.append(self.line, [np.mean(self.arr[-shot_avg:], axis = 0)], axis=0)
        
class motorMover():
    def __init__(self):
        pass
    def initializeMotor(self):
        self.strCurrFile = os.path.abspath(inspect.stack()[0][1])
        print("Executing File = %s\n" % self.strCurrFile)

        # Initialize the DLL folder path to where the DLLs are located
        self.strPathDllFolder = os.path.dirname(self.strCurrFile)
        print("Executing Dir  = %s\n" % self.strPathDllFolder)

        # Add the DLL folder path to the system search path (before adding references)
        sys.path.append(self.strPathDllFolder)

        # Add a reference to each .NET assembly required
        clr.AddReference("UsbDllWrap")

        # Import a class from a namespace
        from Newport.USBComm import USB
        from System.Text import StringBuilder
        from System.Collections import Hashtable
        from System.Collections import IDictionaryEnumerator
        # Call the class constructor to create an object
        self.oUSB = USB(True)

        # Discover all connected devices
        self.bStatus = self.oUSB.OpenDevices(0, True)
        self.strBldr = StringBuilder(64)

    def move(self, device, motornum, step, lim):
        self.initializeMotor()
        strDeviceKey = device
        self.strBldr.Remove(0, self.strBldr.Length)
        # nReturn = self.oUSB.Query(strDeviceKey, "*IDN?", strBldr)
        # print("Return Status = %d" % nReturn)
        # print("*IDN Response = %s\n" % self.strBldr.ToString())
        sgn = int(math.copysign(1, step))
        if abs(step) > lim:
            step = sgn*lim
        command = motornum + "PR" + str(step)
        print(command)
        self.oUSB.Query (strDeviceKey, command, self.strBldr)
        self.close()
#         nReturn = oUSB.Query (strDeviceKey, "1PR?", strBldr)
        
    def close(self):
        # Shut down all communication
        self.oUSB.CloseDevices()
        print ("Devices Closed.\n")
        
class fitParams():
    def __init__(self):
        self.params = np.array([])
    def setParams(self, data):
        self.params = np.array(data)

############################################################# basic settings ##########################################################################

coords = [0,0]
n_cams = [3,2]

screen_width = 1920
screen_height = 1020
window = str(sys.argv[1])
print(window)
winVal = int(window[-1])-1
xorigin = (screen_width//n_cams[0])*winVal if winVal <n_cams[0] else (screen_width//n_cams[0])*(winVal-n_cams[0])
yorigin = 0 if winVal <n_cams[0] else screen_height//2+30

origin = [xorigin, yorigin]

s_sizex = screen_width//n_cams[0]
s_sizey = screen_height//n_cams[1]
screen = str(s_sizex)+"x"+str(s_sizey)+"+"+str(origin[0])+"+"+str(origin[1])

root = tkinter.Tk()

################################################################ read in camera settings; start grabbing #####################################################

iniFname = str(sys.argv[2])
print('reading settings from: %s' %iniFname)
config = configparser.ConfigParser()
config.read(iniFname)
cam_name = tkinter.StringVar()
cam_name.set(config['CAMERA NAMES'][window])
exposure_time = tkinter.IntVar()
exposure_time.set(int(config[window.upper() + ' SETTINGS']['exposure time']))
trigger_mode = tkinter.StringVar()
trigger_mode.set(config[window.upper() + ' SETTINGS']['trigger mode'])
                                
EcalVal = tkinter.DoubleVar(value = float(config[window.upper() + ' SETTINGS']['energy cal']))
energyLimit = tkinter.DoubleVar(value = float(config[window.upper() + ' SETTINGS']['energy warning value']))
                                
ROIEnable = tkinter.BooleanVar(value = config[window.upper() + ' SETTINGS']['roienable']=='True')
ROILeft = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['roileft']))
ROIRight = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['roiright']))
ROIBottom = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['roibottom']))
ROITop = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['roitop']))

saveFolder = StringVar(value = config[window.upper() + ' SETTINGS']['save folder'])

bkgndFile = StringVar(value = config[window.upper() + ' SETTINGS']['bkgnd file'])
subtBkgnd = tkinter.BooleanVar(value = config[window.upper() + ' SETTINGS']['subtract background']=='True')

cursorXfixed = StringVar(value = config[window.upper() + ' SETTINGS']['cursor x pos'])
cursorYfixed = StringVar(value = config[window.upper() + ' SETTINGS']['cursor y pos'])
cursorRad = StringVar(value = config[window.upper() + ' SETTINGS']['cursor radius'])

motorX = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['motor x pos']))
motorY = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['motor y pos'])) 
motorOn = tkinter.BooleanVar(value = config[window.upper() + ' SETTINGS']['motor on']=='True')
motorError = tkinter.DoubleVar(value = float(config[window.upper() + ' SETTINGS']['motor error']))
motorThreshold = tkinter.DoubleVar(value = float(config[window.upper() + ' SETTINGS']['motor threshold']))
motorStep = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['motor step size']))
deviceName = StringVar(value = config[window.upper() + ' SETTINGS']['device name'])
motorXnum = StringVar(value = config[window.upper() + ' SETTINGS']['motor x number'])
motorYnum = StringVar(value = config[window.upper() + ' SETTINGS']['motor y number'])
motorXmult = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['motor x multiplier']))
motorYmult = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['motor y multiplier']))
initializeOn = tkinter.BooleanVar(value = config[window.upper() + ' SETTINGS']['initialize motors at start']=='True')

umperpix = tkinter.DoubleVar(value = float(config[window.upper() + ' SETTINGS']['umperpix']))
fittingini = tkinter.BooleanVar(value = config[window.upper() + ' SETTINGS']['gauss fitting']=='True')
fitting = tkinter.BooleanVar(value = False)
singleShot = tkinter.BooleanVar(value = config[window.upper() + ' SETTINGS']['single shot']=='True')

timeout = tkinter.IntVar(value = 1000)
if trigger_mode.get() == 'on':
    timeout.set(200)
    
#cam = camAcq(synthetic_camera = True)
cam = camAcq()
cam.open_camera(cam_name.get(), exposure_time.get(), trigger_mode.get())

############################################################# Tkinter gui elements setup ###################################################################### 

root.wm_title(window)
root.geometry(screen)
tabControl = ttk.Notebook(root)

labelText = StringVar()
labelText.set("")
labelDir = Label(root, textvariable=labelText, font = ("arial",40))

camNameText = StringVar(value = cam_name.get())
camNameDir = Label(root, textvariable=camNameText, font = ("arial",20))

fwhmxlabelText = StringVar(value = "")
fwhmxlabelDir = Label(root, textvariable=fwhmxlabelText, font = ("arial",10))
fwhmxText = StringVar(value = "")
fwhmxDir = Label(root, textvariable=fwhmxText, font = ("arial",20))

fwhmylabelText = StringVar(value = "")
fwhmylabelDir = Label(root, textvariable=fwhmylabelText, font = ("arial",10))
fwhmyText = StringVar(value = "")
fwhmyDir = Label(root, textvariable=fwhmyText, font = ("arial",20))

energylabelText = StringVar(value = "energy (mJ)")
energylabelDir = Label(root, textvariable=energylabelText, font = ("arial",10))
energyText = StringVar(value = "")
energyDir = Label(root, textvariable=energyText, font = ("arial",20))

stdlabelText = StringVar(value = "std dev (mJ)")
stdlabelDir = Label(root, textvariable=stdlabelText, font = ("arial",10))
stdText = StringVar(value = "")
stdDir = Label(root, textvariable=stdText, font = ("arial",20))

fluencelabelText = StringVar(value = "")
fluencelabelDir = Label(root, textvariable=fluencelabelText, font = ("arial",10))
fluenceText = StringVar(value = "")
fluenceDir = Label(root, textvariable=fluenceText, font = ("arial",20))

gauss_fit_button = ttk.Checkbutton(root, text='Gauss fit', variable=fitting, onvalue=True, offvalue=False)

################################################################ create plot structure ########################################################################

cameraChange = tkinter.BooleanVar(value = True)
cameraChange2 = tkinter.BooleanVar(value = False)

scale = tkinter.DoubleVar(value = s_sizex/cam.width)
if scale.get()*cam.height > 0.8*s_sizey:
    scale.set(0.8*s_sizey/cam.height)
orig_scale = scale.get()
xvals = np.arange(int(scale.get()*cam.width))
yvals = np.arange(int(scale.get()*cam.height))
xvals_max = s_sizex
yvals_max = int(0.8*s_sizey)
px = 1/plt.rcParams['figure.dpi']

x_extent = s_sizex*px
y_extent = int(0.8*s_sizey*px)
rect = patches.Rectangle((-1, -1), xvals_max+1, yvals_max+1, edgecolor='white',facecolor='white')
circ = patches.Ellipse((int(scale.get()*int(cursorXfixed.get())),int(scale.get()*int(cursorYfixed.get()))), 0,0, fill=False, edgecolor='white')
fig = plt.figure(figsize=(x_extent, y_extent))

horizontal_line = plt.axhline(color='white', lw=0.8, ls='-')
vertical_line = plt.axvline(color='white', lw=0.8, ls='-')

horizontal_line_fixed = plt.axhline(color='white', lw=0.8, ls='-')
vertical_line_fixed = plt.axvline(color='white', lw=0.8, ls='-')

x_slice_raw, = plt.plot([0], [0])
y_slice_raw, = plt.plot([0],[0])
x_slice, = plt.plot([0],[0])
y_slice, = plt.plot([0],[0])
energy_plot_line_long, = plt.plot([0],[0], ls = '-', color = 'yellow', alpha = 0.75 )
energy_plot, = plt.plot([0],[0], 'o')
energy_plot_line, = plt.plot([0],[0], 'r-')

cid = fig.canvas.mpl_connect('button_press_event', onclick)
maxText = plt.text(0, 0.8*y_extent/px-1,"", color = 'white', path_effects=[pe.withStroke(linewidth=2, foreground="black")])
midText = plt.text(0, 0.9*y_extent/px-1,"", color = 'white', path_effects=[pe.withStroke(linewidth=2, foreground="black")])
botText = plt.text(0, y_extent/px-1,"", color = 'white', path_effects=[pe.withStroke(linewidth=2, foreground="black")])
maxTextLong = plt.text(x_extent/px-100, 0.8*y_extent/px-1,"", color = 'white', path_effects=[pe.withStroke(linewidth=2, foreground="black")])
midTextLong = plt.text(x_extent/px-100, 0.9*y_extent/px-1,"", color = 'white', path_effects=[pe.withStroke(linewidth=2, foreground="black")])
botTextLong = plt.text(x_extent/px-100, y_extent/px-1,"", color = 'white', path_effects=[pe.withStroke(linewidth=2, foreground="black")])
maxLine = plt.axhline(y=0.8*y_extent/px, color='white', linestyle='--')
midLine = plt.axhline(y=0.9*y_extent/px, color='white', linestyle='--')
botLine = plt.axhline(y=y_extent/px, color='white', linestyle='--')

ROILeftLine = plt.axvline(color='white', lw=0.8, ls='-')
ROIRightLine = plt.axvline(color='white', lw=0.8, ls='-')
ROIBottomLine = plt.axhline(color='white', lw=0.8, ls='-')
ROITopLine = plt.axhline(color='white', lw=0.8, ls='-')

im = plt.imshow(np.zeros((yvals_max, xvals_max)), aspect = 'auto')

plt.axis('off')
fig.tight_layout()

im.axes.add_patch(rect)
im.axes.add_patch(circ)

canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
canvas.draw()

################################################################ camera refresh functions + logic variables  ##################################################

gau = simulateLaser()
eData = dataHistory()
flPeakData = dataHistory()
flAvgData = dataHistory()
fitxData = dataHistory()
fityData = dataHistory()
crdData = coordHistory()
pico = motorMover()
fitx = fitParams()
fity = fitParams()

imsx = tkinter.IntVar(value = xvals_max)
imsy = tkinter.IntVar(value = yvals_max)

energyCal = tkinter.DoubleVar(value = 1e6)
shot_avg = tkinter.IntVar(value = 20)
reset = tkinter.IntVar(value = 100)
resetLong = tkinter.IntVar(value = 10000)
EplotButton = tkinter.IntVar(value = 1)
EplotLong = tkinter.IntVar(value = 0)
numToSum = StringVar(value = '100')
EcalEnergy = tkinter.DoubleVar(value = 100.)
colorbarFl = tkinter.BooleanVar(value = False)
manualScale = tkinter.BooleanVar(value = False)
flMax = tkinter.DoubleVar(value = 2000)
flAng = tkinter.DoubleVar(value = 0.)
showFlMax = tkinter.BooleanVar(value = True)

ROIBboxButton = tkinter.IntVar(value = 0)
ROIShow = tkinter.BooleanVar(value = False)

saveImages = tkinter.BooleanVar(value = False)
saveFreq = tkinter.IntVar(value = 1)
saveFreq2 = tkinter.IntVar(value = 0)
saveTime = tkinter.DoubleVar(value = 0.)
lastSave = tkinter.DoubleVar(value = 0.)
saveName = StringVar(value = cam_name.get())

exposure_num = StringVar(value = str(exposure_time.get()))
clicked = StringVar(value = cam_name.get())
triggerClicked = StringVar(value = trigger_mode.get())

colorClicked = StringVar(value = 'viridis')
clrScale = tkinter.BooleanVar(value = False)

autoCursor = tkinter.BooleanVar(value = True)
cursorX = StringVar(value = '0')
cursorXstd = StringVar(value = '')
cursorY = StringVar(value = '0')
cursorYstd = StringVar(value = '')
cursorAvg = tkinter.IntVar(value = 10)
cursorReset = tkinter.IntVar(value = 100)

motorStep = tkinter.IntVar(value=5)
motorLim = tkinter.IntVar(value=1000)
motorsInitialized = tkinter.BooleanVar(value=False)
frameCounter = tkinter.IntVar(value = 0)
motorFrame = tkinter.BooleanVar(value = False)
aboveThreshold = tkinter.BooleanVar(value = False)

def stop():
    fig.canvas.mpl_disconnect(cid)
    try:
        pico.close()
    except:
        pass
    root.quit()
    root.destroy()    
button_quit = tkinter.Button(master=root, text="Quit", command = stop)

################################################################ Eplot popup box ##############################################################################

def clearEnergy():
    eData.__init__()

def EplotButtonfcn():
    if EplotButton.get() != 1:
        ROIEnable.set(False)
        maxText.set_text("")
        midText.set_text("")
        botText.set_text("")
        maxTextLong.set_text("")
        midTextLong.set_text("")
        botTextLong.set_text("")
        maxLine.set_visible(False)
        midLine.set_visible(False)
        botLine.set_visible(False)
        energy_plot.set_visible(False)
        energy_plot_line_long.set_visible(False)
        energy_plot_line.set_visible(False)
    else:
        energy_plot.set_visible(True)
        if EplotLong.get()==1:
            energy_plot_line_long.set_visible(True)
        energy_plot_line.set_visible(True)
        maxLine.set_visible(True)
        midLine.set_visible(True)
        botLine.set_visible(True)
        
def EplotLongfcn():
    if EplotLong.get() != 1:
        maxTextLong.set_text("")
        midTextLong.set_text("")
        botTextLong.set_text("")
        energy_plot_line_long.set_visible(False)
    else:
        energy_plot_line_long.set_visible(True)
        
def Ecalfcn():
    config.read(iniFname)
    EcalVal.set(EcalEnergy.get()/np.mean(eData.lineLong[-int(numToSum.get()):]))
    config[window.upper() + ' SETTINGS']['energy cal'] = str(EcalVal.get())
    with open(iniFname, 'w') as configfile:
        config.write(configfile)
        
def popup_energy():
    win = tkinter.Toplevel()
    win.wm_title("Energy settings")
    win.geometry("300x250"+"+"+str(origin[0])+"+"+str(origin[1]))

    clrE = ttk.Button(win, text="clear E plot", command=clearEnergy)
    
    showEplotButton = ttk.Checkbutton(win, text='E plot', variable=EplotButton, onvalue=1, offvalue=0, command=EplotButtonfcn)
    showEplotLong = ttk.Checkbutton(win, text='long time scale', variable=EplotLong, onvalue=1, offvalue=0, command=EplotLongfcn)

    EcalButton = ttk.Button(win, text="run E cal", command=Ecalfcn)
    EcallabelText = StringVar(value = "num to sum")
    EcallabelDir = Label(win, textvariable=EcallabelText)
    EcalEntry = ttk.Entry(win, textvariable=numToSum, width=5)
    EcalEnergylabelText = StringVar(value = "energy (mJ)")
    EcalEnergylabelDir = Label(win, textvariable=EcalEnergylabelText)
    EcalEnergyDir = ttk.Entry(win, textvariable=EcalEnergy, width=15)
    shot_avgText = StringVar(value = "shots to avg:")
    shot_avgDir = Label(win, textvariable=shot_avgText)
    shot_avgEntry = ttk.Entry(win, textvariable=shot_avg, width=5)
    energyLimitDir = Label(win, textvariable = StringVar(value = 'energy warning:'))
    energyLimitEntry = ttk.Entry(win, textvariable = energyLimit, width = 5)
    
    colorbarFlButton = ttk.Checkbutton(win, text='fluence colorbar', variable=colorbarFl, onvalue=True, offvalue=False)
    manualScaleButton = ttk.Checkbutton(win, text='manual colorbar', variable=manualScale, onvalue=True, offvalue=False)
    flMaxText = StringVar(value = "colorscale max (mJ/cm^2)")
    flMaxDir = Label(win, textvariable=flMaxText)
    flMaxEntry = ttk.Entry(win, textvariable=flMax, width=5)
    showFlMaxButton = ttk.Checkbutton(win, text='peak/avg fl', variable=showFlMax, onvalue=True, offvalue=False)
    flAngText = StringVar(value = "incidence angle:")
    flAngDir = Label(win, textvariable=flAngText)
    flAngEntry = ttk.Entry(win, textvariable=flAng, width=5)
        
    clrE.place(x=0, y=0)
    showEplotButton.place(x=0, y=25)
    showEplotLong.place(x=0, y=50)
    shot_avgDir.place(x=0, y=75)
    shot_avgEntry.place(x=75, y=75)
    EcallabelDir.place(x=0, y=100)
    EcalEntry.place(x=75, y=100)
    EcalEnergylabelDir.place(x=0, y=125)
    EcalEnergyDir.place(x=0, y=145)
    EcalButton.place(x=0, y=170)
    energyLimitDir.place(x=0, y=195)
    energyLimitEntry.place(x=100, y=195)
    
    colorbarFlButton.place(x=150, y=0)
    manualScaleButton.place(x=150, y=25)
    flMaxDir.place(x=150, y=50)
    flMaxEntry.place(x=150, y=70)
    showFlMaxButton.place(x=150, y=95)
    flAngDir.place(x=150, y=120)
    flAngEntry.place(x=240, y=120)
    
    b = ttk.Button(win, text="close window", command=win.destroy)
    b.place(x=0, y=220)
    
button_energy = ttk.Button(root, text="E settings", command=popup_energy)

################################################################ ROI popup box ################################################################################

def ROIBboxfcn():
    if ROIBboxButton.get()==1:
        ROIShow.set(True)
        if ROIEnable.get():
            cameraChange.set(True)
            ROIEnable.set(False)
        ROILeftLine.set_visible(True)
        ROIRightLine.set_visible(True)
        ROIBottomLine.set_visible(True)
        ROITopLine.set_visible(True)
    else:
        ROIShow.set(False)
        ROILeftLine.set_visible(False)
        ROIRightLine.set_visible(False)
        ROIBottomLine.set_visible(False)
        ROITopLine.set_visible(False)

def ROIChangefcn():
    if ROIBboxButton.get()==1:
        ROIBboxButton.set(0)
        ROIBboxfcn()
        cameraChange.set(True)
    else:
        cameraChange.set(True)
    config.read(iniFname)
    config[window.upper() + ' SETTINGS']['ROIEnable'] = str(ROIEnable.get())
    with open(iniFname, 'w') as configfile:
        config.write(configfile)
        
def saveROIfcn():
    # config.read(iniFname)
    config[window.upper() + ' SETTINGS']['ROILeft'] = str(ROILeft.get())
    config[window.upper() + ' SETTINGS']['ROIRight'] = str(ROIRight.get())
    config[window.upper() + ' SETTINGS']['ROIBottom'] = str(ROIBottom.get())
    config[window.upper() + ' SETTINGS']['ROITop'] = str(ROITop.get())
    with open(iniFname, 'w') as configfile:
        config.write(configfile)
    
def popup_ROI():
    winROI = tkinter.Toplevel()
    winROI.wm_title("ROI settings")
    winROI.geometry("300x150"+"+"+str(int(s_sizex/2+origin[0]))+"+"+str(int(0.8*s_sizey+origin[1])))
    
    showROIBboxButton = ttk.Checkbutton(winROI, text='show ROIBbox', variable=ROIBboxButton, onvalue=1, offvalue=0, command=ROIBboxfcn)
    showROIBboxButton.place(x=0, y=0)
    
    ROIlabelText = StringVar(value = "ROI (left, right, bottom, top)")
    ROIlabelDir = Label(winROI, textvariable=ROIlabelText)
    ROILeftDir = ttk.Entry(winROI, textvariable=ROILeft, width=5)
    ROIRightDir = ttk.Entry(winROI, textvariable=ROIRight, width=5)
    ROIBottomDir = ttk.Entry(winROI, textvariable=ROIBottom, width=5)
    ROITopDir = ttk.Entry(winROI, textvariable=ROITop, width=5)
    
    ROIEnableButton = ttk.Checkbutton(winROI, text='Enable ROI?', variable=ROIEnable, onvalue=True, offvalue=False, command=ROIChangefcn)

    saveROIButton = ttk.Button(winROI, text="Save ROI?", command=saveROIfcn)
    
    ROIlabelDir.place(x=0, y=20)
    ROILeftDir.place(x=0, y=45)
    ROIRightDir.place(x=40, y=45)
    ROIBottomDir.place(x=80, y=45)
    ROITopDir.place(x=120, y=45)
    
    ROIEnableButton.place(x=170, y=0)
    saveROIButton.place(x=170, y=30)
    
    bROI = ttk.Button(winROI, text="close window", command=winROI.destroy)
    bROI.place(x=0, y=70)
    
button_ROI = ttk.Button(root, text="ROI settings", command=popup_ROI)

################################################################ Save popup box ###############################################################################

def setSaveTimefcn():
    lastSave.set(0)

def popup_save():
    winSave = tkinter.Toplevel()
    winSave.wm_title("Save settings")
    winSave.geometry("350x180"+"+"+str(int(s_sizex/2+origin[0]))+"+"+str(int(s_sizey)))
    
    saveImagesButton = ttk.Checkbutton(winSave, text='save images', variable=saveImages, onvalue=True, offvalue=False, command = setSaveTimefcn)
    saveFolderlabelDir = Label(winSave, textvariable=StringVar(value='save folder'))
    saveFolderEntry = ttk.Entry(winSave, textvariable=saveFolder, width=50)
    saveNamelabelDir = Label(winSave, textvariable=StringVar(value='save file name'))
    saveNameEntry = ttk.Entry(winSave, textvariable=saveName, width=50)
    saveFreqlabelText = StringVar(value = "num of images to save every interval:")
    saveFreqlabelDir = Label(winSave, textvariable=saveFreqlabelText)
    saveFreqDir = ttk.Entry(winSave, textvariable=saveFreq, width=5)
    saveTimelabelText = StringVar(value = "interval time separation (minutes); 0 to continously save:")
    saveTimelabelDir = Label(winSave, textvariable=saveTimelabelText)
    saveTimeDir = ttk.Entry(winSave, textvariable=saveTime, width=5)
    
    saveImagesButton.place(x=0, y=0)
    saveFreqlabelDir.place(x=0, y=20)
    saveFreqDir.place(x=200, y=20)
    saveTimelabelDir.place(x=0, y=40)
    saveTimeDir.place(x=300, y=40)
    saveFolderlabelDir.place(x=0, y=60)
    saveFolderEntry.place(x=0, y=80)
    saveNamelabelDir.place(x=0, y=105)
    saveNameEntry.place(x=0, y=125)
    
    bSave = ttk.Button(winSave, text="close window", command=winSave.destroy)
    bSave.place(x=0, y=150)
    
button_save = ttk.Button(root, text="Save settings", command=popup_save)

################################################################ camera popup box ##############################################################################

def camChangefcn():
    if [clicked.get(), int(exposure_num.get()), triggerClicked.get()] != [cam_name.get(), exposure_time.get(), trigger_mode.get()]:
        print("camera change detected")
        config.read(iniFname)
        cam_name.set(clicked.get())
        exposure_time.set(int(exposure_num.get()))
        trigger_mode.set(triggerClicked.get())
        config['CAMERA NAMES'][window] = cam_name.get()
        config[window.upper() + ' SETTINGS']['exposure time'] = str(exposure_time.get())
        config[window.upper() + ' SETTINGS']['trigger mode'] = trigger_mode.get()
        with open(iniFname, 'w') as configfile:
            config.write(configfile)
        cam.camera.Close()
        if trigger_mode.get() == 'on':
            timeout.set(200)
        else:
            timeout.set(1000)
        cam.open_camera(cam_name.get(), exposure_time.get(), trigger_mode.get())
        cameraChange.set(True)

def popup_camera():
    winCam = tkinter.Toplevel()
    winCam.wm_title("Camera settings")
    winCam.geometry("250x170"+"+"+str(int(s_sizex/2+origin[0]))+"+"+str(int(1.5*s_sizey)))
    
    exposure_setting = ttk.Entry(winCam, textvariable=exposure_num, width=15)

    cameraText = StringVar(value = "camera select")
    cameraDir = Label(winCam, textvariable = cameraText)
    drop = OptionMenu(winCam, clicked, *cam.options)
    
    exposureText=StringVar()
    exposureText.set("exposure time (us)")
    exposureDir=Label(winCam, textvariable=exposureText)

    triggerText=StringVar()
    triggerText.set("trigger mode")
    triggerDir=Label(winCam, textvariable=triggerText)
    triggerDrop = OptionMenu(winCam, triggerClicked, *['on', 'off'])
    
    saveCamSettings = ttk.Button(winCam, text="change cam settings", command=camChangefcn)
    
    cameraDir.place(x=0, y=0)
    drop.place(x=0, y=20)
    exposureDir.place(x=0, y=50)
    exposure_setting.place(x=0, y=70)
    triggerDir.place(x=120, y=50)
    triggerDrop.place(x=120, y=70) 
    saveCamSettings.place(x=0, y=95)
    
    bCam = ttk.Button(winCam, text="close window", command=winCam.destroy)
    bCam.place(x=0, y=140)
    
button_cam = ttk.Button(root, text="Cam settings", command=popup_camera)

################################################################ plot settings popup box #######################################################################

def clrScalefcn():
    '''stupid way to add/remove colorbar'''
    if clrScale.get():
        axins = inset_axes(im.axes, # here using axis of the lowest plot
               width="3%",  # width = 5% of parent_bbox width
               height="30%",  # height : 340% good for a (4x4) Grid
               loc='upper left'
               )
        cb = plt.colorbar(cax = axins, orientation = 'vertical')
        cb.ax.yaxis.set_tick_params(color='white')
        plt.setp(plt.getp(cb.ax.axes, 'yticklabels'), color='white')
    else:
        for o in fig.findobj()[-2:]:
            try:
                o.remove()
            except:
                pass
            
def saveBkgndfcn():
    np.save(bkgndFile.get(), cam.arr)

def subtBkgndfcn():
    if subtBkgnd.get():
        cam.bkgnd = np.load(bkgndFile.get())

def saveCursorfcn():
    config.read(iniFname)
    config[window.upper() + ' SETTINGS']['cursor x pos'] = cursorXfixed.get()
    config[window.upper() + ' SETTINGS']['cursor y pos'] = cursorYfixed.get()
    config[window.upper() + ' SETTINGS']['cursor radius'] = cursorRad.get()
    with open(iniFname, 'w') as configfile:
        config.write(configfile)
        
def autoCursorfcn():
    if not autoCursor.get():
        coords[1] = int(float(cursorY.get())*yvals_max/imsy.get())
        coords[0] = int(float(cursorX.get())*xvals_max/imsx.get())
    else:
        cursorX.set(coords[0])
        cursorY.set(coords[1])
            
def popup_plot():
    winPlot = tkinter.Toplevel()
    winPlot.wm_title("Plot settings")
    winPlot.geometry("250x250"+"+"+str(int(s_sizex/2+origin[0]))+"+"+str(int(1.5*s_sizey)))
    
    colorText = StringVar(value = "color scale")
    colorDir = Label(winPlot, textvariable = colorText)
    colorDrop = OptionMenu(winPlot, colorClicked, *['viridis', 'hsv', 'plasma'])
    scaleButton = ttk.Checkbutton(winPlot, text='clrbar on/off', variable=clrScale, onvalue=True, offvalue=False, command = clrScalefcn)
    bkgndPathlabelText = StringVar(value = "Bckgnd file path")
    bkgndPathlabelDir = Label(winPlot, textvariable=bkgndPathlabelText)
    bkgndPath = ttk.Entry(winPlot, textvariable=bkgndFile)
    saveBkgnd = ttk.Button(winPlot, text="save bkgnd", command=saveBkgndfcn)
    subtBkgndButton = ttk.Checkbutton(winPlot, text='subt bkgnd', variable=subtBkgnd, onvalue=True, offvalue=False, command=subtBkgndfcn)
    autoCursorButton = ttk.Checkbutton(winPlot, text='autocursor?', variable=autoCursor, onvalue=True, offvalue=False, command=autoCursorfcn)
    cursorText = StringVar(value = "cursor coords (x, y)")
    cursorDir = Label(winPlot, textvariable = cursorText)
    cursorXlabel = Label(winPlot, textvariable=cursorX, width = 5)
    cursorYlabel = Label(winPlot, textvariable=cursorY, width = 5)
    cursorXstdlabel = Label(winPlot, textvariable=cursorXstd, width=5)
    cursorYstdlabel = Label(winPlot, textvariable=cursorYstd, width=5)
    
    cursorFixedText = StringVar(value = "fixed cursor coords and radius (x, y, rad)")
    cursorFixedDir = Label(winPlot, textvariable = cursorFixedText)
    cursorXentryFixed = ttk.Entry(winPlot, textvariable=cursorXfixed, width = 5)
    cursorYentryFixed = ttk.Entry(winPlot, textvariable=cursorYfixed, width = 5)
    cursorRadentry = ttk.Entry(winPlot, textvariable=cursorRad, width = 5)
    saveCursorButton = ttk.Button(winPlot, text="save settings", command=saveCursorfcn)
    cursorAvgText = StringVar(value = "num to avg:")
    cursorAvgDir = Label(winPlot, textvariable = cursorAvgText)
    cursorAvgentry = ttk.Entry(winPlot, textvariable=cursorAvg, width=5)
    
    scaleButton.place(x=0, y=0)
    colorDir.place(x=0, y=20)
    colorDrop.place(x=0, y=35)
    autoCursorButton.place(x=0, y=65)
    
    cursorDir.place(x=0, y=90)
    cursorXlabel.place(x=0, y=110)
    cursorXstdlabel.place(x=30, y=110)
    cursorYlabel.place(x=70, y=110)
    cursorYstdlabel.place(x=100, y=110)
    cursorAvgDir.place(x=150, y=110)
    cursorAvgentry.place(x=220, y=110)
    
    cursorFixedDir.place(x=0, y=135)
    cursorXentryFixed.place(x=0, y=155)
    cursorYentryFixed.place(x=50, y=155)
    cursorRadentry.place(x=100, y=155)
    saveCursorButton.place(x=0, y=180)
    
    bkgndPathlabelDir.place(x=110, y=0)
    bkgndPath.place(x=110, y=25)
    subtBkgndButton.place(x=110, y=50)
    saveBkgnd.place(x=110, y=80)
    
    bPlot = ttk.Button(winPlot, text="close window", command=winPlot.destroy)
    bPlot.place(x=0, y=210)
    
button_plot = ttk.Button(root, text="Plot settings", command=popup_plot)

################################################################  motor control popup box ######################################################################

def initializeOnfcn():
    if not motorsInitialized.get():
        # pico.initializeMotor()
        print('motors initialized')
        motorsInitialized.set(True)

def saveMotorfcn():
    config.read(iniFname)
    config[window.upper() + ' SETTINGS']['motor x pos'] = str(motorX.get())
    config[window.upper() + ' SETTINGS']['motor y pos'] = str(motorY.get())
    config[window.upper() + ' SETTINGS']['motor on'] = str(motorOn.get())
    config[window.upper() + ' SETTINGS']['motor error'] = str(motorError.get())
    config[window.upper() + ' SETTINGS']['motor threshold'] = str(motorThreshold.get())
    config[window.upper() + ' SETTINGS']['motor step size'] = str(motorStep.get())
    config[window.upper() + ' SETTINGS']['device name'] = str(deviceName.get())
    config[window.upper() + ' SETTINGS']['motor x number'] = str(motorXnum.get())
    config[window.upper() + ' SETTINGS']['motor y number'] = str(motorYnum.get())
    config[window.upper() + ' SETTINGS']['motor x multiplier'] = str(motorXmult.get())
    config[window.upper() + ' SETTINGS']['motor y multiplier'] = str(motorYmult.get())
    with open(iniFname, 'w') as configfile:
        config.write(configfile)

def jogPlusXfcn():
    if motorsInitialized.get():
        pico.move(deviceName.get(), motorXnum.get(), motorXmult.get()*motorStep.get(), motorLim.get())
    
def jogMinusXfcn():
    if motorsInitialized.get():
        pico.move(deviceName.get(), motorXnum.get(), -1*motorXmult.get()*motorStep.get(), motorLim.get())
    
def jogPlusYfcn():
    if motorsInitialized.get():
        pico.move(deviceName.get(), motorYnum.get(), motorYmult.get()*motorStep.get(), motorLim.get())
    
def jogMinusYfcn():
    if motorsInitialized.get():
        pico.move(deviceName.get(), motorYnum.get(), -1*motorYmult.get()*motorStep.get(), motorLim.get())

def popup_motor():
    winMotor = tkinter.Toplevel()
    winMotor.wm_title("Motor settings")
    winMotor.geometry("250x300"+"+"+str(int(s_sizex/2+origin[0]))+"+"+str(int(1.5*s_sizey)))
    
    initializeOnButton = ttk.Button(winMotor, text='init motors', command = initializeOnfcn, width = 10)
    
    motorOnButton = ttk.Checkbutton(winMotor, text='stabilizers on?', variable=motorOn, onvalue=True, offvalue=False)
    
    motorErrorText = StringVar(value = "allowed motor error (pix):")
    motorErrorDir = Label(winMotor, textvariable = motorErrorText)
    motorErrorEntry = ttk.Entry(winMotor, textvariable=motorError, width = 5)
    
    motorText = StringVar(value = "position to center motors to: ")
    motorXDir = Label(winMotor, textvariable = StringVar(value='x'))
    motorYDir = Label(winMotor, textvariable = StringVar(value='y'))
    motorDir = Label(winMotor, textvariable = motorText)
    motorXEntry = ttk.Entry(winMotor, textvariable=motorX, width = 5)
    motorYEntry = ttk.Entry(winMotor, textvariable=motorY, width = 5)
    
    motorThresholdText = StringVar(value = "threshold before motor turns on:")
    motorThresholdDir = Label(winMotor, textvariable = motorThresholdText)
    motorThresholdEntry = ttk.Entry(winMotor, textvariable=motorThreshold, width = 5)
    saveMotorButton = ttk.Button(winMotor, text="save settings", command=saveMotorfcn)
    
    motorStepText = StringVar(value = "motor step size:")
    motorStepDir = Label(winMotor, textvariable = motorStepText)
    motorStepEntry = ttk.Entry(winMotor, textvariable=motorStep, width = 5)
    
    deviceNameDir = Label(winMotor, textvariable=StringVar(value='device name'))
    deviceNameEntry = ttk.Entry(winMotor, textvariable=deviceName, width = 10)
    
    motorXnumDir = Label(winMotor, textvariable=StringVar(value='x num:'))
    motorXnumEntry = ttk.Entry(winMotor, textvariable=motorXnum, width = 5)
    motorYnumDir = Label(winMotor, textvariable=StringVar(value='y num:'))
    motorYnumEntry = ttk.Entry(winMotor, textvariable=motorYnum, width = 5)
    
    motorXmultDir = Label(winMotor, textvariable=StringVar(value='x mult:'))
    motorXmultEntry = ttk.Entry(winMotor, textvariable=motorXmult, width = 5)
    motorYmultDir = Label(winMotor, textvariable=StringVar(value='y mult:'))
    motorYmultEntry = ttk.Entry(winMotor, textvariable=motorYmult, width = 5)
    
    jogDir = Label(winMotor, textvariable=StringVar(value='Jog:'))
    jogPlusXButton = ttk.Button(winMotor, text="X+", command=jogPlusXfcn, width = 5)
    jogMinusXButton = ttk.Button(winMotor, text="X-", command=jogMinusXfcn, width = 5)
    jogPlusYButton = ttk.Button(winMotor, text="Y+", command=jogPlusYfcn, width = 5)
    jogMinusYButton = ttk.Button(winMotor, text="Y-", command=jogMinusYfcn, width = 5)
    
    initializeOnButton.place(x=0, y=0)
    motorOnButton.place(x=70, y=0)
    motorDir.place(x=0, y=25)
    motorXDir.place(x=170, y=5)
    motorYDir.place(x=210, y=5)
    motorXEntry.place(x=160, y=25)
    motorYEntry.place(x=200, y=25)
    motorThresholdDir.place(x=0, y=50)
    motorThresholdEntry.place(x=180, y=50)
    motorErrorDir.place(x=0, y=75)
    motorErrorEntry.place(x=140, y=75)
    motorStepDir.place(x=0, y=100)
    motorStepEntry.place(x=90, y=100)
    motorXnumDir.place(x=0, y=125)
    motorXnumEntry.place(x=50, y=125)
    motorYnumDir.place(x=100, y=125)
    motorYnumEntry.place(x=150, y=125)
    motorXmultDir.place(x=0, y=150)
    motorXmultEntry.place(x=50, y=150)
    motorYmultDir.place(x=100, y=150)
    motorYmultEntry.place(x=150, y=150)
    jogDir.place(x=0, y=175)
    jogPlusXButton.place(x=50, y=175)
    jogMinusXButton.place(x=100, y=175)
    jogPlusYButton.place(x=150, y=175)
    jogMinusYButton.place(x=200, y=175)
    deviceNameDir.place(x=0, y=200)
    deviceNameEntry.place(x=80, y=200)
    saveMotorButton.place(x=0, y=225)
    
    bMotor = ttk.Button(winMotor, text="close window", command=winMotor.destroy)
    bMotor.place(x=0, y=250)
    
buttom_motor = ttk.Button(root, text="Motor settings", command=popup_motor)

################################################################ funcAnimation functions #######################################################################

def grabCamera():
    try:
        if not cam.synthetic:
            grabResult = cam.camera.RetrieveResult(timeout.get(), pylon.TimeoutHandling_ThrowException) 
            arr = grabResult.GetArray()
            img = np.array(arr, dtype = 'float64')
            cam.arr = np.copy(img)
        else:
            img = np.copy(cam.arr)+np.random.normal(scale = 10, size = cam.arr.shape)
    except:
        if not singleShot.get():
            cam.camera.Close()
            cam.open_camera(cam_name.get(), exposure_time.get(), trigger_mode.get())
            grabResult = cam.camera.RetrieveResult(timeout.get(), pylon.TimeoutHandling_ThrowException)        
            arr = grabResult.GetArray()
            img = np.array(arr, dtype = 'float64')
            cam.arr = img
        else:
            if cam.arr.shape[0]>1:
                img = np.copy(cam.arr)
            else:
                img = np.zeros((int(scale.get()*cam.height), int(scale.get()*cam.width)))
    if subtBkgnd.get():
        img -= cam.bkgnd
    if ROIEnable.get():
        img = np.copy(img[int(ROIBottom.get()):int(ROITop.get()), int(ROILeft.get()):int(ROIRight.get())])
#    try:
#        img += gau.gaussarr
#    except:
#        pass
    try:
        energy = eData.line[-1]*EcalVal.get()
        stddev = eData.std[-1]*EcalVal.get()
    except:
        energy = np.sum(img)*EcalVal.get()
        stddev = 0
    energyText.set("%.1f"%(energy))
    if energy <energyLimit.get():
        energylabelDir.config(fg= 'red')
        energyDir.config(fg = 'red')
    else:
        energylabelDir.config(fg= 'black')
        energyDir.config(fg = 'black')
        
    if energy>motorThreshold.get():
        aboveThreshold.set(True)
    else:
        aboveThreshold.set(False)
    
    if fittingini.get() and aboveThreshold.get():
        fitting.set(True)
        fittingini.set(False)
    try:
        flarr = np.copy(img)*EcalVal.get()/(umperpix.get()**2*(1e-4)**2)*np.cos(np.deg2rad(flAng.get()))
    except:
        flarr = np.copy(img)*EcalVal.get()/(umperpix.get()**2*(1e-4)**2)
    flPeakData.addData(np.max(flarr), shot_avg.get(), reset.get(), resetLong.get())
    flAvgData.addData(np.mean(flarr), shot_avg.get(), reset.get(), resetLong.get())
    if showFlMax.get():
        fluencelabelText.set("Peak fl (mJ/cm^2)")
        if flPeakData.line[-1] < 10000:
            fluenceText.set("%.1f"%(flPeakData.line[-1]))
        else:
            fluenceText.set("%.1e"%(flPeakData.line[-1]))
    else:
        fluencelabelText.set("Avg fl (mJ/cm^2)")
        if flPeakData.line[-1] < 10000:
            fluenceText.set("%.1f"%(flPeakData.line[-1]))
        else:
            fluenceText.set("%.1e"%(flPeakData.line[-1]))
    if stddev < 0.1:
        stdText.set("%.1e"%(stddev))
    else:
        stdText.set("%.1f"%(stddev))
    return img

def peak_and_fit(img):
    fittingFailed=False
    s = img.shape
    y1 = np.arange(0,s[0])
    x1 = np.arange(0,s[1])
    yvals = np.arange(int(scale.get()*s[0]))
    xvals = np.arange(int(scale.get()*s[1]))
    keep_running = True
    i = 0
    mcrdmax = np.where(img == np.max(img))
    if not autoCursor.get():
        mcrd = mcrdmax
    else:
        if crdData.arr.any():
            mcrd = np.array([[crdData.arr[-1][1]], [crdData.arr[-1][0]]])
        else:
            mcrd = mcrdmax
    
    slicey = img[:, int(mcrd[1][0])]
    slicex = img[int(mcrd[0][0]),:]
    
    if fitx.params.any() and fity.params.any():
        p0x = fitx.params
        p0y = fity.params
    else:
        p0x = [s[1]//2, 300, 3, np.max(slicex), np.min(slicex)]
        p0y = [s[0]//2, 300, 3, np.max(slicey), np.min(slicey)]
    
    if np.max(slicex)>0:
        xheight = (0.2*yvals_max)/np.max(slicex)
        yheight = (0.2*xvals_max)/np.max(slicey)
    else:
        xheight = (0.2*yvals_max)
        yheight = (0.2*xvals_max)
    
    x_slice_raw.set_data(np.linspace(0, xvals_max, num=len(slicex)),
                         slicex*xheight)
    y_slice_raw.set_data(slicey*yheight,
                         np.linspace(0, yvals_max, num=len(slicey)))
    if fitting.get():
        try:
            poptx, pcovx = curve_fit(supergauss_1D, x1, slicex, p0 = p0x, bounds = [(0, 0, 0, 0, -np.inf), (s[1], np.inf, np.inf, np.inf, np.inf)])
            popty, pcovy = curve_fit(supergauss_1D, y1, slicey, p0 = p0y,  bounds = [(0, 0, 0, 0, -np.inf), (s[0], np.inf, np.inf, np.inf, np.inf)])
            if poptx[1] < 3*s[1]:
                if popty[1] < 3*s[0]:
                    fitx.setParams(poptx)
                    fity.setParams(popty)
                else:
                    fitx.setParams([])
                    fity.setParams([])
            fwhmxText.set("%.2f"%(poptx[1]*umperpix.get()/1000))
            fwhmyText.set("%.2f"%(popty[1]*umperpix.get()/1000))
            fwhmxlabelText.set("FWHMx (mm)")
            fwhmylabelText.set("FWHMy (mm)")
            x_slice.set_data(np.linspace(0, xvals_max, num=len(xvals)),
                             xheight*supergauss_1D(np.linspace(0, len(slicex), num=len(xvals)), *poptx))
            y_slice.set_data(yheight*supergauss_1D(np.linspace(0, len(slicey), num=len(yvals)), *popty),
                             np.linspace(0, yvals_max, num=len(yvals)))
        except:
            fitx.setParams([])
            fity.setParams([])
            fwhmxText.set("")
            fwhmyText.set("")
            fwhmxlabelText.set("")
            fwhmylabelText.set("")
            fittingFailed=True
            print("fitting failed")
    else:
        fitx.setParams([])
        fity.setParams([])
        fwhmxText.set("")
        fwhmyText.set("")
        fwhmxlabelText.set("")
        fwhmylabelText.set("")
        x_slice.set_data([0],[0])
        y_slice.set_data([0],[0])
    try:
        if fitting.get() and not fittingFailed:
            crdData.addData([int(poptx[0]), int(popty[0])], cursorAvg.get(), reset.get())
        else:
            crdData.addData([int(mcrd[1][0]), int(mcrd[0][0])], cursorAvg.get(), reset.get())
    except:
        pass
    if autoCursor.get():
        # why did i do it like this, need to simplify...
        cursorX.set(int(float(np.round(crdData.line[-1][0]))))
        cursorXstd.set('+/-%d'%crdData.std[-1][0])
        cursorY.set(int(float(np.round(crdData.line[-1][1]))))
        cursorYstd.set('+/-%d'%crdData.std[-1][1])
    else:
        cursorX.set(int(coords[0]*imsx.get()/xvals_max))
        cursorXstd.set('')
        cursorY.set(int(coords[1]*imsy.get()/yvals_max))
        cursorYstd.set('')
            
def plotEnergy(img):
    arr = eData.arr*EcalVal.get()
    line = eData.line*EcalVal.get()
    lineLong = eData.lineLong*EcalVal.get()
    if (np.max(arr)-np.min(arr))>0:
        eheight = (0.2*yvals_max)/(np.max(arr)-np.min(arr))
        eheightLong = (0.2*yvals_max)/(np.max(lineLong)-np.min(lineLong))
        
    else:
        eheight = (0.2*yvals_max)
        eheightLong = (0.2*yvals_max)
        
    eplotArr = yvals_max*np.ones_like(arr)-eheight*(arr - np.min(arr))
    eplotLine = yvals_max*np.ones_like(line)-eheight*(line - np.min(arr))
    maxText.set_text("%.1f"%(np.max(arr)))
    midText.set_text("%.1f"%((np.max(arr)+np.min(arr))/2))
    botText.set_text("%.1f"%(np.min(arr)))
    energy_plot.set_data(
        np.linspace(0, xvals_max, num=len(arr)),
        eplotArr
    )
    energy_plot_line.set_data(
        np.linspace(0, xvals_max, num=len(line)),
        eplotLine
    )
    if EplotLong.get() == 1:
        eplotLineLong = yvals_max*np.ones_like(lineLong)-eheightLong*(lineLong - np.min(lineLong))
        maxTextLong.set_text("%.1f"%(np.max(lineLong)))
        midTextLong.set_text("%.1f"%((np.max(lineLong)+np.min(lineLong))/2))
        botTextLong.set_text("%.1f"%(np.min(lineLong)))
        energy_plot_line_long.set_data(
        np.linspace(0, xvals_max, num=len(lineLong)),
        eplotLineLong
        )
        
def plotROI():
    if ROIShow.get():
        try:
            ROILeftLine.set_xdata([int(int(ROILeft.get())*xvals_max/imsx.get())])
            ROIRightLine.set_xdata([int(int(ROIRight.get())*xvals_max/imsx.get())])
            ROIBottomLine.set_ydata([int(int(ROIBottom.get())*yvals_max/imsy.get())])
            ROITopLine.set_ydata([int(int(ROITop.get()*yvals_max/imsy.get()))])
        except:
            pass
        
def plot_and_save(img):
    if saveImages.get():
        et = time.time()
        if et - lastSave.get()  > saveTime.get()*60:
            if saveFreq2.get() < saveFreq.get():
                i = saveFreq2.get()
                i+=1
                saveFreq2.set(i)
                fname = str(datetime.now()) + "_" + saveName.get()
                fname = fname.replace(":", "-")
                fname = fname.replace(".", "-")
                if saveFolder.get()[-1] !="\\":
                    sF = saveFolder.get()
                    saveFolder.set(sF+"\\")
                saveImg(saveFolder.get() + fname, img)
            else:
                if saveTime.get()==0:
                    saveImages.set(False)
                    saveFreq2.set(0)
                lastSave.set(et)
        else:
            saveFreq2.set(0)
    else:
        lastSave.set(0)

def plotCursors():
    if not autoCursor.get():
        horizontal_line.set_ydata([coords[1]])
        vertical_line.set_xdata([coords[0]])
    else:
        horizontal_line.set_ydata([int(float(cursorY.get())*yvals_max/imsy.get())])
        vertical_line.set_xdata([int(float(cursorX.get())*xvals_max/imsx.get())])
    try:
        xpoint = int(int(cursorXfixed.get())*xvals_max/imsx.get())
        ypoint = int(int(cursorYfixed.get())*yvals_max/imsy.get())
        horizontal_line_fixed.set_ydata([ypoint])
        vertical_line_fixed.set_xdata([xpoint])
        circ.set_center((xpoint, ypoint))
        circ.set_width(int(int(cursorRad.get())*xvals_max/imsx.get()))
        circ.set_height(int(int(cursorRad.get())*yvals_max/imsy.get()))
    except:
        pass

def plotImg(img):
    imgnew = np.copy(img)
    if colorbarFl.get():
        try:
            imgnew *= (EcalVal.get()/(umperpix.get()**2*(1e-4)**2)*np.cos(np.deg2rad(flAng.get())))
        except:
            imgnew *= EcalVal.get()/(umperpix.get()**2*(1e-4)**2)
    img2 = rescale(imgnew, scale.get(), anti_aliasing=False)
    im.set_array(img2)
    im.set_cmap(colorClicked.get())
    if manualScale.get():
        try:
            im.set_clim(vmin=np.min(img2), vmax=flMax.get())
        except:
            im.set_clim(vmin=np.min(img2), vmax=np.max(img2))
    else:
        im.set_clim(vmin=np.min(img2), vmax=np.max(img2))
        
def stabilizeBeam():
    if initializeOn.get():
        initializeOnfcn()
    if frameCounter.get() % cursorAvg.get() == 0:
        motorFrame.set(True)
    else:
        motorFrame.set(False)
    if motorOn.get() and motorsInitialized.get() and motorFrame.get() and aboveThreshold.get():
        try:
            xpos = crdData.line[-1][0]
            ypos = crdData.line[-1][1]
            xtarget = motorX.get()
            ytarget = motorY.get()
            xdiff = xpos-xtarget
            xsign = int(math.copysign(1, xdiff))
            ydiff = ypos-ytarget
            ysign = int(math.copysign(1, ydiff))
            for i, mtr in enumerate([motorXnum.get(), motorYnum.get()]):
                if i==0:
                    if abs(xdiff) > motorError.get():
                        pico.move(deviceName.get(), motorXnum.get(), xsign*motorXmult.get()*motorStep.get(), motorLim.get())
                else:
                    if abs(ydiff) > motorError.get():
                        pico.move(deviceName.get(), motorYnum.get(), ysign*motorYmult.get()*motorStep.get(), motorLim.get())
        except:
            print('motor control failed')
    
def clearPlot():
    im.axes.add_patch(rect)
    x_slice_raw.set_data([0], [0])
    y_slice_raw.set_data([0],[0])
    x_slice.set_data([0], [0])
    y_slice.set_data([0],[0])
    energy_plot.set_data([0],[0])
    energy_plot_line.set_data([0],[0])
    energy_plot_line_long.set_data([0],[0])
    horizontal_line.set_color('white')
    vertical_line.set_color('white')
    horizontal_line_fixed.set_color('white')
    vertical_line_fixed.set_color('white')
    maxLine.set_color('white')
    midLine.set_color('white')
    botLine.set_color('white')
    maxText.set_text("")
    midText.set_text("")
    botText.set_text("")
    maxTextLong.set_text("")
    midTextLong.set_text("")
    botTextLong.set_text("")
    ROILeftLine.set_color('white')
    ROIRightLine.set_color('white')
    ROIBottomLine.set_color('white')
    ROITopLine.set_color('white')
    circ.set_color('white')
    
def reAdjustPlot(img):
    s2 = img.shape
    imsx.set(s2[1])
    imsy.set(s2[0])
#    gau.generateArr(s2[0], s2[1])
    px = 1/plt.rcParams['figure.dpi']
    y_extent2 = s2[0]*px
    x_extent2 = s2[1]*px
    scale.set(0.8*s_sizey/s2[0])
    if scale.get()*s2[1] > s_sizex:
        scale.set(s_sizex/s2[1])
    fig.set_size_inches(scale.get()*x_extent2, scale.get()*y_extent2)
    horizontal_line.set_color('yellow')
    vertical_line.set_color('yellow')
    horizontal_line_fixed.set_color('red')
    vertical_line_fixed.set_color('red')
    maxTextLong.set_position((0.85*scale.get()*s2[1], 0.78*scale.get()*s2[0]))
    midTextLong.set_position((0.85*scale.get()*s2[1], 0.88*scale.get()*s2[0]))
    botTextLong.set_position((0.85*scale.get()*s2[1], 0.98*scale.get()*s2[0]))
    maxLine.set_color('orange')
    midLine.set_color('orange')
    botLine.set_color('orange')
    ROILeftLine.set_color('white')
    ROIRightLine.set_color('white')
    ROIBottomLine.set_color('white')
    ROITopLine.set_color('white')
    circ.set_color('red')
    
st = time.time()

############################################################# FuncAnimation loop ##############################################################################

def update_plot(i):
    frameCounter.set(i)
    # et = time.time()
    # labelText.set("%.1f"%(et-st))
    try:
        img = grabCamera()
        if cameraChange.get() and cameraChange2.get():
            reAdjustPlot(img)
            cameraChange.set(False)
            cameraChange2.set(False)
        eData.addData(np.sum(img), shot_avg.get(), reset.get(), resetLong.get())
        plotImg(img)
        try:
            rect.remove()
        except:
            pass
        peak_and_fit(img)
        plotCursors()
        plotROI()
        if EplotButton.get() == 1:
            plotEnergy(img)
        plot_and_save(img)
        stabilizeBeam()
        if cameraChange.get():
            clearPlot()
            cameraChange2.set(True)
    except:
        print("failed to update frame")
    return [im]

############################################################# Tkinter gui elements placement ##################################################################

canvas.get_tk_widget().place(x=0, y=0)
button_quit.place(x=s_sizex - 50, y=s_sizey-30)
labelDir.place(x=500, y=s_sizey-75)
camNameDir.place(x=150, y=s_sizey-60)
energylabelDir.place(x=150, y=s_sizey-110)
energyDir.place(x=150, y=s_sizey-90)
stdlabelDir.place(x=240, y=s_sizey-110)
stdDir.place(x=240, y=s_sizey-90)
fluencelabelDir.place(x=340, y=s_sizey-110)
fluenceDir.place(x=340, y=s_sizey-90)
fwhmxlabelDir.place(x=450, y=s_sizey-110)
fwhmxDir.place(x=450, y=s_sizey-90)
fwhmylabelDir.place(x=540, y=s_sizey-110)
fwhmyDir.place(x=540, y=s_sizey-90)
gauss_fit_button.place(x=0, y=s_sizey-110)
button_energy.place(x=0, y=s_sizey-80)
button_ROI.place(x=0, y=s_sizey-55)
button_save.place(x=75, y=s_sizey-80)
button_cam.place(x=75, y = s_sizey-55)
button_plot.place(x=0, y=s_sizey-30)
buttom_motor.place(x=75, y=s_sizey-30)

############################################################# looping function ################################################################################
if exposure_time.get() < 100000 and trigger_mode.get() != 'on':
    interval = 80
else:
    interval = 0
ani = FuncAnimation(fig, update_plot, interval = interval, repeat=False, cache_frame_data=False)
root.mainloop()