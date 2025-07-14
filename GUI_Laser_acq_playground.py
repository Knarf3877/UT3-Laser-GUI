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
try:
    # Import the .NET Common Language Runtime (CLR) to allow interaction with .NET
    import clr
except ImportError:
    print("no .NET Common Language Runtime (CLR) found. Can't run picomotors...")

def getResolution(monitor):
    crd = monitor.find('x')
    width = int(monitor[:crd])
    height = int(monitor[crd+1:])
    return width, height

class camAcq():
    def __init__(self, synthetic_camera=False, synth_shape=(1000, 1000)):
        self.synthetic = synthetic_camera
        self.options = np.array(["ayy", "lmao"])
        self.arr = np.zeros(synth_shape)
        self.shape = synth_shape
        self.width = self.shape[1]
        self.height = self.shape[0]

def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata
    global coords
    coords = [ix, iy]
    ROIBox[0] = coords
#     if len(coords) == 2:
#         fig.canvas.mpl_disconnect(cid)
    return coords

def onrelease(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata
    global coords
    coords = [ix, iy]
    ROIBox[1] = coords
    if dragToSelectROI:
        fillInROIField(ROIBox)
#     if len(coords) == 2:
#         fig.canvas.mpl_disconnect(cid)
    return coords

def on_tab_change(event):
    tab = event.widget.tab('current')['text']
    global dragToSelectROI
    if tab == 'ROI':
        dragToSelectROI = True
    else:
        dragToSelectROI = False

cam = camAcq()
root = tkinter.Tk()
window = 'window1'
iniFname = 'front_end_settings.ini'
print('reading settings from: %s' % iniFname)
config = configparser.ConfigParser()
config.read(iniFname)
cam_name = tkinter.StringVar()
cam_name.set(config['CAMERA NAMES'][window])
exposure_time = tkinter.IntVar()
exposure_time.set(int(config[window.upper() + ' SETTINGS']['exposure time']))
trigger_mode = tkinter.StringVar()
trigger_mode.set(config[window.upper() + ' SETTINGS']['trigger mode'])
GigE = tkinter.BooleanVar(value=config[window.upper() + ' SETTINGS']['GigE'] == 'True')
packetSize = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['packet size']))
packetDelay = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['packet delay']))
frameDelay = tkinter.DoubleVar(value=float(config[window.upper() + ' SETTINGS']['frame delay']))
GUIPause = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['GUIPause']))

EcalVal = tkinter.DoubleVar(value=float(config[window.upper() + ' SETTINGS']['energy cal']))
energyLimit = tkinter.DoubleVar(value=float(config[window.upper() + ' SETTINGS']['energy warning value']))

ROIEnable = tkinter.BooleanVar(value=config[window.upper() + ' SETTINGS']['roienable'] == 'True')
ROILeft = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['roileft']))
ROIRight = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['roiright']))
ROIBottom = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['roibottom']))
ROITop = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['roitop']))

saveFolder = StringVar(value=config[window.upper() + ' SETTINGS']['save folder'])

bkgndFile = StringVar(value=config[window.upper() + ' SETTINGS']['bkgnd file'])
subtBkgnd = tkinter.BooleanVar(value=config[window.upper() + ' SETTINGS']['subtract background'] == 'True')

cursorXfixed = StringVar(value=config[window.upper() + ' SETTINGS']['cursor x pos'])
cursorYfixed = StringVar(value=config[window.upper() + ' SETTINGS']['cursor y pos'])
cursorRad = StringVar(value=config[window.upper() + ' SETTINGS']['cursor radius'])

motorX = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['motor x pos']))
motorY = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['motor y pos']))
motorOn = tkinter.BooleanVar(value=config[window.upper() + ' SETTINGS']['motor on'] == 'True')
motorError = tkinter.DoubleVar(value=float(config[window.upper() + ' SETTINGS']['motor error']))
motorThreshold = tkinter.DoubleVar(value=float(config[window.upper() + ' SETTINGS']['motor threshold']))
motorStep = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['motor step size']))
deviceName = StringVar(value=config[window.upper() + ' SETTINGS']['device name'])
motorXnum = StringVar(value=config[window.upper() + ' SETTINGS']['motor x number'])
motorYnum = StringVar(value=config[window.upper() + ' SETTINGS']['motor y number'])
motorXmult = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['motor x multiplier']))
motorYmult = tkinter.IntVar(value=int(config[window.upper() + ' SETTINGS']['motor y multiplier']))
initializeOn = tkinter.BooleanVar(value=config[window.upper() + ' SETTINGS']['initialize motors at start'] == 'True')
usingMotors = tkinter.BooleanVar(value=config[window.upper() + ' SETTINGS']['using motors'] == 'True')

umperpix = tkinter.DoubleVar(value=float(config[window.upper() + ' SETTINGS']['umperpix']))
fittingini = tkinter.BooleanVar(value=config[window.upper() + ' SETTINGS']['gauss fitting'] == 'True')
fitting = tkinter.BooleanVar(value=False)
singleShot = tkinter.BooleanVar(value=config[window.upper() + ' SETTINGS']['single shot'] == 'True')

wPosition = config[window.upper() + ' SETTINGS']['position']
screen_size_horiz = int(config[window.upper() + ' SETTINGS']['horizontal screen size'])
screen_size_vert = int(config[window.upper() + ' SETTINGS']['vertical screen size'])

smallAlignmentZone = tkinter.IntVar(value = (config[window.upper() + ' SETTINGS']['small alignment zone']))


dragToSelectROI = False
ROIBox = [[0,0],[0,0]]

mon = config[window.upper() + ' SETTINGS']['monitor']
if mon[-1] == '1':
    monitor = config['MONITOR RESOLUTION'][mon]
else:
    monitor = config['MONITOR RESOLUTION'][mon[:-1]]

timeout = tkinter.IntVar(value = int(config[window.upper() + ' SETTINGS']['timeout']))
timeoutOrig = tkinter.IntVar(value = timeout.get())
#cam = camAcq(synthetic_camera = True)

coords = [0,0]

screen_width, screen_height = getResolution(monitor)
screen_height -=60
position = {'top left':[0, 0],
            'top center':[screen_width//screen_size_horiz, 0],
            'top right':[(screen_width-screen_width//screen_size_horiz), 0],
            'bottom left':[0, screen_height//screen_size_vert+30],
            'bottom center':[screen_width//screen_size_horiz, screen_height//screen_size_vert+30],
            'bottom right':[(screen_width - screen_width//screen_size_horiz), screen_height//screen_size_vert+30]
           }


winVal = 1
n_cams =[3,2]
xorigin = (screen_width//n_cams[0])*winVal if winVal <n_cams[0] else (screen_width//n_cams[0])*(winVal-n_cams[0])
yorigin = 0 if winVal <n_cams[0] else screen_height//2+30
#xorigin, yorigin = position[wPosition]

origin = [xorigin, yorigin]

s_sizex = screen_width//screen_size_horiz
s_sizey = screen_height//screen_size_vert
screen = str(s_sizex)+"x"+str(s_sizey)+"+"+str(origin[0])+"+"+str(origin[1])

root.wm_title(window)
root.geometry(screen)

root.rowconfigure(0, weight=2)  # Top half (img)
root.rowconfigure(1, weight=1)  # Bottom half (tabs)
root.rowconfigure(2, minsize = 25)  # Bottom (quit button)
root.columnconfigure(0, weight=1)

main_frame = tkinter.Frame(root)
main_frame.grid(row=0, column=0, sticky="nsew")
tab_frame = tkinter.Frame(root)
tab_frame.grid(row=1, column=0, sticky="nsew")
bottom_frame = tkinter.Frame(root)
bottom_frame.grid(row=2, column=0, sticky="nsew")

############################## tabs ########################################
tabControl = ttk.Notebook(tab_frame)

mainTab = ttk.Frame(tabControl)
energyTab = ttk.Frame(tabControl)
savingTab = ttk.Frame(tabControl)
ROITab = ttk.Frame(tabControl)
camerasTab = ttk.Frame(tabControl)
plotAdjTab = ttk.Frame(tabControl)
motorsTab = ttk.Frame(tabControl)

tabControl.add(mainTab, text='Main Image')
tabControl.add(energyTab, text='Energy/fl')
tabControl.add(savingTab, text='Saving')
tabControl.add(ROITab, text='ROI')
tabControl.add(camerasTab, text='Cameras')
tabControl.add(plotAdjTab, text='Plot Adj')
tabControl.add(motorsTab, text='Motors')
tabControl.pack(expand=1, fill="both")

###########################################################################

labelText = StringVar()
labelText.set("")
labelDir = Label(mainTab, textvariable=labelText, font = ("arial",40))

camNameText = StringVar(value = 'TEST') #cam_name.get()
camNameDir = Label(bottom_frame, textvariable=camNameText, font = ("arial",10))

fwhmxlabelText = StringVar(value = "")
fwhmxlabelDir = Label(mainTab, textvariable=fwhmxlabelText, font = ("arial",10))
fwhmxText = StringVar(value = "")
fwhmxDir = Label(mainTab, textvariable=fwhmxText, font = ("arial",20))

fwhmylabelText = StringVar(value = "")
fwhmylabelDir = Label(mainTab, textvariable=fwhmylabelText, font = ("arial",10))
fwhmyText = StringVar(value = "")
fwhmyDir = Label(mainTab, textvariable=fwhmyText, font = ("arial",20))

energylabelText = StringVar(value = "energy (mJ)")
energylabelDir = Label(mainTab, textvariable=energylabelText, font = ("arial",10))
energyText = StringVar(value = "")
energyDir = Label(mainTab, textvariable=energyText, font = ("arial",20))

stdlabelText = StringVar(value = "std dev (mJ)")
stdlabelDir = Label(mainTab, textvariable=stdlabelText, font = ("arial",10))
stdText = StringVar(value = "")
stdDir = Label(mainTab, textvariable=stdText, font = ("arial",20))

fluencelabelText = StringVar(value = "")
fluencelabelDir = Label(mainTab, textvariable=fluencelabelText, font = ("arial",10))
fluenceText = StringVar(value = "")
fluenceDir = Label(mainTab, textvariable=fluenceText, font = ("arial",20))

##########################################################################################

def place_gui_elems(yvals_max):
    #offset = 10
    button_quit.place(x= s_sizex - 50, y=0)
    # labelDir.place(x=500, y=s_sizey-75)
    camNameDir.place(x=10, y= 0)
    energylabelDir.place(x=0, y=0)
    energyDir.place(x=0, y=20)
    stdlabelDir.place(x=90, y=0)
    stdDir.place(x=90, y=20)
    fluencelabelDir.place(x=190, y=0)
    fluenceDir.place(x=190, y=20)
    fwhmxlabelDir.place(x=300, y=0)
    fwhmxDir.place(x=300, y=20)
    fwhmylabelDir.place(x=390, y=0)
    fwhmyDir.place(x=390, y=20)

####################################################################

cameraChange = tkinter.BooleanVar(value = True)
cameraChange2 = tkinter.BooleanVar(value = False)
screenScale = 1-80/s_sizey
scale = tkinter.DoubleVar(value = s_sizex/cam.width)
if scale.get()*cam.height > screenScale*s_sizey:
    scale.set(screenScale*s_sizey/cam.height)
orig_scale = scale.get()
xvals = np.arange(int(scale.get()*cam.width))
yvals = np.arange(int(scale.get()*cam.height))
xvals_max = s_sizex
yvals_max = int(screenScale*s_sizey)
px = 1/plt.rcParams['figure.dpi']

x_extent = s_sizex*px
y_extent = screenScale*s_sizey*px
rect = patches.Rectangle((-2, -2), xvals_max+2, yvals_max+2, edgecolor='white',facecolor='white')
circ = patches.Ellipse((int(scale.get()*int(cursorXfixed.get())),int(scale.get()*int(cursorYfixed.get()))), 0,0, fill=False, edgecolor='white')
fig = plt.figure(figsize=(x_extent, y_extent))

horizontal_line = plt.axhline(color='white', lw=0.8, ls='-')
vertical_line = plt.axvline(color='white', lw=0.8, ls='-')

horizontal_line_fixed = plt.axhline(color='white', lw=0.8, ls='-')
vertical_line_fixed = plt.axvline(color='white', lw=0.8, ls='-')

x_slice_raw, = plt.plot([0],[0])
y_slice_raw, = plt.plot([0],[0])
x_slice, = plt.plot([0],[0])
y_slice, = plt.plot([0],[0])
energy_plot_line_long, = plt.plot([0],[0], ls = '-', color = 'yellow', alpha = 0.75 )
energy_plot, = plt.plot([0],[0], 'o')
energy_plot_line, = plt.plot([0],[0], 'r-')

cid = fig.canvas.mpl_connect('button_press_event', onclick)
rid = fig.canvas.mpl_connect('button_release_event', onrelease)
tabControl.bind('<<NotebookTabChanged>>', on_tab_change)
print(tabControl.tab(tabControl.select(), "text"))
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
plt.margins(x=0)
plt.margins(y=0)

plt.axis('off')
fig.tight_layout()

im.axes.add_patch(rect)
im.axes.add_patch(circ)

canvas = FigureCanvasTkAgg(fig, main_frame)  # A tk.DrawingArea.
canvas.draw()

################################################################ camera refresh functions + logic variables  ##################################################

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

skipFrame = tkinter.BooleanVar(value = False)
openBox = tkinter.BooleanVar(value = False)
boxFrame = tkinter.IntVar(value = 0)
startCount = tkinter.BooleanVar(value = False)

def stop():
    fig.canvas.mpl_disconnect(cid)
    fig.canvas.mpl_disconnect(rid)
    try:
        pico.close()
    except:
        pass
    root.quit()
    root.destroy()
button_quit = tkinter.Button(bottom_frame, text="Quit", command = stop)

################################################################################################################################
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
        if EplotLong.get() == 1:
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
    EcalVal.set(EcalEnergy.get() / np.mean(eData.lineLong[-int(numToSum.get()):]))
    config[window.upper() + ' SETTINGS']['energy cal'] = str(EcalVal.get())
    with open(iniFname, 'w') as configfile:
        config.write(configfile)


def popup_energy(tab):
    openBox.set(True)
    win = tab
    clrE = ttk.Button(win, text="clear E plot", command=clearEnergy)

    showEplotButton = ttk.Checkbutton(win, text='E plot', variable=EplotButton, onvalue=1, offvalue=0,
                                      command=EplotButtonfcn)
    showEplotLong = ttk.Checkbutton(win, text='long time scale', variable=EplotLong, onvalue=1, offvalue=0,
                                    command=EplotLongfcn)

    EcalButton = ttk.Button(win, text="run E cal", command=Ecalfcn)
    EcallabelText = StringVar(value="num to sum")
    EcallabelDir = Label(win, textvariable=EcallabelText)
    EcalEntry = ttk.Entry(win, textvariable=numToSum, width=5)
    EcalEnergylabelText = StringVar(value="energy (mJ)")
    EcalEnergylabelDir = Label(win, textvariable=EcalEnergylabelText)
    EcalEnergyDir = ttk.Entry(win, textvariable=EcalEnergy, width=15)
    shot_avgText = StringVar(value="shots to avg:")
    shot_avgDir = Label(win, textvariable=shot_avgText)
    shot_avgEntry = ttk.Entry(win, textvariable=shot_avg, width=5)
    energyLimitDir = Label(win, textvariable=StringVar(value='energy warning:'))
    energyLimitEntry = ttk.Entry(win, textvariable=energyLimit, width=5)

    colorbarFlButton = ttk.Checkbutton(win, text='fluence colorbar', variable=colorbarFl, onvalue=True, offvalue=False)
    manualScaleButton = ttk.Checkbutton(win, text='manual colorbar', variable=manualScale, onvalue=True, offvalue=False)
    flMaxText = StringVar(value="colorscale max (mJ/cm^2)")
    flMaxDir = Label(win, textvariable=flMaxText)
    flMaxEntry = ttk.Entry(win, textvariable=flMax, width=5)
    showFlMaxButton = ttk.Checkbutton(win, text='peak/avg fl', variable=showFlMax, onvalue=True, offvalue=False)
    flAngText = StringVar(value="incidence angle:")
    flAngDir = Label(win, textvariable=flAngText)
    flAngEntry = ttk.Entry(win, textvariable=flAng, width=5)

    clrE.place(x=0, y=0)
    showEplotButton.place(x=0, y=25)
    showEplotLong.place(x=0, y=50)
    shot_avgDir.place(x=0, y=75)
    shot_avgEntry.place(x=75, y=75)
    EcallabelDir.place(x=0, y=100)
    EcalEntry.place(x=75, y=100)
    EcalEnergylabelDir.place(x=320, y=0)
    EcalEnergyDir.place(x=320, y=20)
    EcalButton.place(x=320, y=45)
    energyLimitDir.place(x=320, y=70)
    energyLimitEntry.place(x=420, y=70)

    colorbarFlButton.place(x=150, y=0)
    manualScaleButton.place(x=150, y=25)
    flMaxDir.place(x=150, y=50)
    flMaxEntry.place(x=150, y=70)
    showFlMaxButton.place(x=150, y=95)
    flAngDir.place(x=150, y=120)
    flAngEntry.place(x=240, y=120)

popup_energy(energyTab)


################################################################ ROI popup box ################################################################################

def ROIBboxfcn():
    if ROIBboxButton.get() == 1:
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

def fillInROIField(coord):
    left = int(min(coord[0][0],coord[1][0]))
    bottom = int(min(coord[0][1], coord[1][1]))
    right = int(max(coord[0][0], coord[1][0]))
    top = int(max(coord[0][1], coord[1][1]))

    ROILeft.set(left)
    ROIRight.set(right)
    ROIBottom.set(bottom)
    ROITop.set(top)

def ROIChangefcn():
    if ROIBboxButton.get() == 1:
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


def popup_ROI(tab):
    openBox.set(True)
    win = tab
    showROIBboxButton = ttk.Checkbutton(win, text='show ROIBbox', variable=ROIBboxButton, onvalue=1, offvalue=0,
                                        command=ROIBboxfcn)
    showROIBboxButton.place(x=0, y=0)

    ROIlabelText = StringVar(value="ROI (left, right, bottom, top)")
    ROIlabelDir = Label(win, textvariable=ROIlabelText)
    ROILeftDir = ttk.Entry(win, textvariable=ROILeft, width=5)
    ROIRightDir = ttk.Entry(win, textvariable=ROIRight, width=5)
    ROIBottomDir = ttk.Entry(win, textvariable=ROIBottom, width=5)
    ROITopDir = ttk.Entry(win, textvariable=ROITop, width=5)

    ROIEnableButton = ttk.Checkbutton(win, text='Enable ROI?', variable=ROIEnable, onvalue=True, offvalue=False,
                                      command=ROIChangefcn)

    saveROIButton = ttk.Button(win, text="Save ROI?", command=saveROIfcn)

    ROIlabelDir.place(x=0, y=20)
    ROILeftDir.place(x=0, y=45)
    ROIRightDir.place(x=40, y=45)
    ROIBottomDir.place(x=80, y=45)
    ROITopDir.place(x=120, y=45)

    ROIEnableButton.place(x=170, y=0)
    saveROIButton.place(x=170, y=30)

popup_ROI(ROITab)

################################################################ Save popup box ###############################################################################

def setSaveTimefcn():
    lastSave.set(0)


def popup_save(tab):
    openBox.set(True)
    win = tab
    saveImagesButton = ttk.Checkbutton(win, text='save images', variable=saveImages, onvalue=True, offvalue=False,
                                       command=setSaveTimefcn)
    saveFolderlabelDir = Label(win, textvariable=StringVar(value='save folder'))
    saveFolderEntry = ttk.Entry(win, textvariable=saveFolder, width=50)
    saveNamelabelDir = Label(win, textvariable=StringVar(value='save file name'))
    saveNameEntry = ttk.Entry(win, textvariable=saveName, width=50)
    saveFreqlabelText = StringVar(value="num of images to save every interval:")
    saveFreqlabelDir = Label(win, textvariable=saveFreqlabelText)
    saveFreqDir = ttk.Entry(win, textvariable=saveFreq, width=5)
    saveTimelabelText = StringVar(value="interval time separation (minutes); 0 to continously save:")
    saveTimelabelDir = Label(win, textvariable=saveTimelabelText)
    saveTimeDir = ttk.Entry(win, textvariable=saveTime, width=5)

    saveImagesButton.place(x=0, y=0)
    saveFreqlabelDir.place(x=0, y=20)
    saveFreqDir.place(x=200, y=20)
    saveTimelabelDir.place(x=0, y=45)
    saveTimeDir.place(x=300, y=45)
    saveFolderlabelDir.place(x=0, y=65)
    saveFolderEntry.place(x=0, y=85)
    saveNamelabelDir.place(x=0, y=110)
    saveNameEntry.place(x=0, y=130)

popup_save(savingTab)


################################################################ camera popup box ##############################################################################

def camChangefcn():
    if [clicked.get(), int(exposure_num.get()), triggerClicked.get()] != [cam_name.get(), exposure_time.get(),
                                                                          trigger_mode.get()]:
        print("camera change detected")
        config.read(iniFname)
        cam_name.set(clicked.get())
        exposure_time.set(int(exposure_num.get()))
        trigger_mode.set(triggerClicked.get())
        config['CAMERA NAMES'][window] = cam_name.get()
        config[window.upper() + ' SETTINGS']['exposure time'] = str(exposure_time.get())
        config[window.upper() + ' SETTINGS']['trigger mode'] = trigger_mode.get()
        config[window.upper() + ' SETTINGS']['timeout'] = str(timeout.get())
        with open(iniFname, 'w') as configfile:
            config.write(configfile)
        cam.camera.Close()
        cam.open_camera(cam_name.get(), exposure_time.get(), trigger_mode.get(), GigE.get(), packetSize.get(),
                        packetDelay.get())
        cameraChange.set(True)
    elif timeout.get() != timeoutOrig.get():
        timeoutOrig.set(timeout.get())
        config.read(iniFname)
        config[window.upper() + ' SETTINGS']['timeout'] = str(timeout.get())
        with open(iniFname, 'w') as configfile:
            config.write(configfile)


def popup_camera(tab):
    openBox.set(True)
    win = tab
    exposure_setting = ttk.Entry(win, textvariable=exposure_num, width=15)

    cameraText = StringVar(value="camera select")
    cameraDir = Label(win, textvariable=cameraText)
    drop = OptionMenu(win, clicked, *cam.options)

    exposureText = StringVar()
    exposureText.set("exposure time (us)")
    exposureDir = Label(win, textvariable=exposureText)

    triggerText = StringVar()
    triggerText.set("trigger mode")
    triggerDir = Label(win, textvariable=triggerText)
    triggerDrop = OptionMenu(win, triggerClicked, *['on', 'off'])

    timeoutLabel = Label(win, textvariable=StringVar(value='timeout (ms):'))
    timeoutEntry = ttk.Entry(win, textvariable=timeout, width=10)

    saveCamSettings = ttk.Button(win, text="change cam settings", command=camChangefcn)

    cameraDir.place(x=0, y=0)
    drop.place(x=0, y=20)
    exposureDir.place(x=0, y=50)
    exposure_setting.place(x=0, y=70)
    triggerDir.place(x=120, y=50)
    triggerDrop.place(x=120, y=70)
    timeoutLabel.place(x=0, y=100)
    timeoutEntry.place(x=80, y=100)
    saveCamSettings.place(x=0, y=125)

popup_camera(camerasTab)


################################################################ plot settings popup box #######################################################################

def clrScalefcn():
    '''stupid way to add/remove colorbar'''
    if clrScale.get():
        axins = inset_axes(im.axes,  # here using axis of the lowest plot
                           width="3%",  # width = 5% of parent_bbox width
                           height="30%",  # height : 340% good for a (4x4) Grid
                           loc='upper left'
                           )
        cb = plt.colorbar(cax=axins, orientation='vertical')
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
        coords[1] = int(float(cursorY.get()) * yvals_max / imsy.get())
        coords[0] = int(float(cursorX.get()) * xvals_max / imsx.get())
    else:
        cursorX.set(coords[0])
        cursorY.set(coords[1])


def popup_plot(tab):
    openBox.set(True)
    win = tab

    colorText = StringVar(value="color scale")
    colorDir = Label(win, textvariable=colorText)
    colorDrop = OptionMenu(win, colorClicked, *['viridis', 'hsv', 'plasma'])
    scaleButton = ttk.Checkbutton(win, text='clrbar on/off', variable=clrScale, onvalue=True, offvalue=False,
                                  command=clrScalefcn)
    bkgndPathlabelText = StringVar(value="Bckgnd file path")
    bkgndPathlabelDir = Label(win, textvariable=bkgndPathlabelText)
    bkgndPath = ttk.Entry(win, textvariable=bkgndFile)
    saveBkgnd = ttk.Button(win, text="save bkgnd", command=saveBkgndfcn)
    subtBkgndButton = ttk.Checkbutton(win, text='subt bkgnd', variable=subtBkgnd, onvalue=True, offvalue=False,
                                      command=subtBkgndfcn)
    autoCursorButton = ttk.Checkbutton(win, text='autocursor?', variable=autoCursor, onvalue=True, offvalue=False,
                                       command=autoCursorfcn)
    cursorText = StringVar(value="cursor coords (x, y)")
    cursorDir = Label(win, textvariable=cursorText)
    cursorXlabel = Label(win, textvariable=cursorX, width=5)
    cursorYlabel = Label(win, textvariable=cursorY, width=5)
    cursorXstdlabel = Label(win, textvariable=cursorXstd, width=5)
    cursorYstdlabel = Label(win, textvariable=cursorYstd, width=5)
    gauss_fit_button = ttk.Checkbutton(win, text='Gauss fit', variable=fitting, onvalue=True, offvalue=False)

    cursorFixedText = StringVar(value="fixed cursor coords and radius (x, y, rad)")
    cursorFixedDir = Label(win, textvariable=cursorFixedText)
    cursorXentryFixed = ttk.Entry(win, textvariable=cursorXfixed, width=5)
    cursorYentryFixed = ttk.Entry(win, textvariable=cursorYfixed, width=5)
    cursorRadentry = ttk.Entry(win, textvariable=cursorRad, width=5)
    saveCursorButton = ttk.Button(win, text="save settings", command=saveCursorfcn)
    cursorAvgText = StringVar(value="num to avg:")
    cursorAvgDir = Label(win, textvariable=cursorAvgText)
    cursorAvgentry = ttk.Entry(win, textvariable=cursorAvg, width=5)

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
    gauss_fit_button.place(x=450, y=45)

    cursorFixedDir.place(x=300, y=0)
    cursorXentryFixed.place(x=300, y=20)
    cursorYentryFixed.place(x=350, y=20)
    cursorRadentry.place(x=400, y=20)
    saveCursorButton.place(x=300, y=45)

    bkgndPathlabelDir.place(x=110, y=0)
    bkgndPath.place(x=110, y=25)
    subtBkgndButton.place(x=110, y=50)
    saveBkgnd.place(x=110, y=80)

popup_plot(plotAdjTab)


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
    #config[window.upper() + ' SETTINGS']['motor dead zone'] = str(deadZone.get())
    with open(iniFname, 'w') as configfile:
        config.write(configfile)


def jogPlusXfcn():
    if motorsInitialized.get():
        pico.move(deviceName.get(), motorXnum.get(), motorXmult.get() * motorStep.get(), motorLim.get())


def jogMinusXfcn():
    if motorsInitialized.get():
        pico.move(deviceName.get(), motorXnum.get(), -1 * motorXmult.get() * motorStep.get(), motorLim.get())


def jogPlusYfcn():
    if motorsInitialized.get():
        pico.move(deviceName.get(), motorYnum.get(), motorYmult.get() * motorStep.get(), motorLim.get())


def jogMinusYfcn():
    if motorsInitialized.get():
        pico.move(deviceName.get(), motorYnum.get(), -1 * motorYmult.get() * motorStep.get(), motorLim.get())


def popup_motor(tab):
    openBox.set(True)
    win= tab

    motorOnButton = ttk.Checkbutton(win, text='stabilizers on?', variable=motorOn, onvalue=True, offvalue=False)

    motorErrorText = StringVar(value="allowed motor error (pix):")
    motorErrorDir = Label(win, textvariable=motorErrorText)
    motorErrorEntry = ttk.Entry(win, textvariable=motorError, width=5)

    motorText = StringVar(value="position to center motors to: ")
    motorXDir = Label(win, textvariable=StringVar(value='x'))
    motorYDir = Label(win, textvariable=StringVar(value='y'))
    motorDir = Label(win, textvariable=motorText)
    motorXEntry = ttk.Entry(win, textvariable=motorX, width=5)
    motorYEntry = ttk.Entry(win, textvariable=motorY, width=5)

    motorThresholdText = StringVar(value="threshold before motor turns on:")
    motorThresholdDir = Label(win, textvariable=motorThresholdText)
    motorThresholdEntry = ttk.Entry(win, textvariable=motorThreshold, width=5)
    saveMotorButton = ttk.Button(win, text="save settings", command=saveMotorfcn)

    motorStepText = StringVar(value="motor step size:")
    motorStepDir = Label(win, textvariable=motorStepText)
    motorStepEntry = ttk.Entry(win, textvariable=motorStep, width=5)

    deviceNameDir = Label(win, textvariable=StringVar(value='device name'))
    deviceNameEntry = ttk.Entry(win, textvariable=deviceName, width=10)

    motorXnumDir = Label(win, textvariable=StringVar(value='x num:'))
    motorXnumEntry = ttk.Entry(win, textvariable=motorXnum, width=5)
    motorYnumDir = Label(win, textvariable=StringVar(value='y num:'))
    motorYnumEntry = ttk.Entry(win, textvariable=motorYnum, width=5)

    motorXmultDir = Label(win, textvariable=StringVar(value='x mult:'))
    motorXmultEntry = ttk.Entry(win, textvariable=motorXmult, width=5)
    motorYmultDir = Label(win, textvariable=StringVar(value='y mult:'))
    motorYmultEntry = ttk.Entry(win, textvariable=motorYmult, width=5)

    jogDir = Label(win, textvariable=StringVar(value='Jog:'))
    jogPlusXButton = ttk.Button(win, text="X+", command=jogPlusXfcn, width=5)
    jogMinusXButton = ttk.Button(win, text="X-", command=jogMinusXfcn, width=5)
    jogPlusYButton = ttk.Button(win, text="Y+", command=jogPlusYfcn, width=5)
    jogMinusYButton = ttk.Button(win, text="Y-", command=jogMinusYfcn, width=5)

    smallAlignmentZoneDir = Label(win, textvariable=StringVar(value='small alignment zone:'))
    smallAlignmentZoneEntry = ttk.Entry(win, textvariable=smallAlignmentZone, width=5)
    smallAlignmentZoneDir.place(x = 0, y = 125)
    smallAlignmentZoneEntry.place(x = 125, y = 125)

    motorOnButton.place(x=0, y=0)
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
    motorXnumDir.place(x=275, y=0)
    motorXnumEntry.place(x=325, y=0)
    motorYnumDir.place(x=375, y=0)
    motorYnumEntry.place(x=425, y=0)
    motorXmultDir.place(x=275, y=25)
    motorXmultEntry.place(x=325, y=25)
    motorYmultDir.place(x=375, y=25)
    motorYmultEntry.place(x=425, y=25)
    jogDir.place(x=275, y=50)
    jogPlusXButton.place(x=325, y=50)
    jogMinusXButton.place(x=375, y=50)
    jogPlusYButton.place(x=425, y=50)
    jogMinusYButton.place(x=475, y=50)
    deviceNameDir.place(x=275, y=75)
    deviceNameEntry.place(x=355, y=75)
    saveMotorButton.place(x=275, y=100)

if usingMotors.get():
    popup_motor(motorsTab)
else:
    nullMotorText = Label(motorsTab, textvariable="No motors set in settings")
    nullMotorText.place(pady = 5)

###################################
canvas.get_tk_widget().place(x=0, y=0)
place_gui_elems(yvals_max)

root.mainloop()