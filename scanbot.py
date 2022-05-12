# -*- coding: utf-8 -*-
"""
Created on Fri May  6 15:38:34 2022

@author: jack hellerstedt and julian ceddia
"""

from nanonisTCP import nanonisTCP
from nanonisTCP.Scan import Scan
from nanonisTCP.TipShaper import TipShaper
from nanonisTCP.Bias import Bias

import nanonisUtils as nut

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import nanonispyfit as nap

import time
import ntpath                                                                   # os.path but for windows paths

import global_

class scanbot():
###############################################################################
# Constructor
###############################################################################
    def __init__(self,interface):
        self.interface = interface
        
###############################################################################
# Actions
###############################################################################
    def plot(self,args):
        NTCP,connection_error = self.connect()                                  # Connect to nanonis via TCP
        if(connection_error): return connection_error                           # Return error message if there was a problem connecting        
        
        scan = Scan(NTCP)                                                       # Nanonis scan module
        _,scanData,_ = scan.FrameDataGrab(14, 1)                                # Grab the data within the scan frame. Channel 14 is . 1 is forward data direction
        
        pngFilename = self.makePNG(scanData)                                    # Generate a png from the scan data
        self.interface.sendPNG(pngFilename)                                     # Send a png over zulip
        
        self.disconnect(NTCP)                                                   # Close the TCP connection
        return ""
    
    def stop(self,args):
        NTCP,connection_error = self.connect()                                  # Connect to nanonis via TCP
        if(connection_error): return connection_error                           # Return error message if there was a problem connecting        
                                                                                # Defaults args...
        arg_dict = {'-safe'  : '0'}                                             # -safe means safe mode... withdraw tip etc. (not implemented yet)
        
        for arg in args:                                                        # Override the defaults if user inputs them
            key,value = arg.split('=')
            if(not key in arg_dict):
                self.disconnect(NTCP)                                           # Close the connection and 
                return "invalid argument: " + arg                               # return error message
            arg_dict[key] = value     
        
        scan = Scan(NTCP)                                                       # Nanonis scan module
        scan.Action('stop')                                                     # Stop the current scan
        
        self.disconnect(NTCP)                                                   # Close the NTCP connection
        return ("Stopped!")
    
    def survey(self,args):
        NTCP,connection_error = self.connect()                                  # Connect to nanonis via TCP
        if(connection_error): return connection_error                           # Return error message if there was a problem connecting        
                                                                                # Defaults args...
        arg_dict = {'-i'  : '1',                                                # start the grid from this index
                    '-n'  : '5',                                                # size of the nxn grid of scans
                    '-s'  : 'scanbot',                                          # suffix at the end of autosaved sxm files
                    '-xy' : '100e-9',                                           # length and width of the scan frame (square)
                    '-dx' : '150e-9',                                           # spacing between scans
                    '-oxy': '0,0',                                              # Origin (centre of grid)
                    '-st' : '10',                                               # Sleep time before restarting scan to correct for drift
                    '-bias': '1'}                                               # Scan bias (not implemented yet)
        
        for arg in args:                                                        # Override the defaults if user inputs them
            key,value = arg.split('=')
            if(not key in arg_dict):
                self.disconnect(NTCP)                                           # Close the connection and 
                return "invalid argument: " + arg                               # return error message
            arg_dict[key] = value                    
        
        self.survey_args = arg_dict.copy()                                      # Store these params for enhance to look at
        
        scan = Scan(NTCP)                                                       # Nanonis scan module
        # biasModule = Bias(NTCP)                                                 # Nanonis bias module
        
        basename     = scan.PropsGet()[3]                                       # Get the save basename
        tempBasename = basename + '_' + arg_dict['-s'] + '_'                    # Create a temp basename for this survey
        scan.PropsSet(series_name=tempBasename)                                 # Set the basename in nanonis for this survey
        
        i  = int(arg_dict['-i'])                                                # start the grid at this index
        n  = int(arg_dict['-n'])                                                # size of nxn grid of scans
        dx = float(arg_dict['-dx'])                                             # Scan spacing
        xy = float(arg_dict['-xy'])                                             # Scan size (square)
        ox = float(arg_dict['-oxy'].split(',')[0])                              # Origin of grid
        oy = float(arg_dict['-oxy'].split(',')[1])                              # Origin of grid
        
        count = 0
        frames = []                                                             # [x,y,w,h,angle=0]
        for ii in range(int(-n/2),int(n/2) + n%2):
            jj_range = range(int(-n/2),int(n/2) + n%2)
            if(ii%2): jj_range = reversed(jj_range)                             # Alternate grid direction each row so the grid snakes... better for drift
            for jj in jj_range:
                count += 1
                if(count < i): continue                                         # Skip this frame if it's before the frame index we want to start from
                frames.append([ jj*dx+ox, ii*dx+oy, xy, xy])                    # Build scan frame
        
        count = i-1
        sleepTime = float(arg_dict['-st'])                                      # Time to sleep before restarting scan to reduce drift
        for frame in frames:
            count += 1
            self.currentSurveyIndex = count
            self.interface.sendReply('Running scan ' + str(count) + ': ' + str(frame)) # Send a message that the next scan is starting
            
            scan.FrameSet(*frame)                                               # Set the coordinates and size of the frame window in nanonis
            scan.Action('start')                                                # Start the scan. default direction is "up"
            
            time.sleep(sleepTime)                                               # Wait 10s for drift to settle
            if(self.checkEventFlags()): break
        
            scan.Action('start')                                                # Start the scan again after waiting for drift to settle
            timeoutStatus, _, filePath = scan.WaitEndOfScan()                   # Wait until the scan finishes
            
            _,scanData,_ = scan.FrameDataGrab(14, 1)                            # Grab the data within the scan frame. Channel 14 is . 1 is forward data direction
            
            pngFilename = self.makePNG(scanData, filePath)                      # Generate a png from the scan data
            self.interface.sendPNG(pngFilename)                                 # Send a png over zulip
            
            if(self.checkEventFlags()): break                                   # Check event flags
        
        scan.PropsSet(series_name=basename)                                     # Put back the original basename
        self.disconnect(NTCP)                                                   # Close the TCP connection
        
        global_.running.clear()                                                 # Free up the running flag
        
        self.interface.sendReply('survey \'' + arg_dict['-s'] + '\' done')      # Send a notification that the survey has completed
    
    def enhance(self,args):
                                                                                # Defaults args...
        arg_dict = {'-i'  : '1',                                                # Index of the frame in the last survey to enhance
                    '-n'  : '2',                                                # size of the nxn grid of scans
                    '-s'  : 'enhance',                                          # suffix at the end of autosaved sxm files
                    '-xy' : '0',                                                # length and width of the scan frame (square)
                    '-dx' : '0',                                                # spacing between scans
                    '-st' : '10'}                                               # Sleep time before restarting scan to correct for drift
        
        for arg in args:                                                        # Override the defaults if user inputs them
            key,value = arg.split('=')
            if(not key in arg_dict):
                return "invalid argument: " + arg                               # return error message
            arg_dict[key] = value                    
        
        i = int(arg_dict['-i'])                                                 # Index of the frame to enhance
        n = int(self.survey_args['-n'])                                         # size of the original survey grid
        
        count = 0                                                               # Lazy way to get ii and jj
        for ii in range(int(-n/2),int(n/2) + n%2):
            jj_range = range(int(-n/2),int(n/2) + n%2)
            if(ii%2): jj_range = reversed(jj_range)                             # Alternate grid direction each row so the grid snakes... better for drift
            for jj in jj_range:
                count += 1
                if(count == i): break
            if(count == i): break
        
        ox = float(self.survey_args['-oxy'].split(',')[0])                      # Origin of the original survey grid
        oy = float(self.survey_args['-oxy'].split(',')[1])                      # Origin of the original survey grid
        dx = float(self.survey_args['-dx'])                                     # Frame spacing in the original survey grid
        ox,oy =  jj*dx+ox, ii*dx+oy                                             # New origin for the enhance grid is the centre of the frame to enhance
        
        n  = int(arg_dict['-n'])                                                # size of the nxn grid within the frame to enhance
        xy = float(arg_dict['-xy'])                                             # size of the frames in the enhance grid
        dx = float(arg_dict['-dx'])                                             # spacing of the frames in the enhance grid
        if(xy == 0): xy = float(self.survey_args['-xy'])/n                      # if xy is set to 0, divide the enhance grid fills the frame exactly
        if(dx == 0): dx = xy                                                    # spacing = frame size by default
        oxy = str(ox+dx/2) + ',' + str(oy+dx/2)                                 # Adjust the origin by dx/2 to centre the enhance grid on the survey frame
        
        survey_args = []                                                        # Argument list for survey
        survey_args.append('-n='   + str(n))
        survey_args.append('-s='   + 'enhance')
        survey_args.append('-xy='  + str(xy))
        survey_args.append('-dx='  + str(dx))
        survey_args.append('-oxy=' + oxy)
        survey_args.append('-st='  + arg_dict['-st'])
        
        self.survey(survey_args)                                                # Kick off a survey within the frame we want to enhance
    
    def tipShape(self,args):
        NTCP,connection_error = self.connect()                                  # Connect to nanonis via TCP
        if(connection_error): return connection_error                           # Return error message if there was a problem connecting        
                                                                                # Defaults args... leaving set to default takes settings from nanonis
        arg_dict = {'-sod'    : ['ts','default',lambda x: float(x)],            # switch off delay: the time during which the Z position is averaged before switching the Z controller off.
                    '-cb'     : ['ts','default',lambda x: int(x)],              # Change bias flag
                    '-b1'     : ['ts','default',lambda x: float(x)],            # is the value applied to the Bias signal if cb is true
                    '-z1'     : ['ts','default',lambda x: float(x)],            # first tip lift (m) (i.e. -2e-9)
                    '-t1'     : ['ts','default',lambda x: float(x)],            # defines the time to ramp Z from current Z position to z1
                    '-b2'     : ['ts','default',lambda x: float(x)],            # the Bias voltage applied just after the first Z ramping
                    '-t2'     : ['ts','default',lambda x: float(x)],            # the time to wait after applying the Bias Lift value b2
                    '-z3'     : ['ts','default',lambda x: float(x)],            # the height the tip is going to ramp for the second time (m) i.e. +5nm
                    '-t3'     : ['ts','default',lambda x: float(x)],            # given time to ramp Z in the second ramping [s].
                    '-wait'   : ['ts','default',lambda x: float(x)],            # time to wait after restoring the initial Bias voltage
                    '-fb'     : ['ts','default',lambda x: int(x)],              # selects the initial Z-controller status is off (0) or on (1)
                    # Bias Pulse params
                    '-np'     : ['-','1',       lambda x: int(x)],              # Number of pulses
                    '-pw'     : ['pulse','0.1', lambda x: float(x)],            # Pulse width (duration in seconds)
                    '-bias'   : ['pulse','3',   lambda x: float(x)],            # Bias value (V)
                    '-zhold'  : ['pulse','0',   lambda x: int(x)],              # Z-Controller on hold (0=nanonis setting, 1=deactivated, 2=activated)
                    '-abs'    : ['pulse','0',   lambda x: int(x)]               # Absolute or relative to current bias (0=nanonis setting, 1=relative,2=absolute)
                    }
        
        for arg in args:                                                        # Override the defaults if user inputs them
            key,value = arg.split('=')
            if(not key in arg_dict):
                return "invalid argument: " + arg                               # return error message
            arg_dict[key][1] = value                    
            
        tipShaper  = TipShaper(NTCP)
        biasModule = Bias(NTCP)
        
        default_args = tipShaper.PropsGet()                                     # Store all the current tip shaping settings in nanonis
        
        tipShaperArgs = []
        for i,arg in enumerate(arg_dict):
            if(arg_dict[arg][0] == 'ts'):
                if(arg_dict[arg][1] == 'default'):                              # only process tip shaper params
                    arg_dict[arg][1] = str(default_args[i])                     # set all the remaining 'default' values to values from nanonis
            
                tipShaperArgs.append(arg_dict[arg][2](arg_dict[arg][1]))        # convert the string to required type
        
        tipShaper.PropsSet(*tipShaperArgs)                                      # update the tip shaping params in nanonis
        
        biasPulseArgs = []
        for arg in arg_dict:
            if(arg_dict[arg][0] == 'pulse'):
                biasPulseArgs.append(arg_dict[arg][2](arg_dict[arg][1]))
        
        biasPulseArgs.append(1)                                                 # wait_until_done flag
        pw        = arg_dict['-pw'][2](arg_dict['-pw'][1])                      # pulse width
        numPulses = arg_dict['-np'][2](arg_dict['-np'][1])                      # Number of bias pulses
        
        tipShaper.Start(wait_until_finished=True,timeout=-1)                    # initiate the tip shape
        for n in range(0,numPulses):                                            # Pulse the tip -np times
            biasModule.Pulse(*biasPulseArgs)
            time.sleep(pw + 0.2)
        
        self.disconnect(NTCP)
        
        global_.running.clear()
        
        self.interface.sendReply("Tip-shape complete")
    
    def pulse(self,args):
        NTCP,connection_error = self.connect()                                  # Connect to nanonis via TCP
        if(connection_error): return connection_error                           # Return error message if there was a problem connecting        
                                                                                # Defaults args... leaving set to default takes settings from nanonis
        arg_dict = {'-np'     : ['-','1',       lambda x: int(x)],              # Number of pulses
                    '-pw'     : ['pulse','0.1', lambda x: float(x)],            # Pulse width (duration in seconds)
                    '-bias'   : ['pulse','3',   lambda x: float(x)],            # Bias value (V)
                    '-zhold'  : ['pulse','0',   lambda x: int(x)],              # Z-Controller on hold (0=nanonis setting, 1=deactivated, 2=activated)
                    '-abs'    : ['pulse','0',   lambda x: int(x)]               # Absolute or relative to current bias (0=nanonis setting, 1=relative,2=absolute)
                    }
        
        for arg in args:                                                        # Override the defaults if user inputs them
            key,value = arg.split('=')
            if(not key in arg_dict):
                return "invalid argument: " + arg                               # return error message
            arg_dict[key][1] = value                    
            
        biasModule = Bias(NTCP)
        
        biasPulseArgs = []
        for arg in arg_dict:
            if(arg_dict[arg][0] == 'pulse'):
                biasPulseArgs.append(arg_dict[arg][2](arg_dict[arg][1]))
        
        biasPulseArgs.append(1)                                                 # wait_until_done flag
        pw        = arg_dict['-pw'][2](arg_dict['-pw'][1])                      # pulse width
        
        numPulses = arg_dict['-np'][2](arg_dict['-np'][1])                      # Number of bias pulses
        for n in range(0,numPulses):                                            # Pulse the tip -np times
            biasModule.Pulse(*biasPulseArgs)
            time.sleep(pw + 0.2)
        
        self.disconnect(NTCP)
        
        global_.running.clear()
        
        self.interface.sendReply("Bias pulse complete")
    
    def biasDep(self,args):
        NTCP,connection_error = self.connect()                                  # Connect to nanonis via TCP
        if(connection_error): return connection_error                           # Return error message if there was a problem connecting        
                                                                                # Defaults args
        arg_dict = {'-nb'  : ['5',   lambda x: int(x)],                         # Number of images to take b/w initial and final bias
                    '-bdc' : ['-1',  lambda x: float(x)],                       # Drift correct image bias
                    '-bi'  : ['-1',  lambda x: float(x)],                       # initial bias
                    '-bf'  : ['1',   lambda x: float(x)],                       # final bias
                    '-px'  : ['128', lambda x: int(x)]}                         # dc pixels
        
        for arg in args:                                                        # Override the defaults if user inputs them
            key,value = arg.split('=')
            if(not key in arg_dict):
                return "invalid argument: " + arg                               # return error message
            arg_dict[key][0] = value                    
            
        nb  = arg_dict['-nb'][1](arg_dict['-nb'][0])
        bdc = arg_dict['-bdc'][1](arg_dict['-bdc'][0])
        bi  = arg_dict['-bi'][1](arg_dict['-bi'][0])
        bf  = arg_dict['-bf'][1](arg_dict['-bf'][0])
        px  = arg_dict['-px'][1](arg_dict['-px'][0])
        
        biasList = np.linspace(bi,bf,nb)
        
        scan = Scan(NTCP)
        biasModule = Bias(NTCP)
        
        scanPixels,scanLines = scan.BufferGet()[2:]
        lx = int((scanLines/scanPixels)*px)
        
        x,y,w,h,angle = scan.FrameGet()
        
        ## Initial drift correct frame
        biasModule.Set(bdc)
        scan.BufferSet(pixels=px,lines=lx)
        scan.Action('start',scan_direction='up')
        timeoutStatus, _, filePath = scan.WaitEndOfScan()
        if(timeoutStatus): return "bias dep stopped"
        _,initialDriftCorrect,_ = scan.FrameDataGrab(14, 1)
        
        dx  = w/px
        dy  = h/lx
        dxy = np.array([dx,dy])
        for b in biasList:
            ## Bias dep scan
            biasModule.Set(b)
            scan.BufferSet(pixels=scanPixels,lines=scanLines)
            scan.Action('start',scan_direction='down')
            timeoutStatus, _, filePath = scan.WaitEndOfScan()
            if(timeoutStatus): return "bias dep stopped"
            print("timeout: " + str(timeoutStatus))
            _,scanData,_ = scan.FrameDataGrab(14, 1)
            
            
            pngFilename = self.makePNG(scanData, filePath)                      # Generate a png from the scan data
            self.interface.sendPNG(pngFilename)                                 # Send a png over zulip
            
            if(self.checkEventFlags()): break                                   # Check event flags
            
            ## Drift correct scan
            biasModule.Set(bdc)
            scan.BufferSet(pixels=px,lines=lx)
            scan.Action('start',scan_direction='up')
            timeoutStatus, _, filePath = scan.WaitEndOfScan()
            if(timeoutStatus): return "bias dep stopped"
            _,driftCorrectFrame,_ = scan.FrameDataGrab(14, 1)
            
            if(self.checkEventFlags()): break                                   # Check event flags
            
            ox,oy = nut.getFrameOffset(initialDriftCorrect,driftCorrectFrame,dxy)
            x,y   = np.array([x,y]) - np.array([ox,oy])
            
            scan.FrameSet(x,y,w,h)
            
        self.disconnect(NTCP)
        
        global_.running.clear()
        
        self.interface.sendReply("Bias dependent imaging complete")
###############################################################################
# Utilities
###############################################################################
    def makePNG(self,scanData,filePath=''):
        fig, ax = plt.subplots(1,1)
        
        mask = np.isnan(scanData)                                               # Mask the Nan's
        scanData[mask == True] = np.nanmean(scanData)                           # Replace the Nan's with the mean so it doesn't affect the plane fit
        scanData = nap.plane_fit_2d(scanData)                                   # Flattern the image
        vmin, vmax = nap.filter_sigma(scanData)                                 # cmap saturation
        
        ax.imshow(scanData, cmap='Blues_r', vmin=vmin, vmax=vmax) # Plot
        ax.axis('off')
        
        pngFilename = 'im.png'
        if filePath: pngFilename = ntpath.split(filePath)[1] + '.png'
        
        fig.savefig(pngFilename, dpi=60, bbox_inches='tight', pad_inches=0)
        plt.close('all')
        
        return pngFilename
        
    def checkEventFlags(self):
        if(not global_.running.is_set()): return 1                              # Running flag
        while global_.pause.is_set(): time.sleep(2)                             # Pause flag
    
###############################################################################
# Nanonis TCP Connection
###############################################################################
    def connect(self):
        try:                                                                    # Try to connect to nanonis via TCP
            IP   = self.interface.IP
            PORT = self.interface.portList.pop()
            NTCP = nanonisTCP(IP, PORT)
            return [NTCP,0]
        except Exception as e:
            if(len(self.interface.portList)): return [0,str(e)]                 # If there are ports available then return the exception message
            return [0,"No ports available"]                                     # If no ports are available send this message
    
    def disconnect(self,NTCP):
        NTCP.close_connection()                                                 # Close the TCP connection
        self.interface.portList.append(NTCP.PORT)                               # Free up the port - put it back in the list of available ports