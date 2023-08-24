from nanonisTCP.Bias import Bias                                                # Import the NanonisTCP Bias Module. Used to control the tip bias

from nanonisTCP.Scan import Scan
from nanonisTCP.Signals import Signals
from nanonisTCP.Pattern import Pattern
from nanonisTCP.FolMe import FolMe
from nanonisTCP.BiasSpectr import BiasSpectr
from nanonisTCP.ZController import ZController
from nanonisTCP.Piezo import Piezo

import utilities
import global_                                                                  # Import the global variables
import time                                                                     # Import time for sleeping

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sp


class hk_commands(object):
    def __init__(self,interface):                                               # Set up the constructor like this
        self.interface = interface                                              # Reference to scanbot_interface
        self.initCommandDict()                                                  # Initialise the dictionary containing a list of custom commands

    def initCommandDict(self):
        self.commands = {                                                       # A dictionary that maps commands with function names.
                         'change_bias'  : self.changeBias,                      # An example command without threading. Here, the user will call change_bias
                         'change_bias2' : self.changeBias2,                     # An example command with threading. Here, the user will call change_bias2
                         'drift_sts'    : self.driftSTShook,
                         'cal_drift'    : self.calDrifthook,
                         }

    def changeBias(self,user_args,_help=False):
        """
        This function changes the bias in Nanonis. It performs this task
        without the use of multi-threading which means it cannot Scanbot will
        be busy until the task is complete. This is fine for commands that will
        not interfere with other threaded tasks (e.g. taking control of the 
        motors while a survey is running). Tasks that are not threaded can run
        while a threaded task is already running. Tasks that are not threaded
        cannot be stopped until complete (i.e. running 'stop' command will not
        work).

        Parameters
        ----------
        user_args : arguments passed in by the user when calling the command
        _help     : flag set when user calls "help change_bias"

        """
        arg_dict = {'-V'    : ['0',        lambda x: float(x), "(float) Set the tip bias."],
                    '-arg2' : ['some str', lambda x: str(x),   "(str) An example user argument that defaults to 'some str' is a string."]}

        if(_help): return arg_dict                                              # This will get called when the user runs the command 'help exUnthreaded'

        error,user_arg_dict = self.interface.userArgs(arg_dict,user_args)       # This will validate args V and arg2 as float() and str(), respectively
        if(error):
            return error + "\nRun ```help change_bias``` if you're unsure."     # Inform the user of any errors in their input

        V,arg2 = self.interface.unpackArgs(user_arg_dict)                       # Unpack the arguments

        if(V == 0):                                                             # Validation
            errorMessage = "Cannot set bias to zero!"
            return errorMessage                                                 # Return with error

        NTCP,connection_error = self.interface.scanbot.connect()                # Connect to nanonis via TCP
        if(connection_error):
            return connection_error                                             # Return error message if there was a problem connecting        

        biasModule = Bias(NTCP)                                                 # The NanonisTCP Bias module
        biasModule.Set(V)                                                       # Set the bias in Nanonis

        self.interface.sendReply("Bias set to " + str(V) + "! arg2 = " + arg2)  # This is how you send a reply.

        self.interface.scanbot.disconnect(NTCP)                                 # Remember to free up the TCP port

        return                                                                  # Return None for success

    def changeBias2(self,user_args,_help=False):
        """
        This function will change the bias in Nanonis by threading the function
        "threadedFunction" after validating user input. Only one threaded
        function can run at a time. If the user tries to run a second threaded
        function, they will be presented with an error. This is handled by the
        global flags in global_. Threaded tasks run in the background and can
        be stopped by the user running the "stop" command.

        Parameters
        ----------
        user_args : arguments passed in by the user when calling the command
        _help     : flag set when user calls "help change_bias2"

        """
        arg_dict = {'-V'    : ['0',  lambda x: float(x), "(float) Set the tip bias."],
                    '-wait' : ['10', lambda x: int(x),   "(int) Seconds to wait after changing bias."]}

        if(_help): return arg_dict                                              # This will get called when the user runs the command 'help exUnthreaded'

        error,user_arg_dict = self.interface.userArgs(arg_dict,user_args)       # This will validate args V and wait as float() and int(), respectively
        if(error):
            return error + "\nRun ```help change_bias2``` if you're unsure."    # Inform the user of any errors in their input

        args = self.interface.unpackArgs(user_arg_dict)                         # Unpack the arguments

        func = lambda : self.threadedFunction(*args)                            # Handle to the function to be threaded
        return self.interface.threadTask(func)                                  # Return and thread the function

    def threadedFunction(self,V,wait):
        """
        This function changes the bias in Nanonis and then waits a specified
        amount of time.

        This function is run on a new thread. It will run in the backgound 
        while Scanbot can still reveive commands. It should periodically check
        event flags by calling 'self.interface.scanbot.checkEventFlags()'. If
        True is returned, the 'stop' function has been called by the user.
        Whenever returning, the global running flag should be cleared by 
        calling global_.running.clear().

        Parameters
        ----------
        V    : Change the bias to this value
        wait : Wait this many seconds after changing the bias

        """
        if(V == 0):                                                             # Validation
            errorMessage = "Cannot set bias to zero!"
            global_.running.clear()                                             # Free up the running flag. This must be done whenever exiting a threaded function
            return errorMessage                                                 # Return with error

        NTCP,connection_error = self.interface.scanbot.connect()                # Connect to nanonis via TCP
        if(connection_error):
            global_.running.clear()                                             # Free up the running flag. This must be done whenever exiting a threaded function
            return connection_error                                             # Return error message if there was a problem connecting        

        biasModule = Bias(NTCP)                                                 # The NanonisTCP Bias module
        biasModule.Set(V)                                                       # Set the bias in Nanonis

        self.interface.sendReply("Bias set to " + str(V))                       # This is how you send a reply.

        self.interface.sendReply("Waiting " + str(wait) + " seconds...")        # This is how you send a reply.

        seconds = 0
        while(seconds < wait):
            time.sleep(1)
            seconds += 1                                                        # Keep track of how many seconds have gone by
            if(self.interface.scanbot.checkEventFlags() == True):               # Periodically check event flags when appropriate to see if the user has called "stop"
                self.interface.sendReply("Stopping early!")                     # This is how you send a reply.
                break

        self.interface.sendReply("Done!")                                       # This is how you send a reply.

        self.interface.scanbot.disconnect(NTCP)                                 # Remember to free up the TCP port
        global_.running.clear()                                                 # Free up the running flag. This must be done whenever exiting a threaded function

        return
    
    def getFrameOffset2(self,im1,im2,dxy=[1,1],theta=0):
        """
        Returns the offset of im2 relative to im1. im1 and im2 must be the same
        size and scale. Keep dxy=[1,1] to return offset in units of pixels.
        When using with nanonis to detect drift, take the current scan frame 
        position and subtract ox,oy from it. i.e.: center_x -= ox; center_y -= oy

        Parameters
        ----------
        im1 : image to compare against
        im2 : image to get the offset of
        dxy : pixel size in x and y: [dx,dy]
        theta : angle in degrees

        Returns
        -------
        [ox,oy] : offset in x and y

        """

        im1_diff = np.diff(im1,axis=0)                                              # Differentiate along x
        im2_diff = np.diff(im2,axis=0)                                              # Differentiate along x
            
        xcor = sp.correlate(im1_diff,im2_diff, method='auto', mode='same')
        y,x  = np.unravel_index(xcor.argmax(), xcor.shape)

        ni = np.array(xcor.shape)
        oy,ox = np.array([y,x]).astype(int) - (ni/2).astype(int)
        
        ox += x%2                                                                   # Add in this offset because differentiating results in odd number of px 
        
        theta *= np.pi/180                                                        # Convert to radians
        ox,oy = utilities.rotate([0,0],[ox,oy],theta)

        ox *= dxy[0]
        oy *= -dxy[1]

        return [ox, oy]     

    def driftSTShook(self, user_args, _help=False):

        #########################################
        #------------ INPUT HANDLING -----------#
        #########################################


        arg_dict = {'-pat'      : ['"Line"',        lambda x: str(x),   "(string) Experimental pattern. Must be one of: 'Line', 'Grid', or 'Cloud'."],
                    '-imgV'     : ['1',             lambda x: float(x), "(float) Bias for acquiring images."],
                    '-imgI'     : ['20e-12',        lambda x: float(x), "(float) Set-point current for acquiring images."],
                    '-specV'    : ['1',             lambda x: float(x), "(float) Bias for spectroscopy (relative z-offset)."],
                    '-specI'    : ['100e-12',       lambda x: float(x), "(float) Set-point current for spectroscopy (relative z-offset)."],
                    '-corrN'    : ['1',             lambda x: int(x),   "(int) Correct every (n) experimental points."],
                    '-addN'     : ['1',             lambda x: int(x),   "(int) If no drift is detected, add this number to nth drift correct frame."],
                    '-zDrift'   : [False,           lambda x: bool(x),  "(booolean) Apply piezo drift correction in z before each point."],
                    '-tipSpeed' : ['2e-9',          lambda x: float(x), "(float) Speed of tip to move between each point."],
                    '-maxDrift' : ['20',            lambda x: int(x),   "(int) Maximum drift correction applied as %' of drift frame."],
        }

        if(_help): return arg_dict                                              # This will get called when the user runs the command 'help exUnthreaded'

        error,user_arg_dict = self.interface.userArgs(arg_dict,user_args)       # This will validate args V and wait as float() and int(), respectively
        if(error):
            return error + "\nRun ```help change_bias2``` if you're unsure."    # Inform the user of any errors in their input

        # Unpack the arguments
        args = self.interface.unpackArgs(user_arg_dict)                         

        func = lambda : self.driftSTS(*args)                                    # Handle to the function to be threaded
        return self.interface.threadTask(func)                                  # Return and thread the function


    def driftSTS(self, pat, img_V, img_I, spec_V, spec_I, corr_nth, add_to_nth, zDrift, tip_speed, max_drift):
        """
        Corrects the drift during large STS grids/lines. Particularly useful at 77 K. 
        Pauses the experiment periodically to acquire images. If the sample has drifted,
        the offset will be calculated and applied to both the scan window and pattern 
        location.

        Parameters
        ----------
        pat         : Pattern type: 'Line', 'Grid', or 'Cloud'
        img_V       : Bias (V) for imaging 
        img_I       : Set current (I) for imaging
        spec_V      : Setpoint bias (V) for spectroscopy (relative z-offset)
        spec_I      : Setpoint current (I) for spectroscopy (relative z-offset)
        corr_nth    : Correct drift every (n) points in the experiment
        add_to_nth  : If no drift is detected, add this number to corr_nth. 
                    Now drift will be corrected every corr_nth + add_to_nth point.

        Arguments
        ----------
        user_args : arguments passed in by the user when calling the command
        _help     : flag set when user calls "help change_bias"

        """

        ##########################################################
        ##################### LIST OF OPERATIONS #################
        ##########################################################
        
        # First iteration: #
        # - Take reference image
        # - Get pattern info
        # - Move to first point
        # - Ramp to STS set-point (if not already)
        # - **START** experiment

        # Normal operation: #
        # loop
        # - Resume experiment (i.e. take STS. First point unaffected)
        # - Send pause experiment
        # - Wait end of STS
        # - Experiment now paused
        # - If nth frame to drift correct:
        # -- Ramp to imaging set-point
        # -- Take comparison image
        # -- Calculate and apply drift correction
        # -- If zero drift, add to nth correction frame
        # - Move to point
        # - ramp STS set-point (if not already)
        # loop
        
        #########################################
        #------------ INPUT HANDLING -----------#
        #########################################

        # Convert max_drift to percentage (decimal)
        max_drift /= 100

        #Input checking:
        if (not pat in ['Line', 'Grid', 'Cloud']):
            errorMessage = "Pattern must be one of: 'Line', 'Grid', or 'Cloud'."
            global_.running.clear()                                             # Free up the running flag. This must be done whenever exiting a threaded function
            return errorMessage

        if (img_V == 0 or spec_V == 0):
            errorMessage = "Cannot set bias to zero! Check your inputs."
            global_.running.clear()                                             # Free up the running flag. This must be done whenever exiting a threaded function
            return errorMessage

        if (img_I <= 0 or spec_I <= 0):
            errorMessage = "Cannot set current negative or zero! Check your inputs."
            global_.running.clear()                                             # Free up the running flag. This must be done whenever exiting a threaded function
            return errorMessage
        
        if (img_V * spec_V <= 0):
            errorMessage = "Image and spectroscopy bias must be both positive or both negative to avoid (possible) bandgap."
            global_.running.clear()                                             # Free up the running flag. This must be done whenever exiting a threaded function
            return errorMessage

        
        #########################################
        #------------ NANONIS CONNECT ----------#
        #########################################

        NTCP,connection_error = self.interface.scanbot.connect()                # Connect to nanonis via TCP
        if(connection_error):
                global_.running.clear()                                             # Free up the running flag. This must be done whenever exiting a threaded function
                return connection_error                                             # Return error message if there was a problem connecting        
        
        bias = Bias(NTCP)
        zController = ZController(NTCP)
        scan = Scan(NTCP)
        signals = Signals(NTCP)
        biasSpec = BiasSpectr(NTCP)
        pattern = Pattern(NTCP)
        folme = FolMe(NTCP)
        piezo = Piezo(NTCP)

        #########################################
        #------------- SCAN CHANNELS -----------#
        #########################################

        # Define scan save channels
        num_channels, channel_indexes, pixels, lines = scan.BufferGet()
        # Here I want to use InSlotsGet to find the channel idx of 'Z (m)', but demo spits out error. Instead we find the index of the 'Z (m)' RTidx list returned by NamesGet.
        # signal_names = signals.NamesGet()
        # Here is code for non-demo Nanonis. signal_names have list index = channel, and correspoding signal_indexes = RTidx.
        signal_names, signal_indexes = signals.InSlotsGet()
        Zidx = signal_names.index('Z (m)')

        #########################################
        #---------- ACQUIRE 1st IMAGE ----------#
        #########################################

        # Ramp to imaging bias and set current:
        self.interface.scanbot.rampCurrent(NTCP, set_current=img_I, zhold=False) # dont turn off z-controller.
        self.interface.scanbot.rampBias(NTCP, bias=img_V, zhold=False) # dont turn off z-controller. This should be fine as img_V and spec_V are on the same side of the bandgap.
        
        # Get the tip set-point values with returned float32 precision for boolean checks later on:
        img_V = bias.Get()
        img_I = zController.SetpntGet()

        # Get current frame position:
        ix1, iy1, iw1, ih1, iangle1 = scan.FrameGet()

        # Calculate real-space resolution (metres/pixel)
        dxy = [iw1/pixels, ih1/lines]                                     

        # Start down scan
        scan.Action("start","down")
        """
        Wait end of scan
        """
        timeout_status, file_path_size, file_path = scan.WaitEndOfScan()
        self.interface.sendReply("Wait end of scan")
        self.interface.sendReply("file_path: " + file_path)
    
        # Grab frame data
        channel_name,im1,scan_direction = scan.FrameDataGrab(Zidx, 1)
        # Send image to front panel somehow <----------------------------------------

        # Check if user has called stop and stop the experiment.
        if(self.interface.scanbot.checkEventFlags() == True):               # Periodically check event flags when appropriate to see if the user has called "stop"
            scan.Action('stop')
            self.interface.sendReply("Stopping early!")                     # This is how you send a reply.
            self.interface.scanbot.disconnect(NTCP)                                 # Remember to free up the TCP port
            global_.running.clear()                                                 # Free up the running flag. This must be done whenever exiting a threaded function
            return

        #########################################
        #----------- START EXPERIMENT ----------#
        #########################################
        # Open pattern tab
        pattern.ExpOpen()

        # Grab pattern info: #

        # For lines:
        if pat == 'Line':
            # Get Line information
            num_points, x1, y1, x2, y2 = pattern.LineGet()

            # Calculate all x,y coordinates of the points along the Line:
            x_spacing = (x2 - x1)/(num_points-1)
            y_spacing = (y2 - y1)/(num_points-1)
            x = [x1 + point*x_spacing for point in range(num_points)]
            y = [y1 + point*y_spacing for point in range(num_points)]

            line_dict = {0: (x,y)}

        # For grids:
        if pat == 'Grid':
            # Get grid information
            num_points_x, num_points_y, center_x, center_y, w, h, angle = pattern.GridGet()

            ##NOTE##
            # The experiment points in the grid are the CELL CENTRES.
            # These ARE NOT the 'points' (or cell intersections) of the grid.
            # However, these 'points' are what are returned by nanonis.
            # Therefore we have to offset all of the 'points' in the grid 
            # by half a cell to get the correct experiment locations.

            # Calculate total number of points:
            num_points = num_points_x * num_points_y
            # Calcylate all x,y coordinates of the points in the grid:
            x1, x2 = [center_x - w/2, center_x + w/2] # coordinates of the horizontal ends of the grid
            y1, y2 = [center_y - h/2, center_y + h/2] # coordinates of the vertical ends of the grid

            x_spacing = w/(num_points_x) # Real-space separation of EXPERIMENT points in x-direction
            y_spacing = h/(num_points_y) # Real-space separation of EXPERIMENT points in x-direction

            # Now we calculate the coordinates of the EXPERIMENT points (i.e cell centers, done by shifting grid by half a cell).
            rows = [x1 + (point + 0.5)*x_spacing for point in range(num_points_x)] # List of x-coordinates of points
            cols = [y1 + (point + 0.5)*y_spacing for point in range(num_points_y)] # List of y-coordinates of points

            # If angle is non-zero, calculate rotated coordinates
            if angle != 0.0:
                def DoRotation(ox, oy, xspan, yspan, RotRad=0):
                    """Generate a meshgrid and rotate it by RotRad radians."""
                    # Shift grid to (0,0)
                    xspan = [x - ox for x in xspan]
                    yspan = [y - oy for y in yspan]

                    # Clockwise, 2D rotation matrix
                    RotMatrix = np.array([[np.cos(RotRad),  np.sin(RotRad)],
                                        [-np.sin(RotRad), np.cos(RotRad)]])

                    # Create array of experiment coordinates X=[[x1,x2],[x1,x2]...] and Y=[[y1, y1], [y2,y2]...]
                    X, Y = np.meshgrid(xspan, yspan)

                    # Rotate experiment coordinates by angle
                    Xr, Yr = np.einsum('ji, mni -> jmn', RotMatrix, np.dstack([X, Y]))

                    # Convert this array into an iterable list for drift correct loop
                    xr, yr = [Xr.flatten(), Yr.flatten()]

                    # Shift coordinates back to grid center (x,y)
                    xr = [x + ox for x in xr]
                    yr = [y + oy for y in yr]

                    return [xr, yr]
                # Apply rotation function
                x, y = DoRotation(center_x, center_y, rows, cols, np.pi*angle/180)
            else: # If angle = 0:
                # Create array of experiment coordinates X=[[x1,x2],[x1,x2]...] and Y=[[y1, y1], [y2,y2]...]
                X, Y = np.meshgrid(rows,cols) 
                # Convert this array into an iterable list for drift correct loop
                x, y = [X.flatten(), Y.flatten()]
                

           
            # Create an array where every element is a (x,y) tuple of the experiment points
            exp_coords = np.column_stack((x,y))
            # Save list of coordinates in tuples [(x1,y1), (x2,y2), ...]
            grid_dict = {0: exp_coords}


        # For clouds:
        if pat == 'Cloud':
            # Get cloud information:
            num_points, x, y = pattern.CloudGet()

            # Save cloud points in dictionary
            cloud_dict = {0: (x,y)}

        # Set tip speed (input eventually?)
        folme.SpeedSet(speed=tip_speed, custom_speed=True)

        # Move to first point
        folme.XYPosSet(X=x[0], Y=y[0], Wait_end_of_move=True)

        # Pull STS settings to calculate sweep duration 
        #
        props = biasSpec.PropsGet()
        timing = biasSpec.TimingGet()

        sweep_time = props['num_sweeps'] * (props['num_points']*(timing['integration_time'] + timing['settling_time']) + timing['z_averaging_time'] + timing['initial_settling_time'] + timing['end_settling_time'] + timing['z_control_time'])
        if (props['back_sweep']):
            sweep_time *= 2

        # Ramp to STS set-point: 
        self.interface.scanbot.rampBias(NTCP, bias=spec_V, zhold=False) # dont turn off z-controller. This should be fine as img_V and spec_V are on the same side of the bandgap.
        self.interface.scanbot.rampCurrent(NTCP, set_current=spec_I, zhold=False) # dont turn off z-controller.

        # Get the tip set-point values with returned float32 precision for boolean checks later on:
        spec_V = bias.Get()
        spec_I = zController.SetpntGet()

        # Correct z-creep / drift
        if(zDrift):
            # Set up time axis for gradient calculation
            t = np.arange(0,5,0.25)
            # Grab current compensation:
            status, vx, vy, vz, xsat, ysat, zsat = piezo.DriftCompGet()
            
            # Iterate drift compensation n times:
            for n in range(2):
                # Initialise
                z = []
                # Collect z-position over 5 s (4 samples per second)
                for i in range(5*4):
                    z.append(signals.ValGet(signal_index = Zidx, wait_for_newest_data=True)) #<---- should this be signals.ValGet() or zController.ZPosGet()?
                    time.sleep(0.25)
                #<--------------- ln fit here. have to fit against time axis with 0.25 s spaced points
                m, b = np.polyfit(t, z, 1)
                # Apply piezo drift compensaiton:
                vz += m
                piezo.DriftCompSet(on=True, vx=vx, vy=vy, vz=vz)

        # Start the experiment
        pattern.ExpStart('no change')

        #########################################
        #--------- DRIFT CHECK EVERY N ---------#
        #########################################
        
        # Intialise first drift correction point
        corr_point = corr_nth - 1
        
        for point in range(0,num_points):

            # Check if user has called stop and stop the experiment.
            if(self.interface.scanbot.checkEventFlags() == True):               # Periodically check event flags when appropriate to see if the user has called "stop"
                pattern.ExpStop()                                               # Stop the experiment
                biasSpec.Stop()
                scan.Action('stop')
                self.interface.sendReply("Stopping early!")                     # This is how you send a reply.
                break

            # Send progress to front panel:
            if (self.interface.run_mode == 'p'):
                self.interface.panel.updateProgress(point+1,num_points)
            
            # Resume, i.e. take the next experiment point at the start of each loop.
            # ------- This has no effect on the first point.
            pattern.ExpPause('resume')
            # Wait some time
            time.sleep(1)

            # Exit the loop here if its the last point, no need to drift correct or move to the next point.
            if (point == num_points):
                time.sleep(sweep_time)
                # Check if STS is finished. If not, check every 100 ms until it is.
                while biasSpec.StatusGet():
                    time.sleep(0.1)
                break

            # Pause the experiment (after STS completed)
            pattern.ExpPause('pause')

            # Wait for STS to finish
            time.sleep(sweep_time - 1)
            # Check if STS is finished. If not, check every 100 ms until it is.
            while biasSpec.StatusGet():
                time.sleep(0.1)

            if (point == corr_point):

                # Ramp to imaging set-point:
                self.interface.scanbot.rampCurrent(NTCP, set_current=img_I, zhold=False) # dont turn off z-controller.
                self.interface.scanbot.rampBias(NTCP, bias=img_V, zhold=False) # dont turn off z-controller. This should be fine as img_V and spec_V are on the same side of the bandgap.

                #########################################
                #---- MOVE FRAME (ARTIFICIAL DRIFT) ----#
                #########################################

                # # Shift frame centre by a small amount:
                # ixs = ix1 + iw1*0.02
                # iys = iy1 + ih1*0.04

                # # Apply shift
                # scan.FrameSet(ixs, iys, iw1, ih1, iangle1)

                #########################################
                #---------- ACQUIRE Nth IMAGE ----------#
                #########################################

                # Get current frame position:
                ix2, iy2, iw2, ih2, iangle2 = scan.FrameGet()

                scan.Action("start","down")

                """
                Status Get
                """

                """
                Wait end of scan
                """
                timeout_status, file_path_size, file_path = scan.WaitEndOfScan()
                self.interface.sendReply("Wait end of scan")
                self.interface.sendReply("file_path: " + file_path)

                """
                FrameDataGrab
                """
                channel_name,im2,scan_direction = scan.FrameDataGrab(Zidx, 1)

                #########################################
                #------------ ESTIMATE DRIFT -----------#
                #########################################
    
                ox, oy = self.getFrameOffset2(im1, im2, dxy=dxy, theta = np.pi*iangle1/180)
                # Percentage drift
                pox, poy = [ox/iw1, oy/ih1]

                #########################################
                #-------- APPLY DRIFT CORRECTION -------#
                #########################################

                if (ox == 0 and oy == 0):
                    # Make next nth point to drift correct larger by add_to_nth
                    corr_nth += add_to_nth
                    # Update next drift correction point
                    corr_point += corr_nth
                    self.interface.sendReply('No drift detected')
                    # print("----------------------------------------------------------------------")

                if (pox > max_drift or poy > max_drift):
                    self.interface.sendReply('Calculated drift larger than allowed maximum.')
                    pattern.ExpStop()                                               # Stop the experiment
                    biasSpec.Stop()
                    scan.Action('stop')
                    self.interface.sendReply("Stopping early!")                     # This is how you send a reply.
                    break
                else:
                    """""
                    SCAN FRAME
                    """""
                    # Calculate drift-corrected frame centre
                    ix2 += -ox
                    iy2 += -oy

                    # Apply correction to scan frame
                    scan.FrameSet(ix2, iy2, iw2, ih2, iangle2)

                    """""
                    STS LINE
                    """""
                    # Shift STS Pattern:
                    x = [value - ox for value in x]
                    y = [value - oy for value in y]
                    
                    if pat == 'Line':
                        # Apply shift to STS line:
                        pattern.LineSet(num_points, x[0], y[0], x[-1], y[-1])
                        
                        # Save shifted points in dictionary
                        line_dict[point] = x,y

                    if pat == 'Grid':
                        # Calculate grid shift:
                        center_x += - ox
                        center_y += - oy
                        # Apply shift to STS grid:
                        pattern.GridSet(num_points_x, num_points_y, center_x, center_y, w, h , angle)

                        # Create an array where every element is a (x,y) tuple of the experiment points
                        exp_coords = np.column_stack((x,y))
                        # Save list of coordinates in tuples [(x1,y1), (x2,y2), ...]
                        grid_dict[point] = exp_coords

                    if pat == 'Cloud':
                        # Apply shift to STS cloud:
                        pattern.CloudSet(x,y)

                        # Save shifted points in dictionary
                        cloud_dict[point] = (x,y)

                    # Update next drift correction point
                    corr_point += corr_nth
                    # self.interface.sendReply('Drift correction applied: (%0.2f pm, %0.2f pm)' % ( -ox*1e12, -oy*1e12))
                    # print("----------------------------------------------------------------------")
                

            #########################################
            #---------- SET UP NEXT POINT ----------#
            #########################################
            
            # Move to the next experiment point:
            if point < (num_points-1):
                folme.XYPosSet(x[point+1], y[point+1], Wait_end_of_move=True)

            # Ramp to STS set-point (if not already):
            if bias.Get() != spec_V or zController.SetpntGet() != spec_I:
                self.interface.scanbot.rampBias(NTCP, bias=spec_V, zhold=False) # dont turn off z-controller. This should be fine as img_V and spec_V are on the same side of the bandgap.
                self.interface.scanbot.rampCurrent(NTCP, set_current=spec_I, zhold=False) # dont turn off z-controller.

            # Correct z-creep / drift
            if(zDrift):
                # Set up time axis for gradient calculation
                t = np.arange(0,5,0.25)
                # Grab current compensation:
                status, vx, vy, vz, xsat, ysat, zsat = piezo.DriftCompGet()
                
                # Iterate drift compensation n times:
                for n in range(2):
                    # Initialise
                    z = []
                    # Collect z-position over 5 s (4 samples per second)
                    for i in range(5*4):
                        z.append(signals.ValGet(signal_index = Zidx, wait_for_newest_data=True)) #<---- should this be signals.ValGet() or zController.ZPosGet()?
                        time.sleep(0.25)
                    #<--------------- ln fit here. have to fit against time axis with 0.25 s spaced points
                    m, b = np.polyfit(t, z, 1)
                    # Apply piezo drift compensaiton:
                    vz += m
                    piezo.DriftCompSet(on=True, vx=vx, vy=vy, vz=vz)

        """""
        END OF EXPERIMENT
        """""

        # Park tip at imaging set-point:
        self.interface.scanbot.rampCurrent(NTCP, set_current=img_I, zhold=False) # dont turn off z-controller.
        self.interface.scanbot.rampBias(NTCP, bias=img_V, zhold=False) # dont turn off z-controller. This should be fine as img_V and spec_V are on the same side of the bandgap.
        
        self.interface.sendReply("Experiment completed")

        self.interface.scanbot.disconnect(NTCP)

        global_.running.clear()                                                 # Free up the running flag. This must be done whenever exiting a threaded function

        return

    def calDrifthook(self, user_args, _help=False):
        #########################################
        #------------ INPUT HANDLING -----------#
        #########################################
        """""
        Arguments
        ----------
        user_args : arguments passed in by the user when calling the command
        _help     : flag set when user calls "help cal_drift"
        
        """""


        arg_dict = {'-method'   : ['Recursive',     lambda x: str(x),   "(string) Method for drift calibration."],
                    '-imgV'     : ['1',             lambda x: float(x), "(float) Bias for acquiring images."],
                    '-imgI'     : ['20e-12',        lambda x: float(x), "(float) Set-point current for acquiring images."],
        }

        if(_help): return arg_dict                                              # This will get called when the user runs the command 'help exUnthreaded'

        error,user_arg_dict = self.interface.userArgs(arg_dict,user_args)       # This will validate args V and wait as float() and int(), respectively
        if(error):
            return error + "\nRun ```help cal_drift``` if you're unsure."    # Inform the user of any errors in their input

        # Unpack the arguments
        args = self.interface.unpackArgs(user_arg_dict)                         

        func = lambda : self.calDrift(*args)                                    # Handle to the function to be threaded
        return self.interface.threadTask(func)                                  # Return and thread the function

    def calDrift(self, method, img_V, img_I):
        """
        Determines the x,y drift vector of STM images and applies correction to the piezos.
        Built for experiments while heating the STM stage, e.g. 100 K.
        
        Two methods to be tried: 
        
        - First, a trivial but recursive process of correlating image pairs 
          and determining the offset.
          * Long process, imperfect.
          * Possibly ruin tip in process.
        
        - Second, detection of the image skew axis using lines created in the fourier
          transform. Angle theoretically easy to determine, however, magnitude will 
          be much harder


        Parameters
        ----------
        method      : Method to use for drift calibration.
        img_V       : Bias (V) for imaging 
        img_I       : Set current (I) for imaging

        """
        #########################################
        #------------ INPUT HANDLING -----------#
        #########################################

        if (not method in ['Recursive', 'FFT']):
            errorMessage = "Method must be one of: 'Recursive' or 'FFT'."
            global_.running.clear()
            return errorMessage

        if (img_V == 0):
            errorMessage = "Cannot set bias to zero! Check your inputs."
            global_.running.clear()                                             # Free up the running flag. This must be done whenever exiting a threaded function
            return errorMessage

        if (img_I <= 0):
            errorMessage = "Cannot set current negative or zero! Check your inputs."
            global_.running.clear()                                             # Free up the running flag. This must be done whenever exiting a threaded function
            return errorMessage
        
        #########################################
        #------------ NANONIS CONNECT ----------#
        #########################################

        NTCP,connection_error = self.interface.scanbot.connect()                # Connect to nanonis via TCP
        if(connection_error):
            global_.running.clear()                                         # Free up the running flag. This must be done whenever exiting a threaded function
            return connection_error                                         # Return error message if there was a problem connecting        
        
        scan = Scan(NTCP)
        signals = Signals(NTCP)
        piezo = Piezo(NTCP)

        #########################################
        #------------- SCAN CHANNELS -----------#
        #########################################

        # Define scan save channels
        num_channels, channel_indexes, pixels, lines = scan.BufferGet()
        # Here I want to use InSlotsGet to find the channel idx of 'Z (m)', but demo spits out error. Instead we find the index of the 'Z (m)' RTidx list returned by NamesGet.
        # signal_names = signals.NamesGet()
        # Here is code for non-demo Nanonis. signal_names have list index = channel, and correspoding signal_indexes = RTidx.
        signal_names, signal_indexes = signals.InSlotsGet()
        Zidx = signal_names.index('Z (m)')

        #########################################
        #-------------- CORRECTION -------------#
        #########################################

        # Grab current compensation:
        status, vx, vy, vz, xsat, ysat, zsat = piezo.DriftCompGet()

        """""    
        METHOD 1: Recursive correction

        """""

        self.interface.sendReply("Beginning recursive correction...")

        if (method == 'Recursive'):

            # Ramp to imaging bias and set current:
            self.interface.scanbot.rampCurrent(NTCP, set_current=img_I, zhold=False) # dont turn off z-controller.
            self.interface.scanbot.rampBias(NTCP, bias=img_V, zhold=False) # dont turn off z-controller. This should be fine as img_V and spec_V are on the same side of the bandgap.
        
            # Get current frame position:
            x, y, w, h, angle = scan.FrameGet()

            # Calculate real-space resolution (metres/pixel)
            dxy = [w/pixels, h/lines]      

            #########################################
            #----------- IMAGE ACQUISIITON ---------#
            #########################################

            im = {}                              # Initialise image dictionary
            T = ['', '']                         # Initialise time list

            for i in range(2):
                # Start down scan
                scan.Action("start","down")
                """
                Wait end of scan
                """
                timeout_status, file_path_size, file_path = scan.WaitEndOfScan()
                self.interface.sendReply("Wait end of scan")
                self.interface.sendReply("file_path: " + file_path)
               
                # Record end time of frame
                T[i] = time.time()
                self.interface.sendReply("Time: %3.3f" % T[i])
            
                # Grab frame data
                channel_name, im[i], scan_direction = scan.FrameDataGrab(Zidx, 1)

                # Check if user has called stop and stop the experiment.
                if(self.interface.scanbot.checkEventFlags() == True):               # Periodically check event flags when appropriate to see if the user has called "stop"
                    scan.Action('stop')
                    self.interface.sendReply("Stopping early!")                     # This is how you send a reply.
                    self.interface.scanbot.disconnect(NTCP)                                 # Remember to free up the TCP port
                    global_.running.clear()                                                 # Free up the running flag. This must be done whenever exiting a threaded function
                    return

            #########################################
            #------------ ESTIMATE DRIFT -----------#
            #########################################
            
            # Estimate drift vectors:
            ox, oy = self.getFrameOffset2(im[0], im[1], dxy=dxy, theta = np.pi*angle/180)
            self.interface.sendReply("Drift distance: dx = %1.1e m, dy = %1.1e m." % (ox, oy))

            # Estimate drift speed:
            dT = T[1] - T[0]                                                        # Elapsed time between frames
            self.interface.sendReply("dT = %3.3f" % dT)
            vx, vy = [ox/dT, oy/dT]
            self.interface.sendReply("Drift compensation: vx = %1.1e m/s, vy = %1.1e m/s." % (vx, vy))

            #########################################
            #-------- APPLY DRIFT CORRECTION -------#
            #########################################

            if (ox != 0 and oy != 0):
                """""
                PIEZOS
                """""
                # Apply piezo drift compensaiton:
                piezo.DriftCompSet(on=True, vx=vx, vy=vy, vz=vz)

                """""
                SCAN FRAME
                """""
                # Calculate drift-corrected frame centre
                x += -ox
                y += -oy

                # Apply correction to scan frame
                scan.FrameSet(x, y, w, h, angle)

                self.interface.sendReply("Applied Drift Correction")



        """""
        END OF DRIFT CALIBRATION
        """""
        
        self.interface.sendReply("Drift calibration completed")

        self.interface.scanbot.disconnect(NTCP)

        global_.running.clear()                                                 # Free up the running flag. This must be done whenever exiting a threaded function
        
        return