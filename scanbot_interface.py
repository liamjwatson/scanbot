# -*- coding: utf-8 -*-
"""
Created on Fri May  6 12:06:37 2022

@author: jack hellerstedt and julian ceddia
"""

from scanbot import scanbot
import threading

import firebase_admin
from firebase_admin import credentials, storage

import os
import ipaddress
from pathlib import Path

import global_
import nanonisUtils as nut

import zulip

class scanbot_interface(object):
    bot_message = []
    bot_handler = []
    validUploadMethods = ['zulip','firebase']
    
###############################################################################
# Constructor
###############################################################################
    def __init__(self):
        ## Globals
        global_.tasks   = []
        global_.running = threading.Event()                                     # event to stop threads
        global_.pause   = threading.Event()                                     # event to pause threads
        
        self.init()
        
        self.client = zulip.Client(config_file=self.zuliprc)
        self.getWhitelist()                                                     # Load in whitelist file if there is one
        self.firebaseInit()                                                     # Initialise Firebase
        self.initCommandDict()
        self.scanbot = scanbot(self)
    
###############################################################################
# Initialisation
###############################################################################
    def init(self):
        initDict = {'zuliprc'                   : 'zuliprc',
                    'upload_method'             : 'zulip',
                    'firebase_credentials'      : '',
                    'firebase_storage_bucket'   : '',
                    'firebase_storage_path'     : '',
                    'port_list'                 : '6501,6502,6503,6504',
                    'ip'                        : '127.0.0.1',
                    'notify_list'               : ''}
        
        with open('scanbot_config.txt','r') as f:
            line = "begin"
            while(line):
                line = f.readline()[:-1]
                if(not ': ' in line): print("From config: " + line); continue
                key, value = line.split(': ')
                if(not key in initDict):
                    print("Invalid key in scanbot_config.txt: " + key)
                    continue
                
                initDict[key] = value
                
        self.zuliprc      = initDict['zuliprc']
        self.uploadMethod = initDict['upload_method']
        self.firebaseCert = initDict['firebase_credentials']
        self.firebaseStorageBucket = initDict['firebase_storage_bucket']
        self.firebaseStoragePath   = initDict['firebase_storage_path']
        
        portList = initDict['port_list']
        if(portList):
            if(',' in portList):
                portList = portList.split(',')
                self.setPortList(portList)
            else:
                self.setPortList([portList])
                
        self.setIP([initDict['ip']])
        
        self.notifyUserList = []
        notifyList = initDict['notify_list']
        if(notifyList):
            if(',' in notifyList):
                notifyList = notifyList.split(',')
                self.notifyUserList = notifyList
            else:
                self.notifyUserList = [notifyList]
        
        self.notifications = True
        
    def getWhitelist(self):
        self.whitelist = []
        try:
            with open('whitelist.txt', 'r') as f:
                d = f.read()
                self.whitelist = d.split('\n')[:-1]
        except:
            print('No whitelist... add users to create one')
    
    def firebaseInit(self):
        print("Initialising firebase app")
        try:
            cred = credentials.Certificate(self.firebaseCert)                   # Your firebase credentials
            firebase_admin.initialize_app(cred, {
                'storageBucket': self.firebaseStorageBucket                     # Your firebase storage bucket
            })
        except Exception as e:
            print(e)
    
    def initCommandDict(self):
        self.commands = {'list_commands'    : self.listCommands,
                         'help'             : self._help,
                         'add_user'         : self.addUsers,
                         'list_users'       : lambda args: str(self.whitelist),
                         'noti'             : self.noti,
                         'set_ip'           : self.setIP,
                         'get_ip'           : lambda args: self.IP,
                         'set_portlist'     : self.setPortList,
                         'get_portlist'     : lambda args: self.portList,
                         'set_upload_method': self.setUploadMethod,
                         'get_upload_method': lambda args: self.uploadMethod,
                         'survey'           : self.survey,
                         'stop'             : self.stop,
                         'pause'            : lambda args: global_.pause.set(),
                         'resume'           : lambda args: global_.pause.clear(),
                         'enhance'          : self.enhance,
                         'plot'             : self.plot,
                         'move_tip'         : self.moveTip,
                         'tip_shape_props'  : self.tipShapeProps,
                         'tip_shape'        : self.tipShape,
                         'pulse_props'      : self.pulseProps,
                         'pulse'            : self.pulse,
                         'bias_dep'         : self.biasDep,
                         'set_bias'         : self.setBias,
                         'stitch_survey'    : self.stitchSurvey,
                         'watch'            : self.watch,
                         'creep'            : self.creep,
                         'add_notify_list'  : self.addNotifyUser,
                         'get_notify_list'  : lambda args: self.notifyUserList
        }
    
###############################################################################
# Scanbot
###############################################################################
    def creep(self,user_args,_help=False):
        arg_dict = {'-ip'   : [self.IP, lambda x: str(x), "(str) Creep IP"],
                    '-port' : ['6501',  lambda x: int(x), "(int) Creep Port"]}
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help creep``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        # self.scanbot.watch("",*args)
        func = lambda : self.scanbot.watch("",*args)
        return self.threadTask(func)
    
    def watch(self,user_args,_help=False):
        arg_dict = {'-s'    : ['sbwatch',  lambda x: str(x),   "(str) Suffix at the end of autosaved sxm files"]}
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help watch``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        
        func = lambda : self.scanbot.watch(*args)
        return self.threadTask(func)
        
    def survey(self,user_args,_help=False):
        arg_dict = {'-bias' : ['-default', lambda x: float(x), "(float) Scan bias"],
                    '-n'    : ['5',        lambda x: int(x),   "(int) Size of the nxn grid of scans"],
                    '-i'    : ['1',        lambda x: int(x),   "(int) Start the grid from this index"],
                    '-s'    : ['scanbot',  lambda x: str(x),   "(str) Suffix at the end of autosaved sxm files"],
                    '-xy'   : ['100e-9',   lambda x: float(x), "(float) Length and width of the scan frame (m)"],
                    '-dx'   : ['150e-9',   lambda x: float(x), "(float) Scan grid spacing (m)"],
                    '-st'   : ['10',       lambda x: float(x), "(float) Drift compensation time (s)"]}
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help survey``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        
        func = lambda : self.scanbot.survey(*args)
        return self.threadTask(func)
        
    def enhance(self,user_args,_help=False):
        arg_dict = {'-bias' : ['-default', lambda x: float(x), "(float) Scan bias. (-default=unchanged)"],
                    '-n'    : ['5',        lambda x: int(x),   "(int) Size of nxn grid within enhanced frame"],
                    '-i'    : ['1',        lambda x: int(x),   "(int) Start the grid from this index"],
                    '-s'    : ['enhance',  lambda x: str(x),   "(str) Suffix at the end of autosaved sxm files"],
                    '-st'   : ['-default', lambda x: float(x), "(float) Drift compensation time (s)"]}
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help enhance``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        
        func = lambda : self.scanbot.enhance(*args)
        return self.threadTask(func,override=True)
        
    def plot(self,args):
        self.scanbot.plot(args)
    
    def moveTip(self,user_args,_help=False):
        arg_dict = {'-pos'     : ['0,0', lambda x: [float(p) for p in x.split(',')], "(float array) position of the tip x,y"]}
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help move_tip``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        
        self.scanbot.moveTip(*args)
        
    def tipShape(self,user_args,_help=False):
        arg_dict = {'-np'     : ['1',   lambda x: int(x),   "(int) Number of bias pulses after tip shape"], 
                    '-pw'     : ['0.1', lambda x: float(x), "(float) Pulse width (s)"],
                    '-bias'   : ['3',   lambda x: float(x), "(float) Bias pulse value (V)"],
                    '-zhold'  : ['0',   lambda x: int(x),   "(int) Z-Controller on hold (0=nanonis setting, 1=deactivated, 2=activated)"],
                    '-abs'    : ['0',   lambda x: int(x),   "(int) Abs or rel mode (0=nanonis, 1=relative, 2=absolute)"]}
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help tip_shape``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        
        self.scanbot.tipShape(*args)
        
    def tipShapeProps(self,user_args,_help=False):
        arg_dict = {'-sod'    : ['-default',lambda x: float(x), "(float) Switch off delay: the time (s) during which the Z position is averaged before switching the Z controller off"],
                    '-cb'     : ['-default',lambda x: int(x),   "(int) Change bias flag (0=Nanonis,1=Change Bias,2=Don't Change"],
                    '-b1'     : ['-default',lambda x: float(x), "(float) The value applied to the Bias signal if cb is true"],
                    '-z1'     : ['-default',lambda x: float(x), "(float) First tip lift (m) (i.e. -2e-9)"],
                    '-t1'     : ['-default',lambda x: float(x), "(float) Defines the time to ramp Z from current Z position to z1"],
                    '-b2'     : ['-default',lambda x: float(x), "(float) Bias voltage applied just after the first Z ramping"],
                    '-t2'     : ['-default',lambda x: float(x), "(float) Time to wait after applying the Bias Lift value b2"],
                    '-z3'     : ['-default',lambda x: float(x), "(float) Height the tip is going to ramp for the second time (m) i.e. +5nm"],
                    '-t3'     : ['-default',lambda x: float(x), "(float) Time to ramp Z in the second ramping [s]."],
                    '-wait'   : ['-default',lambda x: float(x), "(float) Time to wait after restoring the initial Bias voltage"],
                    '-fb'     : ['-default',lambda x: int(x),   "(int) Restore the initial Z-Controller status. 0: off. 1: on"],
                    '-area'   : ['-600e-9,-600e-9,300e-9,300e-9', lambda x: [float(p) for p in x.split(',')], "(float array) Designated tip-shaping area [x_centre,y_centre,width,height]"],
                    '-grid'   : ['10',      lambda x: int(x),   "(int) Grid size in the designated area"]}
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help tip_shape_props``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        
        self.scanbot.tipShapeProps(*args)
    
    def pulseProps(self,user_args,_help=False):
        arg_dict = {'-n'      : ['1',  lambda x: int(x),   "(Int) Number of pulses"],
                    '-pw'     : ['.1', lambda x: float(x), "(float) Pulse width (s)"],
                    '-bias'   : ['3',  lambda x: float(x), "(float) Bias value (V)"],
                    '-zhold'  : ['0',  lambda x: int(x),   "(int) Z-Controller on hold (0=nanonis setting, 1=deactivated, 2=activated)"],              # 
                    '-abs'    : ['0',  lambda x: int(x),   "(int) Abs or rel mode (0=nanonis,1=rel,2=abs)"]
                    }
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help pulse_props``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        self.scanbot.pulseProps(*args)
        
    def pulse(self,user_args,_help=False):
        self.scanbot.pulse()
        
    def biasDep(self,user_args,_help=False):
        arg_dict = {'-n'   : ['5',   lambda x: int(x),   "(int) Number of images to take b/w initial and final bias"],
                    '-bdc' : ['-1',  lambda x: float(x), "(float) Drift correct image bias"],
                    '-bi'  : ['-1',  lambda x: float(x), "(float) Initial Bias"],
                    '-bf'  : ['1',   lambda x: float(x), "(float) Final Bias"],
                    '-px'  : ['128', lambda x: int(x),   "(int) Pixels in drift correct image. 0=no drift correction"],
                    '-s'   : ['',    lambda x: str(x),   "(str) Suffix for the set of bias dep sxm files"]}
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help bias_dep``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        
        func = lambda : self.scanbot.biasDep(*args)
        return self.threadTask(func)
    
    def setBias(self,user_args,_help=False):
        arg_dict = {'-bias'   : ['0',   lambda x: float(x), "(float) Change the tip bias to this value. 0=No change"]}
        
        if(_help): return arg_dict
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help set_bias``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        self.scanbot.setBias(*args)
    
    def stop(self,args):
        if(global_.running.is_set()):
            self.scanbot.stop(args)
            global_.running.clear()
            global_.tasks.join()
        self.scanbot.stop(args)
        
    def threadTask(self,func,override=False):
        if(override): self.stop(args=[])
        if global_.running.is_set(): return "Error: something already running"
        global_.running.set()
        t = threading.Thread(target=func)
        global_.tasks = t
        t.start()
        return ""
        
###############################################################################
# Zulip
###############################################################################
    def handle_message(self, message, bot_handler=None):
        self.bot_handler = bot_handler
        if(bot_handler):
            if message['sender_email'] not in self.whitelist and self.whitelist:
                self.sendReply(message['sender_email'])
                self.sendReply('access denied')
                return
            
            self.bot_message = message
            
        command = message['content'].split(' ')[0].lower()
        args    = message['content'].split(' ')[1:]
        
        if(not command in self.commands):
            reply = "Invalid command. Run *list_commands* to see command list"
            self.sendReply(reply)
            return
        
        reply = self.commands[command](args)
        
        if(reply): self.sendReply(reply)
    
    def sendReply(self,reply):
        if(self.notifications):
            if(self.bot_handler):
                self.bot_handler.send_reply(self.bot_message, reply)
                return
        
        print(reply)                                                            # Print reply to console if zulip not available or notis turned off
    
    def reactToMessage(self,reaction):
        react_request = {
            'message_id': self.message['id'],
            'emoji_name': reaction,
            }
        self.client.add_reaction(react_request)
        
    def sendPNG(self,pngFilename,notify=True):
        notifyString = ""
        if(notify):
            for user in self.notifyUserList:
                notifyString += "@**" + user + "** "
            
        if(not self.bot_handler): print("pngs not supported yet"); return       # Don't support png
        path = os.getcwd() + '/' + pngFilename
        path = Path(path)
        path = path.resolve()
        
        if self.uploadMethod == 'zulip':
            upload = self.bot_handler.upload_file_from_path(str(path))
            uploaded_file_reply = "[{}]({})".format(path.name, upload["uri"])
            self.sendReply(notifyString + pngFilename)
            self.sendReply(uploaded_file_reply)
            
        if self.uploadMethod == 'firebase':
            bucket = storage.bucket()
            blob   = bucket.blob(self.firebaseStoragePath + pngFilename)
            blob.upload_from_filename(str(path))
        
            url = blob.generate_signed_url(expiration=9999999999)
            self.sendReply(notifyString + "[" + pngFilename + "](" + url + ")")
        
        os.remove(path)
    
    def addNotifyUser(self,username):
        self.notifyUserList.append(" ".join(username))
        
    def addUsers(self,user):
        try:
            self.whitelist.append(user[0])
            with open('whitelist.txt', 'w') as f:
                for w in self.whitelist:
                    f.write(w+'\n')
        except Exception as e:
            return str(e)
        
        return "user/s added sucessfully"
    
    def setIP(self,IP):
        try:
            IP = IP[0]
            ipaddress.ip_address(IP)
            self.IP = IP
            return "Set IP to " + self.IP
        except Exception as e:
            return str(e)
    
    def setPortList(self,portList):
        self.portList = [int(x) for x in portList]
        return "Updated ports: " + str(self.portList)
    
    def setUploadMethod(self,uploadMethod):
        uploadMethod = uploadMethod[0].lower()
        if(not uploadMethod in self.validUploadMethods):
            return "Invalid Method. Available methods:\n" + "\n". join(self.validUploadMethods)
        self.uploadMethod = uploadMethod
        return "Set upload method to " + self.uploadMethod
    
    def noti(self,args):
        if(not args[0].lower() in ["off","on"]):
            self.sendReply("Choose on or off")
            return
        
        if(args[0].lower() == "on"):
            self.notifications = True
            self.sendReply("Notifications on")
            
        if(args[0].lower() == "off"):
            self.sendReply("Turn notifications back on by")
            self.sendReply("```noti on```")
            self.notifications = False
    
    def listCommands(self,args):
        return "\n". join([c for c in self.commands])
    
###############################################################################
# Utilities
###############################################################################
    def stitchSurvey(self,user_args,_help=False):
        arg_dict = {'-s' : ['*', lambda x: str(x), "(str) Survey name"],
                    '-d' : ['0', lambda x: int(x), "(int) Delete survey after stitch. 0: Don't delete. 1: Delete"]}
        
        if(_help): return arg_dict
        
        if(self.uploadMethod == 'zulip'): return "Unsupported upload method"
        
        error,user_arg_dict = self.userArgs(arg_dict,user_args)
        if(error): return error + "\nRun ```help stitch_survey``` if you're unsure."
        
        args = self.unpackArgs(user_arg_dict)
        return nut.stitchSurvey(storage,self.firebaseStoragePath,*args)
                
    def userArgs(self,arg_dict,user_args):
        error = ""
        for arg in user_args:                                                   # Override the defaults if user inputs them
            try:
                key,value = arg.split('=')
            except:
                error = "Invalid argument"
                break
            if(not key in arg_dict):
                error = "invalid argument: " + key                              # return error message
                break
            try:
                arg_dict[key][1](value)                                         # Validate the value
            except:
                error  = "Invalid value for arg " + key + "."                   # Error if the value doesn't match the required data type
                break
            
            arg_dict[key][0] = value
        
        return [error,arg_dict]
    
    def unpackArgs(self,arg_dict):
        args = []
        for key,value in arg_dict.items():
            if(value[0] == "-default"):                                         # If the value is -default...
                args.append("-default")                                         # leave it so the function can retrieve the value from nanonis
                continue
            
            args.append(value[1](value[0]))                                     # Convert the string into data type
        
        return args
    
    def _help(self,args):
        if(not len(args)):
            helpStr = "Type ```help <command name>``` for more info\n"
            return helpStr + self.listCommands(args=[])
        
        command = args[0]
        if(not command in self.commands):
            return "Run ```list_commands``` to see valid commands"
        
        helpStr = "**" + command + "**\n"
        arg_dict = self.commands[command](args,_help=True)
        for key,value in arg_dict.items():
            helpStr += "```"
            helpStr += key + "```: " + value[2] + ". "
            helpStr += "Default: ```" +  value[0].replace("-default","nanonis") 
            helpStr += "```\n"
        
        return helpStr
    
handler_class = scanbot_interface