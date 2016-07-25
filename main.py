import Tkinter as tk
import glob
import os
import time
from watchdog.observers import Observer as FileAlterationMonitor
from watchdog.events import FileSystemEventHandler, PatternMatchingEventHandler
import json
from pymavlink import mavutil
from pymavlink.dialects.v10 import pixhawk

def foreach_children(widget,func):
        for child in widget.winfo_children():
            foreach_children(child,func)
            func(child)

class StandControlFrame(tk.Frame):
    def __init__(self, parent, name, devicePath, connected):
        tk.Frame.__init__(self, parent, padx=25, pady=25)

        self.name = name
        self.devicePath = devicePath
        self.deviceBaud = 57600

        self.headerLabel = tk.Label(self, text=name, font=("Helvetica", 16, "bold"))
        self.headerLabel.pack()

        self.armBtn = tk.Button(self, text='Arm', bg='lightgreen', command=self.armBtnPress)
        self.startBtn = tk.Button(self, text='Start', bg='lightblue', command=self.startBtnPress)
        self.disarmBtn = tk.Button(self, text='Disarm', bg='pink', command=self.disarmBtnPress)
        self.armBtn.pack()
        self.startBtn.pack()
        self.disarmBtn.pack()

        self.telemetryData = [
            {'name':'Arm', 'value':False},
            {'name':'Thr', 'value':0},
            {'name':'Volt', 'value':0.},
            {'name':'Curr', 'value':0.},
            {'name':'Pow', 'value':0.}
            ]

        self.telemFrame = tk.Frame(self)
        tk.Label(self.telemFrame, text='Telemetry', font=("Helvetica", 12, "bold")).grid(row=0,column=0,columnspan=2)

        self.telemetryLabels = []
        for i in range(len(self.telemetryData)):
            self.telemetryLabels.append({})
            self.telemetryLabels[i]['name'] = tk.Label(self.telemFrame, text=str(self.telemetryData[i]['name'])+':')
            self.telemetryLabels[i]['value'] = tk.Label(self.telemFrame, text=str(self.telemetryData[i]['value']))
            self.telemetryLabels[i]['name'].grid(row=i+1, column=0, sticky=tk.W)
            self.telemetryLabels[i]['value'].grid(row=i+1, column=1, sticky=tk.E)
        self.telemFrame.pack()

        self.removeBtn = tk.Button(self, text='Remove', command=self._removeBtnPress, padx=0, pady=0)
        self.removeBtn.pack()

        self.active = True
        self.deactivate_controls()

        if connected:
            self.connect()
        else:
            self.disconnect()

        self.after(10, self.tick)

    def activate_controls(self):
        if self.active:
            return
        self.active = True

        def configure_normal(widget):
            try:
                widget.configure(state=tk.NORMAL)
            except tk.TclError:
                pass

        foreach_children(self, configure_normal)

    def deactivate_controls(self):
        if not self.active:
            return
        self.active = False

        def configure_disabled(widget):
            try:
                widget.configure(state=tk.DISABLED)
            except tk.TclError:
                pass

        foreach_children(self, configure_disabled)
        self.removeBtn.configure(state=tk.NORMAL)

    def updateTelemLabels(self):
        for i in range(len(self.telemetryData)):
            self.telemetryLabels[i]['name'].config(text=self.telemetryData[i]['name'])
            self.telemetryLabels[i]['value'].config(text=self.telemetryData[i]['value'])

    def armBtnPress(self):
        print "arming %s!" % (self.name,)
        if self.mavlink_conn is not None:
            self.override = [1500,1500,1000,1500,1000,1000,1000,1000]
            self.mavlink_conn.arducopter_arm()

    def startBtnPress(self):
        print "starting %s!" % (self.name,)
        self.override = [1500,1500,1500,1500,1000,1000,1000,1000]


    def disarmBtnPress(self):
        print "disarming %s!" % (self.name,)
        if self.mavlink_conn is not None:
            self.override = [1500,1500,1000,1500,1000,1000,1000,1000]
            self.mavlink_conn.mav.rc_channels_override_send(0, 0, *self.override)
            self.mavlink_conn.arducopter_disarm()
            self.disarm_desired = True
            self.last_rc_override_send = time.time()

    def _removeBtnPress(self):
        self.removeBtnPress(self.devicePath)

    def removeBtnPress(self, devicePath):
        pass

    def connect(self):
        self.mavlink_conn = mavutil.mavserial(self.devicePath, baud=self.deviceBaud, autoreconnect=True)
        self.mavlink_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.mavlink_conn.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
        self.last_heartbeat_send = time.time()
        self.last_rc_override_send = time.time()
        self.last_heartbeat_recv = 0
        self.disarm_desired = False
        self.override = [1500,1500,1000,1500,1000,1000,1000,1000]

    def disconnect(self):
        self.mavlink_conn = None

    def process_mavlink_message(self, msg):
        if msg.get_type() == 'HEARTBEAT':
            self.last_heartbeat_recv = time.time()
            self.armed = bool((msg.base_mode & pixhawk.MAV_MODE_FLAG_DECODE_POSITION_SAFETY) == pixhawk.MAV_MODE_FLAG_DECODE_POSITION_SAFETY)
            if not self.armed:
                self.disarm_desired = False
            self.telemetryData[0]['value'] = self.armed
            self.updateTelemLabels()

        if msg.get_type() == 'VFR_HUD':
            self.telemetryData[1]['value'] = msg.throttle
            self.updateTelemLabels()

        if msg.get_type() == 'SYS_STATUS':
            V = msg.voltage_battery*0.001
            I = msg.current_battery*0.01
            self.telemetryData[2]['value'] = "%.2f" % V
            self.telemetryData[3]['value'] = "%.2f" % I
            self.telemetryData[4]['value'] = "%.2f" % V*I
            self.updateTelemLabels()



    def tick(self):
        self.after(10, self.tick)

        if self.mavlink_conn is not None:
            try:
                while True:
                    msg = self.mavlink_conn.recv_msg()
                    if msg is None:
                        break
                    self.process_mavlink_message(msg)

                    if time.time()-self.last_heartbeat_send >= 1.0:
                        self.mavlink_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                        self.mavlink_conn.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
                        self.last_heartbeat_send = time.time()
                        #self.mavlink_conn.set_mode(0)

                    if time.time()-self.last_rc_override_send >= 0.1:
                        if self.disarm_desired:
                            self.mavlink_conn.arducopter_disarm()
                        self.mavlink_conn.mav.rc_channels_override_send(0, 0, *self.override)
                        self.last_rc_override_send = time.time()

                if time.time()-self.last_heartbeat_recv > 3.0:
                    self.deactivate_controls()
                else:
                    self.activate_controls()
            except:
                pass
        else:
            self.deactivate_controls()



class TestStandMain:
    def __init__(self):
        self.configFilePath = os.path.expanduser('~/.test_stand_config')
        self.config = {'devices':[]}
        if os.path.isfile(self.configFilePath):
            self.readConfigFile()
        else:
            self.writeConfigFile()

        self.devicesPath = '/dev/serial/by-path'
        self.devicesConnected = set(glob.glob(os.path.join(self.devicesPath,'*')))
        self.latestDeviceConnected = None
        self.deviceConnectAfterId = None

        self.plugMonitor = FileAlterationMonitor()
        self.plugHandler = FileSystemEventHandler()
        self.plugHandler.on_modified = lambda event: self.devicesChanged()
        self.plugMonitor.schedule(self.plugHandler, '/dev')
        self.plugMonitor.start()

        self.win = tk.Tk()
        self.standControlsFrame = tk.Frame(padx=25, pady=25,)
        self.connectLabel = tk.Label(self.standControlsFrame, text='To add a test stand, connect or re-connect its USB cable.', font=("Helvetica", 10, "bold"), wraplength=100)
        self.connectLabel.pack()
        self.nameEntry = tk.Entry(self.standControlsFrame, width=11)
        self.nameEntry.bind('<Return>', lambda event: self.addDevice())
        self.addDevBtn = tk.Button(self.standControlsFrame, text='Add Device', command=self.addDevice)
        self.standControlsFrame.pack(side=tk.RIGHT)

        self.standControls = {}
        for deviceConfig in self.config['devices']:
            name = deviceConfig['name']
            devicePath = deviceConfig['path']
            self.standControls[devicePath] = StandControlFrame(self.win, name, devicePath, devicePath in self.devicesConnected)
            self.standControls[devicePath].pack(side=tk.LEFT)
            self.standControls[devicePath].removeBtnPress = self.removeDevice

        self.win.mainloop()

    def addDevice(self):
        name = self.nameEntry.get()
        if self.latestDeviceConnected is None or len(name) == 0:
            return
        self._addDevice(name, self.latestDeviceConnected)
        self.cancelAddDevice()

    def _addDevice(self,name,devicePath):
        if name in [dev['name'] for dev in self.config['devices']] or devicePath in [dev['path'] for dev in self.config['devices']]:
            return

        self.config['devices'].append({'name': name, 'path':devicePath})
        self.writeConfigFile()
        self.standControls[devicePath] = StandControlFrame(self.win, name, devicePath, True)
        self.standControls[devicePath].pack(side=tk.LEFT)
        self.standControls[devicePath].removeBtnPress = self.removeDevice

    def removeDevice(self, devicePath):
        if devicePath not in [dev['path'] for dev in self.config['devices']]:
            return

        idx = [dev['path'] for dev in self.config['devices']].index(devicePath)

        del self.config['devices'][idx]
        self.writeConfigFile()

        self.standControls[devicePath].pack_forget()
        self.standControls[devicePath].destroy()
        del self.standControls[devicePath]

    def cancelAddDevice(self):
        if self.deviceConnectAfterId is not None:
            self.win.after_cancel(self.deviceConnectAfterId)

        self.deviceConnectAfterId = None
        self.connectLabel.config(text='To add a test stand, connect a USB cable.')
        self.nameEntry.delete(0, tk.END)
        self.nameEntry.pack_forget()
        self.addDevBtn.pack_forget()
        self.nameEntry.config(state=tk.DISABLED)
        self.addDevBtn.config(state=tk.DISABLED)

    def devicesAdded(self, addedDevices):
        if len(addedDevices) != 1:
            return

        devicePath = addedDevices.pop()
        if devicePath in [dev['path'] for dev in self.config['devices']]:
            self.standControls[devicePath].connect()
        else:
            print "New device detected: %s" % (devicePath,)
            if self.deviceConnectAfterId is not None:
                self.cancelAddDevice()
            else:
                self.latestDeviceConnected = devicePath
                self.connectLabel.config(text='Enter a name for the new test stand.')
                self.nameEntry.pack()
                self.addDevBtn.pack()
                self.nameEntry.config(state=tk.NORMAL)
                self.addDevBtn.config(state=tk.NORMAL)
                self.deviceConnectAfterId = self.win.after(15000, self.cancelAddDevice)

    def devicesRemoved(self, removedDevices):
        if self.latestDeviceConnected in removedDevices:
            self.cancelAddDevice()

        for devicePath in removedDevices:
            if devicePath in [dev['path'] for dev in self.config['devices']]:
                self.standControls[devicePath].disconnect()

    def devicesChanged(self):
        nextDevicesConnected = set(glob.glob(os.path.join(self.devicesPath,'*')))
        removedDevices = self.devicesConnected-nextDevicesConnected
        addedDevices = nextDevicesConnected-self.devicesConnected

        if removedDevices:
            self.devicesRemoved(removedDevices)

        if addedDevices:
            self.devicesAdded(addedDevices)

        self.devicesConnected = nextDevicesConnected

    def writeConfigFile(self):
        with open(self.configFilePath, 'wb') as f:
            json.dump(self.config, f)

    def readConfigFile(self):
        with open(self.configFilePath, 'rb') as f:
            self.config = json.load(f)


if __name__ == "__main__":
    main = TestStandMain()
