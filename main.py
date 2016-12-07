import Tkinter as tk
import glob
import os
import time
from watchdog.observers import Observer as FileAlterationMonitor
from watchdog.events import FileSystemEventHandler, PatternMatchingEventHandler
import json
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega

def foreach_children(widget,func):
        for child in widget.winfo_children():
            foreach_children(child,func)
            func(child)

def configure_normal(widget):
    try:
        widget.configure(state=tk.NORMAL)
    except tk.TclError:
        pass

def configure_disabled(widget):
    try:
        widget.configure(state=tk.DISABLED)
    except tk.TclError:
        pass

class StandControlFrame(tk.Frame):
    def __init__(self, parent, name, devicePath, connected):
        tk.Frame.__init__(self, parent, padx=25, pady=25)

        self.name = name
        self.devicePath = devicePath
        self.deviceBaud = 57600

        tk.Label(self, text=name, font=("Helvetica", 16, "bold")).grid(row=0, column=0, padx=6, pady=6)


        self.headerFrame = tk.Frame(self)
        self.armBtn = tk.Button(self.headerFrame, text='Arm', bg='lightgreen', command=self._armBtnPress)
        self.startBtn = tk.Button(self.headerFrame, text='Start', bg='lightblue', command=self._startBtnPress)
        self.disarmBtn = tk.Button(self.headerFrame, text='Disarm', bg='pink', command=self._disarmBtnPress)
        self.magCalBtn = tk.Button(self.headerFrame, text='Mag Cal', command=self._magCalBtnPress)
        self.throttleSliderLabel = tk.Label(self.headerFrame, text='Thr')
        self.throttleSlider = tk.Scale(self.headerFrame, from_=1000, to=2000, orient=tk.HORIZONTAL, tickinterval=1000., length=200)
        self.removeBtn = tk.Button(self.headerFrame, text='Remove', command=self._removeBtnPress, padx=0, pady=0)
        self.armBtn.pack(side=tk.LEFT)
        self.startBtn.pack(side=tk.LEFT)
        self.disarmBtn.pack(side=tk.LEFT)
        self.magCalBtn.pack(side=tk.LEFT)
        self.throttleSliderLabel.pack(side=tk.LEFT)
        self.throttleSlider.pack(side=tk.LEFT)
        self.removeBtn.pack(side=tk.LEFT)

        self.headerFrame.grid(row=0, column=1, sticky=tk.W, padx=6, pady=6)

        self.telemetryData = [
            {'name':'Arm', 'value':False},
            {'name':'Thr', 'value':0},
            {'name':'Volt', 'value':0.},
            {'name':'Curr', 'value':0.},
            {'name':'Pow', 'value':0.},
            {'name':'Time', 'value':time.strftime("%H:%M:%S", time.gmtime(0))}
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

        self.telemFrame.grid(row=1, column=0, padx=6, pady=6)

        self.infoLabel = tk.Text(self, bg='black', fg='white', state=tk.DISABLED, width=64, height=8)

        self.infoLabel.grid(row=1, column=1, padx=6, pady=6)

        self.INIT = 0
        self.DISCONNECTED = 1
        self.CONNECTED_DISARMED = 2
        self.CONNECTED_ARMED = 3
        self.CONNECTED_RUNNING = 4
        self.CONNECTED_DISARMING = 5

        self.state = self.INIT

        self.throttleSlider.set(1500)

        if connected:
            self.connect()
        else:
            self.disconnect()

        self.after(10, self.tick)

    def appendInfoText(self, text):
        self.infoLabel.config(state=tk.NORMAL)
        self.infoLabel.insert(tk.END, text+'\n')
        self.infoLabel.config(state=tk.DISABLED)
        self.infoLabel.see(tk.END)

    def setState(self, state):
        assert state in (self.DISCONNECTED, self.CONNECTED_DISARMED, self.CONNECTED_ARMED, self.CONNECTED_RUNNING, self.CONNECTED_DISARMING)
        if state != self.state:
            self.state = state
            self.updateButtonDisableState()
            self.lastStateChangeTime = time.time()
            if state == self.DISCONNECTED:
                self.appendInfoText("State: DISCONNECTED")
            elif state == self.CONNECTED_DISARMED:
                self.appendInfoText("State: CONNECTED_DISARMED")
            elif state == self.CONNECTED_ARMED:
                self.appendInfoText("State: CONNECTED_ARMED")
            elif state == self.CONNECTED_RUNNING:
                self.appendInfoText("State: CONNECTED_RUNNING")
            elif state == self.CONNECTED_DISARMING:
                self.appendInfoText("State: CONNECTED_DISARMING")

    def updateButtonDisableState(self):
        if self.state == self.DISCONNECTED:
            foreach_children(self, configure_disabled)
            self.removeBtn.configure(state=tk.NORMAL)
        elif self.state == self.CONNECTED_DISARMED:
            foreach_children(self, configure_normal)
            self.startBtn.configure(state=tk.DISABLED)
            self.disarmBtn.configure(state=tk.DISABLED)
        elif self.state == self.CONNECTED_DISARMING:
            foreach_children(self, configure_normal)
            self.armBtn.configure(state=tk.DISABLED)
            self.startBtn.configure(state=tk.DISABLED)
            self.disarmBtn.configure(state=tk.DISABLED)
            self.magCalBtn.configure(state=tk.DISABLED)
        elif self.state == self.CONNECTED_ARMED:
            foreach_children(self, configure_normal)
            self.armBtn.configure(state=tk.DISABLED)
            self.magCalBtn.configure(state=tk.DISABLED)
        elif self.state == self.CONNECTED_RUNNING:
            foreach_children(self, configure_normal)
            self.armBtn.configure(state=tk.DISABLED)
            self.startBtn.configure(state=tk.DISABLED)
            self.magCalBtn.configure(state=tk.DISABLED)
        self.throttleSliderLabel.configure(state=tk.NORMAL)
        self.throttleSlider.configure(state=tk.NORMAL)

    def updateTelemLabels(self):
        for i in range(len(self.telemetryData)):
            self.telemetryLabels[i]['name'].config(text=self.telemetryData[i]['name'])
            self.telemetryLabels[i]['value'].config(text=self.telemetryData[i]['value'])

    def _armBtnPress(self):
        if self.state == self.CONNECTED_DISARMED:
            self.mavlink_conn.arducopter_arm()

    def _startBtnPress(self):
        if self.state != self.DISCONNECTED:
            self.setState(self.CONNECTED_RUNNING)

    def _disarmBtnPress(self):
        if self.state != self.DISCONNECTED:
            self.setState(self.CONNECTED_DISARMING)

        if self.state == self.CONNECTED_DISARMING:
            self.mavlink_conn.mav.rc_channels_override_send(0, 0, *self.getRCOverride())
            self.mavlink_conn.arducopter_disarm()

    def _magCalBtnPress(self):
        if self.state == self.CONNECTED_DISARMED:
            self.mavlink_conn.mav.command_long_send(0, 0, 42424, 0, 0, 1, 1, 5, 1, 0, 0)

    def _removeBtnPress(self):
        self.removeBtnPress(self.devicePath)

    def removeBtnPress(self, devicePath):
        pass

    def getRCOverride(self):
        throttle = 1000
        if self.state == self.CONNECTED_RUNNING:
            rampPct = min(max((time.time()-self.lastStateChangeTime)/10., 0.), 1.) # 10 second ramp
            throttle = 1000.+rampPct*(self.throttleSlider.get()-1000.)

        return [1500,1500,throttle,1500,1000,1000,1000,1000]

    def connect(self):
        self.appendInfoText("Plugged in")
        self.mavlink_conn = mavutil.mavserial(self.devicePath, baud=self.deviceBaud, autoreconnect=True)
        self.mavlink_conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.mavlink_conn.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
        self.last_heartbeat_send = time.time()
        self.last_rc_override_send = time.time()
        self.last_heartbeat_recv = 0
        self.setState(self.DISCONNECTED)

    def disconnect(self):
        self.appendInfoText("Unplugged")
        self.mavlink_conn = None
        self.setState(self.DISCONNECTED)

    def process_mavlink_message(self, msg):
        if msg.get_type() == 'HEARTBEAT':
            self.last_heartbeat_recv = time.time()
            armed = bool((msg.base_mode & ardupilotmega.MAV_MODE_FLAG_DECODE_POSITION_SAFETY) == ardupilotmega.MAV_MODE_FLAG_DECODE_POSITION_SAFETY)

            if armed and self.state == self.CONNECTED_DISARMED:
                self.setState(self.CONNECTED_ARMED)

            if not armed and self.state not in (self.DISCONNECTED, self.CONNECTED_DISARMED):
                self.setState(self.CONNECTED_DISARMED)

            self.telemetryData[0]['value'] = armed
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

        if msg.get_type() == 'STATUSTEXT':
            self.appendInfoText(msg.text)

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
                        self.mavlink_conn.set_mode(0)

                    if time.time()-self.last_rc_override_send >= 0.1:
                        if self.state == self.CONNECTED_DISARMING:
                            self.mavlink_conn.arducopter_disarm()
                        override = self.getRCOverride()
                        self.mavlink_conn.mav.rc_channels_override_send(0, 0, *override)
                        self.last_rc_override_send = time.time()
            except:
                pass

        haveHeartbeat = (self.mavlink_conn is not None) and (time.time()-self.last_heartbeat_recv < 5.0)

        if haveHeartbeat and self.state == self.DISCONNECTED:
            self.setState(self.CONNECTED_DISARMED)

        if not haveHeartbeat:
            self.setState(self.DISCONNECTED)

        if self.state == self.CONNECTED_RUNNING:
            secsElapsed = time.time()-self.lastStateChangeTime
            if False:#secsElapsed > 60*60*8:
                self.appendInfoText("Test complete")
                self.setState(self.CONNECTED_DISARMING)
            else:
                self.telemetryData[5]['value'] = str(time.strftime("%H:%M:%S", time.gmtime(secsElapsed)))




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
        addDeviceFrame = tk.Frame(padx=25, pady=25,)
        self.connectLabel = tk.Label(addDeviceFrame, text='To add a test stand, connect or re-connect its USB cable.', font=("Helvetica", 10, "bold"), wraplength=200)
        self.connectLabel.pack()
        self.nameEntry = tk.Entry(addDeviceFrame, width=11)
        self.nameEntry.bind('<Return>', lambda event: self.addDevice())
        self.addDevBtn = tk.Button(addDeviceFrame, text='Add Device', command=self.addDevice)
        addDeviceFrame.pack(side=tk.BOTTOM)

        self.standControls = {}
        for deviceConfig in self.config['devices']:
            name = deviceConfig['name']
            devicePath = deviceConfig['path']
            self.standControls[devicePath] = StandControlFrame(self.win, name, devicePath, devicePath in self.devicesConnected)
            self.standControls[devicePath].pack(side=tk.TOP)
            self.standControls[devicePath].removeBtnPress = self.removeDevice

        self.win.update()
        self.win.minsize(self.win.winfo_width(), self.win.winfo_height())
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
