# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025 Sun Devil Rocketry

import cmd
import shlex

try:
    import readline
except ImportError:
    import pyreadline3 as readline

from serial import SerialException

from SDECv2 import SerialObj
from SDECv2.BaseController import create_controllers, BaseController
from SDECv2.Sensor import SensorSentry

class Cli(cmd.Cmd):
    intro = "SDECv2 CLI"
    prompt = ">> "
    serial_connection = SerialObj()
    rev2_controller = create_controllers.flight_computer_rev2_controller()
    base_controller = BaseController()
    #use sensro sentry for sensor stuff


    def __init__(self):
        super().__init__()
        
        
    def do_sensor_dump(self, line):
        params = shlex.split(line)
        
    def do_sensor_poll(self, line):
        params = shlex.split(line)
        
        
    def do_dashboard_dump(self, line):
        """
        Dumps sensor data
        Usage:
            dashboard-dump 
        """
        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: dashboard-dump")
            return
        
        sensor_dump = SensorSentry.dashboard_dump(self.serial_connection)
        
        for sensor, readout in sensor_dump.items():
            if readout is not None:
                print(f"{sensor.name}: {readout:.2f} {sensor.unit}")
            else:
                print(f"{sensor.name}: 0.0 {sensor.unit}")

    def do_list_comports(self, line):
        """
        List the currently available comports
        Usage:
            list-comports
        """
        
        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: list-comports")
            return
        
        print("Available ports:")
        for port in self.serial_connection.available_comports():
            print(f"  {port}")

    def do_connect(self, line):
        """
        Connect to a comport
        Usage: 
            connect <name> [timeout]
        Arguments:
            name Name of the COM port (COM3, /dev/ttyUSB0)
            timeout Optional timeout in seconds
        Notes:
            Default timeout is 1 second
        """

        params = shlex.split(line)
        if len(params) == 1:
            name = params[0]
            timeout = 1
        elif len(params) == 2:
            name, timeout_str = params
            try: 
                timeout = int(timeout_str) 
                if timeout <= 0: raise ValueError
            except ValueError: 
                print("Timeout must be a positive integer (seconds)")
                return
        else:
            print("Usage: connect <name> [timeout]")
            return 

        self.serial_connection.init_comport(
            name=name.upper(),
            baudrate=921600,
            timeout=timeout
        )

        try:
            if self.serial_connection.open_comport():
                print(f"Successfully opened serial connection on port {name}")
            else:
                print(f"Failed to open serial connection on port {name}")
        except Exception as e:
            print(f"Failed to open serial connection on port {name}: {e}")

    def do_disconnect(self, line):
        """
        Disconnect the current comport
        Usage:
            disconnect
        """

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: disconnect")
            return
        
        try:
            self.serial_connection.comport
        except AttributeError:
            print(f"No initialized comport")
            return

        try:
            if self.serial_connection.close_comport():
                print(f"Successfully closed serial connection on port {self.serial_connection.comport.name}")
            else:
                print(f"Failed to close serial connection on port {self.serial_connection.comport.name}")
        except Exception as e:
            print(f"Failed to close serial connection: {e}")

    def do_quit(self, line):
        """
        Quit the CLI
        Usage:
            quit
        """
        try:
            self.serial_connection.close_comport()
        except:
            pass

        return True
    
    def do_q(self, line):
        """
        Quit the CLI
        Usage:
            q
        """

        return self.do_quit(line)
    
if __name__=="__main__":
    Cli().cmdloop()