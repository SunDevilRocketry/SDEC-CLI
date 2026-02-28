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
from SDECv2.Parser import Parser, create_configs

class Cli(cmd.Cmd):
    intro = "SDECv2 CLI"
    prompt = ">> "
    serial_connection = SerialObj()
    rev2_controller = create_controllers.flight_computer_rev2_controller()
    base_controller = BaseController()
    appa_parser = Parser(
        preset_config=create_configs.appa_preset_config(),
        preset_data=None
    )
    #use sensro sentry for sensor stuff


    def __init__(self):
        super().__init__()
        
        
    def do_sensor_dump(self, line):
        params = shlex.split(line)
        
    def do_sensor_poll(self, line):
        params = shlex.split(line)
        
    def do_flash_extract(self, line):
        """
            Extracts all flash data from the flight computer and optionally stores the preset and data to files
        Usage:
            flash-extract [store_preset] [store_data]
        Arguments:
            store_preset True or False, flag to store the preset to a file (default: False)
            store_data True or False, flag to store the flash data to a file (default: False)
        """
        params = shlex.split(line)
        if len(params) != 2:
            print("Usage: flash-extract [store_preset] [store_data]")
            return
        stored_preset = params[0]
        stored_data = params[1]
        flash_data = self.appa_parser.flash_extract(self.serial_connection, store_preset = stored_preset, store_data = stored_data)
        
    def do_upload_preset(self, line):
        """
        Uploads a preset to the flight computer from a file
        Usage:
            upload-preset <path>
        Arguments:
            path Optional Path to the preset file to upload
        """
        params = shlex.split(line)
        if len(params) == 1:
            the_path = params[0]
        elif len(params) == 0:
            the_path = "a_input/to_upload_preset.json"
        else:
            print("Usage: upload-preset <path>")
            return
        parser = Parser.upload_preset(self.serial_connection, path = the_path)
        
    def do_download_preset(self, line):
        """
        Downloads the current preset from the flight computer and stores it to a file
        Usage:
            download-preset [path]
        Arguments:
            path Optional Path to store the downloaded preset file
        """
        params = shlex.split(line)
        if len(params) == 1:
            the_path = params[0]
        elif len(params) == 0:
            the_path = "a_output/downloaded_preset.json"
        else:
            print("Usage: download-preset [path]")
            return
        self.appa_parser.download_preset(self.serial_connection, path = the_path)
        
    def do_verify_preset(self, line):
        """
        Verifies the current preset on the flight computer against a preset file
        Usage:
            verify-preset
        """
        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: verify-preset")
            return
        downloaded_parser = Parser.from_file(path="a_output/downloaded_preset.json")
        verify_result = downloaded_parser.verify_preset(self.serial_connection)
        print(f"{"Valid Preset" if verify_result else "Invalid Preset"}")
        
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