# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025 Sun Devil Rocketry

import argparse
import cmd
import shlex

try:
    import readline
except ImportError:
    import pyreadline3 as readline

from serial import SerialException

from SDECv2.BaseController import create_controllers, BaseController
from SDECv2.Sensor import SensorSentry, create_sensors
from SDECv2.Parser import Parser, create_configs
from SDECv2.SerialController import SerialObj, Status

class Cli(cmd.Cmd):
    intro = "SDECv2 CLI"
    prompt = ">> "

    # Global objects 
    serial_connection = SerialObj()
    controller = create_controllers.flight_computer_rev2_controller()
    appa_parser = Parser(
        preset_config=create_configs.appa_preset_config(),
        preset_data=None
    )
    sensor_sentry = SensorSentry(create_sensors.flight_computer_rev2_sensors())

    def __init__(self):
        super().__init__()
        
    def do_sensor_dump(self, line):
        """
            Prints one frame of all sensor data
        Usage:
            sensor_dump
        """

        if self.serial_connection.comport.status is not Status.OPEN: 
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: sensor_dump")
            return
        
        sensor_dump = self.sensor_sentry.dump(self.serial_connection)
        for sensor, readout in sensor_dump.items():
            if readout:
                print(f"{sensor.name}: {readout:.2f} {sensor.unit}")
            else:
                print(f"{sensor.name}: 0.0 {sensor.unit}")
         
    def do_sensor_poll(self, line):
        """
            Continues printing frames of all sensor data until timeout or count is reached
        Usage:
            sensor_poll <--timeout> <time> | <--count> | <count>
        Arguments:
            timeout Time in seconds for poll to last
            count Integer of how many sensor frames to poll
        Notes:
            Must provide either a timeout or count
        """

        if self.serial_connection.comport.status is not Status.OPEN: 
            print("Error: No serial connection")
            return

        arg_parser = argparse.ArgumentParser(prog="sensor_poll", add_help=False)
        group = arg_parser.add_mutually_exclusive_group(required=True)
        group.add_argument("--timeout", type=int, help="Time in seconds to poll")
        group.add_argument("--count", type=int, help="Number of sensor frames to poll")
        
        try:
            args = arg_parser.parse_args(shlex.split(line))
        except SystemExit:
            print("Usage: sensor_poll <--timeout> <time> | <--count> <count>")
            return

        if args.count is not None:
            for sensor_poll in self.sensor_sentry.poll(self.serial_connection, count=args.count):
                for sensor, readout in sensor_poll.items():
                    if readout:
                        print(f"{sensor.name}: {readout:.2f} {sensor.unit}")
                    else:
                        print(f"{sensor.name}: 0.0 {sensor.unit}")
        elif args.timeout is not None:
            for sensor_poll in self.sensor_sentry.poll(self.serial_connection, timeout=args.timeout):
                for sensor, readout in sensor_poll.items():
                    if readout:
                        print(f"{sensor.name}: {readout:.2f} {sensor.unit}")
                    else:
                        print(f"{sensor.name}: 0.0 {sensor.unit}")
        
    def do_flash_extract(self, line):
        """
            Extracts all flash data from the flight computer and optionally stores the preset and data to files
        Usage:
            flash_extract [store_preset] [store_data]
        Arguments:
            --store-preset Optional flag to store the preset to a file (default: False)
            --store-data Optional flag to store the flash data to a file (default: False)
        """

        if self.serial_connection.comport.status is not Status.OPEN: 
            print("Error: No serial connection")
            return

        arg_parser = argparse.ArgumentParser(prog="flash_extract", add_help=False)
        arg_parser.add_argument("--store-preset", action="store_true", help="Store preset to a file")
        arg_parser.add_argument("--store-data", action="store_true", help="Store flash data to a file")

        try: 
            args = arg_parser.parse_args(shlex.split(line))
        except SystemExit:
            print("Usage: flash_extract [--store-preset] [--store-data]")
            return

        self.appa_parser.flash_extract(
            self.serial_connection, 
            store_preset=args.store_preset, 
            store_data=args.store_data
        )
        
    def do_upload_preset(self, line):
        """
        Uploads a preset to the flight computer from a file
        Usage:
            upload_preset [path]
        Arguments:
            path Optional Path to the preset file to upload
        """

        if self.serial_connection.comport.status is not Status.OPEN: 
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) == 1:
            path = params[0]
        elif len(params) == 0:
            path = "SDECv2/a_input/to_upload_preset.json"
        else:
            print("Usage: upload_preset [path]")
            return
        
        Parser.upload_preset(self.serial_connection, path=path)
        
    def do_download_preset(self, line):
        """
        Downloads the current preset from the flight computer and stores it to a file
        Usage:
            download_preset [path]
        Arguments:
            path Optional Path to store the downloaded preset file
        """

        if self.serial_connection.comport.status is not Status.OPEN: 
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) == 1:
            path = params[0]
        elif len(params) == 0:
            path = "SDECv2/a_output/downloaded_preset.json"
        else:
            print("Usage: download_preset [path]")
            return
        
        self.appa_parser.download_preset(self.serial_connection, path=path)
        
    def do_verify_preset(self, line):
        """
        Verifies the current preset on the flight computer against a preset file
        Usage:
            verify_preset
        """

        if self.serial_connection.comport.status is not Status.OPEN: 
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: verify_preset")
            return
        
        downloaded_parser = Parser.from_file(path="SDECv2/a_output/downloaded_preset.json")
        verify_result = downloaded_parser.verify_preset(self.serial_connection)
        
        print(f"{"Valid Preset" if verify_result else "Invalid Preset"}")
        
    def do_dashboard_dump(self, line):
        """
        Dumps sensor data
        Usage:
            dashboard_dump 
        """

        if self.serial_connection.comport.status is not Status.OPEN: 
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: dashboard_dump")
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
            list_comports
        """
        
        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: list_comports")
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