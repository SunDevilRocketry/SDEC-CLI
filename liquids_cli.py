# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025 Sun Devil Rocketry

"""
Modular Liquid CLI

Command lind interface that only interacts with the Liquid Avionics. 
Built intently for simplicity and prototyping with liquids-ported SDECV2.
This version should be only used for testing and should not be released and distributed
-- Nick
"""

import argparse
import cmd
import shlex

try:
    import readline
except ImportError:
    import pyreadline3 as readline

from SDECv2.BaseController import create_controllers
from SDECv2.Sensor import SensorSentry, create_sensors
from SDECv2.SerialController import SerialObj, Status
from SDECv2.EngineController import (
    hotfire_abort, preflight_purge, fill_chill, standby, hotfire,
    stop_hotfire, stop_purge, lox_purge, manual_mode,
    get_state, telemetry_request, flash_extract,
    ENGINE_STATE_NAMES,
)

class Cli(cmd.Cmd):
    intro = "SDECv2 Liquids CLI"
    prompt = "Liquids >> "

    # Global objects 
    serial_connection = SerialObj()
    controller = create_controllers.engine_controller_rev5_controller()
    sensor_sentry = SensorSentry(create_sensors.engine_controller_rev5_sensors())

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

        print("NOTE: Currently unsupported by Engine Controller Rev 5 Firmware")

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
            Extracts all flash data from the engine controller and optionally stores it to a CSV file
        Usage:
            flash_extract [--store-data] [--output <path>]
        Arguments:
            --store-data  Optional flag to store flash data to a CSV file (default: False)
            --output      Optional path for CSV output (default: a_output/engine_ctrl_rev5_flash_data.csv)
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        arg_parser = argparse.ArgumentParser(prog="flash_extract", add_help=False)
        arg_parser.add_argument("--store-data", action="store_true", help="Store flash data to a CSV file")
        arg_parser.add_argument("--output", type=str, default="SDECv2/a_output/engine_ctrl_rev5_flash_data.csv",
                                help="Output path for CSV file")

        try:
            args = arg_parser.parse_args(shlex.split(line))
        except SystemExit:
            print("Usage: flash_extract [--store-data] [--output <path>]")
            return

        frames = flash_extract(
            self.serial_connection,
            store_data=args.store_data,
            output_path=args.output,
        )
        print(f"{len(frames)} frames extracted")

    def do_get_state(self, line):
        """
        Query the current engine state
        Usage:
            get_state
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: get_state")
            return

        state = get_state(self.serial_connection)
        if state is None:
            print("Error: No response from engine controller")
        else:
            print(f"Engine State: {ENGINE_STATE_NAMES[state]}")

    def do_abort(self, line):
        """
        Send abort command to the engine controller
        Usage:
            abort
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: abort")
            return

        if hotfire_abort(self.serial_connection):
            print("Abort acknowledged")
        else:
            print("Error: Abort not acknowledged")

    def do_preflight_purge(self, line):
        """
        Initiate the pre-fire purge sequence
        Usage:
            preflight_purge
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: preflight_purge")
            return

        if preflight_purge(self.serial_connection):
            print("Preflight purge acknowledged")
        else:
            print("Error: Preflight purge not acknowledged")

    def do_fill_chill(self, line):
        """
        Initiate the fill-and-chill sequence
        Usage:
            fill_chill
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: fill_chill")
            return

        if fill_chill(self.serial_connection):
            print("Fill and chill acknowledged")
        else:
            print("Error: Fill and chill not acknowledged")

    def do_standby(self, line):
        """
        Transition the engine to standby state
        Usage:
            standby
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: standby")
            return

        if standby(self.serial_connection):
            print("Standby acknowledged")
        else:
            print("Error: Standby not acknowledged")

    def do_hotfire(self, line):
        """
        Initiate ignition
        Usage:
            hotfire
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: hotfire")
            return

        if hotfire(self.serial_connection):
            print("Hotfire acknowledged")
        else:
            print("Error: Hotfire not acknowledged")

    def do_stop_hotfire(self, line):
        """
        Terminate the engine burn
        Usage:
            stop_hotfire
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: stop_hotfire")
            return

        if stop_hotfire(self.serial_connection):
            print("Stop hotfire acknowledged")
        else:
            print("Error: Stop hotfire not acknowledged")

    def do_stop_purge(self, line):
        """
        Stop the post-fire purge
        Usage:
            stop_purge
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: stop_purge")
            return

        if stop_purge(self.serial_connection):
            print("Stop purge acknowledged")
        else:
            print("Error: Stop purge not acknowledged")

    def do_lox_purge(self, line):
        """
        Initiate the LOX tank purge sequence
        Usage:
            lox_purge
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: lox_purge")
            return

        if lox_purge(self.serial_connection):
            print("LOX purge acknowledged")
        else:
            print("Error: LOX purge not acknowledged")

    def do_manual(self, line):
        """
        Enter manual valve control mode
        Usage:
            manual
        """

        if self.serial_connection.comport.status is not Status.OPEN:
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: manual")
            return

        if manual_mode(self.serial_connection):
            print("Manual mode acknowledged")
        else:
            print("Error: Manual mode not acknowledged")

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