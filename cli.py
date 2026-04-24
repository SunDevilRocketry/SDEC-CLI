# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025 Sun Devil Rocketry

import argparse
import shlex
import sys
import time

from prompt_toolkit import PromptSession
from prompt_toolkit.completion import NestedCompleter
from prompt_toolkit.history import InMemoryHistory
from serial import SerialException

from SDECv2.BaseController import create_controllers, BaseController
from SDECv2.Sensor import SensorSentry, create_sensors
from SDECv2.Parser import Parser, create_configs, Telemetry
from SDECv2.SerialController import SerialObj, Status
from SDECv2.Exceptions import InvalidDataError, MissingDataError, ParserError, SDECError

from hw_fw_pairing import HW_FW_PAIRS

COMMANDS = [
    "sensor_dump",
    "sensor_poll",
    "flash",
    "preset",
    "lora",
    "dashboard_dump",
    "list_comports",
    "connect",
    "disconnect",
    "help",
    "quit",
    "q"
]

class Cli:
    intro = "SDECv2 CLI"
    prompt = ">> "

    # Global objects 
    serial_connection = SerialObj()
    hardware_code = b"\x00"
    firmware_code = b"\x00"
    appa_parser = Parser(
        preset_config=create_configs.appa_preset_config(),
        preset_data=None
    )
    sensor_sentry = SensorSentry(create_sensors.flight_computer_rev2_sensors())

    def __init__(self):
        self.session = PromptSession(
            history=InMemoryHistory(),
            completer=NestedCompleter.from_nested_dict({
                "help":          {cmd: None for cmd in COMMANDS},
                "sensor_dump":   None,
                "sensor_poll":   {"--timeout": None, "--count": None},
                "flash":         {"extract": {"--store-preset": None, "--store-data": None, "--no-store-preset": None, "--no-store-data": None}},
                "preset":        {"upload": None, "download": None, "verify": None},
                "lora":          {"preset": {"upload": None, "download": None}},
                "dashboard_dump": None,
                "list_comports": None,
                "connect":       {port for port in self.serial_connection.available_comports()},
                "disconnect":    None,
                "quit":          None,
                "q":             None,
            }),
        )

    def cmdloop(self):
        print(self.intro)
        while True:
            try:
                line = self.session.prompt(self.prompt).strip()
            except KeyboardInterrupt:
                print()
                continue
            except EOFError:
                break

            if not line: continue

            params = shlex.split(line)
            cmd, arg = params[0], params[1:]
            handler = getattr(self, f"do_{cmd}", None)

            if handler is None:
                print(f"Unkown command: {cmd!r} (type 'help' for a list)")
                continue

            result = handler(shlex.join(arg))
            if result: break

    def do_help(self, line):
        """
            Lists available commands or show help for a sepecifc command
        Usage:
            help <command>
        Arguments:
            command The name of the command to show help for
        """  
        params = shlex.split(line)
        if params:
            handler = getattr(self, f"do_{params[0]}", None)
            if handler and handler.__doc__:
                print(handler.__doc__)
            else:
                print(f"No help for {params[0]!r}")
        else:
            print("Available commands:")
            for name in COMMANDS:
                 handler = getattr(self, f"do_{name}", None)
                 doc = (handler.__doc__ or "").strip().splitlines()[0] if handler else ""
                 print(f"   {name:<20} {doc}")
        
    def do_sensor_dump(self, line):
        """
            Prints one frame of all sensor data
        Usage:
            sensor_dump
        """

        if not self.serial_connection.serialObj.is_open: 
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

        self.serial_connection.reset_input_buffer()
         
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

        print("NOTE: Currently unsupported by v2.6.0 of Flight Computer Firmware")

        if not self.serial_connection.serialObj.is_open: 
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

        self.serial_connection.reset_input_buffer()

    def do_flash(self, line):
        """
            Commands for controlling flash and performing flash operations.
        Usage:
            flash extract [--store_preset <path>] [--store_data <path>] [--no-store-preset] [--no-store-data]
        Arguments:
            extract:
                --store-preset Optional flag to specify path to store preset to
                --store-data Optional flag to specify path to store data to
                --no-store-preset Optional flag to not store the preset to a file (default: True)
                --no-store-data Optional flag to not store the flash data to a file (default: True)
        """
        arg_parser = argparse.ArgumentParser(prog="flash", add_help=False)
        sub_parser = arg_parser.add_subparsers(dest="subcommand", required=True)

        extract_parser = sub_parser.add_parser("extract")
        extract_parser.add_argument("--store-preset", type=str, help="Path to store preset to")
        extract_parser.add_argument("--store-data", type=str, help="Path to store data to")
        extract_parser.add_argument("--no-store-preset", action="store_true", help="Disable storing preset to a file")
        extract_parser.add_argument("--no-store-data", action="store_true", help="Disable storing flash data to a file")

        try: 
            args = arg_parser.parse_args(shlex.split(line))
        except SystemExit:
            print("Usage:\n" + 
                  "  flash_extract [--store-preset] [--store-data]"
            )
            return
        
        if not self.serial_connection.serialObj.is_open: 
            print("Error: No serial connection")
            return
        
        match args.subcommand:
            case "extract":
                preset_path = "a_output/extracted_preset.json"
                data_path = "a_output/extracted_data.csv"
                
                if args.store_preset: preset_path = args.store_preset
                if args.store_data: data_path = args.store_data

                if args.no_store_preset: preset_path = ""
                if args.no_store_data: data_path = ""

                try:
                    self.appa_parser.flash_extract(
                    self.serial_connection, 
                    preset_path=preset_path, 
                    data_path=data_path
                    )
                except SDECError as e:
                    print(f"SDEC error: {e}")

    def do_preset(self, line):
        """
            Commands for managing presets on the Flight Computer
        Usage:
            preset upload [path]
            preset download [path]
            preset verify
        Arguments:
            upload:
                path Optional Path to the preset file to upload
            download:
                path Option Path to store the downloaded preset file
        """
        
        arg_parser = argparse.ArgumentParser(prog="preset", add_help=False)
        sub_parser = arg_parser.add_subparsers(dest="subcommand", required=True)

        upload_parser = sub_parser.add_parser("upload")
        upload_parser.add_argument("path", 
                                   nargs="?", 
                                   default="a_input/to_upload_preset.json", 
                                   help="Path to the preset file")
        
        download_parser = sub_parser.add_parser("download")
        download_parser.add_argument("path",
                                     nargs="?",
                                     default="a_output/downloaded_preset.json",
                                     help="Path to store the downloaded preset")
        
        verify_parser = sub_parser.add_parser("verify")

        try:
            args = arg_parser.parse_args(shlex.split(line))
        except SystemExit:
            print("Usage:\n" +
                    "  preset upload [path]\n" +
                    "  preset download [path]\n" +
                    "  preset verify\n"
            )
            return
        
        if not self.serial_connection.serialObj.is_open: 
            print("Error: No serial connection")
            return

        match args.subcommand:
            case "upload":
                try:
                    Parser.upload_preset(self.serial_connection, path=args.path)
                except SDECError as e:
                    print(f"SDEC error: {e}")

            case "download":
                self.appa_parser.download_preset(self.serial_connection, path=args.path)

            case "verify":
                try:
                    downloaded_parser = Parser.from_file(path="a_output/downloaded_preset.json")
                except SDECError as e:
                    print(f"SDEC error: {e}")
                    return
                except FileNotFoundError as e:
                    print(f"File not found: {e}")
                    return
                verify_result = downloaded_parser.verify_preset(self.serial_connection)
                
                print(f"{"Valid Preset" if verify_result else "Invalid Preset"}")

        self.serial_connection.reset_input_buffer()

    def do_dashboard_dump(self, line):
        """
        Dumps sensor data
        Usage:
            dashboard_dump 
        """

        if not self.serial_connection.serialObj.is_open: 
            print("Error: No serial connection")
            return

        params = shlex.split(line)
        if len(params) != 0:
            print("Usage: dashboard_dump")
            return
        
        dashboard_obj = Telemetry()
        dashboard_obj.dashboard_dump(self.serial_connection)
        dashboard_obj.get_latest_dashboard_dump()
        dashboard_dump = dashboard_obj.get_latest_dashboard_dump()

        if dashboard_dump is None:
            print("Dashboard dump not found.")
            return

        for sensor, readout in dashboard_dump.items():
            if readout is not None:
                print(f"{sensor}: {readout:.2f}")
            else:
                print(f"{sensor}: 0.0")

        self.serial_connection.reset_input_buffer()

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
            name=name,
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
            return

        try:
            self.serial_connection.connect()

            if self.serial_connection.target is None:
                raise IndexError()

            self.hardware_code = self.serial_connection.target.controller.id
            self.firmware_code = self.serial_connection.target.firmware.id

            for pair in HW_FW_PAIRS:
                if self.hardware_code == pair.controller.id and self.firmware_code == pair.firmware.id:
                    self.serial_connection.target = pair
                    print(f"Connected to hardware firmware pair {self.hardware_code} {pair.controller.name}>{self.firmware_code} {pair.firmware.name}")
                    break
            else:
                print(f"Unable to connect to unknown hardware firmware pair {self.hardware_code}>{self.firmware_code}")
                return
        except IndexError as e:
            print("No hardware firmware pair received, closing connection")

            if self.serial_connection.close_comport():
                print(f"Successfully closed serial connection on port {self.serial_connection.comport.name}")
            else:
                print(f"Failed to close serial connection on port {self.serial_connection.comport.name}")
        

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
        
        self.serial_connection.reset_input_buffer()

        try:
            if self.serial_connection.close_comport():
                print(f"Successfully closed serial connection on port {self.serial_connection.comport.name}")
            else:
                print(f"Failed to close serial connection on port {self.serial_connection.comport.name}")
        except Exception as e:
            print(f"Failed to close serial connection: {e}")

    def do_lora(self, line):
        """
        Commands for using and configuring LoRA
        Usage:
            lora preset upload [path]
            lora preset download [path]
        Arguments:
            preset upload:
                path Optional Path to the preset file to upload
            preset download:
                path Option Path to store the downloaded preset file        
        """

        arg_parser = argparse.ArgumentParser(prog="lora", add_help=False)
        sub_parser = arg_parser.add_subparsers(dest="command", required=True)
        preset_parser = sub_parser.add_parser("preset")

        preset_parser = preset_parser.add_subparsers(dest="subcommand", required=True)
        
        upload_parser = preset_parser.add_parser("upload")
        upload_parser.add_argument("path", 
                                   nargs="?", 
                                   default="a_input/to_upload_lora_preset.json", 
                                   help="Path to the LoRA preset file")
        
        download_parser = preset_parser.add_parser("download")
        download_parser.add_argument("path",
                                     nargs="?",
                                     default="a_output/downloaded_lora_preset.json",
                                     help="Path to store the LoRA downloaded preset")

        try:
            args = arg_parser.parse_args(shlex.split(line))
        except SystemExit:
            print("Usage:\n" +
                    "  lora preset upload [path]\n" +
                    "  lora preset download [path]\n"
            )
            return
        
        if not self.serial_connection.serialObj.is_open: 
            print("Error: No serial connection")
            return

        match args.command:
            case "preset":
                match args.subcommand:
                    case "upload":
                        self.appa_parser.upload_lora_preset(self.serial_connection, path=args.path)

                    case "download":
                        self.appa_parser.download_lora_preset(self.serial_connection, path=args.path)

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