/*!
 * @file        AlnitakEmu.cs
 *
 * @brief       Command line Alnitak Flat Panel emulator.
 *
 * This auxiliary helper allow Alnitak commands to be sent/received over the
 * specified serial port with input/output data printed on the terminal.
 *
 * @author      Frederick Steiling
 *
 * @license     This project is released under GPLv3.
 */

using System;
using System.IO.Ports;

class AlnitakTester {

    enum ProgramArguments {
        SerialPort,
        NumArguments
    };

    static void PrintUsage() {
        Console.WriteLine(
             "AlnitakTester [COMPort]: A serial program to " +
                "test an Alnitak Flatbox Interface\n" +
             "\tCOM Port: The COM port through which the commands " +
                "are sent & received\n"
        );
    }

    static void PrintInstructions() {
        Console.WriteLine(
                "Select a command to send:\n" +
                "\t\"P\": Ping (ID)\n" +
                "\t\"O\": Open Cover\n" +
                "\t\"C\": Close Cover\n" +
                "\t\"L\": Turn Light On\n" +
                "\t\"D\": Turn Light Off\n" +
                "\t\"B\": Set Brightness\n" +
                "\t\"J\": Get Brightness\n" +
                "\t\"S\": Get Device Status\n" +
                "\t\"V\": Get Firmware Version\n\n" +
                "\tor \"Q\" to quit this program\n"
            );
    }

    enum Command : int {
        Ping,
        OpenCover,
        CloseCover,
        TurnLightOn,
        TurnLightOff,
        SetBrightness,
        GetBrightness,
        GetStatus,
        GetFirmware,
        Quit,
        NumCommands
    };

    public struct CommandSet {
        public char   inputChar;
        public string command;

        public CommandSet(char i, string c) {
            this.inputChar = i;
            this.command = c;
        }
    };

    static int QueryBrightness() {
        int brightness = -1;

        while (true) {
            Console.Write("Enter a brightness value [0-255]: ");

            string s = Console.ReadLine();

            try {
                brightness = Int32.Parse(s);
            }
            catch (Exception) {
                Console.WriteLine(
                    "\nIll-formed integer. " +
                    "A value between 0 and 255 is required.\n");
                continue;
            }

            if ((brightness >= 0) && (brightness <= 255)) {
                break;
            }
            else {
                Console.WriteLine(
                    "\n" + brightness + " is not within the required range");
                Console.WriteLine(
                    "A value between 0 and 255 is required.\n");
            }
        }

        return brightness;
    }

    static readonly CommandSet [] cmdSet = {
        new CommandSet('P', ">P000\n"),     // Ping
        new CommandSet('O', ">O000\n"),     // OpenCover
        new CommandSet('C', ">C000\n"),     // CloseCover
        new CommandSet('L', ">L000\n"),     // TurnLightOn
        new CommandSet('D', ">D000\n"),     // TurnLightOff
        new CommandSet('B', ">B000\n"),     // SetBrightness
        new CommandSet('J', ">J000\n"),     // GetBrightness
        new CommandSet('S', ">S000\n"),     // GetStatus
        new CommandSet('V', ">V000\n"),     // GetFirmware
        new CommandSet('Q', "")             // Quit
    };

    static Command GetCommand() {
        Command retval = Command.NumCommands;

        while (true) {
            int i = 0;

            PrintInstructions();

            // Get a character:
            Console.WriteLine("Enter a command key...");
            char c = Console.ReadKey().KeyChar;

            // Convert to upper-case, if necessary:
            if ((c >= 'a') && (c <= 'z')) {
                c = (char)(c - ('a' - 'A'));
            }

            Console.WriteLine("");

            // See if the command matches anything we'll accept:
            for (i = 0; i < (int)Command.NumCommands; i++) {
                if (cmdSet[i].inputChar == c) {
                    retval = (Command)i;
                    break;
                }
            }

            if (i < (int)Command.NumCommands) break;

            Console.WriteLine("\n\"" + c + "\" is not a recognized command.");
            Console.WriteLine("Press ENTER to retry...");
            Console.ReadLine();
            Console.Clear();
        }

        return retval;
    }

    static void Wait() {
        Console.WriteLine("Press [ENTER] to continue\n");
        Console.ReadLine();
        Console.Clear();
    }

    static int ErrorExit() {
        Console.WriteLine("Press [ENTER] to exit\n");
        Console.ReadLine();
        return -1;
    }

    static int Main(string [] argv) {
        // Make sure the correct number of arguments were provided:
        if (argv.Length != (int)ProgramArguments.NumArguments) {
            Console.WriteLine("\nIncorrect number of arguments provided.\n");
            PrintUsage();
            return ErrorExit();
        }

        // Now make sure we can get to the serial port:
        string[] portNames = SerialPort.GetPortNames();
        bool havePort = false;

        foreach (string s in portNames) {
            if (String.Compare(
                        s, argv[(int)ProgramArguments.SerialPort]) == 0) {
                havePort = true;
                break;
            }
        }
        if (!havePort) {
            Console.WriteLine("\nCannot find port \"" +
                    argv[(int)ProgramArguments.SerialPort] + "\"");
            Console.WriteLine("Bailing...\n");
            return ErrorExit();
        }

        // Open up the port:
        SerialPort sp = null;
       
        try {
            sp = new SerialPort(argv[(int)ProgramArguments.SerialPort]);
        }
        catch (Exception) {
            Console.WriteLine(
                    "Unable to create object for \"" +
                    argv[(int)ProgramArguments.SerialPort] + "\"");
            Console.WriteLine("Bailing...\n");
            return ErrorExit();
        }

        try {
            sp.BaudRate  = 9600;
            sp.DataBits  = 8;
            sp.StopBits  = StopBits.One;
            sp.Parity    = Parity.None;
            sp.Handshake = Handshake.RequestToSend;

            // Set the read timeout at 1/2 second
            sp.ReadTimeout = 500;
        }
        catch (Exception) {
            Console.WriteLine(
                    "Unable to set properties of \"" +
                    argv[(int)ProgramArguments.SerialPort] + "\"");
            Console.WriteLine("Bailing...\n");
            return ErrorExit();
        }

        try {
            sp.Open();
        }
        catch {
            Console.WriteLine(
                    "Unable to open \"" +
                    argv[(int)ProgramArguments.SerialPort] + "\"");
            Console.WriteLine("Bailing...\n");
            return ErrorExit();
        }

        while (true) {
            // Now we're ready to roll.  Present options to the user, and act 
            // accordingly:
            Command c = GetCommand();

            if (c == Command.Quit) break;

            string command = cmdSet[(int)c].command;

            // Some commands require modifications.  Take care of those here:
            if (c == Command.SetBrightness) {
                // Need an additional query to get the desired brightness:
                int brightness = QueryBrightness();

                // Format the parameter:
                string formattedBrightness = brightness.ToString("D3");

                // Replace the 3-digit parameter with the brightness:
                command = command.Replace("000", formattedBrightness);
            }

            // Send out the command:
            try {
                sp.Write(command);
            }
            catch (Exception) {
                Console.WriteLine("Error sending command.");
                Wait();
                continue;
            }

            string resp = "";

            try {
                // Now look for a response:
                resp = sp.ReadTo("\n");
            }
            catch (Exception) {
                // There was no response received within the specified timeout:
                Console.WriteLine("No response received.");
                Wait();
                continue;
            }

            // Print the end status here:
            Console.WriteLine("\n*** Communication Status ***");
            Console.WriteLine("-Sent: " + 
                    command.Substring(0, (command.Length - 1)));
            Console.WriteLine("-Received: " + resp);
            Console.WriteLine("****************************\n");

            // Wait for the user to see the results, then continue:
            Wait();
        }

        // Cleanup the port:
        sp.Close();

        return 0;
    }
};
