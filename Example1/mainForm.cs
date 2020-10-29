#region "Meta Data"
/*  
'   File....... mainForm.CS
'   Purpose.... Provide a GUI for mapping EMG and joystick signals to the Bento Arm
'   Author..... Michael Dawson
'   Help....... mrd1@ualberta.ca
'   Started.... 09/12/2009
'   Updated.... 
'   Version.... 1.0.0.0
'
'   xPC Target (aka Simulink Realtime) .NET API Copyright Mathworks (see matlab R2014a documentation for help)

*/
#endregion

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using System.IO;                // For save/open parameters functionality
using System.IO.Ports;          // For communicating with the simulator
using System.Diagnostics;       // For processing simulator object
using System.Net;               // For simulator communication
using System.Net.Sockets;       // For simulator communication
using System.Threading;
using System.Management;
using XInputDotNetPure;                 // for xbox controller
using dynamixel_sdk;                    // for dynamixel
using MyoSharp.Communication;           // for MyoSharp
using MyoSharp.Device;                  // for MyoSharp
using MyoSharp.Exceptions;              // for MyoSharp
using Clifton.Collections.Generic;      // For simple moving average
using Clifton.Tools.Data;               // For simple moving average
using System.Windows.Input;

namespace brachIOplexus
{
    public partial class mainForm : Form
    {

        #region "Initialization"
        // Create xPC Target parameter object for xPC Target/SLRT interface
        int[] SLRT_ch = new int[8];
        int SLRTflag = 0;   //  Used to cycle through which channels are being read each time step of the main loop (2 channels/time step), initialize to 0 when connecting to SLRT

        // Create TCP listener/client for biopatrec communication and initalize related variables
        TcpListener listener;
        TcpClient client;
        NetworkStream netStream;
        Thread t = null;
        Int32 biopatrec_ID = 0;
        Int32 biopatrec_vel = 0;
        const int CLASS_NUM = 25;                        // Number of classes that are available for mapping
        Stopwatch stopWatch2 = new Stopwatch();
        long milliSec2;     // the timestep of the biopatrec loop in milliseconds

        // Create thread for UDP client for interactiveMyoControl communication and initalize related variables
        Thread t2 = null;
        double[] UDP_ch = new double[11];
        UdpClient udpClientRX;
        IPEndPoint ipEndPointRX;

        // Create a socket client object for legacy simulator
        System.Net.Sockets.TcpClient socketClient = new System.Net.Sockets.TcpClient();

        // Process variable for legacy simulator
        private Process myProcess = new Process();
        // Properties for running the process are stored here
        private ProcessStartInfo procInfo = new ProcessStartInfo();

        // Absolute path to the profiles used as part of the 'Open/Save Profiles' functionality
        string ProfilesPath = @"Resources\Profiles";

        // Create a sound player for sequential switching feedback
        System.Media.SoundPlayer player = new System.Media.SoundPlayer();

        // XInputDotNet - Initialize ReporterState for XBOX controller
        private ReporterState reporterState = new ReporterState();
        bool XboxBuzzFlag = false;
        Stopwatch XboxTimer = new Stopwatch();

        // SimpleMovingAverage - Initialize variables used for simple moving average filter for MYO EMG channels
        static int window = 40;     // The window size for the averaging filters i.e. the number of samples it averages over
        static IMovingAverage avg1 = new SimpleMovingAverage(window);    // for simplemovingaverage filter
        static IMovingAverage avg2 = new SimpleMovingAverage(window);    // for simplemovingaverage filter
        static IMovingAverage avg3 = new SimpleMovingAverage(window);    // for simplemovingaverage filter
        static IMovingAverage avg4 = new SimpleMovingAverage(window);    // for simplemovingaverage filter
        static IMovingAverage avg5 = new SimpleMovingAverage(window);    // for simplemovingaverage filter
        static IMovingAverage avg6 = new SimpleMovingAverage(window);    // for simplemovingaverage filter
        static IMovingAverage avg7 = new SimpleMovingAverage(window);    // for simplemovingaverage filter
        static IMovingAverage avg8 = new SimpleMovingAverage(window);    // for simplemovingaverage filter

        // MyoSharp - Initialize variables
        private IChannel channel;
        private IHub hub;
        float ch1_value;
        float ch2_value;
        float ch3_value;
        float ch4_value;
        float ch5_value;
        float ch6_value;
        float ch7_value;
        float ch8_value;
        bool myoBuzzFlag = false;

        // Keyboard - Initialize variables
        const int KB_NUM = 15;                        // Number of keyboard keys that are available for mapping
        double[] KBvel = new double[KB_NUM];

        // add stopwatch for tracking main loop delay
        Stopwatch stopWatch1 = new Stopwatch();
        long milliSec1;     // the timestep of the main loop in milliseconds -> will vary depending on how many dynamixel servos are connected

        // Quick Profiles - Initialize variables
        bool QuickProfileFlag = false;
        int QuickProfileState = 0;

        // Auto-suspend - Initialize variables
        bool autosuspend_flag = false;

        // serialArduinoInput - initialize variables for reading in strings over serial or bluetooth
        string RxString;
        char[] separatingChars = { 'A', 'B', 'C', 'D' };
        Stopwatch ArduinoStartTimer = new Stopwatch();
        long ArduinoStartDelay = 1000;     // the timestep of the main loop in milliseconds -> will vary depending on how many dynamixel servos are connected

        // serialHANDi - initialize variables for reading in strings over serial or bluetooth via the arduino connected to the HANDi Hand
        string RxString2;
        char[] separatingChars2 = {','};
        Stopwatch HANDiStartTimer = new Stopwatch();
        long HANDiStartDelay = 1000;     // the timestep of the main loop in milliseconds -> will vary depending on how many dynamixel servos are connected
        public const int HANDI_NUM = 6;
        int[] HANDi_pos = new int[6];   // target positions for the HANDi Hand
        Thread t3 = null;
        Stopwatch stopWatch3 = new Stopwatch();
        long milliSec3;     // the timestep of the biopatrec loop in milliseconds

        // Task Timer - Initialize Variables
        Stopwatch Task_Timer = new Stopwatch();  // timer for measuring the total amount of elapsed time
        int TaskTimerState = 0;     // 0 = disabled, 1 = reset(and ready to go), 2 = running, 3 = resetting

        // Usage Log - Initialize Variables
        DateTime StartTime;

        #region "Dynamixel SDK Initilization"
        // DynamixelSDK
        // Control table address
        public const int ADDR_MX_TORQUE_ENABLE = 24;                  // Control table address is different in Dynamixel model
        public const int ADDR_MX_LED = 25;
        public const int ADDR_MX_GOAL_POSITION = 30;
        public const int ADDR_MX_MOVING_SPEED = 32;
        public const int ADDR_MX_TORQUE_LIMIT = 34;
        public const int ADDR_MX_PRESENT_POSITION = 36;
        public const int ADDR_MX_PRESENT_SPEED = 38;
        public const int ADDR_MX_PRESENT_LOAD = 40;
        public const int ADDR_MX_PRESENT_VOLTAGE = 42;
        public const int ADDR_MX_PRESENT_TEMP = 43;
        public const int ADDR_MX_MOVING = 46;

        // Data Byte Length
        public const int LEN_MX_GOAL_POSITION = 2;
        public const int LEN_MX_MOVING_SPEED = 2;
        public const int LEN_MX_GOAL_POS_SPEED = 4;
        public const int LEN_MX_TORQUE_LIMIT = 2;
        public const int LEN_MX_PRESENT_POSITION = 2;
        public const int LEN_MX_PRESENT_SPEED = 2;
        public const int LEN_MX_PRESENT_LOAD = 2;
        public const int LEN_MX_PRESENT_VOLTAGE = 1;
        public const int LEN_MX_PRESENT_TEMP = 1;
        public const int LEN_MX_MOVING = 1;

        // Protocol version
        public const int PROTOCOL_VERSION = 1;                   // See which protocol version is used in the Dynamixel

        // Default setting
        public const int BENTO_NUM = 5;                 // Number of Servos on the bus
        public const int DXL1_ID = 1;                   // Dynamixel ID: 1 (shoulder)
        public const int DXL2_ID = 2;                   // Dynamixel ID: 2 (elbow)
        public const int DXL3_ID = 3;                   // Dynamixel ID: 3 (wrist rot)
        public const int DXL4_ID = 4;                   // Dynamixel ID: 4 (wrist flex)
        public const int DXL5_ID = 5;                   // Dynamixel ID: 5 (hand open/close)
        public const int BAUDRATE = 1000000;
        public const string DEVICENAME = "COM14";      // Check which port is being used on your controller

        public const int TORQUE_ENABLE = 1;                   // Value for enabling the torque
        public const int TORQUE_DISABLE = 0;                   // Value for disabling the torque
        public const int LED_ON = 1;                            // Value for turning on LED
        public const int LED_OFF = 0;                           // Value for turning of LED
        public const int DXL_MINIMUM_POSITION_VALUE = 1100;                 // Dynamixel will rotate between this value
        public const int DXL_MAXIMUM_POSITION_VALUE = 3000;                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        public const int DXL_MOVING_STATUS_THRESHOLD = 10;                  // Dynamixel moving status threshold

        public const byte ESC_ASCII_VALUE = 0x1b;

        public const int COMM_SUCCESS = 0;                   // Communication Success result value
        public const int COMM_TX_FAIL = -1001;               // Communication Tx Failed

        // Initialize PortHandler Structs
        int port_num = 0;

        // Initialize Groupbulkread group
        int read_group_num = 0;
        public const int read_length = 10;                   // The number of bytes to read in the group bulk read packet

        // Initialize Groupsyncwrite group
        int write_group_num = 0;

        int dxl_comm_result = COMM_TX_FAIL;                                   // Communication result
        byte dxl_error = 0;                                                   // Dynamixel error

        bool dxl_addparam_result = false;                                     // AddParam result
        bool dxl_getdata_result = false;                                      // GetParam result

        // Dynamixel servo parameters
        UInt16 ID1_present_position = 0;
        UInt16 ID1_prev_state = 0;   // 0 = stopped, 1 = moving CW, 2 = moving CCW
        UInt16 ID1_goal_position = 0;
        UInt16 ID1_moving_speed = 1;
        UInt16 ID1_connected = 0;      // Is the dynamixel connected (0 = no, 1 = yes)

        UInt16 ID2_present_position = 0;
        UInt16 ID2_prev_state = 0;   // 0 = stopped, 1 = moving CW, 2 = moving CCW
        UInt16 ID2_goal_position = 0;
        UInt16 ID2_moving_speed = 1;
        UInt16 ID2_connected = 0;      // Is the dynamixel connected (0 = no, 1 = yes)

        UInt16 ID3_present_position = 0;
        UInt16 ID3_prev_state = 0;   // 0 = stopped, 1 = moving CW, 2 = moving CCW
        UInt16 ID3_goal_position = 0;
        UInt16 ID3_moving_speed = 1;
        UInt16 ID3_connected = 0;      // Is the dynamixel connected (0 = no, 1 = yes)

        UInt16 ID4_present_position = 0;
        UInt16 ID4_prev_state = 0;   // 0 = stopped, 1 = moving CW, 2 = moving CCW
        UInt16 ID4_goal_position = 0;
        UInt16 ID4_moving_speed = 1;
        UInt16 ID4_connected = 0;      // Is the dynamixel connected (0 = no, 1 = yes)

        UInt16 ID5_present_position = 0;
        int ID5_present_load = 0;
        UInt16 ID5_prev_state = 0;   // 0 = stopped, 1 = moving CW, 2 = moving CCW
        UInt16 ID5_goal_position = 0;
        UInt16 ID5_moving_speed = 1;
        UInt16 ID5_connected = 0;      // Is the dynamixel connected (0 = no, 1 = yes)

        UInt16 BentoOverloadError = 0;    // This stores the status of whether there are any overload errors detected with the dynamixel servos. 0 - no errors, 1 - warnings, 2 - errors
        UInt16 BentoOverheatError = 0;    // This stores the status of whether there are any overload errors detected with the dynamixel servos. 0 - no errors, 1 - warnings, 2 - errors

        // Used as part of the auto suspend functionality (i.e. see run/suspend buttons on bento tab)
        bool bentoSuspend = false;

        #endregion

        #region "Mapping Classes"
        // Classes for storing information about mapping and robot data

        // Class DoF_ stores all the information about each degree of freedom (DoF).
        public class DoF_
        {
            public bool Enabled { get; set; }   // Whether the DoF is enabled or not
            public int flipA { get; set; }       // This value is used to keep track of whether the output order for a given DoF has been flipped from how they are ordered by default in the output list 
            public int flipB { get; set; }       // This value is used to keep track of whether the output order for a given DoF has been flipped from how they are ordered by default in the output list
            public Ch ChA { get; set; }         // Each DoF has two channels by default. This is the first channel.
            public Ch ChB { get; set; }         // This is the second channel.

            public DoF_()
            {
                ChA = new Ch();
                ChB = new Ch();
            }

        }

        // Class Ch stores all the properties of a single channel
        public class Ch
        {
            public Put input { get; set; }    // The input device that the channel is mapped to
            public Put output { get; set; }   // The output device that the channel is mapped to
            public int mapping { get; set; }  // The mapping method used for driving the output device from the input signal. The default method is 'first past smin'
            public int signal { get; set; }   // The signal strength of the input device on the current time step.
            public decimal gain { get; set; }     // The gain that is being applied to the signal strength. Can be used to 'zoom' in on the signal
            public decimal smin { get; set; }     // The minimum threshold below which the motor does not move and above which the motor moves
            public decimal smax { get; set; }     // The maximum threshold above which the motor just continues to move at its maximum velocity
            public bool Enabled { get; set; }   // Whether the channel is enabled or not

            public Ch()
            {
                input = new Put();
                output = new Put();
            }
        }

        // Class Put stores the type and ID of each input device
        public class Put
        {
            public int Type { get; set; }    // The type of device (i.e. 1 = XBox, 2 = Myo, 3 = Keyboard) 
            public int ID { get; set; }      // The index of the device in the CheckedList in the Input/Output tab
        }

        // Class Switching stores all of the properties related to sequential switching
        public class Switching
        {
            public int DoF { get; set; }          // The DoF that will utilize sequential switching
            public int mode { get; set; }         // The switching mode (i.e. 0 = button press, 1 = co-contraction)
            public int input { get; set; }        // The button to use as a trigger if switching mode is set to button press
            public int signal { get; set; }       // The signal strength of the input device on the current time step.
            public decimal gain { get; set; }     // The gain that is being applied to the signal strength. Can be used to 'zoom' in on the signal
            public decimal smin1 { get; set; }     // The minimum threshold below which the motor does not move and above which the motor moves
            public decimal smax1 { get; set; }     // The maximum threshold above which the motor just continues to move at its maximum velocity
            public decimal cctime { get; set; }   // The time lockout where the algorithm neither moves or switches after a signal has crosses threshold
            public decimal smin2 { get; set; }     // The minimum threshold below which the motor does not move and above which the motor moves
            public decimal smax2 { get; set; }     // The maximum threshold above which the motor just continues to move at its maximum velocity
            public bool flag1 { get; set; }        // The flag used to track whether the max threshold of channel 1 was crossed during co-contraction switching (false = not crossed, true = crossed)
            public bool flag2 { get; set; }        // The flag used to track whether the max threshold of channel 2 was crossed during co-contraction switching (false = not crossed, true = crossed)
            public List[] List = new List[SWITCH_NUM];      // The actual switching list

            public Switching()
            {
                List = new List[SWITCH_NUM];
                for (int i = 0; i < SWITCH_NUM; i++)
                {
                    List[i] = new List();
                }
            }
        }

        // Class List stores all the properties for each item in the sequential switching list
        public class List
        {
            public int output { get; set; }     // The output device that the channel is mapped to
            public int flip { get; set; }       // This value is used to keep track of whether the output order for a given DoF has been flipped from how they are ordered by default in the output list 
            public int mapping { get; set; }    // The mapping method used for driving the output device from the input signal. The default method is 'first past smin'
        }

        // Class Robot stores all the properties for controlling a robot
        public class Robot
        {
            public int type { get; set; }       // The type of robot. (i.e. 0 = dynamixel, 1 = RC servo)
            public int torque { get; set; }     // Used for turning the torque on/off
            public int suspend { get; set; }    // Used for connecting or disconnecting the input devices from the output joints
            public int counter = 0;             // Used for counting the instances of the class https://stackoverflow.com/questions/12276641/count-instances-of-the-class
            public Motor[] Motor = new Motor[BENTO_NUM+HANDI_NUM];    // The list of motors
            // The indexing starts at 0, but some of the IDs are offset by 1 to reflect their actualy IDs on the dynamixel bus. For example shoulder is indexed at 0 in this object, but its ID on the bus is 1. Hand is indexed at 4, but its ID on the bus is 5.
            public Robot()
            {
                Interlocked.Increment(ref counter);

                Motor = new Motor[BENTO_NUM + HANDI_NUM];
                for (int i = 0; i < BENTO_NUM + HANDI_NUM; i++)
                {
                    Motor[i] = new Motor();
                }
            }
            ~Robot()
            {
                Interlocked.Decrement(ref counter);
            }

        }

        // Class Motor stores the properties for each motor
        public class Motor
        {
            public int pmin { get; set; }       // the CW angle limit of the motor
            public int pmax { get; set; }       // the CCW angle limit of the motor
            public int p { get; set; }       // the goal position
            public int p_prev { get; set; }     // the previous position of the motor (used for stopping dynamixel motors)
            public int wmin { get; set; }       // the minimum velocity of the motor
            public int wmax { get; set; }       // the maximum velocity of the motor
            public int w { get; set; }       // the goal velocity
            public int w_prev { get; set; }     // the previous velocity of the motor (not currently being used)


        }

        public class State
        {
            public int dofState { get; set; }       // The state of the DoF from the previous timestep. (not currently being used)
            public int switchState { get; set; }    // The state of the sequential switch (i.e. 0 = below threshold, 1 = above threshold -> switch to next item on list, 2 = Don't allow another switching event until both of the channels drops below threshold)                  
            public int listPos { get; set; }        // The position of the sequential switch in the switching order (i.e. cycles between 0 and 5 as)

            public long timer1 { get; set; }        // Counter used for co-contracting switching.
            public long timer2 { get; set; }        // 2nd counter used for co-contracting switching
            public int[] motorState = new int[BENTO_NUM + HANDI_NUM];   // The state of each motor (i.e. 0 = off, 1 = moving in cw direction, 2 = moving in ccw direction, 3 = hanging until co-contraction is finished).
            // The indexing starts at 0, but some of the IDs are offset by 1 to reflect their actualy IDs on the dynamixel bus. For example shoulder is indexed at 0 in this object, but its ID on the bus is 1. Hand is indexed at 4, but its ID on the bus is 5.
            public State()
            {
                motorState = new int[BENTO_NUM + HANDI_NUM];
                for (int i = 0; i < BENTO_NUM + HANDI_NUM; i++)
                {
                    motorState[i] = new int();
                }
            }
        }

        // Class RobotSensors stores the feedback values from a robot's sensors to be used for logging or streaming over UDP to other software
        public class RobotSensors
        {
            public int counter = 0;             // Used for counting the instances of the class https://stackoverflow.com/questions/12276641/count-instances-of-the-class
            public ID[] ID = new ID[BENTO_NUM];    // The list of motors
            // The indexing starts at 0, but some of the IDs are offset by 1 to reflect their actualy IDs on the dynamixel bus. For example shoulder is indexed at 0 in this object, but its ID on the bus is 1. Hand is indexed at 4, but its ID on the bus is 5.
            public RobotSensors()
            {
                Interlocked.Increment(ref counter);

                ID = new ID[BENTO_NUM];
                for (int i = 0; i < BENTO_NUM; i++)
                {
                    ID[i] = new ID();
                }
            }
            ~RobotSensors()
            {
                Interlocked.Decrement(ref counter);
            }

        }

        // Class ID stores the sensor values for each motor from the RobotSensors class
        public class ID
        {
            public ushort pos { get; set; }       // the current position of the motor
            public ushort posf { get; set; }      // the current filtered position of the motor
            public ushort vel { get; set; }       // the current velocity of the motor
            public ushort load { get; set; }      // the current load of the motor
            public ushort loadf { get; set; }      // the current filtered load of the motor
            public ushort volt { get; set; }      // the voltage of the motor
            public ushort temp { get; set; }      // the current temperature of the motor
            public ushort tempf { get; set; }      // the current filtered temperature of the motor
        }

        // Initialize state, robot, and switching object
        State stateObj = new State();
        public const int DOF_NUM = 6;       // the number of DoF in the GUI
        public const int SWITCH_NUM = 5;    // the number of slots for sequential switching

        Robot robotObj = new Robot();
        Switching switchObj = new Switching();
        DoF_[] dofObj = new DoF_[DOF_NUM];
        int[] dofStop = new int[2] { -1, -1};   // Used to selectively stop a joint on a robot when changing the mapping via the input/output tab. The first indice is for type and the second is for ID and correspond to the values in the outputmap array in the mainloop. If set to -1 no stoppage will occur, but if set to other values then that joint will be stopped.
        RobotSensors BentoSense = new RobotSensors();

        #endregion

        public mainForm()
        {
            InitializeComponent();
        }

        private void mainForm_Load(object sender, EventArgs e)
        {

            //// Hide the XPC target and HANDi Hand tab
            //tabControl1.TabPages.Remove(tabXPC);
            tabControl1.TabPages.Remove(tabHANDi);

            // How to find com ports and populate combobox: http://stackoverflow.com/questions/13794376/combo-box-for-serial-port
            string[] ports = SerialPort.GetPortNames();
            cmbSerialPorts.DataSource = ports;

            // Audo-detect com port that is connected to the USB2dynamixel
            string auto_com_port = Autodetect_Dyna_Port();
            // How to check index of combobox based on string: http://stackoverflow.com/questions/13459772/how-to-check-index-of-combobox-based-on-string
            if (auto_com_port != null)
            {
                cmbSerialPorts.SelectedIndex = cmbSerialPorts.Items.IndexOf(auto_com_port);
            }
            else
            {
                cmbSerialPorts.SelectedIndex = cmbSerialPorts.Items.Count - 1;
            }

            // Auto-detect COM port that is connected to the Arduino/Bluetooth mate
            // How to sort the COM ports: http://www.prodigyproductionsllc.com/articles/programming/get-and-sort-com-port-names-using-c/
            // Also need to convert to list: http://stackoverflow.com/questions/9285426/orderby-and-list-vs-iorderedenumerable
            var sortedList = ports.OrderBy(port => Convert.ToInt32(port.Replace("COM", string.Empty))).ToList();
            ArduinoInputCOM.DataSource = sortedList;
            HANDiCOM.DataSource = sortedList;

            // Use this code if you are connecting the arduino via a bluetooth mate
            if (ArduinoInputCOM.Items.Count < 2)
            {
                ArduinoInputCOM.SelectedIndex = ArduinoInputCOM.Items.Count - 1;
            }
            else
            {
                ArduinoInputCOM.SelectedIndex = ArduinoInputCOM.Items.Count - 2;
            }


            //// Use this code if you are connecting the arduino via a USB cable
            //// Reference 1: https://stackoverflow.com/questions/3293889/how-to-auto-detect-arduino-com-port
            //// Reference 2: https://stackoverflow.com/questions/450059/how-do-i-set-the-selected-item-in-a-combobox-to-match-my-string-using-c
            //ArduinoInputCOM.SelectedIndex = ArduinoInputCOM.FindStringExact(AutodetectArduinoPort());

            // Initialize text for degree of freedom comboboxes
            doF1.DoFBox.Text = "Degree of Freedom 1";
            doF2.DoFBox.Text = "Degree of Freedom 2";
            doF3.DoFBox.Text = "Degree of Freedom 3";
            doF4.DoFBox.Text = "Degree of Freedom 4";
            doF5.DoFBox.Text = "Degree of Freedom 5";
            doF6.DoFBox.Text = "Degree of Freedom 6";

            // Initialize mapping objects
            for (int i = 0; i < DOF_NUM; i++)
            {
                dofObj[i] = new DoF_();

            }

            // Used to auto deselect when duplicate input/output values are selected and autofill when paired output values are selected in the mapping tab
            List<DoF> dof_list = this.tabControl1.TabPages[1].Controls.OfType<DoF>().ToList<DoF>();

            foreach (DoF dof in dof_list)
            {
                dof.channel1.OutputIndexChanged += filterOutputComboBox;
                dof.channel2.OutputIndexChanged += filterOutputComboBox;
                dof.channel1.InputIndexChanged += filterInputComboBox;
                dof.channel2.InputIndexChanged += filterInputComboBox;
                dof.channel1.MappingIndexChanged += filterMappingComboBox;
                dof.channel2.MappingIndexChanged += filterMappingComboBox;
                dof.channel1.GainValueChanged += filterGainValue;
                dof.channel2.GainValueChanged += filterGainValue;
                dof.channel1.SminValueChanged += filterSminValue;
                dof.channel2.SminValueChanged += filterSminValue;
                dof.channel1.SmaxValueChanged += filterSmaxValue;
                dof.channel2.SmaxValueChanged += filterSmaxValue;

                // Initialize the event handlers, so they are synched with the values on the GUI
                filterGainValue(dof.channel1, e);
                filterGainValue(dof.channel2, e);
                filterSminValue(dof.channel1, e);
                filterSminValue(dof.channel2, e);
                filterSmaxValue(dof.channel1, e);
                filterSmaxValue(dof.channel2, e);
            }

            // Load default parameters
            try
            {
                if (File.Exists(@"Resources\Profiles\default.dat") == true)
                {
                    //Load profile parameters from default save file
                    LoadParameters(@"Resources\Profiles\default.dat");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }

            // Take a timestamp from when the program first loaded to use as part of the usage log
            StartTime = DateTime.Now;

            // Start pollingWorker! This is the main loop for reading in the keyboard/Xbox inputs and sending packets to the output devices.
            pollingWorker.RunWorkerAsync();
        }

        private void mainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            try
            {
                // Close simulator connection
                socketClient.Close();

                // Dispose of sound player
                player.Dispose();

                // Turn off serial communication with LED display arduino if it has not been already
                if (serialPort1.IsOpen)
                {
                    // need to disable RtsEnable when closing the port otherwise it slows down the Arduino Leonardo/Micro boards. Not necessary for other arduino boards
                    serialPort1.RtsEnable = false;
                    serialPort1.Close();
                }

                // Turn off serial communication with arduino if it has not been already
                if (serialArduinoInput.IsOpen)
                {
                    // need to disable RtsEnable when closing the port otherwise it slows down the Arduino Leonardo/Micro boards. Not necessary for other arduino boards
                    serialArduinoInput.RtsEnable = false;
                    serialArduinoInput.Close();
                }

                // Clean up the UDP objects when the form is closing
                //t2.Abort();
                //udpClientTX.Close();
                //udpClientRX.Close();

                // XInputDotNet - stop pollingWorker
                pollingWorker.CancelAsync();

                // Close port
                if (BentoGroupBox.Enabled == true)
                {
                    dynamixel.closePort(port_num);
                }

                // Write an entry into the usage log to track how much the program is being used
                // CSV labels are as follows: StartTime, EndTime, Total Minutes
                DateTime EndTime = DateTime.Now;
                string OutputString = StartTime.ToString() + ", " + EndTime.ToString() + ", " + Math.Round((EndTime.Subtract(StartTime).TotalMinutes)).ToString();
                File.AppendAllText(@"Resources\usage_log.txt", OutputString + Environment.NewLine);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        // Helper function for figuring out the COM port associated with an Arduino wired to the computer with a USB cable
        private string AutodetectArduinoPort()
        {
            ManagementScope connectionScope = new ManagementScope();
            SelectQuery serialQuery = new SelectQuery("SELECT * FROM Win32_SerialPort");
            ManagementObjectSearcher searcher = new ManagementObjectSearcher(connectionScope, serialQuery);

            try
            {
                foreach (ManagementObject item in searcher.Get())
                {
                    string desc = item["Description"].ToString();
                    string deviceId = item["DeviceID"].ToString();

                    if (desc.Contains("Arduino") || desc.Contains("Genuino"))
                    {
                        return deviceId;
                    }
                }
            }
            catch (ManagementException e)
            {
                /* Do Nothing */
            }

            return null;
        }

        #endregion

        #region "Open/Save Profiles"
        private void OpenToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // Set dialog filters
            OpenFileDialog1.Filter = "All Files (*.*)|*.*|Data Files (*.dat)|*.dat";

            // Specify default filter
            OpenFileDialog1.FilterIndex = 2;
            //OpenFileDialog1.InitialDirectory = "";
            if (Directory.Exists(System.IO.Path.GetFullPath(ProfilesPath)))
            {
                // Specify the default folder path
                OpenFileDialog1.InitialDirectory = System.IO.Path.GetFullPath(ProfilesPath);
            }
            OpenFileDialog1.RestoreDirectory = false;
            OpenFileDialog1.Title = "Open Profile";

            // Display the Open dialog box
            // Reference: http://www.dotnetperls.com/openfiledialog
            DialogResult DidWork = OpenFileDialog1.ShowDialog();


            // Call the open file procedure
            if (DidWork != DialogResult.Cancel)
            {
                LoadParameters(OpenFileDialog1.FileName);
            }
        }

        private void SaveAsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // Set dialog filters
            SaveFileDialog1.Filter = "All Files (*.*)|*.*|Data Files (*.dat)|*.dat";

            // Specify default filter
            SaveFileDialog1.FilterIndex = 2;
            //SaveFileDialog1.InitialDirectory = @"Resources\Profiles";
            if (Directory.Exists(System.IO.Path.GetFullPath(ProfilesPath)))
            {
                // Specify the default folder path
                SaveFileDialog1.InitialDirectory = System.IO.Path.GetFullPath(ProfilesPath);
            }
            SaveFileDialog1.RestoreDirectory = true;
            SaveFileDialog1.Title = "Save Profile As";

            // Display the Save dialog box
            DialogResult DidWork = SaveFileDialog1.ShowDialog();

            // Call the save file procedure
            if (DidWork != DialogResult.Cancel)
            {
                SaveParameters(SaveFileDialog1.FileName);
            }
        }

        // Save the default profile
        private void ToolStripMenuItem1_Click(object sender, EventArgs e)
        {
            SaveParameters(@"Resources\Profiles\default.dat");
        }

        private void NewToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                // Open the empty profile
                LoadParameters(@"Resources\Profiles\empty.dat");
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        public void LoadParameters(string strFileName)
        {
            //Open parameters from selected profile
            //Create a filestream object
            FileStream fsInput = new FileStream(strFileName, FileMode.Open, FileAccess.Read, FileShare.None);

            //Create a binary reader object
            BinaryReader brInput = new BinaryReader(fsInput);

            //Read the profile settings from the input stream
            for (int i = 0; i <= (XBoxList.Items.Count - 1); i++)
            {
                bool m = brInput.ReadBoolean();
                if (m == true)
                {
                    XBoxList.SetItemCheckState(i, CheckState.Checked);
                }
                else
                {
                    XBoxList.SetItemCheckState(i, CheckState.Unchecked);
                }
            }

            for (int i = 0; i <= (MYOlist.Items.Count - 1); i++)
            {
                bool m = brInput.ReadBoolean();
                if (m == true)
                {
                    MYOlist.SetItemCheckState(i, CheckState.Checked);
                }
                else
                {
                    MYOlist.SetItemCheckState(i, CheckState.Unchecked);
                }
            }

            for (int i = 0; i <= (KBlist.Items.Count - 1); i++)
            {
                bool m = brInput.ReadBoolean();
                if (m == true)
                {
                    KBlist.SetItemCheckState(i, CheckState.Checked);
                }
                else
                {
                    KBlist.SetItemCheckState(i, CheckState.Unchecked);
                }
            }

            KBcheckRamp.Checked = brInput.ReadBoolean();

            biopatrecIPaddr.Text = brInput.ReadString();
            biopatrecIPport.Text = brInput.ReadString();

            for (int i = 0; i <= (biopatrecList.Items.Count - 1); i++)
            {
                bool m = brInput.ReadBoolean();
                if (m == true)
                {
                    biopatrecList.SetItemCheckState(i, CheckState.Checked);
                }
                else
                {
                    biopatrecList.SetItemCheckState(i, CheckState.Unchecked);
                }
            }

            for (int i = 0; i <= (ArduinoInputList.Items.Count - 1); i++)
            {
                bool m = brInput.ReadBoolean();
                if (m == true)
                {
                    ArduinoInputList.SetItemCheckState(i, CheckState.Checked);
                }
                else
                {
                    ArduinoInputList.SetItemCheckState(i, CheckState.Unchecked);
                }
            }

            for (int i = 0; i <= (SLRTlist.Items.Count - 1); i++)
            {
                bool m = brInput.ReadBoolean();
                if (m == true)
                {
                    SLRTlist.SetItemCheckState(i, CheckState.Checked);
                }
                else
                {
                    SLRTlist.SetItemCheckState(i, CheckState.Unchecked);
                }
            }

            for (int i = 0; i <= (udpList.Items.Count - 1); i++)
            {
                bool m = brInput.ReadBoolean();
                if (m == true)
                {
                    udpList.SetItemCheckState(i, CheckState.Checked);
                }
                else
                {
                    udpList.SetItemCheckState(i, CheckState.Unchecked);
                }
            }

            for (int i = 0; i <= (BentoList.Items.Count - 1); i++)
            {
                bool m = brInput.ReadBoolean();
                if (m == true)
                {
                    BentoList.SetItemCheckState(i, CheckState.Checked);
                }
                else
                {
                    BentoList.SetItemCheckState(i, CheckState.Unchecked);
                }
            }

            for (int i = 0; i <= (HANDiList.Items.Count - 1); i++)
            {
                bool m = brInput.ReadBoolean();
                if (m == true)
                {
                    HANDiList.SetItemCheckState(i, CheckState.Checked);
                }
                else
                {
                    HANDiList.SetItemCheckState(i, CheckState.Unchecked);
                }
            }

            updateCheckedLists();

            doF1.channel1.inputBox.SelectedIndex = brInput.ReadInt32();
            doF1.channel1.outputBox.SelectedIndex = brInput.ReadInt32();
            doF1.channel1.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF1.channel1.gainCtrl.Value = brInput.ReadDecimal();
            doF1.channel1.sminCtrl.Value = brInput.ReadDecimal();
            doF1.channel1.smaxCtrl.Value = brInput.ReadDecimal();
            doF1.channel2.inputBox.SelectedIndex = brInput.ReadInt32();
            doF1.channel2.outputBox.SelectedIndex = brInput.ReadInt32();
            doF1.channel2.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF1.channel2.gainCtrl.Value = brInput.ReadDecimal();
            doF1.channel2.sminCtrl.Value = brInput.ReadDecimal();
            doF1.channel2.smaxCtrl.Value = brInput.ReadDecimal();

            doF2.channel1.inputBox.SelectedIndex = brInput.ReadInt32();
            doF2.channel1.outputBox.SelectedIndex = brInput.ReadInt32();
            doF2.channel1.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF2.channel1.gainCtrl.Value = brInput.ReadDecimal();
            doF2.channel1.sminCtrl.Value = brInput.ReadDecimal();
            doF2.channel1.smaxCtrl.Value = brInput.ReadDecimal();
            doF2.channel2.inputBox.SelectedIndex = brInput.ReadInt32();
            doF2.channel2.outputBox.SelectedIndex = brInput.ReadInt32();
            doF2.channel2.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF2.channel2.gainCtrl.Value = brInput.ReadDecimal();
            doF2.channel2.sminCtrl.Value = brInput.ReadDecimal();
            doF2.channel2.smaxCtrl.Value = brInput.ReadDecimal();

            doF3.channel1.inputBox.SelectedIndex = brInput.ReadInt32();
            doF3.channel1.outputBox.SelectedIndex = brInput.ReadInt32();
            doF3.channel1.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF3.channel1.gainCtrl.Value = brInput.ReadDecimal();
            doF3.channel1.sminCtrl.Value = brInput.ReadDecimal();
            doF3.channel1.smaxCtrl.Value = brInput.ReadDecimal();
            doF3.channel2.inputBox.SelectedIndex = brInput.ReadInt32();
            doF3.channel2.outputBox.SelectedIndex = brInput.ReadInt32();
            doF3.channel2.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF3.channel2.gainCtrl.Value = brInput.ReadDecimal();
            doF3.channel2.sminCtrl.Value = brInput.ReadDecimal();
            doF3.channel2.smaxCtrl.Value = brInput.ReadDecimal();

            doF4.channel1.inputBox.SelectedIndex = brInput.ReadInt32();
            doF4.channel1.outputBox.SelectedIndex = brInput.ReadInt32();
            doF4.channel1.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF4.channel1.gainCtrl.Value = brInput.ReadDecimal();
            doF4.channel1.sminCtrl.Value = brInput.ReadDecimal();
            doF4.channel1.smaxCtrl.Value = brInput.ReadDecimal();
            doF4.channel2.inputBox.SelectedIndex = brInput.ReadInt32();
            doF4.channel2.outputBox.SelectedIndex = brInput.ReadInt32();
            doF4.channel2.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF4.channel2.gainCtrl.Value = brInput.ReadDecimal();
            doF4.channel2.sminCtrl.Value = brInput.ReadDecimal();
            doF4.channel2.smaxCtrl.Value = brInput.ReadDecimal();

            doF5.channel1.inputBox.SelectedIndex = brInput.ReadInt32();
            doF5.channel1.outputBox.SelectedIndex = brInput.ReadInt32();
            doF5.channel1.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF5.channel1.gainCtrl.Value = brInput.ReadDecimal();
            doF5.channel1.sminCtrl.Value = brInput.ReadDecimal();
            doF5.channel1.smaxCtrl.Value = brInput.ReadDecimal();
            doF5.channel2.inputBox.SelectedIndex = brInput.ReadInt32();
            doF5.channel2.outputBox.SelectedIndex = brInput.ReadInt32();
            doF5.channel2.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF5.channel2.gainCtrl.Value = brInput.ReadDecimal();
            doF5.channel2.sminCtrl.Value = brInput.ReadDecimal();
            doF5.channel2.smaxCtrl.Value = brInput.ReadDecimal();

            doF6.channel1.inputBox.SelectedIndex = brInput.ReadInt32();
            doF6.channel1.outputBox.SelectedIndex = brInput.ReadInt32();
            doF6.channel1.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF6.channel1.gainCtrl.Value = brInput.ReadDecimal();
            doF6.channel1.sminCtrl.Value = brInput.ReadDecimal();
            doF6.channel1.smaxCtrl.Value = brInput.ReadDecimal();
            doF6.channel2.inputBox.SelectedIndex = brInput.ReadInt32();
            doF6.channel2.outputBox.SelectedIndex = brInput.ReadInt32();
            doF6.channel2.mappingBox.SelectedIndex = brInput.ReadInt32();
            doF6.channel2.gainCtrl.Value = brInput.ReadDecimal();
            doF6.channel2.sminCtrl.Value = brInput.ReadDecimal();
            doF6.channel2.smaxCtrl.Value = brInput.ReadDecimal();

            switchDoFbox.SelectedIndex = brInput.ReadInt32();
            switchModeBox.SelectedIndex = brInput.ReadInt32();
            switchInputBox.SelectedIndex = brInput.ReadInt32();
            switchGainCtrl1.Value = brInput.ReadDecimal();
            switchSminCtrl1.Value = brInput.ReadDecimal();
            switchSmaxCtrl1.Value = brInput.ReadDecimal();
            switchTimeCtrl1.Value = brInput.ReadDecimal();
            switchSminCtrl2.Value = brInput.ReadDecimal();
            switchSmaxCtrl2.Value = brInput.ReadDecimal();
            switch1OutputBox.SelectedIndex = brInput.ReadInt32();
            switch2OutputBox.SelectedIndex = brInput.ReadInt32();
            switch3OutputBox.SelectedIndex = brInput.ReadInt32();
            switch4OutputBox.SelectedIndex = brInput.ReadInt32();
            switch5OutputBox.SelectedIndex = brInput.ReadInt32();
            switch1MappingBox.SelectedIndex = brInput.ReadInt32();
            switch2MappingBox.SelectedIndex = brInput.ReadInt32();
            switch3MappingBox.SelectedIndex = brInput.ReadInt32();
            switch4MappingBox.SelectedIndex = brInput.ReadInt32();
            switch5MappingBox.SelectedIndex = brInput.ReadInt32();
            switch1Flip.Checked = brInput.ReadBoolean();
            switch2Flip.Checked = brInput.ReadBoolean();
            switch3Flip.Checked = brInput.ReadBoolean();
            switch4Flip.Checked = brInput.ReadBoolean();
            switch5Flip.Checked = brInput.ReadBoolean();
            textBox.Checked = brInput.ReadBoolean();
            vocalBox.Checked = brInput.ReadBoolean();
            dingBox.Checked = brInput.ReadBoolean();
            myoBuzzBox.Checked = brInput.ReadBoolean();
            XboxBuzzBox.Checked = brInput.ReadBoolean();

            shoulder_pmin_ctrl.Value = brInput.ReadDecimal();
            shoulder_pmax_ctrl.Value = brInput.ReadDecimal();
            shoulder_wmin_ctrl.Value = brInput.ReadDecimal();
            shoulder_wmax_ctrl.Value = brInput.ReadDecimal();

            elbow_pmin_ctrl.Value = brInput.ReadDecimal();
            elbow_pmax_ctrl.Value = brInput.ReadDecimal();
            elbow_wmin_ctrl.Value = brInput.ReadDecimal();
            elbow_wmax_ctrl.Value = brInput.ReadDecimal();

            wristRot_pmin_ctrl.Value = brInput.ReadDecimal();
            wristRot_pmax_ctrl.Value = brInput.ReadDecimal();
            wristRot_wmin_ctrl.Value = brInput.ReadDecimal();
            wristRot_wmax_ctrl.Value = brInput.ReadDecimal();

            wristFlex_pmin_ctrl.Value = brInput.ReadDecimal();
            wristFlex_pmax_ctrl.Value = brInput.ReadDecimal();
            wristFlex_wmin_ctrl.Value = brInput.ReadDecimal();
            wristFlex_wmax_ctrl.Value = brInput.ReadDecimal();

            hand_pmin_ctrl.Value = brInput.ReadDecimal();
            hand_pmax_ctrl.Value = brInput.ReadDecimal();
            hand_wmin_ctrl.Value = brInput.ReadDecimal();
            hand_wmax_ctrl.Value = brInput.ReadDecimal();

            BentoAdaptGripCheck.Checked = brInput.ReadBoolean();
            BentoAdaptGripCtrl.Value = brInput.ReadDecimal();

            D0_pmin_ctrl.Value = brInput.ReadDecimal();
            D0_pmax_ctrl.Value = brInput.ReadDecimal();
            D0_wmin_ctrl.Value = brInput.ReadDecimal();
            D0_wmax_ctrl.Value = brInput.ReadDecimal();

            D1_pmin_ctrl.Value = brInput.ReadDecimal();
            D1_pmax_ctrl.Value = brInput.ReadDecimal();
            D1_wmin_ctrl.Value = brInput.ReadDecimal();
            D1_wmax_ctrl.Value = brInput.ReadDecimal();

            D2_pmin_ctrl.Value = brInput.ReadDecimal();
            D2_pmax_ctrl.Value = brInput.ReadDecimal();
            D2_wmin_ctrl.Value = brInput.ReadDecimal();
            D2_wmax_ctrl.Value = brInput.ReadDecimal();

            D3_pmin_ctrl.Value = brInput.ReadDecimal();
            D3_pmax_ctrl.Value = brInput.ReadDecimal();
            D3_wmin_ctrl.Value = brInput.ReadDecimal();
            D3_wmax_ctrl.Value = brInput.ReadDecimal();

            D4_pmin_ctrl.Value = brInput.ReadDecimal();
            D4_pmax_ctrl.Value = brInput.ReadDecimal();
            D4_wmin_ctrl.Value = brInput.ReadDecimal();
            D4_wmax_ctrl.Value = brInput.ReadDecimal();

            D5_pmin_ctrl.Value = brInput.ReadDecimal();
            D5_pmax_ctrl.Value = brInput.ReadDecimal();
            D5_wmin_ctrl.Value = brInput.ReadDecimal();
            D5_wmax_ctrl.Value = brInput.ReadDecimal();

            //Close and dispose of file writing objects
            brInput.Close();
            fsInput.Close();
            fsInput.Dispose();

            // Call event handlers that are not updated on each iteration of the main loop to ensure synchronization of mapping parameters
            switchDoFbox_SelectedIndexChanged(null, null);
            switchModeBox_SelectedIndexChanged(null, null);
            switchInputBox_SelectedIndexChanged(null, null);
            switchGainCtrl1_ValueChanged(null, null);
            switchSminCtrl1_ValueChanged(null, null);
            switchSmaxCtrl1_ValueChanged(null, null);
            switchTimeCtrl1_ValueChanged(null, null);
            switchSminCtrl2_ValueChanged(null, null);
            switchSmaxCtrl2_ValueChanged(null, null);
            switch1OutputBox_SelectedIndexChanged(null, null);
            switch2OutputBox_SelectedIndexChanged(null, null);
            switch3OutputBox_SelectedIndexChanged(null, null);
            switch4OutputBox_SelectedIndexChanged(null, null);
            switch5OutputBox_SelectedIndexChanged(null, null);
            switch1MappingBox_SelectedIndexChanged(null, null);
            switch2MappingBox_SelectedIndexChanged(null, null);
            switch3MappingBox_SelectedIndexChanged(null, null);
            switch4MappingBox_SelectedIndexChanged(null, null);
            switch5MappingBox_SelectedIndexChanged(null, null);
            switch1Flip_CheckedChanged(null, null);
            switch2Flip_CheckedChanged(null, null);
            switch3Flip_CheckedChanged(null, null);
            switch4Flip_CheckedChanged(null, null);
            switch5Flip_CheckedChanged(null, null);

            // Ensure that the active joint does not get stuck in its sequential switching hang state when switching between profiles
            if ((switchObj.List[stateObj.listPos].output - 1) >= 0)
            {
                stateObj.motorState[switchObj.List[stateObj.listPos].output - 1] = 0;
            }
        }

        public void SaveParameters(string strFileName)
        {
            //Save parameters to selected profile
            //Set the output variables to the controls in the GUI

            //Create a filestream object
            FileStream fsOutput = new FileStream(strFileName, FileMode.Create, FileAccess.Write, FileShare.None);

            //Create a binary writer object
            BinaryWriter bwOutput = new BinaryWriter(fsOutput);

            //Write the profile settings

            for (int i = 0; i <= (XBoxList.Items.Count - 1); i++)
            {
                bwOutput.Write(Convert.ToBoolean(XBoxList.GetItemCheckState(i)));
            }

            for (int i = 0; i <= (MYOlist.Items.Count - 1); i++)
            {
                bwOutput.Write(Convert.ToBoolean(MYOlist.GetItemCheckState(i)));
            }

            for (int i = 0; i <= (KBlist.Items.Count - 1); i++)
            {
                bwOutput.Write(Convert.ToBoolean(KBlist.GetItemCheckState(i)));
            }

            bwOutput.Write(KBcheckRamp.Checked);

            bwOutput.Write(biopatrecIPaddr.Text);

            bwOutput.Write(biopatrecIPport.Text);

            for (int i = 0; i <= (biopatrecList.Items.Count - 1); i++)
            {
                bwOutput.Write(Convert.ToBoolean(biopatrecList.GetItemCheckState(i)));
            }

            for (int i = 0; i <= (ArduinoInputList.Items.Count - 1); i++)
            {
                bwOutput.Write(Convert.ToBoolean(ArduinoInputList.GetItemCheckState(i)));
            }

            for (int i = 0; i <= (SLRTlist.Items.Count - 1); i++)
            {
                bwOutput.Write(Convert.ToBoolean(SLRTlist.GetItemCheckState(i)));
            }

            for (int i = 0; i <= (udpList.Items.Count - 1); i++)
            {
                bwOutput.Write(Convert.ToBoolean(udpList.GetItemCheckState(i)));
            }

            for (int i = 0; i <= (BentoList.Items.Count - 1); i++)
            {
                bwOutput.Write(Convert.ToBoolean(BentoList.GetItemCheckState(i)));
            }

            for (int i = 0; i <= (HANDiList.Items.Count - 1); i++)
            {
                bwOutput.Write(Convert.ToBoolean(HANDiList.GetItemCheckState(i)));
            }

            bwOutput.Write(doF1.channel1.inputBox.SelectedIndex);
            bwOutput.Write(doF1.channel1.outputBox.SelectedIndex);
            bwOutput.Write(doF1.channel1.mappingBox.SelectedIndex);
            bwOutput.Write(doF1.channel1.gainCtrl.Value);
            bwOutput.Write(doF1.channel1.sminCtrl.Value);
            bwOutput.Write(doF1.channel1.smaxCtrl.Value);
            bwOutput.Write(doF1.channel2.inputBox.SelectedIndex);
            bwOutput.Write(doF1.channel2.outputBox.SelectedIndex);
            bwOutput.Write(doF1.channel2.mappingBox.SelectedIndex);
            bwOutput.Write(doF1.channel2.gainCtrl.Value);
            bwOutput.Write(doF1.channel2.sminCtrl.Value);
            bwOutput.Write(doF1.channel2.smaxCtrl.Value);

            bwOutput.Write(doF2.channel1.inputBox.SelectedIndex);
            bwOutput.Write(doF2.channel1.outputBox.SelectedIndex);
            bwOutput.Write(doF2.channel1.mappingBox.SelectedIndex);
            bwOutput.Write(doF2.channel1.gainCtrl.Value);
            bwOutput.Write(doF2.channel1.sminCtrl.Value);
            bwOutput.Write(doF2.channel1.smaxCtrl.Value);
            bwOutput.Write(doF2.channel2.inputBox.SelectedIndex);
            bwOutput.Write(doF2.channel2.outputBox.SelectedIndex);
            bwOutput.Write(doF2.channel2.mappingBox.SelectedIndex);
            bwOutput.Write(doF2.channel2.gainCtrl.Value);
            bwOutput.Write(doF2.channel2.sminCtrl.Value);
            bwOutput.Write(doF2.channel2.smaxCtrl.Value);

            bwOutput.Write(doF3.channel1.inputBox.SelectedIndex);
            bwOutput.Write(doF3.channel1.outputBox.SelectedIndex);
            bwOutput.Write(doF3.channel1.mappingBox.SelectedIndex);
            bwOutput.Write(doF3.channel1.gainCtrl.Value);
            bwOutput.Write(doF3.channel1.sminCtrl.Value);
            bwOutput.Write(doF3.channel1.smaxCtrl.Value);
            bwOutput.Write(doF3.channel2.inputBox.SelectedIndex);
            bwOutput.Write(doF3.channel2.outputBox.SelectedIndex);
            bwOutput.Write(doF3.channel2.mappingBox.SelectedIndex);
            bwOutput.Write(doF3.channel2.gainCtrl.Value);
            bwOutput.Write(doF3.channel2.sminCtrl.Value);
            bwOutput.Write(doF3.channel2.smaxCtrl.Value);

            bwOutput.Write(doF4.channel1.inputBox.SelectedIndex);
            bwOutput.Write(doF4.channel1.outputBox.SelectedIndex);
            bwOutput.Write(doF4.channel1.mappingBox.SelectedIndex);
            bwOutput.Write(doF4.channel1.gainCtrl.Value);
            bwOutput.Write(doF4.channel1.sminCtrl.Value);
            bwOutput.Write(doF4.channel1.smaxCtrl.Value);
            bwOutput.Write(doF4.channel2.inputBox.SelectedIndex);
            bwOutput.Write(doF4.channel2.outputBox.SelectedIndex);
            bwOutput.Write(doF4.channel2.mappingBox.SelectedIndex);
            bwOutput.Write(doF4.channel2.gainCtrl.Value);
            bwOutput.Write(doF4.channel2.sminCtrl.Value);
            bwOutput.Write(doF4.channel2.smaxCtrl.Value);

            bwOutput.Write(doF5.channel1.inputBox.SelectedIndex);
            bwOutput.Write(doF5.channel1.outputBox.SelectedIndex);
            bwOutput.Write(doF5.channel1.mappingBox.SelectedIndex);
            bwOutput.Write(doF5.channel1.gainCtrl.Value);
            bwOutput.Write(doF5.channel1.sminCtrl.Value);
            bwOutput.Write(doF5.channel1.smaxCtrl.Value);
            bwOutput.Write(doF5.channel2.inputBox.SelectedIndex);
            bwOutput.Write(doF5.channel2.outputBox.SelectedIndex);
            bwOutput.Write(doF5.channel2.mappingBox.SelectedIndex);
            bwOutput.Write(doF5.channel2.gainCtrl.Value);
            bwOutput.Write(doF5.channel2.sminCtrl.Value);
            bwOutput.Write(doF5.channel2.smaxCtrl.Value);

            bwOutput.Write(doF6.channel1.inputBox.SelectedIndex);
            bwOutput.Write(doF6.channel1.outputBox.SelectedIndex);
            bwOutput.Write(doF6.channel1.mappingBox.SelectedIndex);
            bwOutput.Write(doF6.channel1.gainCtrl.Value);
            bwOutput.Write(doF6.channel1.sminCtrl.Value);
            bwOutput.Write(doF6.channel1.smaxCtrl.Value);
            bwOutput.Write(doF6.channel2.inputBox.SelectedIndex);
            bwOutput.Write(doF6.channel2.outputBox.SelectedIndex);
            bwOutput.Write(doF6.channel2.mappingBox.SelectedIndex);
            bwOutput.Write(doF6.channel2.gainCtrl.Value);
            bwOutput.Write(doF6.channel2.sminCtrl.Value);
            bwOutput.Write(doF6.channel2.smaxCtrl.Value);

            bwOutput.Write(switchDoFbox.SelectedIndex);
            bwOutput.Write(switchModeBox.SelectedIndex);
            bwOutput.Write(switchInputBox.SelectedIndex);
            bwOutput.Write(switchGainCtrl1.Value);
            bwOutput.Write(switchSminCtrl1.Value);
            bwOutput.Write(switchSmaxCtrl1.Value);
            bwOutput.Write(switchTimeCtrl1.Value);
            bwOutput.Write(switchSminCtrl2.Value);
            bwOutput.Write(switchSmaxCtrl2.Value);
            bwOutput.Write(switch1OutputBox.SelectedIndex);
            bwOutput.Write(switch2OutputBox.SelectedIndex);
            bwOutput.Write(switch3OutputBox.SelectedIndex);
            bwOutput.Write(switch4OutputBox.SelectedIndex);
            bwOutput.Write(switch5OutputBox.SelectedIndex);
            bwOutput.Write(switch1MappingBox.SelectedIndex);
            bwOutput.Write(switch2MappingBox.SelectedIndex);
            bwOutput.Write(switch3MappingBox.SelectedIndex);
            bwOutput.Write(switch4MappingBox.SelectedIndex);
            bwOutput.Write(switch5MappingBox.SelectedIndex);
            bwOutput.Write(switch1Flip.Checked);
            bwOutput.Write(switch2Flip.Checked);
            bwOutput.Write(switch3Flip.Checked);
            bwOutput.Write(switch4Flip.Checked);
            bwOutput.Write(switch5Flip.Checked);
            bwOutput.Write(textBox.Checked);
            bwOutput.Write(vocalBox.Checked);
            bwOutput.Write(dingBox.Checked);
            bwOutput.Write(myoBuzzBox.Checked);
            bwOutput.Write(XboxBuzzBox.Checked);

            bwOutput.Write(shoulder_pmin_ctrl.Value);
            bwOutput.Write(shoulder_pmax_ctrl.Value);
            bwOutput.Write(shoulder_wmin_ctrl.Value);
            bwOutput.Write(shoulder_wmax_ctrl.Value);

            bwOutput.Write(elbow_pmin_ctrl.Value);
            bwOutput.Write(elbow_pmax_ctrl.Value);
            bwOutput.Write(elbow_wmin_ctrl.Value);
            bwOutput.Write(elbow_wmax_ctrl.Value);

            bwOutput.Write(wristRot_pmin_ctrl.Value);
            bwOutput.Write(wristRot_pmax_ctrl.Value);
            bwOutput.Write(wristRot_wmin_ctrl.Value);
            bwOutput.Write(wristRot_wmax_ctrl.Value);

            bwOutput.Write(wristFlex_pmin_ctrl.Value);
            bwOutput.Write(wristFlex_pmax_ctrl.Value);
            bwOutput.Write(wristFlex_wmin_ctrl.Value);
            bwOutput.Write(wristFlex_wmax_ctrl.Value);

            bwOutput.Write(hand_pmin_ctrl.Value);
            bwOutput.Write(hand_pmax_ctrl.Value);
            bwOutput.Write(hand_wmin_ctrl.Value);
            bwOutput.Write(hand_wmax_ctrl.Value);

            bwOutput.Write(BentoAdaptGripCheck.Checked);
            bwOutput.Write(BentoAdaptGripCtrl.Value);

            bwOutput.Write(D0_pmin_ctrl.Value);
            bwOutput.Write(D0_pmax_ctrl.Value);
            bwOutput.Write(D0_wmin_ctrl.Value);
            bwOutput.Write(D0_wmax_ctrl.Value);

            bwOutput.Write(D1_pmin_ctrl.Value);
            bwOutput.Write(D1_pmax_ctrl.Value);
            bwOutput.Write(D1_wmin_ctrl.Value);
            bwOutput.Write(D1_wmax_ctrl.Value);

            bwOutput.Write(D2_pmin_ctrl.Value);
            bwOutput.Write(D2_pmax_ctrl.Value);
            bwOutput.Write(D2_wmin_ctrl.Value);
            bwOutput.Write(D2_wmax_ctrl.Value);

            bwOutput.Write(D3_pmin_ctrl.Value);
            bwOutput.Write(D3_pmax_ctrl.Value);
            bwOutput.Write(D3_wmin_ctrl.Value);
            bwOutput.Write(D3_wmax_ctrl.Value);

            bwOutput.Write(D4_pmin_ctrl.Value);
            bwOutput.Write(D4_pmax_ctrl.Value);
            bwOutput.Write(D4_wmin_ctrl.Value);
            bwOutput.Write(D4_wmax_ctrl.Value);

            bwOutput.Write(D5_pmin_ctrl.Value);
            bwOutput.Write(D5_pmax_ctrl.Value);
            bwOutput.Write(D5_wmin_ctrl.Value);
            bwOutput.Write(D5_wmax_ctrl.Value);

            //Close and dispose of file writing objects
            bwOutput.Close();
            fsOutput.Close();
            fsOutput.Dispose();

        }
        #endregion

        #region "XBOX Controller"
        // Stuff to make XInputDotNet work
        private void XboxConnect_Click(object sender, EventArgs e)
        {
            // Check whether Xbox controller is connected
            if (reporterState.LastActiveState.IsConnected == true)
            {
                // Enable Xbox feedback
                xBoxGroupBox.Enabled = true;
                XboxDisconnect.Enabled = true;
                XboxConnect.Enabled = false;
                XBoxList.Enabled = true;
                XBoxSelectAll.Enabled = true;
                XBoxClearAll.Enabled = true;
            }
        }

        private void XboxDisconnect_Click(object sender, EventArgs e)
        {
            // Re-configure the GUI when the Xbox controller is disconnected
            xBoxGroupBox.Enabled = false;
            XboxDisconnect.Enabled = false;
            XboxConnect.Enabled = true;
            XBoxList.Enabled = false;
            XBoxSelectAll.Enabled = false;
            XBoxClearAll.Enabled = false;
            XboxConnect.Focus();
        }

        private void XBoxSelectAll_Click(object sender, EventArgs e)
        {
            // Select all of the items in the checkedListBox
            for (int i = 0; i < XBoxList.Items.Count; i++)
            {
                XBoxList.SetItemChecked(i, true);
            }
        }

        private void XBoxClearAll_Click(object sender, EventArgs e)
        {
            // Unselect all of the items in the checkedListBox
            for (int i = 0; i < XBoxList.Items.Count; i++)
            {
                XBoxList.SetItemChecked(i, false);
            }
        }

        #endregion

        #region "MYO Armband Controller"

        private void MYOconnect_Click(object sender, EventArgs e)
        {
            try
            {
                // Get Reference to the current Process
                Process thisProc = Process.GetCurrentProcess();

                // Check whether MYO connect is open before starting up the MYO event handlers. If you don't check first it will throw an exception and crash the program
                if (IsProcessOpen("Myo Connect") == true)
                {
                    // MyoSharp - Create a channel and hub that will manage Myo devices for us
                    channel = MyoSharp.Communication.Channel.Create(ChannelDriver.Create(ChannelBridge.Create(), MyoErrorHandlerDriver.Create(MyoErrorHandlerBridge.Create())));
                    hub = Hub.Create(channel);
                    hub.MyoConnected += Hub_MyoConnected;
                    hub.MyoDisconnected += Hub_MyoDisconnected;
                    channel.StartListening();
                    MYOstatus.Text = "Searching for MYO...";

                    // Enable MYO feedback
                    MYOgroupBox.Enabled = true;
                    MYOdisconnect.Enabled = true;
                    MYOconnect.Enabled = false;
                    MYOlist.Enabled = true;
                    MYOselectAll.Enabled = true;
                    MYOclearAll.Enabled = true;
                }
                else
                {
                    MessageBox.Show("Please launch Myo Connect.exe before trying to connect to armband.");
                }
            }


            catch (Exception me)
            {
                MessageBox.Show(me.Message);
            }
        }

        private void MYO_disconnect_Click(object sender, EventArgs e)
        {
            // Re-configure the GUI when the MYO is disconnected and disable the event handlers
            hub.MyoConnected -= Hub_MyoConnected;
            hub.MyoDisconnected -= Hub_MyoDisconnected;
            channel.Dispose();
            hub.Dispose();
            MYOstatus.Text = "Disconnected";

            // Disable MYO feedback
            MYOgroupBox.Enabled = false;
            MYOdisconnect.Enabled = false;
            MYOconnect.Enabled = true;
            MYOlist.Enabled = false;
            MYOselectAll.Enabled = false;
            MYOclearAll.Enabled = false;
            MYOconnect.Focus();
        }

        private void MYOselectAll_Click(object sender, EventArgs e)
        {
            // Select all of the items in the checkedListBox
            for (int i = 0; i < MYOlist.Items.Count; i++)
            {
                MYOlist.SetItemChecked(i, true);

            }
        }

        private void MYOclearAll_Click(object sender, EventArgs e)
        {
            // Unselect all of the items in the checkedListBox
            for (int i = 0; i < MYOlist.Items.Count; i++)
            {
                MYOlist.SetItemChecked(i, false);

            }
        }

        #region "MYO Event Handlers"
        // Event handler for when MYO armband disconnects 
        private void Hub_MyoDisconnected(object sender, MyoEventArgs e)
        {
            SetText("Searching for MYO...");    // Update MYO status in status bar
            e.Myo.SetEmgStreaming(false);
            e.Myo.EmgDataAcquired -= Myo_EmgDataAcquired;
        }

        // Event handler for when MYO armband connects 
        private void Hub_MyoConnected(object sender, MyoEventArgs e)
        {
            SetText("Connected");   // Update MYO status in status bar                           
            e.Myo.Vibrate(VibrationType.Medium);    // Vibrate the MYO when it connects
            e.Myo.EmgDataAcquired += Myo_EmgDataAcquired;
            e.Myo.SetEmgStreaming(true);
        }


        // Cross thread operation not valid: https://stackoverflow.com/questions/10775367/cross-thread-operation-not-valid-control-textbox1-accessed-from-a-thread-othe
        // Need to do the following to set the text property of MYOstatus in the UI thread
        delegate void SetTextCallback(string text);

        private void SetText(string text)
        {
            // InvokeRequired required compares the thread ID of the
            // calling thread to the thread ID of the creating thread.
            // If these threads are different, it returns true.
            if (this.MYOstatus.InvokeRequired)
            {
                SetTextCallback d = new SetTextCallback(SetText);
                this.Invoke(d, new object[] { text });
            }
            else
            {
                this.MYOstatus.Text = text;
            }
        }

        private void Myo_EmgDataAcquired(object sender, EmgDataEventArgs e)
        {
            // Sample EMG value from each sensor
            // This event handler runs on its own thread and will run as fast possible (delay is typically less than a millisecond)
            // Code for all 8 channels

            avg1.AddSample(System.Math.Abs(e.EmgData.GetDataForSensor(0)));
            avg2.AddSample(System.Math.Abs(e.EmgData.GetDataForSensor(1)));
            avg3.AddSample(System.Math.Abs(e.EmgData.GetDataForSensor(2)));
            avg4.AddSample(System.Math.Abs(e.EmgData.GetDataForSensor(3)));
            avg5.AddSample(System.Math.Abs(e.EmgData.GetDataForSensor(4)));
            avg6.AddSample(System.Math.Abs(e.EmgData.GetDataForSensor(5)));
            avg7.AddSample(System.Math.Abs(e.EmgData.GetDataForSensor(6)));
            avg8.AddSample(System.Math.Abs(e.EmgData.GetDataForSensor(7)));
            ch1_value = avg1.Average;
            ch2_value = avg2.Average;
            ch3_value = avg3.Average;
            ch4_value = avg4.Average;
            ch5_value = avg5.Average;
            ch6_value = avg6.Average;
            ch7_value = avg7.Average;
            ch8_value = avg8.Average;

            if (myoBuzzBox.Checked == true && myoBuzzFlag == true)
            {
                e.Myo.Vibrate(VibrationType.Short);
                myoBuzzFlag = false;
            }
        }

        // Function for checking whether Myo Connect is running as a background process
        public bool IsProcessOpen(string name)
        {
            foreach (Process clsProcess in Process.GetProcesses())
            {
                if (clsProcess.ProcessName.Contains(name))
                {
                    return true;
                }
            }

            return false;
        }
        #endregion

        #endregion

        #region "Keyboard Controller"

        private void KB_connect_Click(object sender, EventArgs e)
        {
            // Re-configure the GUI when the keyboard is connected
            KBgroupBox.Enabled = true;
            KBdisconnect.Enabled = true;
            KBconnect.Enabled = false;
            KBcheckRamp.Enabled = true;
            KBlabelRamp.Enabled = true;
            KBlist.Enabled = true;
            KBselectAll.Enabled = true;
            KBclearAll.Enabled = true;
        }

        private void KB_disconnect_Click(object sender, EventArgs e)
        {
            // Re-configure the GUI when the keyboard is disconnected
            KBgroupBox.Enabled = false;
            KBdisconnect.Enabled = false;
            KBconnect.Enabled = true;
            KBcheckRamp.Enabled = false;
            KBlabelRamp.Enabled = false;
            KBlist.Enabled = false;
            KBselectAll.Enabled = false;
            KBclearAll.Enabled = false;
            KBconnect.Focus();
        }

        private void KBselectAll_Click(object sender, EventArgs e)
        {
            // Select all of the items in the checkedListBox
            for (int i = 0; i < KBlist.Items.Count; i++)
            {
                KBlist.SetItemChecked(i, true);

            }
        }

        private void KBclearAll_Click(object sender, EventArgs e)
        {
            // Unselect all of the items in the checkedListBox
            for (int i = 0; i < KBlist.Items.Count; i++)
            {
                KBlist.SetItemChecked(i, false);

            }
        }

        // Disable/block keyboard inputs and navigation on the form if the keyboard input device is connected
        // This is done to prevent controls from accidentally changing on the form when users are controlling the robot with the keyboard 
        private void mainForm_KeyDown(object sender, System.Windows.Forms.KeyEventArgs e)
        {
            if (KBconnect.Enabled == false)
            {
                //e.Handled = true;
                e.SuppressKeyPress = true;
            }
        }

        #endregion

        #region "Arduino Input Device"
        private void ArduinoInputConnect_Click(object sender, EventArgs e)
        {
            try
            {
                if (ArduinoInputCOM.SelectedIndex > -1)
                {
                    // Define the settings for serial communication and open the serial port
                    // To find the portname search for 'device manager' in windows search and then look under Ports (Com & LPT)
                    serialArduinoInput.PortName = ArduinoInputCOM.SelectedItem.ToString();
                    //serialPort1.BaudRate = 9600; default arduino baud rate
                    serialArduinoInput.BaudRate = 9600; // default bluetooth baud rate
                    //serialPort1.DataBits = 8;
                    //serialPort1.Parity = Parity.None; 
                    //serialPort1.StopBits = StopBits.One;
                    //serialPort1.Handshake = Handshake.None;
                    //serialPort1.NewLine = "\r";
                    // NOTE: for serial communication to work with leonardo or micro over USB to a c# program the RTSenable property for serialport1 needs to be set to "true"
                    // ref: http://forum.arduino.cc/index.php?topic=119557.0
                    serialArduinoInput.RtsEnable = true;
                    serialArduinoInput.Open();
                    if (serialArduinoInput.IsOpen)
                    {
                        // Re-configure the GUI when the arduino is connected
                        ArduinoInputGroupBox.Enabled = true;
                        ArduinoInputDisconnect.Enabled = true;
                        ArduinoInputConnect.Enabled = false;
                        ArduinoInputList.Enabled = true;
                        ArduinoInputSelectAll.Enabled = true;
                        ArduinoInputClearAll.Enabled = true;

                        // Start timer to begin streaming (throw away first second, because it is often garbage data)
                        ArduinoStartTimer.Restart();


                    }
                }
                else
                {
                    MessageBox.Show("Please select a port first");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void ArduinoInputDisconnect_Click(object sender, EventArgs e)
        {
            try
            {
                if (serialArduinoInput.IsOpen)
                {
                    // Close the serial port
                    // need to disable RtsEnable when closing the port otherwise it slows down the Arduino Leonardo/Micro boards. Not necessary for other arduino boards
                    serialArduinoInput.RtsEnable = false;
                    serialArduinoInput.Close();

                    // Re-configure the GUI when the arduino is disconnected
                    ArduinoInputGroupBox.Enabled = false;
                    ArduinoInputDisconnect.Enabled = false;
                    ArduinoInputConnect.Enabled = true;
                    ArduinoInputList.Enabled = false;
                    ArduinoInputSelectAll.Enabled = false;
                    ArduinoInputClearAll.Enabled = false;
                    ArduinoInputConnect.Focus();

                    // Reset the timer start timer
                    ArduinoStartTimer.Reset();
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void ArduinoInputSelectAll_Click(object sender, EventArgs e)
        {
            // Select all of the items in the checkedListBox
            for (int i = 0; i < ArduinoInputList.Items.Count; i++)
            {
                ArduinoInputList.SetItemChecked(i, true);

            }
        }

        private void ArduinoInputClearAll_Click(object sender, EventArgs e)
        {
            // Unselect all of the items in the checkedListBox
            for (int i = 0; i < ArduinoInputList.Items.Count; i++)
            {
                ArduinoInputList.SetItemChecked(i, false);

            }
        }

        // Read in the serial data being sent from the arduino
        // reference: http://csharp.simpleserial.com/
        // issue with closing serial port with ReadLine: http://stackoverflow.com/questions/6452330/serialport-in-wpf-threw-i-o-exception
        private void serialArduinoInput_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                if (serialArduinoInput.IsOpen)
                {
                    RxString = serialArduinoInput.ReadLine();
                }

            }
            catch (Exception ex)
            {

            }
        }

        #endregion

        #region "Simulink Realtime Controller"

        private void SLRTconnect_Click(object sender, EventArgs e)
        {
            //try
            //{
            //    // only set filename if it has not already been set in the xPC target tab
            //    if (tg.DLMFileName == "")
            //    {
            //        //For testing on white cart at UofA
            //        //tg.DLMFileName = @"two_state_controller_8ch_PCI6259_BENTO_rev22.dlm";

            //        // For testing at Glenrose with embedeed computer
            //        tg.DLMFileName = @"two_state_controller_8ch_PC104_BENTO_rev22.dlm";
            //    }

            //    InvokeOnClick(connectButton, new EventArgs());      // Programatically click the 'Connect' button in the xPC target tab

            //    // Check whether the target has connected
            //    if (tg.IsConnected == true)
            //    {
            //        InvokeOnClick(loadButton, new EventArgs());      // Programatically click the 'Load' button in the xPC target tab
            //        InvokeOnClick(startButton, new EventArgs());      // Programatically click the 'Start' button in the xPC target tab

            //        // Enable SLRT feedback and reconfigure the GUI
            //        SLRTgroupBox.Enabled = true;
            //        SLRTdisconnect.Enabled = true;
            //        SLRTconnect.Enabled = false;
            //        SLRTlist.Enabled = true;
            //        SLRTselectAll.Enabled = true;
            //        SLRTclearAll.Enabled = true;
            //    }

            //}
            //catch (Exception ex)
            //{
            //    MessageBox.Show(ex.Message);
            //}
        }

        private void SLRTdisconnect_Click(object sender, EventArgs e)
        {
            //if (tg.IsConnected == true)
            //{
            //    InvokeOnClick(stopButton, new EventArgs());      // Programatically click the 'Stop' button in the xPC target tab
            //    InvokeOnClick(disconnectButton, new EventArgs());      // Programatically click the 'Disconnect' button in the xPC target tab

            //    // Re-configure the GUI when the simulink realtime is disconnected
            //    SLRTgroupBox.Enabled = false;
            //    SLRTdisconnect.Enabled = false;
            //    SLRTconnect.Enabled = true;
            //    SLRTlist.Enabled = false;
            //    SLRTselectAll.Enabled = false;
            //    SLRTclearAll.Enabled = false;
            //    SLRTconnect.Focus();
            //}
        }

        private void SLRTselectAll_Click(object sender, EventArgs e)
        {
            // Select all of the items in the checkedListBox
            for (int i = 0; i < SLRTlist.Items.Count; i++)
            {
                SLRTlist.SetItemChecked(i, true);
            }
        }

        private void SLRTclearAll_Click(object sender, EventArgs e)
        {
            // Unselect all of the items in the checkedListBox
            for (int i = 0; i < SLRTlist.Items.Count; i++)
            {
                SLRTlist.SetItemChecked(i, false);
            }
        }
        #endregion

        #region "Dynamixel Controller"

        // Connect to the dynamixel bus!
        private void dynaConnect_Click(object sender, EventArgs e)
        {

            // Added this code for the Task timer
            TaskTimerGroupBox.Enabled = true;
            for (int i = 0; i <= BENTO_NUM - 1; i++)
            {
                BentoSense.ID[i].pos = 2048;
                BentoSense.ID[i].vel = 1023;    // initialize velocity and load values to 1023, so that unconnected devices appear to have "0" velocity and loads in the GUI and in the surprise demo
                BentoSense.ID[i].load = 1023;
            }
            try

            {
                // DynamixelSDK - Initialize PortHandler Structs
                // Set the port path
                // Get methods and members of PortHandlerLinux or PortHandlerWindows
                port_num = dynamixel.portHandler(cmbSerialPorts.SelectedItem.ToString());

                // DynamixelSDK - Initialize PacketHandler Structs
                dynamixel.packetHandler();

                // Initialize Groupsyncwrite instance
                write_group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POS_SPEED);

                #region "Initialize syncwrite parameters"
                // Add Dynamixel#1 goal position value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL1_ID, ID1_goal_position, LEN_MX_GOAL_POSITION);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }

                // Add Dynamixel#1 moving speed value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL1_ID, ID1_moving_speed, LEN_MX_MOVING_SPEED);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }

                // Add Dynamixel#2 goal position value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL2_ID, ID2_goal_position, LEN_MX_GOAL_POSITION);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }

                // Add Dynamixel#2 moving speed value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL2_ID, ID2_moving_speed, LEN_MX_MOVING_SPEED);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }

                // Add Dynamixel#3 goal position value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL3_ID, ID3_goal_position, LEN_MX_GOAL_POSITION);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }

                // Add Dynamixel#3 moving speed value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL3_ID, ID3_moving_speed, LEN_MX_MOVING_SPEED);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }

                // Add Dynamixel#4 goal position value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL4_ID, ID4_goal_position, LEN_MX_GOAL_POSITION);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }

                // Add Dynamixel#4 moving speed value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL4_ID, ID4_moving_speed, LEN_MX_MOVING_SPEED);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }

                // Add Dynamixel#5 goal position value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL5_ID, ID5_goal_position, LEN_MX_GOAL_POSITION);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }

                // Add Dynamixel#5 moving speed value to the Syncwrite storage
                dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL5_ID, ID5_moving_speed, LEN_MX_MOVING_SPEED);
                if (dxl_addparam_result != true)
                {
                    MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                    return;
                }
                #endregion

                // Initialize Groupbulkread Structs
                read_group_num = dynamixel.groupBulkRead(port_num, PROTOCOL_VERSION);

                if (cmbSerialPorts.SelectedIndex > -1)
                {
                    // MessageBox.Show(String.Format("You selected port '{0}'", cmbSerialPorts.SelectedItem));
                    // Define the settings for serial communication and open the serial port
                    // To find the portname search for 'device manager' in windows search and then look under Ports (Com & LPT)

                    try
                    {
                        port_num = dynamixel.portHandler(cmbSerialPorts.SelectedItem.ToString());
                    }
                    catch (InvalidCastException ex)
                    {
                        MessageBox.Show(ex.Message);
                    }

                    // Open port
                    if (dynamixel.openPort(port_num))
                    {
                        dynaStatus.Text = String.Format("Port {0} opened successfully", cmbSerialPorts.SelectedItem.ToString());
                    }
                    else
                    {
                        dynaStatus.Text = String.Format("Failed to open port {0}", cmbSerialPorts.SelectedItem.ToString());
                        return;
                    }
                }
                else
                {
                    dynaStatus.Text = "Please select a port first";
                }

                // Set port baudrate
                if (dynamixel.setBaudRate(port_num, BAUDRATE))
                {
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Changed baudrate to {0} bps", BAUDRATE);
                }
                else
                {
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + "Failed to change the baudrate";
                    return;
                }

                // Ping each dynamixel that is supposed to be on the bus

                // ID1
                dynamixel.ping(port_num, PROTOCOL_VERSION, DXL1_ID);
                if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                {
                    dynaCommResult.Text = Convert.ToString(dxl_comm_result);
                    if (Convert.ToString(dxl_comm_result) == "-1001")
                    {
                        dynaStatus.Text = dynaStatus.Text + Environment.NewLine + "Incorrect COM port selected";
                    }
                    else if (Convert.ToString(dxl_comm_result) == "-3001")
                    {
                        ID1_connected = 0;
                        dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is not connected", DXL1_ID);
                    }
                    else if (Convert.ToString(dxl_comm_result) == "-3002")
                    {
                        ID1_connected = 0;
                        dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Incorrect COM port selected", DXL1_ID);
                    }
                }
                else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
                {
                    dynaError.Text = Convert.ToString(dxl_error);

                }
                else
                {
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL1_ID);

                    // Add parameter storage for Dynamixel#1 feedback: position, speed, load, voltage, temperature
                    dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL1_ID, ADDR_MX_TORQUE_LIMIT, read_length);
                    if (dxl_addparam_result != true)
                    {
                        dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("ID{0} groupBulkRead addparam failed", DXL1_ID);
                        return;
                    }

                    ID1_connected = 1;
                }

                // ID2
                dynamixel.ping(port_num, PROTOCOL_VERSION, DXL2_ID);
                if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                {
                    dynaCommResult.Text = Convert.ToString(dxl_comm_result);
                    ID2_connected = 0;
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is not connected", DXL2_ID);
                }
                else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
                {
                    dynaError.Text = Convert.ToString(dxl_error);
                }
                else
                {
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL2_ID);

                    // Add parameter storage for Dynamixel#2 feedback: position, speed, load, voltage, temperature
                    dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL2_ID, ADDR_MX_TORQUE_LIMIT, read_length);
                    if (dxl_addparam_result != true)
                    {
                        dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("ID{0} groupBulkRead addparam failed", DXL2_ID);
                        return;
                    }

                    ID2_connected = 1;
                }

                // ID3
                dynamixel.ping(port_num, PROTOCOL_VERSION, DXL3_ID);
                if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                {
                    dynaCommResult.Text = Convert.ToString(dxl_comm_result);
                    ID3_connected = 0;
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is not connected", DXL3_ID);
                }
                else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
                {
                    dynaError.Text = Convert.ToString(dxl_error);

                }
                else
                {
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL3_ID);

                    // Add parameter storage for Dynamixel#3 feedback: position, speed, load, voltage, temperature
                    dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL3_ID, ADDR_MX_TORQUE_LIMIT, read_length);
                    if (dxl_addparam_result != true)
                    {
                        dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("ID{0} groupBulkRead addparam failed", DXL3_ID);
                        return;
                    }

                    ID3_connected = 1;
                }

                // ID4
                dynamixel.ping(port_num, PROTOCOL_VERSION, DXL4_ID);
                if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                {
                    dynaCommResult.Text = Convert.ToString(dxl_comm_result);
                    ID4_connected = 0;
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is not connected", DXL4_ID);
                }
                else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
                {
                    dynaError.Text = Convert.ToString(dxl_error);

                }
                else
                {
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL4_ID);

                    // Add parameter storage for Dynamixel#4 feedback: position, speed, load, voltage, temperature
                    dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL4_ID, ADDR_MX_TORQUE_LIMIT, read_length);
                    if (dxl_addparam_result != true)
                    {
                        //MessageBox.Show("[ID: {0}] groupBulkRead addparam failed");
                        dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("ID{0} groupBulkRead addparam failed", DXL4_ID);
                        return;
                    }
                    ID4_connected = 1;
                }

                // ID5
                dynamixel.ping(port_num, PROTOCOL_VERSION, DXL5_ID);
                if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                {
                    dynaCommResult.Text = Convert.ToString(dxl_comm_result);
                    ID5_connected = 0;
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is not connected", DXL5_ID);
                }
                else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
                {
                    dynaError.Text = Convert.ToString(dxl_error);

                }
                else
                {
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL5_ID);

                    // Add parameter storage for Dynamixel#5 feedback: position, speed, load, voltage, temperature
                    dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL5_ID, ADDR_MX_TORQUE_LIMIT, read_length);
                    if (dxl_addparam_result != true)
                    {
                        dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("ID{0} groupBulkRead addparam failed", DXL5_ID);
                        return;
                    }

                    ID5_connected = 1;
                }

                // Enable/disable relevant controls if at least one servo connected properly
                if (ID1_connected == 1 || ID2_connected == 1 || ID3_connected == 1 || ID4_connected == 1 || ID5_connected == 1)
                {
                    dynaConnect.Enabled = false;
                    dynaDisconnect.Enabled = true;
                    BentoGroupBox.Enabled = true;
                    TorqueOn.Enabled = true;
                    TorqueOff.Enabled = false;
                    RobotParamBox.Enabled = true;
                    RobotFeedbackBox.Enabled = true;
                    BentoEnvLimitsBox.Enabled = true;
                    BentoAdaptGripBox.Enabled = true;
                    BentoStatus.Text = "Connected / Torque Off";
                    BentoList.Enabled = true;
                    BentoSelectAll.Enabled = true;
                    BentoClearAll.Enabled = true;
                    dynaDisconnect.Focus();

                }
                else
                {
                    // Close port
                    dynamixel.closePort(port_num);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void dynaDisconnect_Click(object sender, EventArgs e)
        {

            // Added this code for the Task timer
            TaskTimerGroupBox.Enabled = false;

            // Re-configure the GUI when the disconnecting from the dynamixel bus

            // Reset the connection state for each motor
            ID1_connected = 0;
            ID2_connected = 0;
            ID3_connected = 0;
            ID4_connected = 0;
            ID5_connected = 0;

            // Enable/disable relevant controls
            dynaConnect.Enabled = true;
            dynaDisconnect.Enabled = false;
            BentoGroupBox.Enabled = false;
            TorqueOn.Enabled = true;
            TorqueOff.Enabled = false;
            moveCW.Enabled = false;
            moveCCW.Enabled = false;
            RobotParamBox.Enabled = false;
            RobotFeedbackBox.Enabled = false;
            BentoEnvLimitsBox.Enabled = false;
            BentoAdaptGripBox.Enabled = false;
            BentoStatus.Text = "Disconnected";
            BentoRunStatus.Text = "Suspend";
            BentoRunStatus.Enabled = false;
            BentoList.Enabled = false;
            BentoSelectAll.Enabled = false;
            BentoClearAll.Enabled = false;
            dynaConnect.Focus();

            // Close port
            dynamixel.closePort(port_num);

            // Update status text
            dynaStatus.Text = "Disconnected";
        }

        private void BentoSelectAll_Click(object sender, EventArgs e)
        {
            // Select all of the items in the checkedListBox
            for (int i = 0; i < BentoList.Items.Count; i++)
            {
                BentoList.SetItemChecked(i, true);

            }
        }

        private void BentoClearAll_Click(object sender, EventArgs e)
        {
            // Unselect all of the items in the checkedListBox
            for (int i = 0; i < BentoList.Items.Count; i++)
            {
                BentoList.SetItemChecked(i, false);

            }
        }

        private void TorqueOn_Click(object sender, EventArgs e)
        {
            // Read feedback values back from the motors
            readDyna();

            // initialize dynamixel positions when first turning on the torque
            robotObj.Motor[0].p = robotObj.Motor[0].p_prev;
            robotObj.Motor[1].p = robotObj.Motor[1].p_prev;
            robotObj.Motor[2].p = robotObj.Motor[2].p_prev;
            robotObj.Motor[3].p = robotObj.Motor[3].p_prev;
            robotObj.Motor[4].p = robotObj.Motor[4].p_prev;

            // initialize dynamixel velocities when first turning on the torque
            // choose a relatively slow value in case the servo is outside of its normal range it will slowly move to the closest end limit
            robotObj.Motor[0].w = 5;
            robotObj.Motor[1].w = 5;
            robotObj.Motor[2].w = 5;
            robotObj.Motor[3].w = 5;
            robotObj.Motor[4].w = 5;

            // Enable Dynamixel Torque
            // ID1
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }
            // ID2
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }
            // ID3
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }
            // ID4
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }
            // ID5
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }

            // Enable/disable relevant controls
            TorqueOn.Enabled = false;
            TorqueOff.Enabled = true;
            moveCW.Enabled = true;
            moveCCW.Enabled = true;
            TorqueOff.Focus();
            bentoSuspend = false;
            BentoRun.Enabled = false;
            BentoSuspend.Enabled = true;
            BentoStatus.Text = "Torque On / Running";
            BentoRunStatus.Enabled = true;

        }

        private void TorqueOff_Click(object sender, EventArgs e)
        {
            // Disable Dynamixel Torque
            // ID1
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }
            // ID2
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }
            // ID3
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }
            // ID4
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }
            // ID5
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynaError.Text = Convert.ToString(dxl_error);
            }

            // Enable/disable relevant controls
            TorqueOn.Enabled = true;
            TorqueOff.Enabled = false;
            moveCW.Enabled = false;
            moveCCW.Enabled = false;
            TorqueOn.Focus();
            BentoRun.Enabled = false;
            BentoSuspend.Enabled = false;
            bentoSuspend = true;
            BentoStatus.Text = "Connected / Torque Off";
            BentoRunStatus.Text = "Suspend";
            BentoRunStatus.Enabled = false;
        }

        private void BentoRun_Click(object sender, EventArgs e)
        {
            // Connect inputs to the bento arm. This happens by default when the torque is turned on
            if (TorqueOn.Enabled == false)
            {
                bentoSuspend = false;
                BentoRun.Enabled = false;
                BentoSuspend.Enabled = true;
                BentoStatus.Text = "Torque On / Running";
                BentoRunStatus.Text = "Suspend";
            }
        }

        private void BentoSuspend_Click(object sender, EventArgs e)
        {
            // Disconnect inputs from the bento arm
            bentoSuspend = true;

            if ((TorqueOn.Enabled == false && BentoGroupBox.Enabled == true) || (HANDiGroupBox.Enabled == true))
            {


                BentoRun.Enabled = true;
                BentoSuspend.Enabled = false;
                BentoStatus.Text = "Torque On / Suspended";
                BentoRunStatus.Text = "Run";

                // Stop the arm and hold torque
                for (int k = 0; k < BENTO_NUM; k++)
                {
                    robotObj.Motor[k].p = robotObj.Motor[k].p_prev;
                    robotObj.Motor[k].w = robotObj.Motor[k].wmax;
                    stateObj.motorState[k] = 0;
                }
            }

        }

        private void BentoRunStatus_Click(object sender, EventArgs e)
        {
            if (BentoRunStatus.Text == "Run")
            {
                // Programatically click the 'BentoRun' button
                BentoRun_Click(sender, e);
            }
            else if (BentoRunStatus.Text == "Suspend")
            {
                // Programatically click the 'BentoSuspend' button
                BentoSuspend_Click(sender, e);
            }
        }

        #region "Robotic Arm - Parameters"
            // Control the robotic arm parameter values using the tagged block parameters from the xPC Target model
            #region "Shoulder"
            private void shoulder_pmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void shoulder_pmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void shoulder_wmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void shoulder_wmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }
            #endregion
            #region "Elbow"
            private void elbow_pmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void elbow_pmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void elbow_wmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void elbow_wmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }
            #endregion
            #region "Wrist Rotate"
            private void wristRot_pmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void wristRot_pmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void wristRot_wmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void wristRot_wmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }
            #endregion
            #region "Wrist Flex"
            private void wristFlex_pmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void wristFlex_pmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void wristFlex_wmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void wristFlex_wmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }
            #endregion
            #region "Hand"
            private void hand_pmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void hand_pmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void hand_wmin_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            private void hand_wmax_ctrl_ValueChanged(object sender, EventArgs e)
            {
                // Auto-suspend the Bento Arm as soon as the control enters focus
                InvokeOnClick(BentoSuspend, new EventArgs());
            }
            #endregion
            #endregion

        private void writeDyna()
        {
            // Write goal position to each motor on the dynamixel bus

            ushort pos1 = Convert.ToUInt16(robotObj.Motor[0].p);
            ushort vel1 = Convert.ToUInt16(robotObj.Motor[0].w);
            ushort pos2 = Convert.ToUInt16(robotObj.Motor[1].p);
            ushort vel2 = Convert.ToUInt16(robotObj.Motor[1].w);
            ushort pos3 = Convert.ToUInt16(robotObj.Motor[2].p);
            ushort vel3 = Convert.ToUInt16(robotObj.Motor[2].w);
            ushort pos4 = Convert.ToUInt16(robotObj.Motor[3].p);
            ushort vel4 = Convert.ToUInt16(robotObj.Motor[3].w);
            ushort pos5 = Convert.ToUInt16(robotObj.Motor[4].p);
            ushort vel5 = Convert.ToUInt16(robotObj.Motor[4].w);

            // Add Dynamixel#1 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL1_ID, pos1, LEN_MX_GOAL_POSITION, 0);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Add Dynamixel#1 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL1_ID, vel1, LEN_MX_MOVING_SPEED, 2);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Add Dynamixel#2 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL2_ID, pos2, LEN_MX_GOAL_POSITION, 0);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Add Dynamixel#2 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL2_ID, vel2, LEN_MX_MOVING_SPEED, 2);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Add Dynamixel#3 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL3_ID, pos3, LEN_MX_GOAL_POSITION, 0);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Add Dynamixel#3 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL3_ID, vel3, LEN_MX_MOVING_SPEED, 2);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Add Dynamixel#4 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL4_ID, pos4, LEN_MX_GOAL_POSITION, 0);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Add Dynamixel#4 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL4_ID, vel4, LEN_MX_MOVING_SPEED, 2);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Add Dynamixel#5 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL5_ID, pos5, LEN_MX_GOAL_POSITION, 0);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Add Dynamixel#5 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteChangeParam(write_group_num, DXL5_ID, vel5, LEN_MX_MOVING_SPEED, 2);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite changeparam failed");
                return;
            }

            // Syncwrite goal position
            dynamixel.groupSyncWriteTxPacket(write_group_num);

            //// Clear syncwrite parameter storage
            //dynamixel.groupSyncWriteClearParam(write_group_num);

        }

        private void moveCW_Click(object sender, EventArgs e)
        {
            // Old testing button. Not currently in use.
            ID5_moving_speed = 50;
            ID5_goal_position = 2000;
        }

        private void moveCCW_Click(object sender, EventArgs e)
        {
            // Old testing button. Not currently in use.
            ID5_moving_speed = 50;
            ID5_goal_position = 2700;
        }

        private void LEDon_Click(object sender, EventArgs e)
        {
            // Old testing button. Not currently in use.
            // Turn on LED
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_LED, LED_ON);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
            }
        }

        private void LEDoff_Click(object sender, EventArgs e)
        {
            // Old testing button. Not currently in use.
            // Turn on LED
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_LED, LED_OFF);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
            }
        }

        private void readDyna()
        {
            // Read feedback from each dynamixel motor
            // Bulkread present position and moving status
            dynamixel.groupBulkReadTxRxPacket(read_group_num);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);

            dxl_getdata_result = dynamixel.groupBulkReadIsAvailable(read_group_num, DXL1_ID, ADDR_MX_TORQUE_LIMIT, read_length);
            if (dxl_getdata_result != true)
            {
                MessageBox.Show("[ID: {0}] groupBulkRead getdata failed");
            }

            // Get Dynamixel#1 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive

            if (ID1_connected == 1)
            {
                BentoSense.ID[0].pos = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION); ;
                BentoSense.ID[0].vel = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)) + 1023);
                BentoSense.ID[0].load = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)) + 1023);
                BentoSense.ID[0].volt = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE);
                BentoSense.ID[0].temp = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP);

                ID1_present_position = BentoSense.ID[0].pos;
                robotObj.Motor[0].p_prev = ID1_present_position;

                // Filter the position and load values using a filter
                //ID1_pos_avg.AddSample(BentoSense.ID[0].pos);
                //ID1_load_avg.AddSample(BentoSense.ID[0].load);
                //BentoSense.ID[0].posf = (ushort)ID1_pos_avg.Average;
                //BentoSense.ID[0].loadf = (ushort)ID1_load_avg.Average;
                //BentoSense.ID[0].posf = FilterSensor(BentoSense.ID[0].pos, BentoSense.ID[0].posf, BentoSense.ID[0].vel);
                BentoSense.ID[0].posf = FilterSensor2(BentoSense.ID[0].pos, BentoSense.ID[0].posf, 2);
                BentoSense.ID[0].loadf = FilterSensor2(BentoSense.ID[0].load, BentoSense.ID[0].loadf, 9);
                BentoSense.ID[0].tempf = FilterSensor2(BentoSense.ID[0].temp, BentoSense.ID[0].tempf, 2);

                Pos1.Text = Convert.ToString(BentoSense.ID[0].posf);
                Vel1.Text = Convert.ToString(BentoSense.ID[0].vel - 1023);
                Load1.Text = Convert.ToString(BentoSense.ID[0].loadf - 1023);
                Volt1.Text = Convert.ToString(BentoSense.ID[0].volt / 10);
                Temp1.Text = Convert.ToString(BentoSense.ID[0].tempf);
                check_overheat(DXL1_ID, BentoSense.ID[0].temp);
                check_overload(DXL1_ID, (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_TORQUE_LIMIT, LEN_MX_TORQUE_LIMIT));
            }

            // Get Dynamixel#2 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive
            if (ID2_connected == 1)
            {
                BentoSense.ID[1].pos = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION); ;
                BentoSense.ID[1].vel = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)) + 1023);
                BentoSense.ID[1].load = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)) + 1023);
                BentoSense.ID[1].volt = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE);
                BentoSense.ID[1].temp = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP);

                ID2_present_position = BentoSense.ID[1].pos;
                robotObj.Motor[1].p_prev = ID2_present_position;

                // Filter the position and load values using a filter
                //ID2_pos_avg.AddSample(BentoSense.ID[1].pos);
                //ID2_load_avg.AddSample(BentoSense.ID[1].load);
                //BentoSense.ID[1].posf = (ushort)ID2_pos_avg.Average;
                //BentoSense.ID[1].loadf = (ushort)ID2_load_avg.Average;
                BentoSense.ID[1].posf = FilterSensor2(BentoSense.ID[1].pos, BentoSense.ID[1].posf, 2);
                BentoSense.ID[1].loadf = FilterSensor2(BentoSense.ID[1].load, BentoSense.ID[1].loadf, 9);
                BentoSense.ID[1].tempf = FilterSensor2(BentoSense.ID[1].temp, BentoSense.ID[1].tempf, 2);

                Pos2.Text = Convert.ToString(BentoSense.ID[1].posf);
                Vel2.Text = Convert.ToString(BentoSense.ID[1].vel - 1023);
                Load2.Text = Convert.ToString(BentoSense.ID[1].loadf - 1023);
                Volt2.Text = Convert.ToString(BentoSense.ID[1].volt / 10);
                Temp2.Text = Convert.ToString(BentoSense.ID[1].tempf);
                check_overheat(DXL2_ID, BentoSense.ID[1].temp);
                check_overload(DXL2_ID, (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_TORQUE_LIMIT, LEN_MX_TORQUE_LIMIT));
            }

            // Get Dynamixel#3 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive
            if (ID3_connected == 1)
            {
                BentoSense.ID[2].pos = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION); ;
                BentoSense.ID[2].vel = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)) + 1023);
                BentoSense.ID[2].load = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)) + 1023);
                BentoSense.ID[2].volt = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE);
                BentoSense.ID[2].temp = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP);

                ID3_present_position = BentoSense.ID[2].pos;
                robotObj.Motor[2].p_prev = ID3_present_position;

                // Filter the position and load values using a filter
                //ID3_pos_avg.AddSample(BentoSense.ID[2].pos);
                //ID3_load_avg.AddSample(BentoSense.ID[2].load);
                //BentoSense.ID[2].posf = (ushort)ID3_pos_avg.Average;
                //BentoSense.ID[2].loadf = (ushort)ID3_load_avg.Average;
                BentoSense.ID[2].posf = FilterSensor2(BentoSense.ID[2].pos, BentoSense.ID[2].posf, 2);
                BentoSense.ID[2].loadf = FilterSensor2(BentoSense.ID[2].load, BentoSense.ID[2].loadf, 9);
                BentoSense.ID[2].tempf = FilterSensor2(BentoSense.ID[2].temp, BentoSense.ID[2].tempf, 2);

                Pos3.Text = Convert.ToString(BentoSense.ID[2].posf);
                Vel3.Text = Convert.ToString(BentoSense.ID[2].vel - 1023);
                Load3.Text = Convert.ToString(BentoSense.ID[2].loadf - 1023);
                Volt3.Text = Convert.ToString(BentoSense.ID[2].volt / 10);
                Temp3.Text = Convert.ToString(BentoSense.ID[2].tempf);
                check_overheat(DXL3_ID, BentoSense.ID[2].temp);
                check_overload(DXL3_ID, (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_TORQUE_LIMIT, LEN_MX_TORQUE_LIMIT));
            }

            // Get Dynamixel#4 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive
            if (ID4_connected == 1)
            {
                BentoSense.ID[3].pos = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION); ;
                BentoSense.ID[3].vel = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)) + 1023);
                BentoSense.ID[3].load = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)) + 1023);
                BentoSense.ID[3].volt = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE);
                BentoSense.ID[3].temp = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP);

                ID4_present_position = BentoSense.ID[3].pos;
                robotObj.Motor[3].p_prev = ID4_present_position;

                // Filter the position and load values using a filter
                //ID4_pos_avg.AddSample(BentoSense.ID[3].pos);
                //ID4_load_avg.AddSample(BentoSense.ID[3].load);
                //BentoSense.ID[3].posf = (ushort)ID4_pos_avg.Average;
                //BentoSense.ID[3].loadf = (ushort)ID4_load_avg.Average;
                BentoSense.ID[3].posf = FilterSensor2(BentoSense.ID[3].pos, BentoSense.ID[3].posf, 2);
                BentoSense.ID[3].loadf = FilterSensor2(BentoSense.ID[3].load, BentoSense.ID[3].loadf, 9);
                BentoSense.ID[3].tempf = FilterSensor2(BentoSense.ID[3].temp, BentoSense.ID[3].tempf, 2);

                Pos4.Text = Convert.ToString(BentoSense.ID[3].posf);
                Vel4.Text = Convert.ToString(BentoSense.ID[3].vel - 1023);
                Load4.Text = Convert.ToString(BentoSense.ID[3].loadf - 1023);
                Volt4.Text = Convert.ToString(BentoSense.ID[3].volt / 10);
                Temp4.Text = Convert.ToString(BentoSense.ID[3].tempf);
                check_overheat(DXL4_ID, BentoSense.ID[3].temp);
                check_overload(DXL4_ID, (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_TORQUE_LIMIT, LEN_MX_TORQUE_LIMIT));
            }
            // Get Dynamixel#5 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive
            if (ID5_connected == 1)
            {
                BentoSense.ID[4].pos = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
                BentoSense.ID[4].vel = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)) + 1023);
                BentoSense.ID[4].load = (ushort)(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)) + 1023);
                BentoSense.ID[4].volt = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE);
                BentoSense.ID[4].temp = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP);

                ID5_present_load = BentoSense.ID[4].load - 1023;
                ID5_present_position = BentoSense.ID[4].pos;
                robotObj.Motor[4].p_prev = ID5_present_position;

                // Filter the position and load values using a filter
                //ID5_pos_avg.AddSample(BentoSense.ID[4].pos);
                //ID5_load_avg.AddSample(BentoSense.ID[4].load);
                //BentoSense.ID[4].posf = (ushort)ID5_pos_avg.Average;
                //BentoSense.ID[4].loadf = (ushort)ID5_load_avg.Average;
                BentoSense.ID[4].posf = FilterSensor2(BentoSense.ID[4].pos, BentoSense.ID[4].posf, 2);
                BentoSense.ID[4].loadf = FilterSensor2(BentoSense.ID[4].load, BentoSense.ID[4].loadf, 9);
                BentoSense.ID[4].tempf = FilterSensor2(BentoSense.ID[4].temp, BentoSense.ID[4].tempf, 2);

                Pos5.Text = Convert.ToString(BentoSense.ID[4].posf);
                Vel5.Text = Convert.ToString(BentoSense.ID[4].vel - 1023);
                Load5.Text = Convert.ToString(BentoSense.ID[4].loadf - 1023);
                Volt5.Text = Convert.ToString(BentoSense.ID[4].volt / 10);
                Temp5.Text = Convert.ToString(BentoSense.ID[4].tempf);
                check_overheat(DXL5_ID, BentoSense.ID[4].temp);
                check_overload(DXL5_ID, (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_TORQUE_LIMIT, LEN_MX_TORQUE_LIMIT));

            }

            if (BentoOverloadError == 0 && BentoOverheatError == 0)
            {
                BentoErrorColor.Text = "";
                BentoErrorText.Text = "";
            }
        }

        // Check whether a dynamixel servo has overloaded
        private void check_overload(int ID, UInt16 torque_limit)
        {
            if (torque_limit == 0)
            {
                BentoErrorColor.Text = "Error - ";
                BentoErrorColor.ForeColor = Color.Firebrick;
                BentoErrorText.Text = String.Format("ID{0} is overloaded", ID);
                BentoOverloadError = 2;
            }
            else
            {
                BentoOverloadError = 0;
            }
        }

        // Check whether a dynamixel servo is overheating or has overheated
        private void check_overheat(int ID, UInt16 temp)
        {
            if (temp >= 80)
            {
                BentoErrorColor.Text = "Error - ";
                BentoErrorColor.ForeColor = Color.Firebrick;
                BentoErrorText.Text = String.Format("ID{0} is overheated", ID);
                BentoOverheatError = 2;
            }
            else if (temp >= 65)
            {
                BentoErrorColor.Text = "Warning - ";
                BentoErrorColor.ForeColor = Color.DarkGoldenrod;
                BentoErrorText.Text = String.Format("ID{0} is overheating", ID);
                BentoOverheatError = 1;
            }
            else
            {
                BentoOverheatError = 0;
            }
        }

        // Filter that only changes the value of the sensor reading if the velocity is not zero (i.e. zero velocity is vel = 2013)
        private ushort FilterSensor(ushort raw, ushort filt, ushort vel)
        {
            if (vel == 1023)
            {
                return filt;
            }
            else
            {
                filt = raw;
                return raw;
            }
        }

        // Filter that only changes the value of the sensor reading if it changes by more than 2 values
        private ushort FilterSensor2(ushort raw, ushort filt, ushort tolerance)
        {
            if (Math.Abs(raw - filt) <= tolerance)
            {
                return filt;
            }
            else
            {
                filt = raw;
                return raw;
            }
        }

        private void readFeedback_Click(object sender, EventArgs e)
        {
            // Old testing button. Not currently in use.
            readDyna();
        }


        private int parse_load(UInt16 value)
        {
            if (value > 1023)
            {
                return 1023 - value;
            }
            else if (value < 1023)
            {
                return value;
            }
            else
            {
                return 0;
            }
        }

        private void cmbSerialRefresh_Click(object sender, EventArgs e)
        {
            // Refresh the com ports
            // How to find com ports and populate combobox: http://stackoverflow.com/questions/13794376/combo-box-for-serial-port
            var ports = SerialPort.GetPortNames();
            cmbSerialPorts.DataSource = ports;
            cmbSerialPorts.SelectedIndex = cmbSerialPorts.Items.Count - 1;
        }

        private string Autodetect_Dyna_Port()
        {
            // How to auto detect com ports with unique descriptions:http://stackoverflow.com/questions/3293889/how-to-auto-detect-arduino-com-port
            // Different selectquery term for USB Serial Port - http://stackoverflow.com/questions/11458835/finding-information-about-all-serial-devices-connected-through-usb-in-c-sharp
            ManagementScope connectionScope = new ManagementScope();
            SelectQuery serialQuery = new SelectQuery("SELECT * FROM Win32_PnPEntity WHERE ClassGuid=\"{4d36e978-e325-11ce-bfc1-08002be10318}\"");
            ManagementObjectSearcher searcher = new ManagementObjectSearcher(connectionScope, serialQuery);

            try
            {
                foreach (ManagementObject item in searcher.Get())
                {
                    string desc = item["Description"].ToString();
                    string name = item["Name"].ToString();
                    // How to extract text that lies between round brackets: http://stackoverflow.com/questions/378415/how-do-i-extract-text-that-lies-between-parentheses-round-brackets
                    string extracted_com_port = name.Split('(', ')')[1];

                    if (desc.Contains("USB Serial Port"))
                    {
                        return extracted_com_port;
                    }
                }
            }
            catch (ManagementException ex)
            {
                MessageBox.Show(ex.Message);
            }

            return null;
        }

        #region "Bento Joint Limit Profiles"
        private void BentoProfileSave_Click(object sender, EventArgs e)
        {
            try
            {
                if (BentoProfileBox.SelectedIndex > -1)
                {
                    String saveString = String.Format(@"Resources\Profiles\BentoProfiles\profile{0}.dat", BentoProfileBox.SelectedIndex);
                    SaveParameters2(saveString);
                    BentoProfileBox.Focus();
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void BentoProfileOpen_Click(object sender, EventArgs e)
        {
            try
            {
                if (BentoProfileBox.SelectedIndex > -1)
                {
                    String loadString = String.Format(@"Resources\Profiles\BentoProfiles\profile{0}.dat", BentoProfileBox.SelectedIndex);
                    LoadParameters2(loadString);
                    BentoProfileBox.Focus();
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        public void SaveParameters2(string strFileName)
        {
            //Save parameters to selected profile
            //Set the output variables to the controls in the GUI

            //Create a filestream object
            FileStream fsOutput = new FileStream(strFileName, FileMode.Create, FileAccess.Write, FileShare.None);

            //Create a binary writer object
            BinaryWriter bwOutput = new BinaryWriter(fsOutput);

            //Write the profile settings
            bwOutput.Write(shoulder_pmin_ctrl.Value);
            bwOutput.Write(shoulder_pmax_ctrl.Value);
            bwOutput.Write(shoulder_wmin_ctrl.Value);
            bwOutput.Write(shoulder_wmax_ctrl.Value);

            bwOutput.Write(elbow_pmin_ctrl.Value);
            bwOutput.Write(elbow_pmax_ctrl.Value);
            bwOutput.Write(elbow_wmin_ctrl.Value);
            bwOutput.Write(elbow_wmax_ctrl.Value);

            bwOutput.Write(wristRot_pmin_ctrl.Value);
            bwOutput.Write(wristRot_pmax_ctrl.Value);
            bwOutput.Write(wristRot_wmin_ctrl.Value);
            bwOutput.Write(wristRot_wmax_ctrl.Value);

            bwOutput.Write(wristFlex_pmin_ctrl.Value);
            bwOutput.Write(wristFlex_pmax_ctrl.Value);
            bwOutput.Write(wristFlex_wmin_ctrl.Value);
            bwOutput.Write(wristFlex_wmax_ctrl.Value);

            bwOutput.Write(hand_pmin_ctrl.Value);
            bwOutput.Write(hand_pmax_ctrl.Value);
            bwOutput.Write(hand_wmin_ctrl.Value);
            bwOutput.Write(hand_wmax_ctrl.Value);

            bwOutput.Write(BentoAdaptGripCheck.Checked);
            bwOutput.Write(BentoAdaptGripCtrl.Value);

            //Close and dispose of file writing objects
            bwOutput.Close();
            fsOutput.Close();
            fsOutput.Dispose();

        }

        public void LoadParameters2(string strFileName)
        {
            //Open parameters from selected profile
            //Create a filestream object
            FileStream fsInput = new FileStream(strFileName, FileMode.Open, FileAccess.Read, FileShare.None);

            //Create a binary reader object
            BinaryReader brInput = new BinaryReader(fsInput);

            //Read the profile settings from the input stream
            shoulder_pmin_ctrl.Value = brInput.ReadDecimal();
            shoulder_pmax_ctrl.Value = brInput.ReadDecimal();
            shoulder_wmin_ctrl.Value = brInput.ReadDecimal();
            shoulder_wmax_ctrl.Value = brInput.ReadDecimal();

            elbow_pmin_ctrl.Value = brInput.ReadDecimal();
            elbow_pmax_ctrl.Value = brInput.ReadDecimal();
            elbow_wmin_ctrl.Value = brInput.ReadDecimal();
            elbow_wmax_ctrl.Value = brInput.ReadDecimal();

            wristRot_pmin_ctrl.Value = brInput.ReadDecimal();
            wristRot_pmax_ctrl.Value = brInput.ReadDecimal();
            wristRot_wmin_ctrl.Value = brInput.ReadDecimal();
            wristRot_wmax_ctrl.Value = brInput.ReadDecimal();

            wristFlex_pmin_ctrl.Value = brInput.ReadDecimal();
            wristFlex_pmax_ctrl.Value = brInput.ReadDecimal();
            wristFlex_wmin_ctrl.Value = brInput.ReadDecimal();
            wristFlex_wmax_ctrl.Value = brInput.ReadDecimal();

            hand_pmin_ctrl.Value = brInput.ReadDecimal();
            hand_pmax_ctrl.Value = brInput.ReadDecimal();
            hand_wmin_ctrl.Value = brInput.ReadDecimal();
            hand_wmax_ctrl.Value = brInput.ReadDecimal();

            BentoAdaptGripCheck.Checked = brInput.ReadBoolean();
            BentoAdaptGripCtrl.Value = brInput.ReadDecimal();

            //Close and dispose of file writing objects
            brInput.Close();
            fsInput.Close();
            fsInput.Dispose();
        }

        #endregion

        #endregion

        #region "HANDi Hand"

        private void HANDiConnect_Click(object sender, EventArgs e)
        {
            try
            {
                if (HANDiCOM.SelectedIndex > -1)
                {
                    // Define the settings for serial communication and open the serial port
                    // To find the portname search for 'device manager' in windows search and then look under Ports (Com & LPT)
                    serialHANDi.PortName = HANDiCOM.SelectedItem.ToString();
                    //serialPort1.BaudRate = 9600; default arduino baud rate
                    serialHANDi.BaudRate = 9600; // default bluetooth baud rate
                    //serialPort1.DataBits = 8;
                    //serialPort1.Parity = Parity.None; 
                    //serialPort1.StopBits = StopBits.One;
                    //serialPort1.Handshake = Handshake.None;
                    //serialPort1.NewLine = "\r";
                    // NOTE: for serial communication to work with leonardo or micro over USB to a c# program the RTSenable property for serialport1 needs to be set to "true"
                    // ref: http://forum.arduino.cc/index.php?topic=119557.0
                    serialHANDi.RtsEnable = true;
                    serialHANDi.Open();
                    if (serialHANDi.IsOpen)
                    {
                        // Re-configure the GUI when the arduino is connected
                        HANDiDisconnect.Enabled = true;
                        HANDiConnect.Enabled = false;
                        HANDiList.Enabled = true;
                        HANDiSelectAll.Enabled = true;
                        HANDiClearAll.Enabled = true;
                        HANDiGroupBox.Enabled = true;
                        HANDiParamBox.Enabled = true;
                        HANDiFeedbackBox.Enabled = true;

                        // Programatically click the 'BentoRun' button
                        //BentoRun.Enabled = true;
                        //BentoSuspend.Enabled = false;
                        HANDiRun.Enabled = false;
                        HANDiSuspend.Enabled = true;
                        BentoRun_Click(sender, e);
                        BentoRunStatus.Enabled = true;

                        // Start timer to begin streaming (throw away first second, because it is often garbage data)
                        HANDiStartTimer.Restart();

                        // Start the thread that will send the serial packets out to the arduino
                        t3 = new Thread(DoWork3);
                        t3.Start();
                    }
                }
                else
                {
                    MessageBox.Show("Please select a port first");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void HANDiDisconnect_Click(object sender, EventArgs e)
        {
            try
            {
                if (serialHANDi.IsOpen)
                {
                    // Close the serial port
                    // need to disable RtsEnable when closing the port otherwise it slows down the Arduino Leonardo/Micro boards. Not necessary for other arduino boards
                    serialHANDi.RtsEnable = false;
                    serialHANDi.Close();

                    // Re-configure the GUI when the arduino is disconnected
                    HANDiDisconnect.Enabled = false;
                    HANDiConnect.Enabled = true;
                    HANDiList.Enabled = false;
                    HANDiSelectAll.Enabled = false;
                    HANDiClearAll.Enabled = false;
                    HANDiGroupBox.Enabled = false;
                    HANDiParamBox.Enabled = false;
                    HANDiFeedbackBox.Enabled = false;
                    HANDiConnect.Focus();

                    // Programatically click the 'BentoSuspend' button
                    //BentoRun.Enabled = false;
                    //BentoSuspend.Enabled = true;
                    HANDiRun.Enabled = true;
                    HANDiSuspend.Enabled = false;
                    BentoSuspend_Click(sender, e);
                    BentoRunStatus.Enabled = false;

                    // Reset the timer start timer
                    HANDiStartTimer.Reset();
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void HANDiSelectAll_Click(object sender, EventArgs e)
        {
            // Select all of the items in the checkedListBox
            for (int i = 0; i < HANDiList.Items.Count; i++)
            {
                HANDiList.SetItemChecked(i, true);

            }
        }

        private void HANDiClearAll_Click(object sender, EventArgs e)
        {
            // Unselect all of the items in the checkedListBox
            for (int i = 0; i < HANDiList.Items.Count; i++)
            {
                HANDiList.SetItemChecked(i, false);

            }
        }

        private void serialHANDi_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                if (serialHANDi.IsOpen)
                {
                    RxString2 = serialHANDi.ReadLine();
                }

            }
            catch (Exception ex)
            {

            }
        }

        private void HANDiRefresh_Click(object sender, EventArgs e)
        {
            // Auto-detect COM port that is connected to the Arduino/Bluetooth mate
            // How to sort the COM ports: http://www.prodigyproductionsllc.com/articles/programming/get-and-sort-com-port-names-using-c/
            // Also need to convert to list: http://stackoverflow.com/questions/9285426/orderby-and-list-vs-iorderedenumerable
            var ports = SerialPort.GetPortNames();
            var sortedList = ports.OrderBy(port => Convert.ToInt32(port.Replace("COM", string.Empty))).ToList();
            HANDiCOM.DataSource = sortedList;

            // Use this code if you are connecting the arduino via a USB cable
            // Reference 1: https://stackoverflow.com/questions/3293889/how-to-auto-detect-arduino-com-port
            // Reference 2: https://stackoverflow.com/questions/450059/how-do-i-set-the-selected-item-in-a-combobox-to-match-my-string-using-c
            ArduinoInputCOM.SelectedIndex = ArduinoInputCOM.FindStringExact(AutodetectArduinoPort());
        }

        private void HANDiRun_Click(object sender, EventArgs e)
        {
            // Programatically click the 'BentoRun' button
            //BentoRun.Enabled = true;
            //BentoSuspend.Enabled = false;
            HANDiRun.Enabled = false;
            HANDiSuspend.Enabled = true;
            BentoRun_Click(sender, e);
        }

        private void HANDiSuspend_Click(object sender, EventArgs e)
        {
            // Programatically click the 'BentoSuspend' button
            //BentoRun.Enabled = false;
            //BentoSuspend.Enabled = true;
            HANDiRun.Enabled = true;
            HANDiSuspend.Enabled = false;
            BentoSuspend_Click(sender, e);
        }

        // This is a separate thread that is used for communicating with the client
        public void DoWork3()
        {
            try
            {
                stopWatch3.Start();         // start the stop watch
                long HANDi_timestep = 20;     // this is how often in (ms) that brachIOplexus will send a command to the arduino
                while (HANDiGroupBox.Enabled)
                {
                    if (stopWatch3.ElapsedMilliseconds >= HANDi_timestep)
                    {
                        // Stop stopwatch and record how long everything in the main loop took to execute as well as how long it took to retrigger the main loop
                        stopWatch3.Stop();
                        milliSec3 = stopWatch3.ElapsedMilliseconds;
                        //HANDiDelay.Text = Convert.ToString(milliSec3);

                        //if (HANDiDelay.InvokeRequired)
                        //{
                        //    HANDiDelay.Invoke(new MethodInvoker(delegate { HANDiDelay.Text = Convert.ToString(milliSec3); }));
                        //}

                        // Send the target positions to the servo
                        if (serialHANDi.IsOpen)
                        {
                            // Define the message for serial output
                            string message = HANDi_pos[0] + "," + HANDi_pos[1] + "," + HANDi_pos[2] + "," + HANDi_pos[3] + "," + HANDi_pos[4] + "," + HANDi_pos[5] + ".";

                            // Send the string
                            serialHANDi.Write(message);
                        }

                        // Reset and start the stop watch
                        stopWatch3.Restart();
                    }
                }
                
                
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }
        #endregion

        #region "Main LOOP"
        // Main loop for sending out and reading back commands from input and output devices (mostly runs on a background thread).
        // The loop runs as fast as possible (typically 1-3 ms when disconnected from dynamixel bus and 2-6 ms when connected to dynamixel bus)

        private void pollingWorker_DoWork(object sender, DoWorkEventArgs e)
        {
            try
            {
                while (!e.Cancel)
                {
                    if (reporterState.Poll())
                    {
                        Invoke(new Action(UpdateState));
                    }
                }
            }
            catch (Exception ex)
            {
                //MessageBox.Show(ex.Message);
            }
        }

        private void UpdateState()
        {
            try
            {
                // Stop stopwatch and record how long everything in the main loop took to execute as well as how long it took to retrigger the main loop
                stopWatch1.Stop();
                milliSec1 = stopWatch1.ElapsedMilliseconds;
                delay.Text = Convert.ToString(milliSec1);

                // Check to see if the previous delay is the new maximum delay
                if (milliSec1 > Convert.ToDecimal(delay_max.Text))
                {
                    delay_max.Text = Convert.ToString(milliSec1);
                }

                // Reset and start the stop watch
                stopWatch1.Reset();
                stopWatch1.Start();

                #region "Initialize Input/Output Arrays"
                // Initialize input mapping array
                int[,] InputMap = new int[7, 25];

                // Define output mapping array
                int[,] OutputMap = new int[3, 15];

                // Bento Arm
                OutputMap[0, 0] = 0;        // Map to Shoulder Rotation (ID:1)
                OutputMap[0, 1] = 0;        // Map to Shoulder Rotation (ID:1)
                OutputMap[0, 2] = 1;        // Map to Elbow Flexion (ID:2)
                OutputMap[0, 3] = 1;        // Map to Elbow Flexion (ID:2)
                OutputMap[0, 4] = 2;        // Map to Wrist Rotate (ID:3)
                OutputMap[0, 5] = 2;        // Map to Wrist Rotate (ID:3)
                OutputMap[0, 6] = 3;        // Map to Wrist Flexion (ID:4)
                OutputMap[0, 7] = 3;        // Map to Wrist Flexion (ID:5)
                OutputMap[0, 8] = 4;        // Map to Hand (ID:5)
                OutputMap[0, 9] = 4;        // Map to Hand (ID:5)
                OutputMap[0, 10] = -2;      // Map to torque on/off
                OutputMap[0, 11] = -3;      // Map to toggle suspend/run state
                OutputMap[0, 12] = 6;       // Reserved for future use (i.e. multi-servo grippers)
                OutputMap[0, 13] = 7;       // Reserved for future use (i.e. multi-servo grippers)

                // HANDi Hand
                OutputMap[1, 0] = 5;
                OutputMap[1, 1] = 5;
                OutputMap[1, 2] = 6;
                OutputMap[1, 3] = 6;
                OutputMap[1, 4] = 7;
                OutputMap[1, 5] = 7;
                OutputMap[1, 6] = 8;
                OutputMap[1, 7] = 8;
                OutputMap[1, 8] = 9;
                OutputMap[1, 9] = 9;
                OutputMap[1, 10] = 10;
                OutputMap[1, 11] = 10;
                OutputMap[1, 12] = -3;

                #endregion

                #region "Update Input Signals"
                // Update feedback values that show up in the Visualization tab
                // Update Xbox values
                if (xBoxGroupBox.Enabled == true)
                {
                    int preGain = 500;

                    InputMap[0, 0] = splitAxis(Convert.ToInt32(reporterState.LastActiveState.ThumbSticks.Left.X * preGain), true);
                    InputMap[0, 1] = splitAxis(Convert.ToInt32(reporterState.LastActiveState.ThumbSticks.Left.X * preGain), false);
                    InputMap[0, 2] = splitAxis(Convert.ToInt32(reporterState.LastActiveState.ThumbSticks.Left.Y * preGain), true);
                    InputMap[0, 3] = splitAxis(Convert.ToInt32(reporterState.LastActiveState.ThumbSticks.Left.Y * preGain), false);
                    InputMap[0, 4] = splitAxis(Convert.ToInt32(reporterState.LastActiveState.ThumbSticks.Right.X * preGain), true);
                    InputMap[0, 5] = splitAxis(Convert.ToInt32(reporterState.LastActiveState.ThumbSticks.Right.X * preGain), false);
                    InputMap[0, 6] = splitAxis(Convert.ToInt32(reporterState.LastActiveState.ThumbSticks.Right.Y * preGain), true);
                    InputMap[0, 7] = splitAxis(Convert.ToInt32(reporterState.LastActiveState.ThumbSticks.Right.Y * preGain), false);

                    InputMap[0, 8] = Convert.ToInt32(reporterState.LastActiveState.Triggers.Left * preGain);
                    InputMap[0, 9] = Convert.ToInt32(reporterState.LastActiveState.Triggers.Right * preGain);

                    InputMap[0, 10] = Convert.ToInt32(reporterState.LastActiveState.Buttons.LeftStick == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 11] = Convert.ToInt32(reporterState.LastActiveState.Buttons.RightStick == XInputDotNetPure.ButtonState.Pressed) * preGain;

                    InputMap[0, 12] = Convert.ToInt32(reporterState.LastActiveState.Buttons.LeftShoulder == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 13] = Convert.ToInt32(reporterState.LastActiveState.Buttons.RightShoulder == XInputDotNetPure.ButtonState.Pressed) * preGain;

                    InputMap[0, 14] = Convert.ToInt32(reporterState.LastActiveState.DPad.Up == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 15] = Convert.ToInt32(reporterState.LastActiveState.DPad.Right == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 16] = Convert.ToInt32(reporterState.LastActiveState.DPad.Down == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 17] = Convert.ToInt32(reporterState.LastActiveState.DPad.Left == XInputDotNetPure.ButtonState.Pressed) * preGain;

                    InputMap[0, 18] = Convert.ToInt32(reporterState.LastActiveState.Buttons.A == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 19] = Convert.ToInt32(reporterState.LastActiveState.Buttons.B == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 20] = Convert.ToInt32(reporterState.LastActiveState.Buttons.X == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 21] = Convert.ToInt32(reporterState.LastActiveState.Buttons.Y == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 22] = Convert.ToInt32(reporterState.LastActiveState.Buttons.Start == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 23] = Convert.ToInt32(reporterState.LastActiveState.Buttons.Back == XInputDotNetPure.ButtonState.Pressed) * preGain;
                    InputMap[0, 24] = Convert.ToInt32(reporterState.LastActiveState.Buttons.Guide == XInputDotNetPure.ButtonState.Pressed) * preGain;

                    labelStickLeftX.Text = FormatFloat(reporterState.LastActiveState.ThumbSticks.Left.X);
                    labelStickLeftY.Text = FormatFloat(reporterState.LastActiveState.ThumbSticks.Left.Y);
                    labelStickRightX.Text = FormatFloat(reporterState.LastActiveState.ThumbSticks.Right.X);
                    labelStickRightY.Text = FormatFloat(reporterState.LastActiveState.ThumbSticks.Right.Y);
                    checkStickLeft.Checked = reporterState.LastActiveState.Buttons.LeftStick == XInputDotNetPure.ButtonState.Pressed;
                    checkStickRight.Checked = reporterState.LastActiveState.Buttons.RightStick == XInputDotNetPure.ButtonState.Pressed;

                    labelTriggerLeft.Text = FormatFloat(reporterState.LastActiveState.Triggers.Left);
                    labelTriggerRight.Text = FormatFloat(reporterState.LastActiveState.Triggers.Right);

                    checkDPadUp.Checked = reporterState.LastActiveState.DPad.Up == XInputDotNetPure.ButtonState.Pressed;
                    checkDPadRight.Checked = reporterState.LastActiveState.DPad.Right == XInputDotNetPure.ButtonState.Pressed;
                    checkDPadDown.Checked = reporterState.LastActiveState.DPad.Down == XInputDotNetPure.ButtonState.Pressed;
                    checkDPadLeft.Checked = reporterState.LastActiveState.DPad.Left == XInputDotNetPure.ButtonState.Pressed;

                    checkShoulderLeft.Checked = reporterState.LastActiveState.Buttons.LeftShoulder == XInputDotNetPure.ButtonState.Pressed;
                    checkShoulderRight.Checked = reporterState.LastActiveState.Buttons.RightShoulder == XInputDotNetPure.ButtonState.Pressed;

                    checkA.Checked = reporterState.LastActiveState.Buttons.A == XInputDotNetPure.ButtonState.Pressed;
                    checkB.Checked = reporterState.LastActiveState.Buttons.B == XInputDotNetPure.ButtonState.Pressed;
                    checkX.Checked = reporterState.LastActiveState.Buttons.X == XInputDotNetPure.ButtonState.Pressed;
                    checkY.Checked = reporterState.LastActiveState.Buttons.Y == XInputDotNetPure.ButtonState.Pressed;
                    checkStart.Checked = reporterState.LastActiveState.Buttons.Start == XInputDotNetPure.ButtonState.Pressed;
                    checkBack.Checked = reporterState.LastActiveState.Buttons.Back == XInputDotNetPure.ButtonState.Pressed;
                    checkGuide.Checked = reporterState.LastActiveState.Buttons.Guide == XInputDotNetPure.ButtonState.Pressed;

                    if (XboxBuzzBox.Checked == true && XboxBuzzFlag == true)
                    {
                        reporterState.ActivateVibration = true;
                        XboxTimer.Reset();
                        XboxTimer.Start();
                        XboxBuzzFlag = false;
                    }
                    else if (XboxBuzzBox.Checked == true && XboxTimer.ElapsedMilliseconds >= 250)
                    {
                        XboxTimer.Stop();
                        reporterState.ActivateVibration = false;
                    }

                    // Allows for using a button on the joystick to access quick profile switching
                    //if (reporterState.LastActiveState.Buttons.Start == XInputDotNetPure.ButtonState.Pressed && QuickProfileFlag == false)
                    //{
                    //    // QuickProfileState = 0 (Xbox profile), 1 (MYO profile)
                    //    // QuickProfileFlag = false (waiting for trigger), true (triggered and need to release the button before it can be triggered again) 

                    //    if (QuickProfileState == 0)
                    //    {
                    //        InvokeOnClick(demoMYObutton, new EventArgs());
                    //        QuickProfileState = 1;
                    //    }
                    //    else if (QuickProfileState == 1)
                    //    {
                    //        InvokeOnClick(demoXBoxButton, new EventArgs());
                    //        QuickProfileState = 0;
                    //    }
                    //    QuickProfileFlag = true;
                    //}
                    //else if (reporterState.LastActiveState.Buttons.Start == XInputDotNetPure.ButtonState.Released)
                    //{
                    //    QuickProfileFlag = false;
                    //}

                }
                // Update MYO EMG values
                if (MYOgroupBox.Enabled == true)
                {
                    InputMap[1, 0] = Convert.ToInt32(ch1_value);
                    InputMap[1, 1] = Convert.ToInt32(ch2_value);
                    InputMap[1, 2] = Convert.ToInt32(ch3_value);
                    InputMap[1, 3] = Convert.ToInt32(ch4_value);
                    InputMap[1, 4] = Convert.ToInt32(ch5_value);
                    InputMap[1, 5] = Convert.ToInt32(ch6_value);
                    InputMap[1, 6] = Convert.ToInt32(ch7_value);
                    InputMap[1, 7] = Convert.ToInt32(ch8_value);

                    myo_ch1.Text = Convert.ToString(ch1_value);
                    myo_ch2.Text = Convert.ToString(ch2_value);
                    myo_ch3.Text = Convert.ToString(ch3_value);
                    myo_ch4.Text = Convert.ToString(ch4_value);
                    myo_ch5.Text = Convert.ToString(ch5_value);
                    myo_ch6.Text = Convert.ToString(ch6_value);
                    myo_ch7.Text = Convert.ToString(ch7_value);
                    myo_ch8.Text = Convert.ToString(ch8_value);
                }

                // Update keyboard values
                if (KBgroupBox.Enabled == true)
                {
                    double ramp_delay = 264;      // the time in milliseconds that it takes for the keyboard to ramp up from stopped (0) to full speed (1).
                    int preGain = 500;

                    InputMap[2, 0] = velocity_ramp(ref KBvel[0], Keyboard.IsKeyDown(Key.W), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 1] = velocity_ramp(ref KBvel[1], Keyboard.IsKeyDown(Key.A), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 2] = velocity_ramp(ref KBvel[2], Keyboard.IsKeyDown(Key.S), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 3] = velocity_ramp(ref KBvel[3], Keyboard.IsKeyDown(Key.D), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);

                    InputMap[2, 4] = velocity_ramp(ref KBvel[4], Keyboard.IsKeyDown(Key.O), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 5] = velocity_ramp(ref KBvel[5], Keyboard.IsKeyDown(Key.K), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 6] = velocity_ramp(ref KBvel[6], Keyboard.IsKeyDown(Key.L), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 7] = velocity_ramp(ref KBvel[7], Keyboard.IsKeyDown(Key.OemSemicolon), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);

                    InputMap[2, 8] = velocity_ramp(ref KBvel[8], Keyboard.IsKeyDown(Key.Up), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 9] = velocity_ramp(ref KBvel[9], Keyboard.IsKeyDown(Key.Down), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 10] = velocity_ramp(ref KBvel[10], Keyboard.IsKeyDown(Key.Left), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 11] = velocity_ramp(ref KBvel[11], Keyboard.IsKeyDown(Key.Right), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);

                    InputMap[2, 12] = velocity_ramp(ref KBvel[12], Keyboard.IsKeyDown(Key.LeftAlt), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 13] = velocity_ramp(ref KBvel[13], Keyboard.IsKeyDown(Key.RightAlt), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);
                    InputMap[2, 14] = velocity_ramp(ref KBvel[14], Keyboard.IsKeyDown(Key.Space), KBcheckRamp.Checked, preGain / (ramp_delay / milliSec1), preGain);

                    KBcheckW.Checked = Keyboard.IsKeyDown(Key.W);
                    KBcheckA.Checked = Keyboard.IsKeyDown(Key.A);
                    KBcheckS.Checked = Keyboard.IsKeyDown(Key.S);
                    KBcheckD.Checked = Keyboard.IsKeyDown(Key.D);

                    KBcheckO.Checked = Keyboard.IsKeyDown(Key.O);
                    KBcheckK.Checked = Keyboard.IsKeyDown(Key.K);
                    KBcheckL.Checked = Keyboard.IsKeyDown(Key.L);
                    KBcheckSemiColon.Checked = Keyboard.IsKeyDown(Key.OemSemicolon);

                    KBcheckUp.Checked = Keyboard.IsKeyDown(Key.Up);
                    KBcheckDown.Checked = Keyboard.IsKeyDown(Key.Down);
                    KBcheckLeft.Checked = Keyboard.IsKeyDown(Key.Left);
                    KBcheckRight.Checked = Keyboard.IsKeyDown(Key.Right);

                    KBcheckLeftAlt.Checked = Keyboard.IsKeyDown(Key.LeftAlt);
                    KBcheckRightAlt.Checked = Keyboard.IsKeyDown(Key.RightAlt);
                    KBcheckSpace.Checked = Keyboard.IsKeyDown(Key.Space);

                    KBrampW.Text = Convert.ToString(InputMap[2, 0]);
                    KBrampA.Text = Convert.ToString(InputMap[2, 1]);
                    KBrampS.Text = Convert.ToString(InputMap[2, 2]);
                    KBrampD.Text = Convert.ToString(InputMap[2, 3]);
                }

                // Update BioPatRec values
                if (biopatrecGroupBox.Enabled == true)
                {
                    // Scale factor so that progress bar control can show a finer resolution
                    Int32 scale_factor = 1;

                    for (int i = 0; i < CLASS_NUM; i++)
                    {
                        InputMap[3, i] = 0;
                    }

                    InputMap[3, biopatrec_ID] = biopatrec_vel * scale_factor;

                    BPRclass0.Checked = greaterThanZero(InputMap[3, 0]);
                    BPRclass1.Checked = greaterThanZero(InputMap[3, 1]);
                    BPRclass2.Checked = greaterThanZero(InputMap[3, 2]);
                    BPRclass3.Checked = greaterThanZero(InputMap[3, 3]);
                    BPRclass4.Checked = greaterThanZero(InputMap[3, 4]);
                    BPRclass5.Checked = greaterThanZero(InputMap[3, 5]);
                    BPRclass6.Checked = greaterThanZero(InputMap[3, 6]);
                    BPRclass7.Checked = greaterThanZero(InputMap[3, 7]);
                    BPRclass8.Checked = greaterThanZero(InputMap[3, 8]);
                    BPRclass9.Checked = greaterThanZero(InputMap[3, 9]);
                    BPRclass10.Checked = greaterThanZero(InputMap[3, 10]);
                    BPRclass11.Checked = greaterThanZero(InputMap[3, 11]);
                    BPRclass12.Checked = greaterThanZero(InputMap[3, 12]);
                    BPRclass13.Checked = greaterThanZero(InputMap[3, 13]);
                    BPRclass14.Checked = greaterThanZero(InputMap[3, 14]);
                    BPRclass15.Checked = greaterThanZero(InputMap[3, 15]);
                    BPRclass16.Checked = greaterThanZero(InputMap[3, 16]);
                    BPRclass17.Checked = greaterThanZero(InputMap[3, 17]);
                    BPRclass18.Checked = greaterThanZero(InputMap[3, 18]);
                    BPRclass19.Checked = greaterThanZero(InputMap[3, 19]);
                    BPRclass20.Checked = greaterThanZero(InputMap[3, 20]);
                    BPRclass21.Checked = greaterThanZero(InputMap[3, 21]);
                    BPRclass22.Checked = greaterThanZero(InputMap[3, 22]);
                    BPRclass23.Checked = greaterThanZero(InputMap[3, 23]);
                    BPRclass24.Checked = greaterThanZero(InputMap[3, 24]);

                }

                // Update Arduino Analog Input Values
                if (ArduinoInputGroupBox.Enabled == true)
                {
                    // Scale factor so that progress bar control can show a finer resolution
                    Int32 scale_factor = 2;

                    try
                    {
                        if (serialArduinoInput.IsOpen && ArduinoStartTimer.ElapsedMilliseconds > ArduinoStartDelay)
                        {
                            // Stop the start timer if it is running
                            if (ArduinoStartTimer.IsRunning == true)
                            {
                                ArduinoStartTimer.Stop();
                            }
                            // how to separate string into individual words using predefined separatingChars: https://msdn.microsoft.com/en-ca/library/ms228388.aspx
                            // how to remove leading zeros: http://stackoverflow.com/questions/7010702/how-to-remove-leading-zeros
                            // only channels A0-A3 are enabled for now. The rest are set to 0
                            string RxString_local = RxString.TrimEnd('\r', '\n');    // remove carriage return as it messes up the TrimStart when the last channel is set to 0
                            string[] words = RxString_local.Split(separatingChars);

                            InputMap[4, 0] = Convert.ToInt32(words[1].TrimStart('0').Length > 0 ? words[1].TrimStart('0') : "0") / scale_factor;
                            InputMap[4, 1] = Convert.ToInt32(words[2].TrimStart('0').Length > 0 ? words[2].TrimStart('0') : "0") / scale_factor;
                            InputMap[4, 2] = Convert.ToInt32(words[3].TrimStart('0').Length > 0 ? words[3].TrimStart('0') : "0") / scale_factor;
                            InputMap[4, 3] = Convert.ToInt32(words[4].TrimStart('0').Length > 0 ? words[4].TrimStart('0') : "0") / scale_factor;
                            InputMap[4, 4] = Convert.ToInt32(words[5].TrimStart('0').Length > 0 ? words[5].TrimStart('0') : "0") / scale_factor;
                            InputMap[4, 5] = Convert.ToInt32(words[6].TrimStart('0').Length > 0 ? words[6].TrimStart('0') : "0") / scale_factor;
                            InputMap[4, 6] = Convert.ToInt32(words[7].TrimStart('0').Length > 0 ? words[7].TrimStart('0') : "0") / scale_factor;
                            InputMap[4, 7] = Convert.ToInt32(words[8].TrimStart('0').Length > 0 ? words[8].TrimStart('0') : "0") / scale_factor;
                        }
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show(ex.Message);
                    }

                    arduino_A0.Text = Convert.ToString(InputMap[4, 0]);
                    arduino_A1.Text = Convert.ToString(InputMap[4, 1]);
                    arduino_A2.Text = Convert.ToString(InputMap[4, 2]);
                    arduino_A3.Text = Convert.ToString(InputMap[4, 3]);
                    arduino_A4.Text = Convert.ToString(InputMap[4, 4]);
                    arduino_A5.Text = Convert.ToString(InputMap[4, 5]);
                    arduino_A6.Text = Convert.ToString(InputMap[4, 6]);
                    arduino_A7.Text = Convert.ToString(InputMap[4, 7]);
                }

                // Update UDP values
                if (udpGroupBox.Enabled == true)
                {
                    // Scale factor so that progress bar control can show a finer resolution
                    Int32 scale_factor = 2;

                    InputMap[6, 0] = Convert.ToInt32(UDP_ch[0] * scale_factor);
                    InputMap[6, 1] = Convert.ToInt32(UDP_ch[1] * scale_factor);
                    InputMap[6, 2] = Convert.ToInt32(UDP_ch[2] * scale_factor);
                    InputMap[6, 3] = Convert.ToInt32(UDP_ch[3] * scale_factor);
                    InputMap[6, 4] = Convert.ToInt32(UDP_ch[4] * scale_factor);
                    InputMap[6, 5] = Convert.ToInt32(UDP_ch[5] * scale_factor);
                    InputMap[6, 6] = Convert.ToInt32(UDP_ch[6] * scale_factor);
                    InputMap[6, 7] = Convert.ToInt32(UDP_ch[7] * scale_factor);
                    InputMap[6, 8] = Convert.ToInt32(UDP_ch[8] * scale_factor);
                    InputMap[6, 9] = Convert.ToInt32(UDP_ch[9] * scale_factor);
                    InputMap[6, 10] = Convert.ToInt32(UDP_ch[10] * scale_factor);

                    udp_ch1.Text = Convert.ToString(InputMap[6, 0]);
                    udp_ch2.Text = Convert.ToString(InputMap[6, 1]);
                    udp_ch3.Text = Convert.ToString(InputMap[6, 2]);
                    udp_ch4.Text = Convert.ToString(InputMap[6, 3]);
                    udp_ch5.Text = Convert.ToString(InputMap[6, 4]);
                    udp_ch6.Text = Convert.ToString(InputMap[6, 5]);
                    udp_ch7.Text = Convert.ToString(InputMap[6, 6]);
                    udp_ch8.Text = Convert.ToString(InputMap[6, 7]);
                    udp_ch9.Text = Convert.ToString(InputMap[6, 8]);
                    udp_ch10.Text = Convert.ToString(InputMap[6, 9]);
                    udp_ch11.Text = Convert.ToString(InputMap[6, 10]);
                }
                #endregion 

                #region "Update DoF Parameters"
                // Update the mapping parameters
                // Update DoFObj with latest values
                UpdateDoF(dofObj[0], doF1, InputMap, 0);
                UpdateDoF(dofObj[1], doF2, InputMap, 1);
                UpdateDoF(dofObj[2], doF3, InputMap, 2);
                UpdateDoF(dofObj[3], doF4, InputMap, 3);
                UpdateDoF(dofObj[4], doF5, InputMap, 4);
                UpdateDoF(dofObj[5], doF6, InputMap, 5);

                // Reset Auto-suspend flag
                if (autosuspend_flag == true)
                {
                    autosuspend_flag = false;
                }

                #endregion

                #region "Update Robot Parameters"
                // Bento Arm
                robotObj.type = 0;

                robotObj.Motor[0].pmin = Convert.ToInt32(shoulder_pmin_ctrl.Value);
                robotObj.Motor[0].pmax = Convert.ToInt32(shoulder_pmax_ctrl.Value);
                robotObj.Motor[0].wmin = Convert.ToInt32(shoulder_wmin_ctrl.Value);
                robotObj.Motor[0].wmax = Convert.ToInt32(shoulder_wmax_ctrl.Value);

                robotObj.Motor[1].pmin = Convert.ToInt32(elbow_pmin_ctrl.Value);
                robotObj.Motor[1].pmax = Convert.ToInt32(elbow_pmax_ctrl.Value);
                robotObj.Motor[1].wmin = Convert.ToInt32(elbow_wmin_ctrl.Value);
                robotObj.Motor[1].wmax = Convert.ToInt32(elbow_wmax_ctrl.Value);

                robotObj.Motor[2].pmin = Convert.ToInt32(wristRot_pmin_ctrl.Value);
                robotObj.Motor[2].pmax = Convert.ToInt32(wristRot_pmax_ctrl.Value);
                robotObj.Motor[2].wmin = Convert.ToInt32(wristRot_wmin_ctrl.Value);
                robotObj.Motor[2].wmax = Convert.ToInt32(wristRot_wmax_ctrl.Value);

                robotObj.Motor[3].pmin = Convert.ToInt32(wristFlex_pmin_ctrl.Value);
                robotObj.Motor[3].pmax = Convert.ToInt32(wristFlex_pmax_ctrl.Value);
                robotObj.Motor[3].wmin = Convert.ToInt32(wristFlex_wmin_ctrl.Value);
                robotObj.Motor[3].wmax = Convert.ToInt32(wristFlex_wmax_ctrl.Value);

                robotObj.Motor[4].pmin = Convert.ToInt32(hand_pmin_ctrl.Value);
                robotObj.Motor[4].pmax = Convert.ToInt32(hand_pmax_ctrl.Value);
                robotObj.Motor[4].wmin = Convert.ToInt32(hand_wmin_ctrl.Value);
                robotObj.Motor[4].wmax = Convert.ToInt32(hand_wmax_ctrl.Value);

                //// HANDi HAnd -> smushed into bento robotObj
                robotObj.Motor[5].pmin = Convert.ToInt32(D0_pmin_ctrl.Value);
                robotObj.Motor[5].pmax = Convert.ToInt32(D0_pmax_ctrl.Value);
                robotObj.Motor[5].wmin = Convert.ToInt32(D0_wmin_ctrl.Value);
                robotObj.Motor[5].wmax = Convert.ToInt32(D0_wmax_ctrl.Value);

                robotObj.Motor[6].pmin = Convert.ToInt32(D1_pmin_ctrl.Value);
                robotObj.Motor[6].pmax = Convert.ToInt32(D1_pmax_ctrl.Value);
                robotObj.Motor[6].wmin = Convert.ToInt32(D1_wmin_ctrl.Value);
                robotObj.Motor[6].wmax = Convert.ToInt32(D1_wmax_ctrl.Value);

                robotObj.Motor[7].pmin = Convert.ToInt32(D2_pmin_ctrl.Value);
                robotObj.Motor[7].pmax = Convert.ToInt32(D2_pmax_ctrl.Value);
                robotObj.Motor[7].wmin = Convert.ToInt32(D2_wmin_ctrl.Value);
                robotObj.Motor[7].wmax = Convert.ToInt32(D2_wmax_ctrl.Value);

                robotObj.Motor[8].pmin = Convert.ToInt32(D3_pmin_ctrl.Value);
                robotObj.Motor[8].pmax = Convert.ToInt32(D3_pmax_ctrl.Value);
                robotObj.Motor[8].wmin = Convert.ToInt32(D3_wmin_ctrl.Value);
                robotObj.Motor[8].wmax = Convert.ToInt32(D3_wmax_ctrl.Value);

                robotObj.Motor[9].pmin = Convert.ToInt32(D4_pmin_ctrl.Value);
                robotObj.Motor[9].pmax = Convert.ToInt32(D4_pmax_ctrl.Value);
                robotObj.Motor[9].wmin = Convert.ToInt32(D4_wmin_ctrl.Value);
                robotObj.Motor[9].wmax = Convert.ToInt32(D4_wmax_ctrl.Value);

                robotObj.Motor[10].pmin = Convert.ToInt32(D5_pmin_ctrl.Value);
                robotObj.Motor[10].pmax = Convert.ToInt32(D5_pmax_ctrl.Value);
                robotObj.Motor[10].wmin = Convert.ToInt32(D5_wmin_ctrl.Value);
                robotObj.Motor[10].wmax = Convert.ToInt32(D5_wmax_ctrl.Value);

                #endregion

                #region "Update Sequential Switch Parameters
                // Update the signal information for the sequential switch
                if (switchModeBox.SelectedIndex == 0 && switchInputBox.SelectedIndex > 0)
                {
                    // If switch mode is set to button press map the first signal bar to the value of the assigned button
                    switchObj.signal = InputMap[(switchInputBox.SelectedItem as ComboboxItem).Type, (switchInputBox.SelectedItem as ComboboxItem).ID] * Convert.ToInt32(switchObj.gain);
                    switchSignalBar1.Value = signalRail(switchObj.signal, switchSignalBar1);
                }
                else if (switchModeBox.SelectedIndex == 1 && switchDoFbox.SelectedIndex > 0)
                {
                    // If switch mode is set to co-contraction then map the signal bars to the DoF that sequential switching is set to such that they mirror that channel pair
                    switchSignalBar1.Value = signalRail(dofObj[switchDoFbox.SelectedIndex - 1].ChA.signal, switchSignalBar1);
                    switchSignalBar2.Value = signalRail(dofObj[switchDoFbox.SelectedIndex - 1].ChB.signal, switchSignalBar2);

                    // Do not allow smin in sequential switching be  greater than smin in dof section
                    if ((switchSminCtrl1.Value * 100) > dofObj[switchDoFbox.SelectedIndex - 1].ChA.smin)
                    {
                        switchSminCtrl1.Value = dofObj[switchDoFbox.SelectedIndex - 1].ChA.smin / 100;
                    }
                    if ((switchSminCtrl2.Value * 100) > dofObj[switchDoFbox.SelectedIndex - 1].ChB.smin)
                    {
                        switchSminCtrl2.Value = dofObj[switchDoFbox.SelectedIndex - 1].ChB.smin / 100;
                    }
                }
                #endregion

                //// for debugging state variables
                //ID2_state.Text = Convert.ToString(switchObj.List[stateObj.listPos].output - 1); //Convert.ToString(stateObj.motorState[1]);
                //ID2_state.Text = Convert.ToString(stateObj.motorState[1]);
                //ID2_state.Text = Convert.ToString(robotObj.Motor[4].w);

                // Calculate joint positions and velocities and update states for active degrees of freedom
                for (int i = 0; i < DOF_NUM; i++)
                {
                    // update k and m if they haven't already been assigned on a previous iteration through this loop
                    int k = OutputMap[dofObj[i].ChA.output.Type, dofObj[i].ChA.output.ID];
                    int m = OutputMap[dofObj[i].ChB.output.Type, dofObj[i].ChB.output.ID];

                    #region "Sequential Switch"

                    if (i == switchObj.DoF - 1)
                    {
                        // If no switching has occured then set current mapping to previous mapping
                        k = switchObj.List[stateObj.listPos].output - 1;
                        dofObj[i].ChA.mapping = switchObj.List[stateObj.listPos].mapping;

                        // Check for switching events
                        switch (switchObj.mode)
                        {
                            // Use button press
                            case 0:
                                if (switchObj.signal > switchObj.smin1 && stateObj.switchState == 0)
                                {
                                    // Grab last position feedback and STOP! (added in to prevent state variable from staying high if active during a switching event)
                                    if (k >= 0)
                                    {
                                        robotObj.Motor[k].p = robotObj.Motor[k].p_prev;
                                        robotObj.Motor[k].w = robotObj.Motor[k].wmax;
                                        stateObj.motorState[k] = 0;
                                    }

                                    // Update list position
                                    stateObj.listPos = updateList(stateObj.listPos);

                                    if (k >= -1)
                                    {
                                        while (switchObj.List[stateObj.listPos].output <= 0)
                                        {
                                            stateObj.listPos = updateList(stateObj.listPos);
                                        }
                                    }

                                    k = switchObj.List[stateObj.listPos].output - 1;
                                    dofObj[i].ChA.mapping = switchObj.List[stateObj.listPos].mapping;

                                    stateObj.switchState = 1;

                                    // Update switch feedback with current item in switching list
                                    updateSwitchFeedback();
                                    myoBuzzFlag = true;
                                    XboxBuzzFlag = true;
                                }
                                else if (switchObj.signal < switchObj.smin1)
                                {
                                    stateObj.switchState = 0;
                                }
                                break;
                            // Use co-contraction switching
                            case 1:
                                if ((dofObj[i].ChA.signal >= switchObj.smin1 || dofObj[i].ChB.signal >= switchObj.smin2) && stateObj.switchState == 0)
                                {
                                    // Start the co-contraction timer
                                    stateObj.timer1 = 0;
                                    stateObj.switchState = 1;
                                }
                                if (stateObj.switchState == 1)
                                {
                                    // Increment the timer
                                    stateObj.timer1 = stateObj.timer1 + milliSec1;

                                    // Turn off the dof while checking that the co-contraction were met for the duration of cctimer
                                    if (k >= 0)
                                    {
                                        stateObj.motorState[k] = 3;
                                    }

                                    // Check to see if signals are co-contracted above threshold
                                    if (dofObj[i].ChA.signal >= switchObj.smax1)
                                    {
                                        switchObj.flag1 = true;
                                    }

                                    if (dofObj[i].ChB.signal >= switchObj.smax2)
                                    {
                                        switchObj.flag2 = true;
                                    }

                                    if (switchObj.flag1 == true && switchObj.flag2 == true)
                                    {
                                        // Co-contract conditions were met, so initiate switching event. 

                                        // Reset previous joint so it can't move anymore
                                        if (k >= 0)
                                        {
                                            stateObj.motorState[k] = 0;
                                        }

                                        // Update list position
                                        stateObj.listPos = updateList(stateObj.listPos);

                                        if (k >= -1)
                                        {
                                            while (switchObj.List[stateObj.listPos].output <= 0)
                                            {
                                                stateObj.listPos = updateList(stateObj.listPos);
                                            }

                                        }

                                        k = switchObj.List[stateObj.listPos].output - 1;
                                        dofObj[i].ChA.mapping = switchObj.List[stateObj.listPos].mapping;

                                        // Set switched to joint so it can't move until drop below threshold
                                        if (k >= 0)
                                        {
                                            stateObj.motorState[k] = 3;
                                        }

                                        // Reset co-contraction flags for each channel
                                        switchObj.flag1 = false;
                                        switchObj.flag2 = false;

                                        // Update switch feedback with current item in switching list
                                        updateSwitchFeedback();
                                        myoBuzzFlag = true;
                                        XboxBuzzFlag = true;

                                        // Do not allow the arm to move until both signals have dropped below threshold
                                        stateObj.switchState = 2;
                                    }
                                    else if (stateObj.timer1 >= switchObj.cctime)
                                    {
                                        // Co-contract conditions were not met, so allow the arm to move
                                        k = switchObj.List[stateObj.listPos].output - 1;
                                        dofObj[i].ChA.mapping = switchObj.List[stateObj.listPos].mapping;
                                        if (k >= 0)
                                        {
                                            stateObj.motorState[k] = 0;
                                        }

                                        // Reset co-contraction flags for each channel
                                        switchObj.flag1 = false;
                                        switchObj.flag2 = false;

                                        // Don't allow another switching event until both of the channels drops below threshold
                                        stateObj.switchState = 2;
                                    }

                                }
                                else if (dofObj[i].ChA.signal < switchObj.smin1 && dofObj[i].ChB.signal < switchObj.smin2)
                                {
                                    // reset the co-contraction state variable and timer
                                    stateObj.switchState = 0;
                                    stateObj.timer1 = 0;

                                    // Reset co-contraction flags for each channel
                                    switchObj.flag1 = false;
                                    switchObj.flag2 = false;

                                    // Allow the arm to move
                                    k = switchObj.List[stateObj.listPos].output - 1;
                                    dofObj[i].ChA.mapping = switchObj.List[stateObj.listPos].mapping;

                                }
                                break;
                        }


                    }
                    else if (switchObj.DoF == 0)
                    {
                        stateObj.switchState = 0;
                        k = OutputMap[dofObj[i].ChA.output.Type, dofObj[i].ChA.output.ID];
                    }
                    #endregion

                    // Test output for co-contraction debugging
                    timer1_label.Text = Convert.ToString(stateObj.timer1);
                    flag1_label.Text = Convert.ToString(switchObj.flag1);
                    flag2_label.Text = Convert.ToString(switchObj.flag2);
                    switchState_label.Text = Convert.ToString(stateObj.switchState);

                    if (dofObj[i].Enabled)
                    {
                        if (bentoSuspend == false || biopatrecMode.SelectedIndex == 1)  // only connect inputs to outputs if Bento is in 'Run' mode
                        //if (true)  // for testing with HANDi hand
                        {
                            switch (dofObj[i].ChA.mapping)
                            {
                                // Use first past the post control option
                                case 0:
                                    if (k >= 0)
                                    {
                                        post(dofObj[i], k, i);
                                    }
                                    break;
                                // Use greatest signal wins mapping
                                case 1:

                                    if (k >= 0)
                                    {
                                        wins(dofObj[i], k, i);
                                    }
                                    break;
                                // Use Single Site - Voluntary Open/Close Automatic Return Mapping (2 state)
                                case 2:

                                    if (k >= 0)
                                    {
                                        single2(dofObj[i], k, i);
                                    }
                                    break;
                                // Use Joint Position2 Mapping (map from the analog signals of two channels to the position of one of the joints on the robot)
                                case 3:

                                    if (k >= 0)
                                    {
                                        joint_position2(dofObj[i], k, i);
                                    }
                                    break;
                                // Use Joint Position1 Mapping (map from the analog signal of a single channel to the position of one of the joints on the robot)
                                case 4:

                                    if (k >= 0 && dofObj[i].ChA.Enabled)
                                    {
                                        joint_position1(dofObj[i].ChA, k, check_flip(i, dofObj[i].flipA));
                                    }
                                    break;
                            }

                            // Process second channel if using mappings that only use 1 channel (i.e. so you could control a DoF with each channel)
                            if (dofObj[i].ChA.mapping == 4 || k < -1 || m < -1)
                            {
                                switch (dofObj[i].ChB.mapping)
                                {
                                    // Use Joint Position1 Mapping (map from the analog signal of a single channel to the position of one of the joints on the robot)
                                    case 2:
                                        if (m >= 0 && dofObj[i].ChB.Enabled)
                                        {
                                            joint_position1(dofObj[i].ChB, m, check_flip(i, dofObj[i].flipB));
                                        }
                                        break;
                                }
                            }

                            // Stop the joint that was previously assigned in order to prevent it from continuing to move
                            if (dofStop[0] > -1)
                            {
                                StopVelocity(OutputMap[dofStop[0], dofStop[1]]);
                                dofStop[0] = -1;    // Reset dofStop to -1, so that it only stops the joint once
                                dofStop[1] = -1;
                            }
                        }

                        // Check if additional Bento functions such as torque on/off or suspend/run are selected
                        if (k < -1)
                        {
                            switch (k)
                            {
                                case -2:
                                    robotObj.torque = toggle(dofObj[i].ChA, robotObj.torque, TorqueOn, TorqueOff);
                                    break;
                                case -3:
                                    robotObj.suspend = toggle(dofObj[i].ChA, robotObj.suspend, BentoRun, BentoSuspend);
                                    break;
                            }
                        }
                        if (m < -1)
                        {
                            switch (m)
                            {
                                case -2:
                                    robotObj.torque = toggle(dofObj[i].ChB, robotObj.torque, TorqueOn, TorqueOff);
                                    break;
                                case -3:
                                    robotObj.suspend = toggle(dofObj[i].ChB, robotObj.suspend, BentoRun, BentoSuspend);
                                    break;
                            }
                        }
                    }
                }

                if (BentoGroupBox.Enabled == true)
                {
                    // Update feedback
                    readDyna();

                    // Only run the task timer code if it is enabled in the GUI
                    //TaskTimerStateLabel.Text = Convert.ToString(TaskTimerState);
                    if (TaskTimerState != 0)
                    {
                        UpdateTaskTimer();
                    }

                    // Move dynamixel motors
                    if (TorqueOn.Enabled == false && biopatrecMode.SelectedIndex != 1)
                    {
                        // Write goal position and moving speeds to each servo on bus using syncwrite command
                        writeDyna();
                    }
                }
                else if (BentoGroupBox.Enabled == false)
                {
                    // Add delay on background thread so that timers work properly even if not sending out commands or reading from dynamixels
                    // This is needed because without the delay the time reported by the stopwatch is 0 which means we can't update our timers properly
                    System.Threading.Thread.Sleep(2);
                }

                if (HANDiGroupBox.Enabled == true)
                {
                    HANDi_pos[0] = robotObj.Motor[5].p;
                    HANDi_pos[1] = robotObj.Motor[6].p;
                    HANDi_pos[2] = robotObj.Motor[7].p;
                    HANDi_pos[3] = robotObj.Motor[8].p;
                    HANDi_pos[4] = robotObj.Motor[9].p;
                    HANDi_pos[5] = robotObj.Motor[10].p;
                }

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        #region "Helper Functions"

        // Updates a degree of freedom object with the latest info from the GUI and input devices
        private void UpdateDoF(DoF_ dofObj, DoF dofA, int[,] InputMap, int i)
        {

            if (dofA.channel1.inputBox.SelectedIndex > 0)
            {
                dofObj.ChA.signal = InputMap[dofObj.ChA.input.Type, dofObj.ChA.input.ID] * Convert.ToInt32(dofObj.ChA.gain);
                dofA.channel1.signalBar.Value = signalRail(dofObj.ChA.signal, dofA.channel1.signalBar);
            }

            if (dofA.channel2.inputBox.SelectedIndex > 0)
            {
                dofObj.ChB.signal = InputMap[dofObj.ChB.input.Type, dofObj.ChB.input.ID] * Convert.ToInt32(dofObj.ChB.gain);
                dofA.channel2.signalBar.Value = signalRail(dofObj.ChB.signal, dofA.channel2.signalBar);
            }

            // Check whether the dof is enabled
            if ((dofObj.ChA.Enabled == true) || (dofObj.ChB.Enabled == true) || (i == switchObj.DoF - 1 && dofA.channel1.inputBox.SelectedIndex > 0 && dofA.channel2.inputBox.SelectedIndex > 0))
            {
                dofObj.Enabled = true;
            }
            else
            {
                dofObj.Enabled = false;
            }

            // Auto-suspend functionality
            if (autosuspend_flag == true)
            {
                if ((dofObj.ChA.signal >= dofObj.ChA.smin || dofObj.ChB.signal >= dofObj.ChB.smin) && dofObj.ChB.smin != 0)
                {
                    InvokeOnClick(BentoSuspend, new EventArgs());
                    autosuspend_flag = false;
                }
            }

        }

        private void post(DoF_ dofObj, int k, int i)
        {
            // This function acts as the first past the post control option where only
            // the first EMG channel to reach threshold of the pair gets priority for
            // controlling the function on the robotic arm.i.e.If MAV1 = hand open
            // and MAV2 = hand closed then if MAV1 reaches threshold first the hand
            // will open.The hand will not be able to be closed until the MAV1 drops
            // below threshold
            //  
            // Definitions:
            // w = angular velocity
            // p = goal position(controls the direction of rotation)
            // state 0 = off
            // state 1 = open or cw
            // state 2 = closed or cww
            // state 3 = hanging until co-contraction is finished

            // Check whether outputs have been reversed in output comboboxes or in sequential switching list 
            int global_flip = 1;
            //if ((switchObj.List[k].flip == 1 && i == switchObj.DoF - 1) || (dofObj.flip != 0 && i != switchObj.DoF - 1))
            //{
            //    global_flip = -1;
            //}
            if ((switchObj.List[stateObj.listPos].flip == 1 && i == switchObj.DoF - 1) || (dofObj.flipA != 0 && i != switchObj.DoF - 1))
            {
                global_flip = -1;
            }

            // Apply the first past the post algorithm
            if (dofObj.ChA.signal >= dofObj.ChA.smin && stateObj.motorState[k] != 2 && stateObj.motorState[k] != 3)
            {
                // Move CW 
                stateObj.motorState[k] = 1;
                robotObj.Motor[k].w = linear_mapping(dofObj.ChA, robotObj.Motor[k].wmax, robotObj.Motor[k].wmin);

                // Use fake velocity method if grip force limit is enabled
                if (k == 4 && BentoAdaptGripCheck.Checked == true)
                {
                    MoveFakeVelocity(k, global_flip, stateObj.motorState[k]);
                }
                // Elsewise use regular velocity method
                else
                {
                    MoveVelocity(k, global_flip, stateObj.motorState[k]);
                }
            }
            else if (dofObj.ChB.signal >= dofObj.ChB.smin && stateObj.motorState[k] != 1 && stateObj.motorState[k] != 3)
            {
                // Move CCW 
                stateObj.motorState[k] = 2;
                robotObj.Motor[k].w = linear_mapping(dofObj.ChB, robotObj.Motor[k].wmax, robotObj.Motor[k].wmin);

                if (k == 4 && BentoAdaptGripCheck.Checked == true)
                {
                    MoveFakeVelocity(k, global_flip, stateObj.motorState[k]);
                }
                // Elsewise use regular velocity method
                else
                {
                    MoveVelocity(k, global_flip, stateObj.motorState[k]);
                }
            }
            else if ((dofObj.ChA.signal < dofObj.ChA.smin && stateObj.motorState[k] == 1) || (dofObj.ChB.signal < dofObj.ChB.smin && stateObj.motorState[k] == 2))
            {
                // Stop the motor
                stateObj.motorState[k] = 0;

                if (k != 4 || BentoAdaptGripCheck.Checked == false)
                {
                    StopVelocity(k);
                }
            }

            // Bound the position values
            robotObj.Motor[k].p = bound(robotObj.Motor[k].p, robotObj.Motor[k].pmin, robotObj.Motor[k].pmax);

            // Bound the velocity values if not using grip force limit
            if (k != 4 || BentoAdaptGripCheck.Checked == false)
            {
                robotObj.Motor[k].w = bound(robotObj.Motor[k].w, robotObj.Motor[k].wmin, robotObj.Motor[k].wmax);
            }
            // Elsewise use the following code to help grip force limit work a bit better
            else
            {
                if (ID5_present_load > BentoAdaptGripCtrl.Value)
                {
                    robotObj.Motor[k].p = robotObj.Motor[k].p + 1;
                }
                else if (ID5_present_load < -BentoAdaptGripCtrl.Value)
                {
                    robotObj.Motor[k].p = robotObj.Motor[k].p - 1;
                }
            }
        }

        private void wins(DoF_ dofObj, int k, int i)
        {
            // This function acts as the greatest signal wins control option which
            // operates as follows:

            // let ch1 be an EMG sensor placed over a flexor muscle
            // let ch2 be an EMG sensor placed over an extensor muscle

            // 1)  If the signal strength of ch1 and ch2 are below threshold output
            // zero velocity(i.e.the robotic arm does not move)
            // 2)  If the signal strength of ch1 or ch2 has just increased past the
            // threshold then the channel with the highest signal strength is
            // the winner and the corresponding movement is performed with
            // proportional velocity.
            //
            // Definitions:
            // w = angular velocity
            // p = goal position(controls the direction of rotation)
            // state 0 = off
            // state 1 = open or cw
            // state 2 = closed or cww
            // state 3 = hanging until co-contraction is finished


            // Check whether outputs have been reversed in output comboboxes or in sequential switching list 
            int global_flip = 1;
            //if ((switchObj.List[k].flip == 1 && i == switchObj.DoF - 1) || (dofObj.flip != 0 && i != switchObj.DoF - 1))
            //{
            //    global_flip = -1;
            //}
            if ((switchObj.List[stateObj.listPos].flip == 1 && i == switchObj.DoF - 1) || (dofObj.flipA != 0 && i != switchObj.DoF - 1))
            {
                global_flip = -1;
            }

            // Apply the greatest signal wins algorithm
            if (dofObj.ChA.signal >= dofObj.ChA.smin && dofObj.ChA.signal >= dofObj.ChB.signal && stateObj.motorState[k] != 3)
            {
                // Move CW 
                stateObj.motorState[k] = 1;
                robotObj.Motor[k].w = linear_mapping(dofObj.ChA, robotObj.Motor[k].wmax, robotObj.Motor[k].wmin);

                // Use fake velocity method if grip force limit is enabled
                if (k == 4 && BentoAdaptGripCheck.Checked == true)
                {
                    MoveFakeVelocity(k, global_flip, stateObj.motorState[k]);
                }
                // Elsewise use regular velocity method
                else
                {
                    MoveVelocity(k, global_flip, stateObj.motorState[k]);
                }
            }
            else if (dofObj.ChB.signal >= dofObj.ChB.smin && dofObj.ChB.signal > dofObj.ChA.signal && stateObj.motorState[k] != 3)
            {
                // Move CCW 
                stateObj.motorState[k] = 2;
                robotObj.Motor[k].w = linear_mapping(dofObj.ChB, robotObj.Motor[k].wmax, robotObj.Motor[k].wmin);

                if (k == 4 && BentoAdaptGripCheck.Checked == true)
                {
                    MoveFakeVelocity(k, global_flip, stateObj.motorState[k]);
                }
                // Elsewise use regular velocity method
                else
                {
                    MoveVelocity(k, global_flip, stateObj.motorState[k]);
                }
            }
            else if ((dofObj.ChA.signal < dofObj.ChA.smin && stateObj.motorState[k] == 1) || (dofObj.ChB.signal < dofObj.ChB.smin && stateObj.motorState[k] == 2))
            {
                // Stop the motor
                stateObj.motorState[k] = 0;

                if (k != 4 || BentoAdaptGripCheck.Checked == false)
                {
                    StopVelocity(k);
                }
            }

            // Bound the position values
            robotObj.Motor[k].p = bound(robotObj.Motor[k].p, robotObj.Motor[k].pmin, robotObj.Motor[k].pmax);

            // Bound the velocity values if not using grip force limit
            if (k != 4 || BentoAdaptGripCheck.Checked == false)
            {
                robotObj.Motor[k].w = bound(robotObj.Motor[k].w, robotObj.Motor[k].wmin, robotObj.Motor[k].wmax);
            }
            // Elsewise use the following code to help grip force limit work a bit better
            else
            {
                if (ID5_present_load > BentoAdaptGripCtrl.Value)
                {
                    robotObj.Motor[k].p = robotObj.Motor[k].p + 1;
                }
                else if (ID5_present_load < -BentoAdaptGripCtrl.Value)
                {
                    robotObj.Motor[k].p = robotObj.Motor[k].p - 1;
                }
            }
        }

        private void single2(DoF_ dofObj, int k, int i)
        {
            // This function acts as the single state control option where
            // a single EMG channel is used to voluntarily control one joint
            // while otherwise the opposite joint direction will be on all the time. 
            // i.e.If MAV1 = hand open and MAV2 = hand closed then if MAV1 reaches
            // threshold the hand will start opening and if MAV1 drops below threshold
            // the hand will start closing.
            //
            // Definitions:
            // w = angular velocity
            // p = goal position(controls the direction of rotation)
            // state 0 = off
            // state 1 = open or cw
            // state 2 = closed or cww
            // state 3 = hanging until co-contraction is finished


            // Check whether outputs have been reversed in output comboboxes or in sequential switching list 
            int global_flip = 1;
            //if ((switchObj.List[k].flip == 1 && i == switchObj.DoF - 1) || (dofObj.flip != 0 && i != switchObj.DoF - 1))
            //{
            //    global_flip = -1;
            //}
            if ((switchObj.List[stateObj.listPos].flip == 1 && i == switchObj.DoF - 1) || (dofObj.flipA != 0 && i != switchObj.DoF - 1))
            {
                global_flip = -1;
            }

            // Apply the greatest signal wins algorithm
            if (dofObj.ChA.signal >= dofObj.ChA.smin && stateObj.motorState[k] != 3)
            {
                // Move CW 
                stateObj.motorState[k] = 1;
                robotObj.Motor[k].w = linear_mapping(dofObj.ChA, robotObj.Motor[k].wmax, robotObj.Motor[k].wmin);

                // Use fake velocity method if grip force limit is enabled
                if (k == 4 && BentoAdaptGripCheck.Checked == true)
                {
                    MoveFakeVelocity(k, global_flip, stateObj.motorState[k]);
                }
                // Elsewise use regular velocity method
                else
                {
                    MoveVelocity(k, global_flip, stateObj.motorState[k]);
                }
            }
            else if (dofObj.ChB.signal >= dofObj.ChB.smin && stateObj.motorState[k] != 3)
            {
                // Move CCW 
                stateObj.motorState[k] = 2;
                robotObj.Motor[k].w = linear_mapping(dofObj.ChB, robotObj.Motor[k].wmax, robotObj.Motor[k].wmin);

                if (k == 4 && BentoAdaptGripCheck.Checked == true)
                {
                    MoveFakeVelocity(k, global_flip, stateObj.motorState[k]);
                }
                // Elsewise use regular velocity method
                else
                {
                    MoveVelocity(k, global_flip, stateObj.motorState[k]);
                }
            }
            else if (dofObj.ChA.signal < dofObj.ChA.smin && stateObj.motorState[k] == 1)
            {
                // Send the joint back to its default position
                stateObj.motorState[k] = 2;
                robotObj.Motor[k].w = robotObj.Motor[k].wmax;
                MoveVelocity(k, global_flip, stateObj.motorState[k]);
                stateObj.motorState[k] = 4;
            }
            else if (dofObj.ChB.signal < dofObj.ChB.smin && stateObj.motorState[k] == 2)
            {
                // Send the joint back to its default position
                stateObj.motorState[k] = 1;
                robotObj.Motor[k].w = robotObj.Motor[k].wmax;
                MoveVelocity(k, global_flip, stateObj.motorState[k]);
                stateObj.motorState[k] = 4;
            }

            // Bound the position values
                robotObj.Motor[k].p = bound(robotObj.Motor[k].p, robotObj.Motor[k].pmin, robotObj.Motor[k].pmax);

            // Bound the velocity values if not using grip force limit
            if (k != 4 || BentoAdaptGripCheck.Checked == false)
            {
                robotObj.Motor[k].w = bound(robotObj.Motor[k].w, robotObj.Motor[k].wmin, robotObj.Motor[k].wmax);
            }
            // Elsewise use the following code to help grip force limit work a bit better
            else
            {
                if (ID5_present_load > BentoAdaptGripCtrl.Value)
                {
                    robotObj.Motor[k].p = robotObj.Motor[k].p + 1;
                }
                else if (ID5_present_load < -BentoAdaptGripCtrl.Value)
                {
                    robotObj.Motor[k].p = robotObj.Motor[k].p - 1;
                }
            }
        }

        private void joint_position2(DoF_ dofObj, int k, int i)
        {
            // This function acts as a joint position mapping option where the analog
            // signal from a two paired channels are mapped to the joint position of a given 
            // degree of freedom. The output device defines the direction of movement
            // i.e. if you reach smax you will be at the maximum range of motion of the
            // degree of freedom/direction defined in the output dropdown box. If you
            // reach smin then you will be at the maximum range of motion in the opposite
            // direction as the one defined in the output dropdown box.
            //  
            // Definitions:
            // w = angular velocity
            // p = goal position(controls the direction of rotation)
            // state 0 = off
            // state 1 = open or cw
            // state 2 = closed or cww
            // state 3 = hanging until co-contraction is finished

            // Check whether outputs have been reversed in output comboboxes or in sequential switching list 
            int global_flip = 1;
            int midpoint = ((robotObj.Motor[k].pmax - robotObj.Motor[k].pmin) / 2) + robotObj.Motor[k].pmin;
            //if ((switchObj.List[k].flip == 1 && i == switchObj.DoF - 1) || (dofObj.flip != 0 && i != switchObj.DoF - 1))
            //{
            //    global_flip = -1;
            //}
            if ((switchObj.List[stateObj.listPos].flip == 1 && i == switchObj.DoF - 1) || (dofObj.flipA != 0 && i != switchObj.DoF - 1))
            {
                global_flip = -1;
            }

            // Apply the first past the post algorithm
            if (dofObj.ChA.signal >= dofObj.ChA.smin && stateObj.motorState[k] != 2 && stateObj.motorState[k] != 3)
            {
                // Move CW 
                stateObj.motorState[k] = 1;
                if (global_flip == 1)
                {
                    robotObj.Motor[k].p = linear_mapping(dofObj.ChA, robotObj.Motor[k].pmin, midpoint);
                }
                else if (global_flip == -1)
                {
                    robotObj.Motor[k].p = linear_mapping(dofObj.ChA, robotObj.Motor[k].pmax, midpoint);
                }
                robotObj.Motor[k].w = 1023;
            }
            else if (dofObj.ChB.signal >= dofObj.ChB.smin && stateObj.motorState[k] != 1 && stateObj.motorState[k] != 3)
            {
                // Move CCW 
                stateObj.motorState[k] = 2;
                if (global_flip == 1)
                {
                    robotObj.Motor[k].p = linear_mapping(dofObj.ChB, robotObj.Motor[k].pmax, midpoint);
                }
                else if (global_flip == -1)
                {
                    robotObj.Motor[k].p = linear_mapping(dofObj.ChB, robotObj.Motor[k].pmin, midpoint);
                }
                robotObj.Motor[k].w = 1023;
            }
            else if ((dofObj.ChA.signal < dofObj.ChA.smin && stateObj.motorState[k] == 1) || (dofObj.ChB.signal < dofObj.ChB.smin && stateObj.motorState[k] == 2))
            {
                // Stop the motor
                stateObj.motorState[k] = 0;
                robotObj.Motor[k].p = midpoint;
                robotObj.Motor[k].w = 1023;

            }

            // Bound the position values
            robotObj.Motor[k].p = bound(robotObj.Motor[k].p, robotObj.Motor[k].pmin, robotObj.Motor[k].pmax);
        }

        private void joint_position1(Ch channel, int k, int flip)
        {
            // This function acts as a joint position mapping option where the analog
            // signal from a single channel is mapped to the joint position of a given 
            // degree of freedom. The output device defines the direction of movement
            // i.e. if you reach smax you will be at the maximum range of motion of the
            // degree of freedom/direction defined in the output dropdown box. If you
            // reach smin then you will be at the maximum range of motion in the opposite
            // direction as the one defined in the output dropdown box.
            //  
            // Definitions:
            // w = angular velocity
            // p = goal position(controls the direction of rotation)
            // state 0 = off
            // state 1 = open or cw
            // state 2 = closed or cww
            // state 3 = hanging until co-contraction is finished

            // Bound the signal values
            //channel.signal = bound(channel.signal, Convert.ToInt32(channel.smin), Convert.ToInt32(channel.smax)); 

            // Apply the joint position1 algorithm
            if (stateObj.motorState[k] != 3)
            {
                // Move CW 
                //stateObj.motorState[k] = 1;

                if (flip == 1)
                {
                    robotObj.Motor[k].p = linear_mapping(channel, robotObj.Motor[k].pmin, robotObj.Motor[k].pmax);
                }
                else if (flip == -1)
                {
                    robotObj.Motor[k].p = linear_mapping(channel, robotObj.Motor[k].pmax, robotObj.Motor[k].pmin);
                }
                robotObj.Motor[k].w = 400;

            }

            // Bound the position values
            robotObj.Motor[k].p = bound(robotObj.Motor[k].p, robotObj.Motor[k].pmin, robotObj.Motor[k].pmax);

            //// Bound the velocity values if not using grip force lmit
            //if (k != 4 || BentoAdaptGripCheck.Checked == false)
            //{
            //    robotObj.Motor[k].w = bound(robotObj.Motor[k].w, robotObj.Motor[k].wmin, robotObj.Motor[k].wmax);
            //}
        }

        private int toggle(Ch channel, int statePressed, Button state1, Button state2)
        {
            // This function acts as a momentary switch that allows for toggling between
            // two states on the robot such as torque on/off and run/suspend.
            // i.e. If button B is mapped to torque on/off then when it exceeds threshold
            // it will turn the torque off. If it settles below threshold and then exceeds
            // threshold it will turn the torque on and so on.
            //  
            // Definitions:
            // channel = channel class that contains the signal and threshold
            // state1,state2 = the complementary buttons in the GUI that switch between two states on the bento arm
            // statePressed = 0 -> momentary button unpressed
            // statePressed = 1 -> momentary button pressed

            if (channel.signal >= channel.smin && statePressed != 1)
            {
                // Press the button that is pressable
                if (state1.Enabled == true)
                {
                    //state1.PerformClick();
                    InvokeOnClick(state1, new EventArgs());
                }
                else if (state2.Enabled == true)
                {
                    //state2.PerformClick();
                    InvokeOnClick(state2, new EventArgs());
                }
                statePressed = 1;
                return statePressed;
            }
            else if (channel.signal < channel.smin)
            {
                // Reset the momentary button when it falls below threshold
                statePressed = 0;
                return statePressed;
            }

            return statePressed;
        }

        //private int linear_mapping(Ch channel, int k)
        //{
        //    // Cap the maximum EMG signal at smax
        //    if (channel.signal >= channel.smax)
        //    {
        //        channel.signal = Convert.ToInt32(channel.smax);
        //    }

        //    // Linear proportional mapping between signal strength and angular velocity of motor
        //    return Convert.ToInt32((robotObj.Motor[k].wmax - robotObj.Motor[k].wmin) / (channel.smax - channel.smin) * (channel.signal - channel.smin) + robotObj.Motor[k].wmin);
        //}

        private int linear_mapping(Ch channel, int motor_max, int motor_min)
        {
            // Cap the maximum EMG signal at smax
            if (channel.signal >= channel.smax)
            {
                channel.signal = Convert.ToInt32(channel.smax);
            }

            // Linear proportional mapping between signal strength and angular velocity/position of motor
            return Convert.ToInt32((motor_max - motor_min) / (channel.smax - channel.smin) * (channel.signal - channel.smin) + motor_min);
        }

        private int check_flip(int val, int dof_flip)
        {
            // Check whether outputs have been reversed in output comboboxes or in sequential switching list 
            if ((switchObj.List[stateObj.listPos].flip == 1 && val == switchObj.DoF - 1) || (dof_flip != 0 && val != switchObj.DoF - 1))
            {
                return -1;
            }
            else
            {
                return 1;
            }
        }
        private void StopVelocity(int j)
        {
            // Grab last position feedback and STOP!
            robotObj.Motor[j].p = robotObj.Motor[j].p_prev;
            robotObj.Motor[j].w = robotObj.Motor[j].wmax;
        }

        private void MoveVelocity(int j, int flip, int motorState)
        {
            // Joint velocity control where the output is a direction and a velocity
            // This particular function just needs to assign a min or max joint limit as a direction
            // CW direction is the minimum joint limit and CCW is the maximum joint limit

            // Move CW
            //if (motorState == 1 && envlimit[i])
            if (motorState == 1)
            {
                // Flip direction of movement if the output devices have been swapped from their original order or if the checkbox in the sequential switch has been checked
                if (flip < 0)
                {
                    robotObj.Motor[j].p = robotObj.Motor[j].pmax;
                }
                else
                {
                    robotObj.Motor[j].p = robotObj.Motor[j].pmin;
                }
            }
            // Move CCW 
            else if (motorState == 2)
            {
                // Flip direction of movement if the output devices have been swapped from their original order or if the checkbox in the sequential switch has been checked
                if (flip < 0)
                {
                    robotObj.Motor[j].p = robotObj.Motor[j].pmin;
                }
                else
                {
                    robotObj.Motor[j].p = robotObj.Motor[j].pmax;
                }
            }
        }

        private void MoveFakeVelocity(int j, int flip, int motorState)
        {
            // Move with fake velocity method
            // Instead of giving a direction and velocity you increment the current position
            // A slow speed is a small increment and a fast speed is a large increment
            // i.e. new position = old position + velocity_term
            // This is the preferred method when using the grip force lmit for the Bento Arm

            // Apply grip force limit if enabled
            if (j == 4 && BentoAdaptGripCheck.Checked == true)
            {
                // Move CW
                if (motorState == 1)
                {
                    if (flip > 0 ? ID5_present_load < BentoAdaptGripCtrl.Value - 10 : ID5_present_load > -BentoAdaptGripCtrl.Value + 10)
                    {
                        robotObj.Motor[j].p = robotObj.Motor[j].p - robotObj.Motor[j].w * flip / 18;
                    }
                    else if (flip > 0 ? ID5_present_load > BentoAdaptGripCtrl.Value : ID5_present_load < -BentoAdaptGripCtrl.Value)
                    {
                        robotObj.Motor[j].p = robotObj.Motor[j].p + 1 * flip;
                    }
                    robotObj.Motor[j].w = 0;
                }
                // Move CCW
                else if (motorState == 2)
                {
                    if (flip > 0 ? ID5_present_load > -BentoAdaptGripCtrl.Value + 10 : ID5_present_load < BentoAdaptGripCtrl.Value - 10)
                    {
                        robotObj.Motor[j].p = robotObj.Motor[j].p + robotObj.Motor[j].w * flip / 18;
                    }
                    else if (flip > 0 ? ID5_present_load < -BentoAdaptGripCtrl.Value : ID5_present_load > BentoAdaptGripCtrl.Value)
                    {
                        robotObj.Motor[j].p = robotObj.Motor[j].p - 1 * flip;
                    }
                    robotObj.Motor[j].w = 0;
                }
            }
            // Elsewise use fake velocity without grip force limit
            else
            {
                // Move CW
                if (motorState == 1)
                {
                    robotObj.Motor[j].p = robotObj.Motor[j].p - robotObj.Motor[j].w * flip / 18;
                    robotObj.Motor[j].w = 0;
                }
                // Move CCW
                else if (motorState == 2)
                {
                    robotObj.Motor[j].p = robotObj.Motor[j].p + robotObj.Motor[j].w * flip / 18;
                    robotObj.Motor[j].w = 0;
                }
            }
        }

        private int splitAxis(int axisValue, bool upperAxis)
        {
            // Splits analog inputs that have negative and positive components into two separate channels that are both positive
            if ((axisValue < 0) && (upperAxis == true))
            {
                return axisValue = 0;
            }
            else if ((axisValue > 0) && (upperAxis == false))
            {
                return axisValue = 0;
            }
            else
            {
                return Math.Abs(axisValue);
            }

        }

        // Helper function that filters values to see if they are greater than 0
        private bool greaterThanZero(int value)
        {
            if (value > 0)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // Helper function to convert floats to strings
        private static string FormatFloat(float v)
        {
            return string.Format("{0:F3}", v);
        }

        // Helper function to update the switching list
        private int updateList(int listPos)
        {
            if (listPos == SWITCH_NUM - 1)
            {
                return 0;
            }
            else
            {
                return listPos + 1;
            }

        }

        //Helper function to cap the MAV display at the rail
        private int signalRail(int MAV, ProgressBar bar)
        {
            if (MAV > bar.Maximum)
            {
                bar.ForeColor = Color.DarkRed;
                return bar.Maximum;
            }
            else if (MAV < bar.Minimum)
            {
                return bar.Minimum;
            }
            else
            {
                bar.ForeColor = SystemColors.Highlight;
                return MAV;
            }
        }

        // Implements the velocity ramping option used mainly with the keyboard control
        private int velocity_ramp(ref double vel, bool IsKeyDown, bool ramp_enabled, double increment, int max)
        {
            if (ramp_enabled == true)
            {
                if (IsKeyDown)
                {
                    vel = vel + increment;
                    if (vel > max)
                    {
                        vel = max;
                    }
                    return Convert.ToInt32(vel);
                }
                else if (vel > 0)
                {
                    vel = vel - increment;
                    if (vel < 0)
                    {
                        vel = 0;
                    }
                    return Convert.ToInt32(vel);
                }
            }

            return Convert.ToInt32(IsKeyDown) * max;
        }

        // bound the value between its min and max values
        private int bound(int value, int min, int max)
        {
            if (value > max)
            {
                return max;
            }
            else if (value < min)
            {
                return min;
            }
            else
            {
                return value;
            }
        }

        void update_SLRT()
        {
            //try
            //{
            //    // Scale factor so that progress bar control can show a finer resolution
            //    double scale_factor = 100;

            //    SLRT_ch[0] = Convert.ToInt32(tg.Application.Signals["MAV/s1"].GetValue() * scale_factor);
            //    SLRT_ch[1] = Convert.ToInt32(tg.Application.Signals["MAV/s2"].GetValue() * scale_factor);
            //    SLRT_ch[2] = Convert.ToInt32(tg.Application.Signals["MAV/s3"].GetValue() * scale_factor);
            //    SLRT_ch[3] = Convert.ToInt32(tg.Application.Signals["MAV/s4"].GetValue() * scale_factor);
            //    SLRT_ch[4] = Convert.ToInt32(tg.Application.Signals["MAV/s5"].GetValue() * scale_factor);
            //    SLRT_ch[5] = Convert.ToInt32(tg.Application.Signals["MAV/s6"].GetValue() * scale_factor);
            //    SLRT_ch[6] = Convert.ToInt32(tg.Application.Signals["MAV/s7"].GetValue() * scale_factor);
            //    SLRT_ch[7] = Convert.ToInt32(tg.Application.Signals["MAV/s8"].GetValue() * scale_factor);
            //}
            //catch (Exception ex)
            //{
            //    MessageBox.Show(ex.Message);
            //}

        }

        #endregion

        #endregion

        #region "Toolstrip Menu Items"
        private void AboutToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // Open the about window
            // How to make about dialog box: http://stackoverflow.com/questions/19675910/creating-about-dialog-box-in-c-sharp-form-application
            AboutBox1 a = new AboutBox1();
            a.StartPosition = FormStartPosition.CenterParent;
            a.ShowDialog();

        }

        private void ContentsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                // Open the user guide in an external pdf viewer
                // How to open file with associated application: http://stackoverflow.com/questions/10174156/open-file-with-associated-application
                System.Diagnostics.Process.Start(@"Resources\brachIOplexus_User_Guide.pdf");
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void xBoxToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                // Open mapping diagram in external picture viewer
                System.Diagnostics.Process.Start(@"Resources\xbox_controller_mapping.png");
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void mYOSequentialLeftToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                // Open mapping diagram in external picture viewer
                System.Diagnostics.Process.Start(@"Resources\myo_mapping_left.png");
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void mYOSequentialRightToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                // Open mapping diagram in external picture viewer
                System.Diagnostics.Process.Start(@"Resources\myo_mapping_right.png");
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void keyboardMultijointToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                // Open mapping diagram in external picture viewer
                System.Diagnostics.Process.Start(@"Resources\keyboard_mapping.png");
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void ExitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        #endregion

        #region "Input/Output ComboBoxes"
        // Define a class for a combo box item
        public class ComboboxItem
        {

            public int ID { get; set; }         // the ID of the combobox item (i.e. 0 is the first item in the list, 1 is the 2nd item in the last, ect)
            public int Type { get; set; }       // The type of combobox item (i.e. for input devices 0 = xbox, 1 = MYO, 2 = keyboard, ect)
            public string Text { get; set; }    // The corresponding text that is displayed for the item in the combo box

            public override string ToString()
            {
                return Text;
            }
        }

        private void tabControl1_Deselecting(object sender, TabControlCancelEventArgs e)
        {
            if (tabControl1.SelectedTab.Name == "tabIO")
            {
                // Update the combobox lists when changing from the input/output tab to some other tab
                updateCheckedLists();
            }
        }

        private void tabControl1_SelectedIndexChanged(object sender, EventArgs e)
        {
            // Manually calling the Refresh() command on mainForm causes the tabs to redraw significantly faster when changing from one tab to another
            this.Refresh();
        }

        private void updateCheckedLists()
        {
            //// Add selected items to combobox
            //// Include type and ID properties that will help for sorting: https://stackoverflow.com/questions/3063320/combobox-adding-text-and-value-to-an-item-no-binding-source
            string InputBoxText = InputComboBox.GetItemText(InputComboBox.SelectedItem);
            string OutputBoxText = OutputComboBox.GetItemText(OutputComboBox.SelectedItem);
            InputComboBox.Items.Clear();
            OutputComboBox.Items.Clear();

            // Update input device list
            ComboboxItem item = new ComboboxItem();
            item.ID = 0;
            item.Type = 0;
            item.Text = "Off";
            InputComboBox.Items.Add(item);

            comboBox_AddItems(0, InputComboBox, XBoxList);
            comboBox_AddItems(1, InputComboBox, MYOlist);
            comboBox_AddItems(2, InputComboBox, KBlist);
            comboBox_AddItems(3, InputComboBox, biopatrecList);
            comboBox_AddItems(4, InputComboBox, ArduinoInputList);
            comboBox_AddItems(5, InputComboBox, SLRTlist);
            comboBox_AddItems(6, InputComboBox, udpList);
            InputComboBox.SelectedIndex = InputComboBox.FindStringExact(InputBoxText); // Keep selected item persistant when the list changes


            // Update output device list
            item = new ComboboxItem();
            item.ID = 0;
            item.Type = 0;
            item.Text = "Off";
            OutputComboBox.Items.Add(item);

            comboBox_AddItems(0, OutputComboBox, BentoList);
            comboBox_AddItems(1, OutputComboBox, HANDiList);
            OutputComboBox.SelectedIndex = OutputComboBox.FindStringExact(OutputBoxText); // Keep selected item persistant when the list changes 

            // Copy list to the other input device comboboxes on the mapping tab
            var list = InputComboBox.Items.Cast<Object>().ToArray();
            comboBox_CopyItems(doF1.channel1.inputBox, list);
            comboBox_CopyItems(doF1.channel2.inputBox, list);
            comboBox_CopyItems(doF2.channel1.inputBox, list);
            comboBox_CopyItems(doF2.channel2.inputBox, list);
            comboBox_CopyItems(doF3.channel1.inputBox, list);
            comboBox_CopyItems(doF3.channel2.inputBox, list);
            comboBox_CopyItems(doF4.channel1.inputBox, list);
            comboBox_CopyItems(doF4.channel2.inputBox, list);
            comboBox_CopyItems(doF5.channel1.inputBox, list);
            comboBox_CopyItems(doF5.channel2.inputBox, list);
            comboBox_CopyItems(doF6.channel1.inputBox, list);
            comboBox_CopyItems(doF6.channel2.inputBox, list);
            comboBox_CopyItems(switchInputBox, list);

            // Copy list to the other output device comboboxes on the mapping tab
            list = OutputComboBox.Items.Cast<Object>().ToArray();
            comboBox_CopyItems(doF1.channel1.outputBox, list);
            comboBox_CopyItems(doF1.channel2.outputBox, list);
            comboBox_CopyItems(doF2.channel1.outputBox, list);
            comboBox_CopyItems(doF2.channel2.outputBox, list);
            comboBox_CopyItems(doF3.channel1.outputBox, list);
            comboBox_CopyItems(doF3.channel2.outputBox, list);
            comboBox_CopyItems(doF4.channel1.outputBox, list);
            comboBox_CopyItems(doF4.channel2.outputBox, list);
            comboBox_CopyItems(doF5.channel1.outputBox, list);
            comboBox_CopyItems(doF5.channel2.outputBox, list);
            comboBox_CopyItems(doF6.channel1.outputBox, list);
            comboBox_CopyItems(doF6.channel2.outputBox, list);
        }

        // Populate the comboboxes from the selected items in the corresponding checked list box
        private void comboBox_AddItems(int Type, ComboBox DeviceList, CheckedListBox CheckedList)
        {
            foreach (object itemChecked in CheckedList.CheckedItems)
            {
                ComboboxItem item = new ComboboxItem();

                item.ID = Convert.ToInt32(CheckedList.FindString(itemChecked.ToString()));
                item.Type = Type;
                item.Text = itemChecked.ToString();

                DeviceList.Items.Add(item);
            }
        }

        // Copy items from one combobox to another
        private void comboBox_CopyItems(ComboBox DeviceList, object[] items)
        {
            string BoxText = DeviceList.GetItemText(DeviceList.SelectedItem);
            DeviceList.Items.Clear();
            DeviceList.Items.AddRange(items);
            DeviceList.SelectedIndex = DeviceList.FindStringExact(BoxText);
        }

        // filter the values from the output combobox
        private void filterOutputComboBox(object sender, EventArgs e)
        {
            Channel changed = (Channel)sender;
            List<DoF> dof_list = this.tabControl1.TabPages[1].Controls.OfType<DoF>().ToList<DoF>();
            dof_list.Reverse();

            int index = 0;
            foreach (DoF dof in dof_list)
            {
                // Assign the new values to the appropriate channel NOTE: The list needed to be reversed in order for this to work properly with the index starting at 0 and incrementing.
                if (dof.channel1 == changed)
                {
                    if (dof.channel1.outputBox.SelectedIndex > 0)
                    {
                        // Stop the joint that was previously assigned in order to prevent it from continuing to move
                        if (dofObj[index].ChA.signal >= dofObj[index].ChA.smin)
                        {
                            dofStop[0] = dofObj[index].ChA.output.Type;
                            dofStop[1] = dofObj[index].ChA.output.ID;
                        }
                        dofObj[index].ChA.output.Type = (changed.outputBox.SelectedItem as ComboboxItem).Type;
                        dofObj[index].ChA.output.ID = (changed.outputBox.SelectedItem as ComboboxItem).ID;
                        dofObj[index].flipA = (changed.outputBox.SelectedItem as ComboboxItem).ID % 2;     // checks whether it is even or odd (!= 0 means its odd, == 0 means its even)
                    }
                    dofObj[index].ChA.Enabled = setEnabled(dof.channel1.inputBox.SelectedIndex, dof.channel1.outputBox.SelectedIndex);
                }
                else if (dof.channel2 == changed)
                {
                    if (dof.channel2.outputBox.SelectedIndex > 0)
                    {
                        // Stop the joint that was previously assigned in order to prevent it from continuing to move
                        if (dofObj[index].ChB.signal >= dofObj[index].ChB.smin)
                        {
                            dofStop[0] = dofObj[index].ChB.output.Type;
                            dofStop[1] = dofObj[index].ChB.output.ID;
                        }
                        dofObj[index].ChB.output.Type = (changed.outputBox.SelectedItem as ComboboxItem).Type;
                        dofObj[index].ChB.output.ID = (changed.outputBox.SelectedItem as ComboboxItem).ID;
                        dofObj[index].flipB = (changed.outputBox.SelectedItem as ComboboxItem).ID % 2;     // checks whether it is even or odd (!= 0 means its odd, == 0 means its even)
                    }
                    dofObj[index].ChB.Enabled = setEnabled(dof.channel2.inputBox.SelectedIndex, dof.channel2.outputBox.SelectedIndex);
                }

                // Only apply these automations if two channel mappings are selected. Otherwise ignore them
                if (changed.mappingBox.SelectedIndex <= 3)
                {
                    autoFill(dof.channel1.outputBox, dof.channel2.outputBox, changed.outputBox, 10);
                    autoOff(dof.channel1.outputBox, dof.channel2.outputBox, changed.outputBox, 10);
                }
                autoDeselect(dof.channel1.outputBox, dof.channel2.outputBox, changed.outputBox);
                index++;
            }
        }

        // filter the values from the input combobox
        private void filterInputComboBox(object sender, EventArgs e)
        {
            // Auto-suspend when input device is changed in mapping tab
            //InvokeOnClick(BentoSuspend, new EventArgs());
            autosuspend_flag = true;

            Channel changed = (Channel)sender;
            List<DoF> dof_list = this.tabControl1.TabPages[1].Controls.OfType<DoF>().ToList<DoF>();
            dof_list.Reverse();

            int index = 0;
            foreach (DoF dof in dof_list)
            {
                // Assign the new values to the appropriate channel NOTE: The list needed to be reversed in order for this to work properly with the index starting at 0 and incrementing.
                if (dof.channel1 == changed)
                {
                    if (dof.channel1.inputBox.SelectedIndex > 0)
                    {
                        dofObj[index].ChA.input.Type = (changed.inputBox.SelectedItem as ComboboxItem).Type;
                        dofObj[index].ChA.input.ID = (changed.inputBox.SelectedItem as ComboboxItem).ID;
                    }
                    dofObj[index].ChA.Enabled = setEnabled(dof.channel1.inputBox.SelectedIndex, dof.channel1.outputBox.SelectedIndex);
                }
                else if (dof.channel2 == changed)
                {
                    if (dof.channel2.inputBox.SelectedIndex > 0)
                    {
                        dofObj[index].ChB.input.Type = (changed.inputBox.SelectedItem as ComboboxItem).Type;
                        dofObj[index].ChB.input.ID = (changed.inputBox.SelectedItem as ComboboxItem).ID;
                    }
                    dofObj[index].ChB.Enabled = setEnabled(dof.channel2.inputBox.SelectedIndex, dof.channel2.outputBox.SelectedIndex);
                }
                autoFill(dof.channel1.inputBox, dof.channel2.inputBox, changed.inputBox, 8);
                index++;
            }
        }

        // filter the values from the mapping combobox
        private void filterMappingComboBox(object sender, EventArgs e)
        {
            Channel changed = (Channel)sender;
            List<DoF> dof_list = this.tabControl1.TabPages[1].Controls.OfType<DoF>().ToList<DoF>();
            dof_list.Reverse();

            // Auto-suspend the arm if the mapping is set to Joint Velocity 1
            if (changed.mappingBox.SelectedIndex == 4)
            {
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            int index = 0;
            foreach (DoF dof in dof_list)
            {
                // Assign the new values to the appropriate channel NOTE: The list needed to be reversed in order for this to work properly with the index starting at 0 and incrementing.
                if (dof.channel1 == changed)
                {
                    dofObj[index].ChA.mapping = Convert.ToInt32(changed.mappingBox.SelectedIndex);
                }
                else if (dof.channel2 == changed)
                {
                    dofObj[index].ChB.mapping = Convert.ToInt32(changed.mappingBox.SelectedIndex);
                }
                autoFillMapping(dof.channel1.mappingBox, dof.channel2.mappingBox, changed.mappingBox, 3);
                index++;
            }
        }

        // filter the values from the Gain control
        private void filterGainValue(object sender, EventArgs e)
        {
            Channel changed = (Channel)sender;
            List<DoF> dof_list = this.tabControl1.TabPages[1].Controls.OfType<DoF>().ToList<DoF>();
            dof_list.Reverse();

            int index = 0;
            foreach (DoF dof in dof_list)
            {
                // Assign the new values to the appropriate channel NOTE: The list needed to be reversed in order for this to work properly with the index starting at 0 and incrementing.
                if (dof.channel1 == changed)
                {
                    dofObj[index].ChA.gain = changed.gainCtrl.Value;
                }
                else if (dof.channel2 == changed)
                {
                    dofObj[index].ChB.gain = changed.gainCtrl.Value;
                }
                index++;
            }
        }

        // filter the values from the Smin control
        private void filterSminValue(object sender, EventArgs e)
        {
            Channel changed = (Channel)sender;
            List<DoF> dof_list = this.tabControl1.TabPages[1].Controls.OfType<DoF>().ToList<DoF>();
            dof_list.Reverse();

            int index = 0;
            foreach (DoF dof in dof_list)
            {
                // Assign the new values to the appropriate channel NOTE: The list needed to be reversed in order for this to work properly with the index starting at 0 and incrementing.
                if (dof.channel1 == changed)
                {
                    dofObj[index].ChA.smin = changed.sminCtrl.Value * 100;
                }
                else if (dof.channel2 == changed)
                {
                    dofObj[index].ChB.smin = changed.sminCtrl.Value * 100;
                }
                index++;
            }
        }

        // filter the values from the Smin control
        private void filterSmaxValue(object sender, EventArgs e)
        {
            Channel changed = (Channel)sender;
            List<DoF> dof_list = this.tabControl1.TabPages[1].Controls.OfType<DoF>().ToList<DoF>();
            dof_list.Reverse();

            int index = 0;
            foreach (DoF dof in dof_list)
            {
                // Assign the new values to the appropriate channel NOTE: The list needed to be reversed in order for this to work properly with the index starting at 0 and incrementing.
                if (dof.channel1 == changed)
                {
                    dofObj[index].ChA.smax = changed.smaxCtrl.Value * 100;
                }
                else if (dof.channel2 == changed)
                {
                    dofObj[index].ChB.smax = changed.smaxCtrl.Value * 100;
                }
                index++;
            }
        }

        // Used to set whether a channel is enabled or not. The enabled state is used in the main loop to determine whether a given DoF is enabled or not.
        private bool setEnabled(int valueA, int valueB)
        {
            if (valueA > 0 && valueB > 0)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // used to auto deselect when duplicate values are selected
        private void autoDeselect(ComboBox comboBox1, ComboBox comboBox2, ComboBox changed)
        {

            if (comboBox1 != changed && OutputComboBox.GetItemText(comboBox1.SelectedItem) == changed.Text)
            {
                comboBox1.SelectedIndex = 0;
            }

            if (comboBox2 != changed && OutputComboBox.GetItemText(comboBox2.SelectedItem) == changed.Text)
            {
                comboBox2.SelectedIndex = 0;
            }
        }

        // Used to autofill output values when one of the bento arm DoF are selected in the output dropboxes 
        private void autoFill(ComboBox comboBox1, ComboBox comboBox2, ComboBox changed, int index)
        {
            if (comboBox1 == changed && changed.SelectedIndex > 0 && (changed.SelectedItem as ComboboxItem).Type == 0)
            {
                if ((changed.SelectedItem as ComboboxItem).Type == 0 && (changed.SelectedItem as ComboboxItem).ID < index)
                {
                    if (((changed.SelectedItem as ComboboxItem).ID % 2) == 0)
                    {
                        //if even
                        comboBox2.SelectedIndex = comboBox1.SelectedIndex + 1;
                    }
                    else
                    {
                        //if odd
                        comboBox2.SelectedIndex = comboBox1.SelectedIndex - 1;
                    }
                }
            }
            else if (comboBox2 == changed && changed.SelectedIndex > 0 && (changed.SelectedItem as ComboboxItem).Type == 0)
            {
                if ((changed.SelectedItem as ComboboxItem).Type == 0 && (changed.SelectedItem as ComboboxItem).ID < index)
                {
                    if (((changed.SelectedItem as ComboboxItem).ID % 2) == 0)
                    {
                        //if even
                        comboBox1.SelectedIndex = comboBox2.SelectedIndex + 1;
                    }
                    else
                    {
                        //if odd
                        comboBox1.SelectedIndex = comboBox2.SelectedIndex - 1;
                    }
                }
            }
        }

        // used to autofill entries into mapping combobox if a two channel mapping is selected. If a one channel mapping is selected nothing will happen.
        private void autoFillMapping(ComboBox comboBox1, ComboBox comboBox2, ComboBox changed, int index)
        {

            if (comboBox1 == changed && changed.SelectedIndex <= index)
            {
                comboBox2.SelectedIndex = changed.SelectedIndex;
            }
            else if (comboBox2 == changed && changed.SelectedIndex <= index)
            {
                comboBox1.SelectedIndex = changed.SelectedIndex;
            }
        }

        // auto fill both channels with "off" if one of them is assigned to a DoF. This will need to be changed when implementing position control, so that it does not autofill in that case.
        private void autoOff(ComboBox comboBox1, ComboBox comboBox2, ComboBox changed, int index)
        {
            if (comboBox2.SelectedItem != null)
            {
                if (comboBox1 == changed && changed.SelectedIndex == 0 && (comboBox2.SelectedItem as ComboboxItem).ID < index)
                {
                    if (comboBox2.SelectedIndex > 0)
                    {
                        comboBox2.SelectedIndex = 0;
                    }
                }
            }
            if (comboBox1.SelectedItem != null)
            {
                if (comboBox2 == changed && changed.SelectedIndex == 0 && (comboBox1.SelectedItem as ComboboxItem).ID < index)
                {
                    if (comboBox1.SelectedIndex > 0)
                    {
                        comboBox1.SelectedIndex = 0;
                    }
                }
            }
        }

        private void BentoList_ItemCheck(object sender, ItemCheckEventArgs e)
        {
            this.BeginInvoke((MethodInvoker)delegate
            {
                // Double check corresponding items on the bento arm checked list box. i.e. if you select hand open it will autoselect hand close
                double_check(BentoList, 9, e);
            });
        }

        private void XBoxList_ItemCheck(object sender, ItemCheckEventArgs e)
        {
            this.BeginInvoke((MethodInvoker)delegate
            {
                // Double check corresponding items on the xbox checked list box. i.e. if you select 'StickLeftX1' it will autoselect 'StickLeftX2'
                double_check(XBoxList, 7, e);
            });
        }

        // Double check corresponding items on a list
        private void double_check(CheckedListBox CheckedList, int index, ItemCheckEventArgs e)
        {
            if (CheckedList.SelectedIndex <= index)
            {
                if (e.NewValue == CheckState.Checked)
                {
                    {
                        if ((CheckedList.SelectedIndex % 2) == 0)
                        {
                            //if even
                            CheckedList.SetItemChecked(CheckedList.SelectedIndex + 1, true);
                        }
                        else if (CheckedList.SelectedIndex > 0)
                        {
                            //if odd
                            CheckedList.SetItemChecked(CheckedList.SelectedIndex - 1, true);
                        }
                    }
                }
                else if (e.NewValue == CheckState.Unchecked)
                {
                    if ((CheckedList.SelectedIndex % 2) == 0)
                    {
                        //if even
                        CheckedList.SetItemChecked(CheckedList.SelectedIndex + 1, false);
                    }
                    else if (CheckedList.SelectedIndex > 0)
                    {
                        //if odd
                        CheckedList.SetItemChecked(CheckedList.SelectedIndex - 1, false);
                    }
                }
            }
        }

        // Used for initial testing. Not currently in use.
        private void button1_Click(object sender, EventArgs e)
        {

            // Add selected items to combobox
            // Include type and ID properties that will help for sorting: https://stackoverflow.com/questions/3063320/combobox-adding-text-and-value-to-an-item-no-binding-source
            comboBox2.Items.Clear();

            foreach (object itemChecked in checkedListDairy.CheckedItems)
            {
                //comboBox2.Items.Add(itemChecked);
                ComboboxItem item = new ComboboxItem();

                item.ID = Convert.ToInt32(checkedListDairy.FindString(itemChecked.ToString()) + 1);
                item.Type = 1;
                item.Text = itemChecked.ToString();


                comboBox2.Items.Add(item);
            }

            foreach (object itemChecked in checkedListFruit.CheckedItems)
            {
                //comboBox2.Items.Add(itemChecked);
                ComboboxItem item = new ComboboxItem();

                item.ID = Convert.ToInt32(checkedListFruit.FindString(itemChecked.ToString()) + 1);
                item.Type = 2;
                item.Text = itemChecked.ToString();


                comboBox2.Items.Add(item);
            }

            // comboBox2.SelectedIndex = -1;


        }

        // Used for initial testing. Not currently in use.
        private void button14_Click(object sender, EventArgs e)
        {
            if (doF1.channel1.inputBox.SelectedIndex > -1)
            {
                labelID.Text = (doF1.channel1.outputBox.SelectedItem as ComboboxItem).ID.ToString();
                labelType.Text = (doF1.channel1.outputBox.SelectedItem as ComboboxItem).Type.ToString();
                labelText.Text = (doF1.channel1.outputBox.SelectedItem as ComboboxItem).Text;

            }
            //BentoList.SetItemChecked(3, false);

            // Write the profile settings

            bool[] test = new bool[XBoxList.Items.Count];

            for (int i = 0; i <= (XBoxList.Items.Count - 1); i++)
            {
                test[i] = Convert.ToBoolean(XBoxList.GetItemCheckState(i));
            }
        }
        #endregion

        #region "Sequential Switch"
        // Do the following when the DoF is changed in the sequential switching groupbox
        private void switchDoFbox_SelectedIndexChanged(object sender, EventArgs e)
        {
            switchObj.DoF = switchDoFbox.SelectedIndex;
            if (switchObj.DoF == 0)
            {
                for (int k = 0; k < BENTO_NUM; k++)
                {
                    stateObj.motorState[k] = 0;
                }
            }

            // Used to grey out output device selection when the sequential switch is assigned to a dof
            // reference: https://stackoverflow.com/questions/43021/how-do-you-get-the-index-of-the-current-iteration-of-a-foreach-loop
            List<DoF> dof_list = this.tabControl1.TabPages[1].Controls.OfType<DoF>().ToList<DoF>();
            dof_list.Reverse(); // Need to do this because for some reason the list enumerates backwards

            var listEnumerator = dof_list.GetEnumerator();  // Get enumerator

            for (var i = 0; listEnumerator.MoveNext() == true; i++)
            {
                DoF dof = listEnumerator.Current; // Get current item.
                if (i == switchObj.DoF - 1)
                {
                    dof.channel1.outputBox.Enabled = false;
                    dof.channel2.outputBox.Enabled = false;
                    dof.channel1.mappingBox.Enabled = false;
                    dof.channel2.mappingBox.Enabled = false;
                }
                else
                {
                    dof.channel1.outputBox.Enabled = true;
                    dof.channel2.outputBox.Enabled = true;
                    dof.channel1.mappingBox.Enabled = true;
                    dof.channel2.mappingBox.Enabled = true;
                }

            }

        }

        private void switchDoFbox_Enter(object sender, EventArgs e)
        {
            // Auto-suspend the Bento Arm as soon as the control enters focus
            //InvokeOnClick(BentoSuspend, new EventArgs());
            autosuspend_flag = true;
        }

        // Update the switching mode when the index is changed
        private void switchModeBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            switchObj.mode = switchModeBox.SelectedIndex;
            stateObj.switchState = 0;

            // If co-contraction is enabled then disable switchInputBox
            if (switchModeBox.SelectedIndex == 1)
            {
                switchInputBox.Enabled = false;
                switchSminCtrl2.Enabled = true;
                switchSmaxCtrl2.Enabled = true;
            }
            else
            {
                switchInputBox.Enabled = true;
                switchSminCtrl2.Enabled = false;
                switchSmaxCtrl2.Enabled = false;
            }
        }

        // Update the button used for triggering switching when the index is changed
        private void switchInputBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            switchObj.input = switchInputBox.SelectedIndex;
        }

        // Update the gain for the switching channel
        private void switchGainCtrl1_ValueChanged(object sender, EventArgs e)
        {
            switchObj.gain = switchGainCtrl1.Value;
        }

        // Update the minimum threshold for the switching channel
        private void switchSminCtrl1_ValueChanged(object sender, EventArgs e)
        {
            switchObj.smin1 = switchSminCtrl1.Value * 100;
            // Adjust the position of the smin tick and label to reflect changes to the smin value
            switchSminTick1.Location = new Point(switch_tick_position(Convert.ToDouble(switchSminCtrl1.Value)), switchSminTick1.Location.Y);
            switchSminLabel1.Location = new Point(switch_tick_position(Convert.ToDouble(switchSminCtrl1.Value)) - switchSminLabel1.Width / 2 + switchSminTick1.Width / 2, switchSminLabel1.Location.Y);
        }

        // Update the maximum threshold for the switching channel
        private void switchSmaxCtrl1_ValueChanged(object sender, EventArgs e)
        {
            switchObj.smax1 = switchSmaxCtrl1.Value * 100;
            // Adjust the position of the smin tick and label to reflect changes to the smin value
            switchSmaxTick1.Location = new Point(switch_tick_position(Convert.ToDouble(switchSmaxCtrl1.Value)), switchSmaxTick1.Location.Y);
            switchSmaxLabel1.Location = new Point(switch_tick_position(Convert.ToDouble(switchSmaxCtrl1.Value)) - switchSmaxLabel1.Width / 2 + switchSmaxTick1.Width / 2, switchSmaxLabel1.Location.Y);
        }

        private void switchSminCtrl2_ValueChanged(object sender, EventArgs e)
        {
            switchObj.smin2 = switchSminCtrl2.Value * 100;
            // Adjust the position of the smin tick and label to reflect changes to the smin value
            switchSminTick2.Location = new Point(switch_tick_position(Convert.ToDouble(switchSminCtrl2.Value)), switchSminTick2.Location.Y);
            switchSminLabel2.Location = new Point(switch_tick_position(Convert.ToDouble(switchSminCtrl2.Value)) - switchSminLabel2.Width / 2 + switchSminTick2.Width / 2, switchSminLabel2.Location.Y);
        }

        private void switchSmaxCtrl2_ValueChanged(object sender, EventArgs e)
        {
            switchObj.smax2 = switchSmaxCtrl2.Value * 100;
            // Adjust the position of the smin tick and label to reflect changes to the smin value
            switchSmaxTick2.Location = new Point(switch_tick_position(Convert.ToDouble(switchSmaxCtrl2.Value)), switchSmaxTick2.Location.Y);
            switchSmaxLabel2.Location = new Point(switch_tick_position(Convert.ToDouble(switchSmaxCtrl2.Value)) - switchSmaxLabel2.Width / 2 + switchSmaxTick2.Width / 2, switchSmaxLabel2.Location.Y);
        }

        //Helper function to control position of threshold ticks and labels
        public int switch_tick_position(double x)
        {
            //return Convert.ToInt32(35.4 * voltage + 52);
            return Convert.ToInt32(switchSignalBar1.Width / Convert.ToDouble(switchSminCtrl1.Maximum) * x + switchSignalBar1.Location.X);
        }

        private void switchTimeCtrl1_ValueChanged(object sender, EventArgs e)
        {
            switchObj.cctime = switchTimeCtrl1.Value;
        }

        private void switch1OutputBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            switchObj.List[0].output = switch1OutputBox.SelectedIndex;
        }

        private void switch2OutputBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            switchObj.List[1].output = switch2OutputBox.SelectedIndex;
        }

        private void switch3OutputBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            switchObj.List[2].output = switch3OutputBox.SelectedIndex;
        }

        private void switch4OutputBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            switchObj.List[3].output = switch4OutputBox.SelectedIndex;
        }

        private void switch5OutputBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            switchObj.List[4].output = switch5OutputBox.SelectedIndex;
        }

        private void switch1MappingBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            // Auto-suspend the arm if the mapping is set to Joint Velocity 1
            if (switch1MappingBox.SelectedIndex == 2)
            {
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            switchObj.List[0].mapping = switch1MappingBox.SelectedIndex;
        }

        private void switch2MappingBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            // Auto-suspend the arm if the mapping is set to Joint Velocity 1
            if (switch2MappingBox.SelectedIndex == 2)
            {
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            switchObj.List[1].mapping = switch2MappingBox.SelectedIndex;
        }

        private void switch3MappingBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            // Auto-suspend the arm if the mapping is set to Joint Velocity 1
            if (switch3MappingBox.SelectedIndex == 2)
            {
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            switchObj.List[2].mapping = switch3MappingBox.SelectedIndex;
        }

        private void switch4MappingBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            // Auto-suspend the arm if the mapping is set to Joint Velocity 1
            if (switch4MappingBox.SelectedIndex == 2)
            {
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            switchObj.List[3].mapping = switch4MappingBox.SelectedIndex;
        }

        private void switch5MappingBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            // Auto-suspend the arm if the mapping is set to Joint Velocity 1
            if (switch5MappingBox.SelectedIndex == 2)
            {
                InvokeOnClick(BentoSuspend, new EventArgs());
            }

            switchObj.List[4].mapping = switch5MappingBox.SelectedIndex;
        }

        private void switch1Flip_CheckedChanged(object sender, EventArgs e)
        {
            switchObj.List[0].flip = Convert.ToInt32(switch1Flip.Checked);
        }

        private void switch2Flip_CheckedChanged(object sender, EventArgs e)
        {
            switchObj.List[1].flip = Convert.ToInt32(switch2Flip.Checked);
        }

        private void switch3Flip_CheckedChanged(object sender, EventArgs e)
        {
            switchObj.List[2].flip = Convert.ToInt32(switch3Flip.Checked);
        }

        private void switch4Flip_CheckedChanged(object sender, EventArgs e)
        {
            switchObj.List[3].flip = Convert.ToInt32(switch4Flip.Checked);
        }

        private void switch5Flip_CheckedChanged(object sender, EventArgs e)
        {
            switchObj.List[4].flip = Convert.ToInt32(switch5Flip.Checked);
        }

        // Provide user selectable feedback to the user when switching events occur
        private void updateSwitchFeedback()
        {
            if (textBox.Checked == true)
            {
                switchLabel.Text = switch1OutputBox.Items[switchObj.List[stateObj.listPos].output].ToString();
            }

            if (dingBox.Checked == true)
            {
                //System.Media.SoundPlayer player = new System.Media.SoundPlayer();
                player.SoundLocation = @"C:\windows\media\ding.wav";
                player.Play();
            }

            int DoF_test = switchObj.List[stateObj.listPos].output;

            if (vocalBox.Checked == true)
            {
                try
                {
                    switch (DoF_test)
                    {
                        case 0:
                            player.SoundLocation = @"C:\windows\media\ding.wav";
                            player.Play();
                            break;
                        case 1:
                            player.SoundLocation = @"Resources\switching_sounds\should02.wav";
                            player.Play();
                            break;
                        case 2:
                            player.SoundLocation = @"Resources\switching_sounds\elbow001.wav";
                            player.Play();
                            break;
                        case 3:
                            player.SoundLocation = @"Resources\switching_sounds\rotate01.wav";
                            player.Play();
                            break;
                        case 4:
                            player.SoundLocation = @"Resources\switching_sounds\wrist001.wav";
                            player.Play();
                            break;
                        case 5:
                            player.SoundLocation = @"Resources\switching_sounds\hand0001.wav";
                            player.Play();
                            break;
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            }
        }

        // Check mouse click events to hide/show the sequential switching groupbox
        // Any mouse click will make the group box visible and a right click will hide the group box
        private void switchSignalBar1_MouseClick(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            if (e.Button == System.Windows.Forms.MouseButtons.Right && groupBox16.Visible == true)
            {
                groupBox16.Visible = false;
            }
            else if (groupBox16.Visible == false)
            {
                groupBox16.Visible = true;
            }
        }

        // Only allow one of the two auditory feedback methods for sequential switching to be active at a time
        private void dingBox_CheckedChanged(object sender, EventArgs e)
        {
            if (dingBox.Checked == true && vocalBox.Checked == true)
            {
                vocalBox.Checked = false;
            }
        }

        private void vocalBox_CheckedChanged(object sender, EventArgs e)
        {
            if (dingBox.Checked == true && vocalBox.Checked == true)
            {
                dingBox.Checked = false;
            }
        }

        #endregion

        #region "BioPatRec Communication"

        private void biopatrecConnect_Click(object sender, EventArgs e)
        {
            try
            {
                // Set port and address for the TcpListener
                Int32 port = Convert.ToInt32(biopatrecIPport.Text);
                //IPAddress localAddr = IPAddress.Parse("127.0.0.1");     // address for localhost
                //IPAddress localAddr = IPAddress.Any;                    // listen for client activity on all network interfaces (equivalent to 0.0.0.0)
                IPAddress localAddr = IPAddress.Parse(biopatrecIPaddr.Text);                     // listen for client activity on all network interfaces (equivalent to 0.0.0.0)

                // Initialize the server object and wait for client to connect
                listener = new TcpListener(localAddr, port);
                listener.Start();
                client = listener.AcceptTcpClient();
                netStream = client.GetStream();
                t = new Thread(DoWork);
                t.Start();

                // Re-configure the GUI when BioPatRec is connected
                biopatrecGroupBox.Enabled = true;
                biopatrecDisconnect.Enabled = true;
                biopatrecConnect.Enabled = false;
                biopatrecList.Enabled = true;
                biopatrecSelectAll.Enabled = true;
                biopatrecClearAll.Enabled = true;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }

        }

        private void biopatrecDisconnect_Click(object sender, EventArgs e)
        {
            // Re-configure the GUI when BioPatRec is disconnected
            biopatrecGroupBox.Enabled = false;
            biopatrecDisconnect.Enabled = false;
            biopatrecConnect.Enabled = true;
            biopatrecList.Enabled = false;
            biopatrecSelectAll.Enabled = false;
            biopatrecClearAll.Enabled = false;
            biopatrecConnect.Focus();
        }

        private void biopatrecSelectAll_Click(object sender, EventArgs e)
        {
            // Select all of the items in the checkedListBox
            for (int i = 0; i < biopatrecList.Items.Count; i++)
            {
                biopatrecList.SetItemChecked(i, true);
            }
        }

        private void biopatrecClearall_Click(object sender, EventArgs e)
        {
            // Unselect all of the items in the checkedListBox
            for (int i = 0; i < biopatrecList.Items.Count; i++)
            {
                biopatrecList.SetItemChecked(i, false);
            }
        }

        // This is a separate thread that is used for communicating with the client
        public void DoWork()
        {
            try
            {
                byte[] bytes = new byte[1024];

                // need to do some invoking in order to avoi invalid cross-thread operation
                // https://stackoverflow.com/questions/142003/cross-thread-operation-not-valid-control-accessed-from-a-thread-other-than-the
                int mode = 0;
                if (biopatrecMode.InvokeRequired)
                {
                    biopatrecMode.Invoke(new MethodInvoker(delegate { mode = biopatrecMode.SelectedIndex; }));
                }

                // Output - TAC mode -> use conventional EMG outputs from brachI/Oplexus to drive the simulator in biopatrec
                if (mode == 1)
                {
                    stopWatch2.Start();         // start the stop watch
                    long TAC_timestep = 30;     // this is how often in (ms) that biopatrec will send a command to biopatrec if one of the joints is active
                    while (true)
                    {
                        if (stopWatch2.ElapsedMilliseconds >= TAC_timestep)
                        {
                            // Stop stopwatch and record how long everything in the main loop took to execute as well as how long it took to retrigger the main loop
                            stopWatch2.Stop();
                            milliSec2 = stopWatch2.ElapsedMilliseconds;
                            //biopatrecDelay.Text = Convert.ToString(milliSec2);

                            if (biopatrecDelay.InvokeRequired)
                            {
                                biopatrecDelay.Invoke(new MethodInvoker(delegate { biopatrecDelay.Text = Convert.ToString(milliSec2); }));
                            }

                            // Send the class ID and velocity command to biopatrec
                            // Elbow
                            if (stateObj.motorState[1] == 1)
                            {
                                bytes[0] = 20;
                                bytes[1] = Convert.ToByte(robotObj.Motor[1].w);
                                netStream.Write(bytes, 0, 2);
                            }
                            else if (stateObj.motorState[1] == 2)
                            {
                                bytes[0] = 21;
                                bytes[1] = Convert.ToByte(robotObj.Motor[1].w);
                                netStream.Write(bytes, 0, 2);
                            }
                            // Wrist rotation
                            else if (stateObj.motorState[2] == 1)
                            {
                                bytes[0] = 5;
                                bytes[1] = Convert.ToByte(robotObj.Motor[2].w);
                                netStream.Write(bytes, 0, 2);
                            }
                            else if (stateObj.motorState[2] == 2)
                            {
                                bytes[0] = 6;
                                bytes[1] = Convert.ToByte(robotObj.Motor[2].w);
                                netStream.Write(bytes, 0, 2);
                            }
                            // Wrist flexion
                            else if (stateObj.motorState[3] == 1)
                            {
                                bytes[0] = 3;
                                bytes[1] = Convert.ToByte(robotObj.Motor[3].w);
                                netStream.Write(bytes, 0, 2);
                            }
                            else if (stateObj.motorState[3] == 2)
                            {
                                bytes[0] = 4;
                                bytes[1] = Convert.ToByte(robotObj.Motor[3].w);
                                netStream.Write(bytes, 0, 2);
                            }
                            // Hand Open/Close
                            else if (stateObj.motorState[4] == 1)
                            {
                                bytes[0] = 1;
                                bytes[1] = Convert.ToByte(robotObj.Motor[4].w);
                                netStream.Write(bytes, 0, 2);
                            }
                            else if (stateObj.motorState[4] == 2)
                            {
                                bytes[0] = 2;
                                bytes[1] = Convert.ToByte(robotObj.Motor[4].w);
                                netStream.Write(bytes, 0, 2);
                            }

                            // Used for testing/troubleshooting
                            //bytes[0] = 15;
                            //bytes[1] = 80;
                            //if (ID2_state.InvokeRequired)
                            //{
                            //    ID2_state.Invoke(new MethodInvoker(delegate { ID2_state.Text = Convert.ToString(bytes[1]); }));
                            //}

                            // Reset and start the stop watch
                            stopWatch2.Restart();
                        }
                    }
                }
                // Input mode -> use class outputs from biopatrec to drive the bento arm
                else
                {
                    //byte[] bytes = new byte[1024];
                    while (true)
                    {
                        int bytesRead = netStream.Read(bytes, 0, bytes.Length);
                        //this.SetText(Encoding.ASCII.GetString(bytes, 0, bytesRead));
                        //this.SetText(Convert.ToString(bytes[0]));
                        //string received_msg = "";
                        for (int i = 0; i < bytesRead; i++)
                        {
                            if (bytes[i] <= 24)
                            {
                                //received_msg = received_msg + "ID:" + Convert.ToString(bytes[i]) + " ";
                                biopatrec_vel = 0;
                                biopatrec_ID = bytes[i];
                                netStream.Write(bytes, i, 1);
                            }
                            if (bytes[i] > 24)
                            {
                                biopatrec_vel = bytes[i] - 25;
                                //received_msg = received_msg + "Speed:" + Convert.ToString(bytes[i]) + " ";
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        #endregion

        #region "UDP Communication"
        private void udpConnect_Click(object sender, EventArgs e)
        {
            try
            {
                // Initialize the server object and wait for client to connect
                t2 = new Thread(DoWork2);
                t2.IsBackground = true;
                Int32 portRX = Convert.ToInt32(udpIPport.Text);
                IPAddress localAddr = IPAddress.Parse(udpIPaddr.Text);                     // listen for client activity on all network interfaces (equivalent to 0.0.0.0)
                udpClientRX = new UdpClient(portRX);
                ipEndPointRX = new IPEndPoint(localAddr, portRX);
                t2.Start();

                // Re-configure the GUI when DUP is connected
                udpGroupBox.Enabled = true;
                udpDisconnect.Enabled = true;
                udpConnect.Enabled = false;
                udpList.Enabled = true;
                udpSelectAll.Enabled = true;
                udpClearAll.Enabled = true;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void udpDisconnect_Click(object sender, EventArgs e)
        {
            // Clean up the UDP client objects
            t2.Abort();
            udpClientRX.Close();     // Close the UDP client    

            // Re-configure the GUI when UDP is disconnected
            udpGroupBox.Enabled = false;
            udpDisconnect.Enabled = false;
            udpConnect.Enabled = true;
            udpList.Enabled = false;
            udpSelectAll.Enabled = false;
            udpClearAll.Enabled = false;
            udpConnect.Focus();
        }

        private void udpSelectAll_Click(object sender, EventArgs e)
        {
            // Select all of the items in the checkedListBox
            for (int i = 0; i < udpList.Items.Count; i++)
            {
                udpList.SetItemChecked(i, true);
            }
        }

        private void udpClearAll_Click(object sender, EventArgs e)
        {
            // Unselect all of the items in the checkedListBox
            for (int i = 0; i < udpList.Items.Count; i++)
            {
                udpList.SetItemChecked(i, false);
            }
        }
        // This is a separate thread that is used for communicating with the client
        public void DoWork2()
        {
            while (true)
            {

                byte[] bytes = udpClientRX.Receive(ref ipEndPointRX);

                try
                {
                    // Decode packets from UNB ACE using packet structure from UDP_Comm_Protocol_IBT_150917
                    // Calculate the checksum for the packet
                    int checksum = 0;
                    for (int p = 2; p < bytes.Length - 1; p++)
                    {
                        checksum = checksum + bytes[p];     // Add up all the bytes in the LENGTH and DATA section of hte packet
                    }

                    checksum = (byte)~checksum;     // return the bitwise complement which is equivalent to the NOT operator

                    // Only update the input values if the packet is valid
                    if (checksum == bytes[bytes.Length - 1] && bytes[0] == 255 && bytes[1] == 255)
                    {
                        for (int m = 3; m < bytes.Length - 1; m = m + 2)
                        {
                            UDP_ch[bytes[m]] = bytes[m + 1];
                        }
                    }

                    //// This is stuff for decoding packets from interactiveMyocontrol
                    //string text = Encoding.ASCII.GetString(bytes).Trim(new Char[] { '[', ']' });
                    //// Instructions how to parse strings: https://docs.microsoft.com/en-us/dotnet/csharp/how-to/parse-strings-using-split
                    //// Trim can remove the starting and ending [,] characters and split can remove the commas and split into individual strings
                    //char[] delimiterChars = { ',' };
                    //string[] words = text.Split(delimiterChars);
                    //int i = 0;
                    //foreach (var word in words)
                    //{
                    //    if (word != "")
                    //    {
                    //        UDP_ch[i] = Convert.ToDouble(word);
                    //        i++;
                    //    }
                    //}
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
                //this.SetText2(Encoding.ASCII.GetString(bytes));
            }
        }

        // Used for testing purposes
        //private void SetText2(string text)
        //{
        //    // InvokeRequired required compares the thread ID of the
        //    // calling thread to the thread ID of the creating thread.
        //    // If these threads are different, it returns true.
        //    if (this.label236.InvokeRequired)
        //    {
        //        SetTextCallback d = new SetTextCallback(SetText2);
        //        this.Invoke(d, new object[] { text });
        //    }
        //    else
        //    {
        //        this.label236.Text = text;    
        //    }
        //}
        #endregion

        #region "Task Timer"
        // Create a Task Timer for tracking how long it takes to do tasks with the Bento Arm
        // The timer is triggered when the arm first detects movement and stops when it moves back to the reset position
        // The rest of the code for this is in the main loop
        private void TaskTimerEnable_Click(object sender, EventArgs e)
        {
            if (TaskTimerEnable.Text == "Enable\r\nTimer")
            {
                TaskTimerState = 5;     // 0 = disabled, 1 = reset(and waiting for trigger), 2 = running (before movement threshold), 3 = running (after movement threshold), 4 = task completed (timer stopped, waiting for reset),  5 = resetting
                TaskTimerReset.Enabled = true;
                TaskTimerLabel.Enabled = true;
                TaskTimerValue.Enabled = true;
                TaskTimerEnable.Text = "Disable\r\nTimer";

            }
            else if (TaskTimerEnable.Text == "Disable\r\nTimer")
            {
                TaskTimerState = 0;     // 0 = disabled, 1 = reset(and waiting for trigger), 2 = running (before movement threshold), 3 = running (after movement threshold), 4 = task completed (timer stopped and waiting for reset),  5 = resetting
                TaskTimerReset.Enabled = false;
                TaskTimerLabel.Enabled = false;
                TaskTimerValue.Enabled = false;
                TaskTimerEnable.Text = "Enable\r\nTimer";
            }
        }

        // Reset the position of the arm to the reset position
        private void TaskTimerReset_Click(object sender, EventArgs e)
        {
            TaskTimerState = 5;
        }

        // Update the states of the Task Timer in the main loop
        private void UpdateTaskTimer()
        {
            if (TaskTimerState == 1 && (Math.Abs(BentoSense.ID[0].vel - 1023) > 5 || Math.Abs(BentoSense.ID[1].vel - 1023) > 5 || Math.Abs(BentoSense.ID[2].vel - 1023) > 5 || Math.Abs(BentoSense.ID[3].vel - 1023) > 5 || Math.Abs(BentoSense.ID[4].vel - 1023) > 5))
            {
                Task_Timer.Restart();
                TaskTimerState = 2;     // 0 = disabled, 1 = reset(and waiting for trigger), 2 = running (before movement threshold), 3 = running (after movement threshold), 4 = task completed (stop timer and wait for reset),  5 = resetting
                TaskTimerValue.Text = string.Format("{0:0.00}", Convert.ToDouble(Task_Timer.ElapsedMilliseconds) / 1000);
            }
            // Timer is triggered so update the label on the GUI (not yet reached the movement threshold)
            else if (TaskTimerState == 2)
            {
                TaskTimerValue.Text = string.Format("{0:0.00}", Convert.ToDouble(Task_Timer.ElapsedMilliseconds) / 1000);
                // Since stopping the timer is triggered by a position threshold (i.e. returning to the starting position) we change the state befor eand after the threshold to prevent accidental triggers
                if (BentoSense.ID[0].pos >= 1724 && BentoSense.ID[1].pos <= 2530)
                {
                    TaskTimerState = 3;
                }
            }
            // Timer is triggered so update the label on the GUI (reached the movement threshold, so if they go back to reset location then the timer will stop)
            else if (TaskTimerState == 3)
            {
                TaskTimerValue.Text = string.Format("{0:0.00}", Convert.ToDouble(Task_Timer.ElapsedMilliseconds) / 1000);
                if (BentoSense.ID[0].pos <= 1704 && BentoSense.ID[1].pos >= 2550)
                {
                    Task_Timer.Stop();
                    TaskTimerValue.Text = string.Format("{0:0.00}", Convert.ToDouble(Task_Timer.ElapsedMilliseconds) / 1000);
                    TaskTimerState = 4;     // User has returned to reset position, so stop the timer
                }
            }
            // The reset button has been triggered, so move the arm back to it's reset position. Make sure it keeps moving until it has reached the threshold
            else if (TaskTimerState == 5)
            {

                // Set the Timer back to the ready state once it has finished moving to the reset position and stopped moving
                //if (BentoSense.ID[0].pos <= 1704 && BentoSense.ID[1].pos >= 2550)
                if (BentoSense.ID[0].pos <= 1704 && BentoSense.ID[1].pos >= 2550 && Math.Abs(BentoSense.ID[2].pos - 2048) < 20 && Math.Abs(BentoSense.ID[3].pos - 2048) < 20 && Math.Abs(BentoSense.ID[4].pos - 2048) < 20 && BentoSense.ID[0].vel == 1023 && BentoSense.ID[1].vel == 1023 && BentoSense.ID[2].vel == 1023 && BentoSense.ID[3].vel == 1023 && BentoSense.ID[4].vel == 1023)
                {
                    TaskTimerValue.Text = "Ready";
                    TaskTimerState = 1;
                }
                else
                {
                    robotObj.Motor[0].p = 1684;
                    robotObj.Motor[0].w = 50;
                    robotObj.Motor[1].p = 2570;
                    robotObj.Motor[1].w = 50;
                    robotObj.Motor[2].p = 2048;
                    robotObj.Motor[2].w = 50;
                    robotObj.Motor[3].p = 2048;
                    robotObj.Motor[3].w = 50;
                    robotObj.Motor[4].p = 2048;
                    robotObj.Motor[4].w = 50;
                    TaskTimerValue.Text = "Resetting";
                }
            }
        }

        // Check mouse click events to hide/show the task timer groupbox
        // Any mouse click will make the group box visible and a right click will hide the group box
        private void TaskTimerValue_MouseClick(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            if (e.Button == System.Windows.Forms.MouseButtons.Right && TaskTimerGroupBox.Visible == true)
            {
                TaskTimerGroupBox.Visible = false;
            }
            else if (TaskTimerGroupBox.Visible == false)
            {
                TaskTimerGroupBox.Visible = true;
            }
        }

        // Add in buttons to allow clinician or user to move the gripper positions to fully opened or fully closed
        private void TaskTimerOpen_Click(object sender, EventArgs e)
        {
            robotObj.Motor[4].p = Convert.ToInt16(hand_pmax_ctrl.Value);
            robotObj.Motor[4].w = 70;
        }

        private void TaskTimerClose_Click(object sender, EventArgs e)
        {
            robotObj.Motor[4].p = Convert.ToInt16(hand_pmin_ctrl.Value);
            robotObj.Motor[4].w = 70;
        }

        #endregion

        #region "Quick Profiles"
        private void demoDelsysButton_Click(object sender, EventArgs e)
        {
            try
            {
                if (dynaConnect.Enabled == true)
                {
                    InvokeOnClick(dynaConnect, new EventArgs());
                    if (dynaConnect.Enabled == true)
                    {
                        MessageBox.Show("Could not connect to the Bento Arm. Please make sure the USB2dynamixel is plugged in and the correct comm port is selected and try again");
                    }
                    else
                    {
                        InvokeOnClick(TorqueOn, new EventArgs());
                         
                    }
                }

                InvokeOnClick(SLRTconnect, new EventArgs());
                InvokeOnClick(XboxConnect, new EventArgs());
                InvokeOnClick(BentoSuspend, new EventArgs());
                if (SLRTconnect.Enabled == true)
                {
                    MessageBox.Show("Could not connect to the SLRT computer. Please make sure the SLRT computer is turned on and that you have waited at least 60 seconds for it to boot up.");
                }
                else
                {
                    LoadParameters(@"Resources\Profiles\SLRT_multi.dat");
                    QuickProfileState = 0;
                    tabControl1.SelectedIndex = 1;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }
        private void demoXBoxButton_Click(object sender, EventArgs e)
        {
            try
            {
                if (dynaConnect.Enabled == true)
                {
                    InvokeOnClick(dynaConnect, new EventArgs());
                    if (dynaConnect.Enabled == true)
                    {
                        MessageBox.Show("Could not connect to the Bento Arm. Please make sure the USB2dynamixel is plugged in and the correct comm port is selected and try again");
                    }
                    else
                    {
                        InvokeOnClick(TorqueOn, new EventArgs());
                        InvokeOnClick(BentoRun, new EventArgs());
                    }
                }

                InvokeOnClick(XboxConnect, new EventArgs());
                if (XboxConnect.Enabled == true)
                {
                    MessageBox.Show("Could not connect to Xbox Controller. Please make sure the controller is plugged in and try again");
                }
                else
                {
                    LoadParameters(@"Resources\Profiles\xbox_multi.dat");
                    QuickProfileState = 0;
                    tabControl1.SelectedIndex = 1;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void demoMYObutton_Click(object sender, EventArgs e)
        {
            try
            {
                if (dynaConnect.Enabled == true)
                {
                    InvokeOnClick(dynaConnect, new EventArgs());
                    if (dynaConnect.Enabled == true)
                    {
                        MessageBox.Show("Could not connect to the Bento Arm. Please make sure the USB2dynamixel is plugged in and the correct comm port is selected and try again");
                    }
                    else
                    {
                        InvokeOnClick(TorqueOn, new EventArgs());
                        InvokeOnClick(BentoRun, new EventArgs());
                    }
                }
                if (MYOconnect.Enabled == true)
                {
                    InvokeOnClick(MYOconnect, new EventArgs());
                    if (MYOconnect.Enabled == true)
                    {
                        MessageBox.Show("Could not connect to MYO armband. Please make sure that the wireless dongle is plugged in and that the MYO armband is turned on and connected in the MYO connect software.");
                    }
                }
                LoadParameters(@"Resources\Profiles\MYO_sequential_left.dat");
                QuickProfileState = 1;
                tabControl1.SelectedIndex = 1;
                InvokeOnClick(XboxConnect, new EventArgs());        // Connect to xbox, so that quick profile changing can be triggered with start button on xbox
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void demoShutdownButton_Click(object sender, EventArgs e)
        {
            try
            {
                DialogResult result1 = MessageBox.Show("Please support the Bento Arm under the wrist connector to prevent it from falling. Click OK to finish shutting down the program", "Important Reminder", MessageBoxButtons.OKCancel);
                if (result1 == DialogResult.OK)
                {
                    if (dynaConnect.Enabled == false)
                    {
                        InvokeOnClick(TorqueOff, new EventArgs());
                    }
                    this.Close();
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }











        #endregion

    }
}
