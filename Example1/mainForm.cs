#region "Copyright"
/*  
'   File....... mainForm.CS
'   Purpose.... Provide a GUI for mapping EMG and joystick signals to the AX18 smart arm and the Bento Arm
'   Author..... Michael Dawson
'   Help....... mrd1@ualberta.ca
'   Started.... 7/13/2007
'   Updated.... 
'   Version.... 1.9.0.0
'
'   Simulink Realtime Dynamixel API Copyright BLINC Lab (Michael Dawson) http://forum.agaverobotics.com/
'   Simulink Realtime .NET API Copyright Mathworks (see matlab R2014a documentation for help)

'   Tool for converting from vb to c#: http://www.developerfusion.com/tools/convert/vb-to-csharp/
'   Another tool for vb to c#: http://converter.telerik.com/
*/
#endregion

using System;
using System.ComponentModel;
using System.Windows.Forms;
using System.IO.Ports;  // For communicating with the simulator
using System.Diagnostics;   // For processing simulator object
using System.Threading;
using System.Management;
using dynamixel_sdk;        // for dynamixel

namespace brachIOplexus
{
    public partial class mainForm : Form
    {

        #region "Initialization"
        // XInputDotNet - Initialize ReporterState for XBOX controller
        private ReporterState reporterState = new ReporterState();

        // add stopwatch
        Stopwatch stopWatch1 = new Stopwatch();

        // Mapping parameters
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
        UInt16 ID5_prev_state = 0;   // 0 = stopped, 1 = moving CW, 2 = moving CCW
        UInt16 ID5_goal_position = 0;
        UInt16 ID5_moving_speed = 1;
        UInt16 ID5_connected = 0;      // Is the dynamixel connected (0 = no, 1 = yes)

        // DynamixelSDK
        // Control table address
        public const int ADDR_MX_TORQUE_ENABLE = 24;                  // Control table address is different in Dynamixel model
        public const int ADDR_MX_LED = 25;
        public const int ADDR_MX_GOAL_POSITION = 30;
        public const int ADDR_MX_MOVING_SPEED = 32;
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
        public const int LEN_MX_PRESENT_POSITION = 2;
        public const int LEN_MX_PRESENT_SPEED = 2;
        public const int LEN_MX_PRESENT_LOAD = 2;
        public const int LEN_MX_PRESENT_VOLTAGE = 1;
        public const int LEN_MX_PRESENT_TEMP = 1;
        public const int LEN_MX_MOVING = 1;

        // Protocol version
        public const int PROTOCOL_VERSION = 1;                   // See which protocol version is used in the Dynamixel

        // Default setting
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

        // Initialize Groupsyncwrite group
        int write_group_num = 0;

        int dxl_comm_result = COMM_TX_FAIL;                                   // Communication result
        byte dxl_error = 0;                                                   // Dynamixel error

        bool dxl_addparam_result = false;                                     // AddParam result
        bool dxl_getdata_result = false;                                      // GetParam result


        public mainForm()
        {
            InitializeComponent();
        }

        private void mainForm_Load(object sender, EventArgs e)
        {
            // How to find com ports and populate combobox: http://stackoverflow.com/questions/13794376/combo-box-for-serial-port
            var ports = SerialPort.GetPortNames();
            cmbSerialPorts.DataSource = ports;
            
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

            // XInputDotNet - start pollingWorker
            pollingWorker.RunWorkerAsync();

        }

        private void mainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            try
            {
                // XInputDotNet - stop pollingWorker
                pollingWorker.CancelAsync();

                // Close Dynamixel port
                if (BentoGroupBox.Enabled == true)
                {
                    dynamixel.closePort(port_num);
                }
            }
            catch (Exception me)
            {
                MessageBox.Show(me.Message);
            }
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
            }
        }

        private void XboxDisconnect_Click(object sender, EventArgs e)
        {
            xBoxGroupBox.Enabled = false;
            XboxDisconnect.Enabled = false;
            XboxConnect.Enabled = true;
        }

        #endregion

        #region "Dynamixel Controller"
        // Stuff to work with DynamixelSDK
        private void dynaConnect_Click(object sender, EventArgs e)
        {
            // DynamixelSDK - Initialize PortHandler Structs
            // Set the port path
            // Get methods and members of PortHandlerLinux or PortHandlerWindows
            port_num = dynamixel.portHandler(cmbSerialPorts.SelectedItem.ToString());

            // DynamixelSDK - Initialize PacketHandler Structs
            dynamixel.packetHandler();

            //if (syncwrite_flag == false)
            //{
            //    // Initialize Groupsyncwrite instance
            //    write_group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POS_SPEED);
            //    syncwrite_flag = true;
            //}

            // Initialize Groupsyncwrite instance
            write_group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POS_SPEED);

            // Initialize Groupbulkread Structs
            read_group_num = dynamixel.groupBulkRead(port_num, PROTOCOL_VERSION);

            try
            {
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
                        // MessageBox.Show("Succeeded to open the port!");
                        dynaStatus.Text = String.Format("Port {0} opened successfully", cmbSerialPorts.SelectedItem.ToString());
                    }
                    else
                    {
                        // MessageBox.Show("Failed to open the port!");
                        dynaStatus.Text = String.Format("Failed to open port {0}", cmbSerialPorts.SelectedItem.ToString());
                        return;
                    }
                }
                else
                {
                    // MessageBox.Show("Please select a port first");
                    dynaStatus.Text = "Please select a port first";
                }

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }


            // Set port baudrate
            if (dynamixel.setBaudRate(port_num, BAUDRATE))
            {
                // MessageBox.Show("Succeeded to change the baudrate!");
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Changed baudrate to {0} bps", BAUDRATE);
            }
            else
            {
                // MessageBox.Show("Failed to change the baudrate!");
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + "Failed to change the baudrate";
                return;
            }

            // Ping each dynamixel that is supposed to be on the bus

            // ID1
            dynamixel.ping(port_num, PROTOCOL_VERSION, DXL1_ID);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
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
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);

            }
            else
            {
                // MessageBox.Show("Dynamixel ID1 has been successfully connected");
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL1_ID);

                // Add parameter storage for Dynamixel#1 feedback: position, speed, load, voltage, temperature
                dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, 10);
                if (dxl_addparam_result != true)
                {
                    // MessageBox.Show("[ID: {0}] groupBulkRead addparam failed");
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("ID{0} groupBulkRead addparam failed", DXL1_ID);
                    return;
                }

                ID1_connected = 1;
            }

            // ID2
            dynamixel.ping(port_num, PROTOCOL_VERSION, DXL2_ID);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
                ID2_connected = 0;
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is not connected", DXL2_ID);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }
            else
            {
                //MessageBox.Show("Dynamixel ID2 has been successfully connected");
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL2_ID);

                // Add parameter storage for Dynamixel#2 feedback: position, speed, load, voltage, temperature
                dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL2_ID, ADDR_MX_PRESENT_POSITION, 10);
                if (dxl_addparam_result != true)
                {
                    //MessageBox.Show("[ID: {0}] groupBulkRead addparam failed");
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("ID{0} groupBulkRead addparam failed", DXL2_ID);
                    return;
                }

                ID2_connected = 1;
            }

            // ID3
            dynamixel.ping(port_num, PROTOCOL_VERSION, DXL3_ID);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
                ID3_connected = 0;
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is not connected", DXL3_ID);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);

            }
            else
            {
                //MessageBox.Show("Dynamixel ID3 has been successfully connected");
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL3_ID);

                // Add parameter storage for Dynamixel#3 feedback: position, speed, load, voltage, temperature
                dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL3_ID, ADDR_MX_PRESENT_POSITION, 10);
                if (dxl_addparam_result != true)
                {
                    //MessageBox.Show("[ID: {0}] groupBulkRead addparam failed");
                    dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("ID{0} groupBulkRead addparam failed", DXL3_ID);
                    return;
                }

                ID3_connected = 1;
            }

            // ID4
            dynamixel.ping(port_num, PROTOCOL_VERSION, DXL4_ID);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
                ID4_connected = 0;
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is not connected", DXL4_ID);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);

            }
            else
            {
                //MessageBox.Show("Dynamixel ID4 has been successfully connected");
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL4_ID);

                // Add parameter storage for Dynamixel#4 feedback: position, speed, load, voltage, temperature
                dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL4_ID, ADDR_MX_PRESENT_POSITION, 10);
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
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
                ID5_connected = 0;
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is not connected", DXL5_ID);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);

            }
            else
            {
                //MessageBox.Show("Dynamixel ID5 has been successfully connected");
                dynaStatus.Text = dynaStatus.Text + Environment.NewLine + String.Format("Dynamixel ID{0} is connected", DXL5_ID);

                // Add parameter storage for Dynamixel#5 feedback: position, speed, load, voltage, temperature
                dxl_addparam_result = dynamixel.groupBulkReadAddParam(read_group_num, DXL5_ID, ADDR_MX_PRESENT_POSITION, 10);
                if (dxl_addparam_result != true)
                {
                    //MessageBox.Show("[ID: {0}] groupBulkRead addparam failed");
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

            }
            else
            {
                // Close port
                dynamixel.closePort(port_num);
            }

        }

        private void dynaDisconnect_Click(object sender, EventArgs e)
        {
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

            // Close port
            dynamixel.closePort(port_num);

            // Update status text
            dynaStatus.Text = "Disconnected";

        }

        private void TorqueOn_Click(object sender, EventArgs e)
        {
            readDyna();
            // initialize dynamixel positions when first turning on the torque
            ID1_goal_position = ID1_present_position;
            ID2_goal_position = ID2_present_position;
            ID3_goal_position = ID3_present_position;
            ID4_goal_position = ID4_present_position;
            ID5_goal_position = ID5_present_position;

            // Enable Dynamixel Torque
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            // Enable/disable relevant controls
            TorqueOn.Enabled = false;
            TorqueOff.Enabled = true;
            moveCW.Enabled = true;
            moveCCW.Enabled = true;
            TorqueOff.Focus();
            
        }

        private void TorqueOff_Click(object sender, EventArgs e)
        {
            // Disable Dynamixel Torque
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                //dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
                dynaCommResult.Text = Convert.ToString(dxl_comm_result);
            }
            else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
                //dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
                dynaError.Text = Convert.ToString(dxl_error);
            }

            // Enable/disable relevant controls
            TorqueOn.Enabled = true;
            TorqueOff.Enabled = false;
            moveCW.Enabled = false;
            moveCCW.Enabled = false;
            TorqueOn.Focus();
        }

        private void writeDyna(UInt16 pos1, UInt16 vel1, UInt16 pos2, UInt16 vel2, UInt16 pos3, UInt16 vel3, UInt16 pos4, UInt16 vel4, UInt16 pos5, UInt16 vel5)
        {
            //// Write goal position
            //dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_GOAL_POSITION, pos);
            ////dynamixel.write2ByteTxOnly(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position);
            //if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            //{
            //    dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
            //}
            //else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            //{
            //    dynamixel.printRxPacketError(PROTOCOL_VERSION, dxl_error);
            //}

            // Check velocity value to make sure it is bounded and not accidentally set to 0 which would make the dynamixel servo move as FAST as possible
            if (vel1 < 1)
            {
                vel1 = 1;
            }

            if (vel2 < 1)
            {
                vel2 = 1;
            }

            if (vel3 < 1)
            {
                vel3 = 1;
            }

            if (vel4 < 1)
            {
                vel4 = 1;
            }

            if (vel5 < 1)
            {
                vel5 = 1;
            }

            // Clear syncwrite parameter storage
            dynamixel.groupSyncWriteClearParam(write_group_num);

            //// need to only run this if syncwrite has been added in the past -- check with syncwrite_flag
            //if (syncwrite_flag == true)
            //{
            //    dynamixel.groupSyncWriteClearParam(write_group_num);
            //}

            // Add Dynamixel#1 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL1_ID, pos1, LEN_MX_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#1 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL1_ID, vel1, LEN_MX_MOVING_SPEED);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#2 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL2_ID, pos2, LEN_MX_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#2 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL2_ID, vel2, LEN_MX_MOVING_SPEED);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#3 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL3_ID, pos3, LEN_MX_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#3 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL3_ID, vel3, LEN_MX_MOVING_SPEED);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#4 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL4_ID, pos4, LEN_MX_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#4 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL4_ID, vel4, LEN_MX_MOVING_SPEED);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#5 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL5_ID, pos5, LEN_MX_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#5 moving speed value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_group_num, DXL5_ID, vel5, LEN_MX_MOVING_SPEED);
            if (dxl_addparam_result != true)
            {
                MessageBox.Show("[ID: {0}] groupSyncWrite addparam failed");
                return;
            }

            // Syncwrite goal position
            dynamixel.groupSyncWriteTxPacket(write_group_num);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);

        }

        private void moveCW_Click(object sender, EventArgs e)
        {

            ID5_moving_speed = 50;
            ID5_goal_position = 2000;
        }

        private void moveCCW_Click(object sender, EventArgs e)
        {
            ID5_moving_speed = 50;
            ID5_goal_position = 2700;
        }

        private void LEDon_Click(object sender, EventArgs e)
        {
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
            // Read feedback from dynamixel motor
            // Bulkread present position and moving status
            dynamixel.groupBulkReadTxRxPacket(read_group_num);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);

            dxl_getdata_result = dynamixel.groupBulkReadIsAvailable(read_group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, 10);
            if (dxl_getdata_result != true)
            {
                MessageBox.Show("[ID: {0}] groupBulkRead getdata failed");
            }

            // Get Dynamixel#1 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive

            if (ID1_connected == 1)
            {
                ID1_present_position = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
                Pos1.Text = Convert.ToString(ID1_present_position);
                Vel1.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)));
                Load1.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)));
                Volt1.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE) / 10);
                Temp1.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL1_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP));
            }

            // Get Dynamixel#2 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive
            if (ID2_connected == 1)
            {
                ID2_present_position = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
                Pos2.Text = Convert.ToString(ID2_present_position);
                Vel2.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)));
                Load2.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)));
                Volt2.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE) / 10);
                Temp2.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL2_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP));
            }

            // Get Dynamixel#3 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive
            if (ID3_connected == 1)
            {
                ID3_present_position = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
                Pos3.Text = Convert.ToString(ID3_present_position);
                Vel3.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)));
                Load3.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)));
                Volt3.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE) / 10);
                Temp3.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL3_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP));
            }

            // Get Dynamixel#4 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive
            if (ID4_connected == 1)
            {
                ID4_present_position = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
                Pos4.Text = Convert.ToString(ID4_present_position);
                Vel4.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)));
                Load4.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)));
                Volt4.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE) / 10);
                Temp4.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL4_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP));
            }
            // Get Dynamixel#5 present position value
            // If apply load in CCW direction load value is positive
            // If move in CCW direction velocity value is positive
            if (ID5_connected == 1)
            {
                ID5_present_position = (UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
                Pos5.Text = Convert.ToString(ID5_present_position);
                Vel5.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_SPEED)));
                Load5.Text = Convert.ToString(parse_load((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)));
                Volt5.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_VOLTAGE, LEN_MX_PRESENT_VOLTAGE) / 10);
                Temp5.Text = Convert.ToString((UInt16)dynamixel.groupBulkReadGetData(read_group_num, DXL5_ID, ADDR_MX_PRESENT_TEMP, LEN_MX_PRESENT_TEMP));
            }
        }

        private void readFeedback_Click(object sender, EventArgs e)
        {
            readDyna();
        }

        //private int parse_vel(UInt16 value)
        //{
        //    if (value == 0)
        //    {
        //        return 0;
        //    }
        //    else
        //    {
        //        return 1023 - value;
        //    }
        //}

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


        #endregion

        #region "Main LOOP"
        // Main loop for sending out and reading back commands from input and output devices (mostly runs on a background thread).

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
                MessageBox.Show(ex.Message);
            }
        }

        private void UpdateState()
        {
            // start stop watch
            stopWatch1.Reset();
            stopWatch1.Start();

            if (xBoxGroupBox.Enabled == true)
            {
                // Update xbox control values
                checkA.Checked = reporterState.LastActiveState.Buttons.A == XInputDotNetPure.ButtonState.Pressed;
                checkB.Checked = reporterState.LastActiveState.Buttons.B == XInputDotNetPure.ButtonState.Pressed;
                checkX.Checked = reporterState.LastActiveState.Buttons.X == XInputDotNetPure.ButtonState.Pressed;
                checkY.Checked = reporterState.LastActiveState.Buttons.Y == XInputDotNetPure.ButtonState.Pressed;
                checkStart.Checked = reporterState.LastActiveState.Buttons.Start == XInputDotNetPure.ButtonState.Pressed;
                checkBack.Checked = reporterState.LastActiveState.Buttons.Back == XInputDotNetPure.ButtonState.Pressed;
                checkGuide.Checked = reporterState.LastActiveState.Buttons.Guide == XInputDotNetPure.ButtonState.Pressed;
                checkStickLeft.Checked = reporterState.LastActiveState.Buttons.LeftStick == XInputDotNetPure.ButtonState.Pressed;
                checkStickRight.Checked = reporterState.LastActiveState.Buttons.RightStick == XInputDotNetPure.ButtonState.Pressed;
                checkShoulderLeft.Checked = reporterState.LastActiveState.Buttons.LeftShoulder == XInputDotNetPure.ButtonState.Pressed;
                checkShoulderRight.Checked = reporterState.LastActiveState.Buttons.RightShoulder == XInputDotNetPure.ButtonState.Pressed;

                checkDPadUp.Checked = reporterState.LastActiveState.DPad.Up == XInputDotNetPure.ButtonState.Pressed;
                checkDPadRight.Checked = reporterState.LastActiveState.DPad.Right == XInputDotNetPure.ButtonState.Pressed;
                checkDPadDown.Checked = reporterState.LastActiveState.DPad.Down == XInputDotNetPure.ButtonState.Pressed;
                checkDPadLeft.Checked = reporterState.LastActiveState.DPad.Left == XInputDotNetPure.ButtonState.Pressed;

                labelTriggerLeft.Text = FormatFloat(reporterState.LastActiveState.Triggers.Left);
                labelTriggerRight.Text = FormatFloat(reporterState.LastActiveState.Triggers.Right);

                labelStickLeftX.Text = FormatFloat(reporterState.LastActiveState.ThumbSticks.Left.X);
                labelStickLeftY.Text = FormatFloat(reporterState.LastActiveState.ThumbSticks.Left.Y);
                labelStickRightX.Text = FormatFloat(reporterState.LastActiveState.ThumbSticks.Right.X);
                labelStickRightY.Text = FormatFloat(reporterState.LastActiveState.ThumbSticks.Right.Y);
            }



            if (BentoGroupBox.Enabled == true)
            {
                // Update feedback
                readDyna();

                // Move dynamixel motors

                // ID1 -> Shoulder Rotation
                if (reporterState.LastActiveState.ThumbSticks.Left.X < -0.001)
                {
                    // Move CW (Shoulder Rotate Left)
                    ID1_goal_position = 1028;
                    ID1_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.ThumbSticks.Left.X) * 50);
                    ID1_prev_state = 1;
                }
                else if (reporterState.LastActiveState.ThumbSticks.Left.X > 0.001)
                {
                    // Move CCW (Shoulder Rotate Right)
                    ID1_goal_position = 3073;
                    ID1_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.ThumbSticks.Left.X) * 50);
                    ID1_prev_state = 2;
                }
                else if (ID1_prev_state == 1 || ID1_prev_state == 2)
                {
                    // Grab last position feedback and STOP!
                    ID1_goal_position = ID1_present_position;
                    ID1_moving_speed = 50;
                    ID1_prev_state = 0;
                }

                // ID2 -> Elbow flexion
                if (reporterState.LastActiveState.ThumbSticks.Left.Y < -0.001)
                {
                    // Move CW (Elbow flex up)
                    ID2_goal_position = 2225;
                    ID2_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.ThumbSticks.Left.Y) * 70);
                    ID2_prev_state = 1;
                }
                else if (reporterState.LastActiveState.ThumbSticks.Left.Y > 0.001)
                {
                    // Move CCW (Elbow extend down)
                    ID2_goal_position = 2590;
                    ID2_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.ThumbSticks.Left.Y) * 70);
                    ID2_prev_state = 2;
                }
                else if (ID2_prev_state == 1 || ID2_prev_state == 2)
                {
                    // Grab last position feedback and STOP!
                    ID2_goal_position = ID2_present_position;
                    ID2_moving_speed = 70;
                    ID2_prev_state = 0;
                }

                // ID3 -> Wrist Rotation
                if (reporterState.LastActiveState.ThumbSticks.Right.X < -0.001)
                {
                    // Move CW (rotate left)
                    ID3_goal_position = 1028;
                    ID3_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.ThumbSticks.Right.X) * 90);
                    ID3_prev_state = 1;
                }
                else if (reporterState.LastActiveState.ThumbSticks.Right.X > 0.001)
                {
                    // Move CCW (rotate right)
                    ID3_goal_position = 3073;
                    ID3_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.ThumbSticks.Right.X) * 90);
                    ID3_prev_state = 2;
                }
                else if (ID3_prev_state == 1 || ID3_prev_state == 2)
                {
                    // Grab last position feedback and STOP!
                    ID3_goal_position = ID3_present_position;
                    ID3_moving_speed = 90;
                    ID3_prev_state = 0;
                }

                // ID4 -> Wrist Flexion
                if (reporterState.LastActiveState.ThumbSticks.Right.Y > 0.001)
                {
                    // Move CW (flex down)
                    ID4_goal_position = 1330;
                    ID4_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.ThumbSticks.Right.Y) * 90);
                    ID4_prev_state = 1;
                }
                else if (reporterState.LastActiveState.ThumbSticks.Right.Y < -0.001)
                {
                    // Move CCW (flex up)
                    ID4_goal_position = 3130;
                    ID4_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.ThumbSticks.Right.Y) * 90);
                    ID4_prev_state = 2;
                }
                else if (ID4_prev_state == 1 || ID4_prev_state == 2)
                {
                    // Grab last position feedback and STOP!
                    ID4_goal_position = ID4_present_position;
                    ID4_moving_speed = 90;
                    ID4_prev_state = 0;
                }

                // ID5 -> Hand close/open
                if (reporterState.LastActiveState.Triggers.Right > 0.001)
                {
                    // Move CW (close)
                    ID5_goal_position = 2000;
                    ID5_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.Triggers.Right) * 90);
                    ID5_prev_state = 1;
                }
                else if (reporterState.LastActiveState.Triggers.Left > 0.001)
                {
                    // Move CCW (open)
                    ID5_goal_position = 2700;
                    ID5_moving_speed = Convert.ToUInt16(Math.Abs(reporterState.LastActiveState.Triggers.Left) * 90);
                    ID5_prev_state = 2;
                }
                else if (ID5_prev_state == 1 || ID5_prev_state == 2)
                {
                    // Grab last position feedback and STOP!
                    ID5_goal_position = ID5_present_position;
                    ID5_moving_speed = 90;
                    ID5_prev_state = 0;
                }

                if (TorqueOn.Enabled == false)
                {
                    // Write goal position and moving speeds to each servo on bus using syncwrite command
                    writeDyna(ID1_goal_position, ID1_moving_speed, ID2_goal_position, ID2_moving_speed, ID3_goal_position, ID3_moving_speed, ID4_goal_position, ID4_moving_speed, ID5_goal_position, ID5_moving_speed);
                }
            }
            // Thread.Sleep(15);
            //stop stop watch
            stopWatch1.Stop();
            long milliSec1 = stopWatch1.ElapsedMilliseconds;
            delay.Text = Convert.ToString(milliSec1);
            if (milliSec1 > Convert.ToDecimal(delay_max.Text))
            {
                delay_max.Text = Convert.ToString(milliSec1);
            }
        }

        private static string FormatFloat(float v)
        {
            return string.Format("{0:F3}", v);
        }
        #endregion

        #region "Toolstrip Menu Items"
        private void AboutToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // Open the about window
            // How to make about dialog box: http://stackoverflow.com/questions/19675910/creating-about-dialog-box-in-c-sharp-form-application
            AboutBox1 a = new AboutBox1();
            a.Show();
            
        }

        private void mappingGraphicToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // How to open file with associated application: http://stackoverflow.com/questions/10174156/open-file-with-associated-application
            System.Diagnostics.Process.Start(@"resources\xbox_controller_mapping_170417.png");
        }

        private void ContentsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            // How to open file with associated application: http://stackoverflow.com/questions/10174156/open-file-with-associated-application
            System.Diagnostics.Process.Start(@"resources\brachIOplexus_User_Guide_rev1.pdf");
        }

        private void ExitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        #endregion

    }
    
}
