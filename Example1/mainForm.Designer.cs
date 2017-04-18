namespace brachIOplexus
{
    partial class mainForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(mainForm));
            this.MenuStrip1 = new System.Windows.Forms.MenuStrip();
            this.FileToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripSeparator = new System.Windows.Forms.ToolStripSeparator();
            this.toolStripSeparator2 = new System.Windows.Forms.ToolStripSeparator();
            this.ExitToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.HelpToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.ContentsToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.mappingGraphicToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripSeparator5 = new System.Windows.Forms.ToolStripSeparator();
            this.AboutToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.label7 = new System.Windows.Forms.Label();
            this.Label18 = new System.Windows.Forms.Label();
            this.Label20 = new System.Windows.Forms.Label();
            this.Label21 = new System.Windows.Forms.Label();
            this.Label19 = new System.Windows.Forms.Label();
            this.RobotFeedbackBox = new System.Windows.Forms.GroupBox();
            this.Temp5 = new System.Windows.Forms.Label();
            this.Volt5 = new System.Windows.Forms.Label();
            this.Load5 = new System.Windows.Forms.Label();
            this.Vel5 = new System.Windows.Forms.Label();
            this.Pos5 = new System.Windows.Forms.Label();
            this.Temp3 = new System.Windows.Forms.Label();
            this.Volt3 = new System.Windows.Forms.Label();
            this.Load3 = new System.Windows.Forms.Label();
            this.Vel3 = new System.Windows.Forms.Label();
            this.Pos3 = new System.Windows.Forms.Label();
            this.Temp2 = new System.Windows.Forms.Label();
            this.Volt2 = new System.Windows.Forms.Label();
            this.Load2 = new System.Windows.Forms.Label();
            this.Vel2 = new System.Windows.Forms.Label();
            this.Pos2 = new System.Windows.Forms.Label();
            this.Temp1 = new System.Windows.Forms.Label();
            this.Volt1 = new System.Windows.Forms.Label();
            this.Load1 = new System.Windows.Forms.Label();
            this.Vel1 = new System.Windows.Forms.Label();
            this.Pos1 = new System.Windows.Forms.Label();
            this.label109 = new System.Windows.Forms.Label();
            this.label108 = new System.Windows.Forms.Label();
            this.label107 = new System.Windows.Forms.Label();
            this.Temp4 = new System.Windows.Forms.Label();
            this.label106 = new System.Windows.Forms.Label();
            this.label200 = new System.Windows.Forms.Label();
            this.Volt4 = new System.Windows.Forms.Label();
            this.Load4 = new System.Windows.Forms.Label();
            this.Vel4 = new System.Windows.Forms.Label();
            this.Pos4 = new System.Windows.Forms.Label();
            this.cmbSerialPorts = new System.Windows.Forms.ComboBox();
            this.HelpProvider1 = new System.Windows.Forms.HelpProvider();
            this.checkGuide = new System.Windows.Forms.CheckBox();
            this.labelStickRightY = new System.Windows.Forms.Label();
            this.labelStickRightX = new System.Windows.Forms.Label();
            this.labelStickLeftY = new System.Windows.Forms.Label();
            this.labelStickLeftX = new System.Windows.Forms.Label();
            this.labelTriggerRight = new System.Windows.Forms.Label();
            this.labelTriggerLeft = new System.Windows.Forms.Label();
            this.checkDPadLeft = new System.Windows.Forms.CheckBox();
            this.checkDPadDown = new System.Windows.Forms.CheckBox();
            this.checkDPadRight = new System.Windows.Forms.CheckBox();
            this.checkDPadUp = new System.Windows.Forms.CheckBox();
            this.checkStickLeft = new System.Windows.Forms.CheckBox();
            this.checkStickRight = new System.Windows.Forms.CheckBox();
            this.checkBack = new System.Windows.Forms.CheckBox();
            this.checkStart = new System.Windows.Forms.CheckBox();
            this.checkA = new System.Windows.Forms.CheckBox();
            this.checkB = new System.Windows.Forms.CheckBox();
            this.checkX = new System.Windows.Forms.CheckBox();
            this.checkY = new System.Windows.Forms.CheckBox();
            this.checkShoulderRight = new System.Windows.Forms.CheckBox();
            this.label59 = new System.Windows.Forms.Label();
            this.label61 = new System.Windows.Forms.Label();
            this.label62 = new System.Windows.Forms.Label();
            this.label69 = new System.Windows.Forms.Label();
            this.label71 = new System.Windows.Forms.Label();
            this.label73 = new System.Windows.Forms.Label();
            this.label76 = new System.Windows.Forms.Label();
            this.label80 = new System.Windows.Forms.Label();
            this.label81 = new System.Windows.Forms.Label();
            this.label82 = new System.Windows.Forms.Label();
            this.label84 = new System.Windows.Forms.Label();
            this.label85 = new System.Windows.Forms.Label();
            this.label86 = new System.Windows.Forms.Label();
            this.label87 = new System.Windows.Forms.Label();
            this.label88 = new System.Windows.Forms.Label();
            this.checkShoulderLeft = new System.Windows.Forms.CheckBox();
            this.label105 = new System.Windows.Forms.Label();
            this.label111 = new System.Windows.Forms.Label();
            this.label112 = new System.Windows.Forms.Label();
            this.label113 = new System.Windows.Forms.Label();
            this.label114 = new System.Windows.Forms.Label();
            this.label115 = new System.Windows.Forms.Label();
            this.pollingWorker = new System.ComponentModel.BackgroundWorker();
            this.dynaConnect = new System.Windows.Forms.Button();
            this.dynaDisconnect = new System.Windows.Forms.Button();
            this.TorqueOn = new System.Windows.Forms.Button();
            this.TorqueOff = new System.Windows.Forms.Button();
            this.LEDon = new System.Windows.Forms.Button();
            this.LEDoff = new System.Windows.Forms.Button();
            this.moveCW = new System.Windows.Forms.Button();
            this.moveCCW = new System.Windows.Forms.Button();
            this.label116 = new System.Windows.Forms.Label();
            this.label117 = new System.Windows.Forms.Label();
            this.delay = new System.Windows.Forms.Label();
            this.label118 = new System.Windows.Forms.Label();
            this.dynaCommResult = new System.Windows.Forms.Label();
            this.label120 = new System.Windows.Forms.Label();
            this.dynaError = new System.Windows.Forms.Label();
            this.delay_max = new System.Windows.Forms.Label();
            this.label121 = new System.Windows.Forms.Label();
            this.label119 = new System.Windows.Forms.Label();
            this.dynaStatus = new System.Windows.Forms.Label();
            this.cmbSerialRefresh = new System.Windows.Forms.Button();
            this.BentoGroupBox = new System.Windows.Forms.GroupBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.xBoxGroupBox = new System.Windows.Forms.GroupBox();
            this.XboxDisconnect = new System.Windows.Forms.Button();
            this.XboxConnect = new System.Windows.Forms.Button();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.MenuStrip1.SuspendLayout();
            this.RobotFeedbackBox.SuspendLayout();
            this.BentoGroupBox.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.xBoxGroupBox.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.groupBox6.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.SuspendLayout();
            // 
            // MenuStrip1
            // 
            this.MenuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.FileToolStripMenuItem,
            this.HelpToolStripMenuItem});
            this.MenuStrip1.Location = new System.Drawing.Point(0, 0);
            this.MenuStrip1.Name = "MenuStrip1";
            this.MenuStrip1.Padding = new System.Windows.Forms.Padding(4, 2, 0, 2);
            this.MenuStrip1.Size = new System.Drawing.Size(628, 24);
            this.MenuStrip1.TabIndex = 54;
            this.MenuStrip1.Text = "MenuStrip1";
            // 
            // FileToolStripMenuItem
            // 
            this.FileToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripSeparator,
            this.toolStripSeparator2,
            this.ExitToolStripMenuItem});
            this.FileToolStripMenuItem.Name = "FileToolStripMenuItem";
            this.FileToolStripMenuItem.Size = new System.Drawing.Size(37, 20);
            this.FileToolStripMenuItem.Text = "&File";
            // 
            // toolStripSeparator
            // 
            this.toolStripSeparator.Name = "toolStripSeparator";
            this.toolStripSeparator.Size = new System.Drawing.Size(149, 6);
            // 
            // toolStripSeparator2
            // 
            this.toolStripSeparator2.Name = "toolStripSeparator2";
            this.toolStripSeparator2.Size = new System.Drawing.Size(149, 6);
            // 
            // ExitToolStripMenuItem
            // 
            this.ExitToolStripMenuItem.Name = "ExitToolStripMenuItem";
            this.ExitToolStripMenuItem.Size = new System.Drawing.Size(152, 22);
            this.ExitToolStripMenuItem.Text = "E&xit";
            this.ExitToolStripMenuItem.Click += new System.EventHandler(this.ExitToolStripMenuItem_Click);
            // 
            // HelpToolStripMenuItem
            // 
            this.HelpToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.ContentsToolStripMenuItem,
            this.mappingGraphicToolStripMenuItem,
            this.toolStripSeparator5,
            this.AboutToolStripMenuItem});
            this.HelpToolStripMenuItem.Name = "HelpToolStripMenuItem";
            this.HelpToolStripMenuItem.Size = new System.Drawing.Size(44, 20);
            this.HelpToolStripMenuItem.Text = "&Help";
            // 
            // ContentsToolStripMenuItem
            // 
            this.ContentsToolStripMenuItem.Name = "ContentsToolStripMenuItem";
            this.ContentsToolStripMenuItem.Size = new System.Drawing.Size(166, 22);
            this.ContentsToolStripMenuItem.Text = "&User Manual";
            this.ContentsToolStripMenuItem.Click += new System.EventHandler(this.ContentsToolStripMenuItem_Click);
            // 
            // mappingGraphicToolStripMenuItem
            // 
            this.mappingGraphicToolStripMenuItem.Name = "mappingGraphicToolStripMenuItem";
            this.mappingGraphicToolStripMenuItem.Size = new System.Drawing.Size(166, 22);
            this.mappingGraphicToolStripMenuItem.Text = "Mapping Graphic";
            this.mappingGraphicToolStripMenuItem.Click += new System.EventHandler(this.mappingGraphicToolStripMenuItem_Click);
            // 
            // toolStripSeparator5
            // 
            this.toolStripSeparator5.Name = "toolStripSeparator5";
            this.toolStripSeparator5.Size = new System.Drawing.Size(163, 6);
            // 
            // AboutToolStripMenuItem
            // 
            this.AboutToolStripMenuItem.Name = "AboutToolStripMenuItem";
            this.AboutToolStripMenuItem.Size = new System.Drawing.Size(166, 22);
            this.AboutToolStripMenuItem.Text = "&About...";
            this.AboutToolStripMenuItem.Click += new System.EventHandler(this.AboutToolStripMenuItem_Click);
            // 
            // label7
            // 
            this.label7.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label7.Location = new System.Drawing.Point(8, 128);
            this.label7.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(100, 15);
            this.label7.TabIndex = 145;
            this.label7.Text = "Hand Open/Close:";
            // 
            // Label18
            // 
            this.Label18.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Label18.Location = new System.Drawing.Point(8, 81);
            this.Label18.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Label18.Name = "Label18";
            this.Label18.Size = new System.Drawing.Size(82, 15);
            this.Label18.TabIndex = 142;
            this.Label18.Text = "Wrist Rotation:";
            // 
            // Label20
            // 
            this.Label20.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Label20.Location = new System.Drawing.Point(8, 57);
            this.Label20.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Label20.Name = "Label20";
            this.Label20.Size = new System.Drawing.Size(100, 15);
            this.Label20.TabIndex = 135;
            this.Label20.Text = "Elbow Flexion:";
            // 
            // Label21
            // 
            this.Label21.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Label21.Location = new System.Drawing.Point(8, 34);
            this.Label21.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Label21.Name = "Label21";
            this.Label21.Size = new System.Drawing.Size(100, 15);
            this.Label21.TabIndex = 130;
            this.Label21.Text = "Shoulder Rotation:";
            // 
            // Label19
            // 
            this.Label19.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Label19.Location = new System.Drawing.Point(8, 105);
            this.Label19.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Label19.Name = "Label19";
            this.Label19.Size = new System.Drawing.Size(82, 15);
            this.Label19.TabIndex = 129;
            this.Label19.Text = "Wrist Flexion:";
            // 
            // RobotFeedbackBox
            // 
            this.RobotFeedbackBox.Controls.Add(this.Temp5);
            this.RobotFeedbackBox.Controls.Add(this.Volt5);
            this.RobotFeedbackBox.Controls.Add(this.Load5);
            this.RobotFeedbackBox.Controls.Add(this.Vel5);
            this.RobotFeedbackBox.Controls.Add(this.label7);
            this.RobotFeedbackBox.Controls.Add(this.Pos5);
            this.RobotFeedbackBox.Controls.Add(this.Temp3);
            this.RobotFeedbackBox.Controls.Add(this.Label18);
            this.RobotFeedbackBox.Controls.Add(this.Volt3);
            this.RobotFeedbackBox.Controls.Add(this.Load3);
            this.RobotFeedbackBox.Controls.Add(this.Vel3);
            this.RobotFeedbackBox.Controls.Add(this.Pos3);
            this.RobotFeedbackBox.Controls.Add(this.Temp2);
            this.RobotFeedbackBox.Controls.Add(this.Volt2);
            this.RobotFeedbackBox.Controls.Add(this.Load2);
            this.RobotFeedbackBox.Controls.Add(this.Label20);
            this.RobotFeedbackBox.Controls.Add(this.Vel2);
            this.RobotFeedbackBox.Controls.Add(this.Pos2);
            this.RobotFeedbackBox.Controls.Add(this.Temp1);
            this.RobotFeedbackBox.Controls.Add(this.Volt1);
            this.RobotFeedbackBox.Controls.Add(this.Load1);
            this.RobotFeedbackBox.Controls.Add(this.Label21);
            this.RobotFeedbackBox.Controls.Add(this.Vel1);
            this.RobotFeedbackBox.Controls.Add(this.Label19);
            this.RobotFeedbackBox.Controls.Add(this.Pos1);
            this.RobotFeedbackBox.Controls.Add(this.label109);
            this.RobotFeedbackBox.Controls.Add(this.label108);
            this.RobotFeedbackBox.Controls.Add(this.label107);
            this.RobotFeedbackBox.Controls.Add(this.Temp4);
            this.RobotFeedbackBox.Controls.Add(this.label106);
            this.RobotFeedbackBox.Controls.Add(this.label200);
            this.RobotFeedbackBox.Controls.Add(this.Volt4);
            this.RobotFeedbackBox.Controls.Add(this.Load4);
            this.RobotFeedbackBox.Controls.Add(this.Vel4);
            this.RobotFeedbackBox.Controls.Add(this.Pos4);
            this.RobotFeedbackBox.Location = new System.Drawing.Point(8, 90);
            this.RobotFeedbackBox.Margin = new System.Windows.Forms.Padding(2);
            this.RobotFeedbackBox.Name = "RobotFeedbackBox";
            this.RobotFeedbackBox.Padding = new System.Windows.Forms.Padding(2);
            this.RobotFeedbackBox.Size = new System.Drawing.Size(324, 152);
            this.RobotFeedbackBox.TabIndex = 141;
            this.RobotFeedbackBox.TabStop = false;
            this.RobotFeedbackBox.Text = "Feedback";
            // 
            // Temp5
            // 
            this.Temp5.AutoSize = true;
            this.Temp5.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Temp5.Location = new System.Drawing.Point(284, 129);
            this.Temp5.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Temp5.Name = "Temp5";
            this.Temp5.Size = new System.Drawing.Size(13, 13);
            this.Temp5.TabIndex = 169;
            this.Temp5.Text = "--";
            // 
            // Volt5
            // 
            this.Volt5.AutoSize = true;
            this.Volt5.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Volt5.Location = new System.Drawing.Point(250, 129);
            this.Volt5.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Volt5.Name = "Volt5";
            this.Volt5.Size = new System.Drawing.Size(13, 13);
            this.Volt5.TabIndex = 168;
            this.Volt5.Text = "--";
            // 
            // Load5
            // 
            this.Load5.AutoSize = true;
            this.Load5.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Load5.Location = new System.Drawing.Point(210, 129);
            this.Load5.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Load5.Name = "Load5";
            this.Load5.Size = new System.Drawing.Size(13, 13);
            this.Load5.TabIndex = 167;
            this.Load5.Text = "--";
            // 
            // Vel5
            // 
            this.Vel5.AutoSize = true;
            this.Vel5.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Vel5.Location = new System.Drawing.Point(158, 129);
            this.Vel5.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Vel5.Name = "Vel5";
            this.Vel5.Size = new System.Drawing.Size(13, 13);
            this.Vel5.TabIndex = 166;
            this.Vel5.Text = "--";
            // 
            // Pos5
            // 
            this.Pos5.AutoSize = true;
            this.Pos5.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Pos5.Location = new System.Drawing.Point(105, 129);
            this.Pos5.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Pos5.Name = "Pos5";
            this.Pos5.Size = new System.Drawing.Size(13, 13);
            this.Pos5.TabIndex = 165;
            this.Pos5.Text = "--";
            // 
            // Temp3
            // 
            this.Temp3.AutoSize = true;
            this.Temp3.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Temp3.Location = new System.Drawing.Point(284, 81);
            this.Temp3.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Temp3.Name = "Temp3";
            this.Temp3.Size = new System.Drawing.Size(13, 13);
            this.Temp3.TabIndex = 163;
            this.Temp3.Text = "--";
            // 
            // Volt3
            // 
            this.Volt3.AutoSize = true;
            this.Volt3.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Volt3.Location = new System.Drawing.Point(250, 81);
            this.Volt3.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Volt3.Name = "Volt3";
            this.Volt3.Size = new System.Drawing.Size(13, 13);
            this.Volt3.TabIndex = 162;
            this.Volt3.Text = "--";
            // 
            // Load3
            // 
            this.Load3.AutoSize = true;
            this.Load3.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Load3.Location = new System.Drawing.Point(210, 81);
            this.Load3.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Load3.Name = "Load3";
            this.Load3.Size = new System.Drawing.Size(13, 13);
            this.Load3.TabIndex = 161;
            this.Load3.Text = "--";
            // 
            // Vel3
            // 
            this.Vel3.AutoSize = true;
            this.Vel3.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Vel3.Location = new System.Drawing.Point(158, 81);
            this.Vel3.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Vel3.Name = "Vel3";
            this.Vel3.Size = new System.Drawing.Size(13, 13);
            this.Vel3.TabIndex = 160;
            this.Vel3.Text = "--";
            // 
            // Pos3
            // 
            this.Pos3.AutoSize = true;
            this.Pos3.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Pos3.Location = new System.Drawing.Point(105, 81);
            this.Pos3.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Pos3.Name = "Pos3";
            this.Pos3.Size = new System.Drawing.Size(13, 13);
            this.Pos3.TabIndex = 159;
            this.Pos3.Text = "--";
            // 
            // Temp2
            // 
            this.Temp2.AutoSize = true;
            this.Temp2.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Temp2.Location = new System.Drawing.Point(284, 57);
            this.Temp2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Temp2.Name = "Temp2";
            this.Temp2.Size = new System.Drawing.Size(13, 13);
            this.Temp2.TabIndex = 157;
            this.Temp2.Text = "--";
            // 
            // Volt2
            // 
            this.Volt2.AutoSize = true;
            this.Volt2.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Volt2.Location = new System.Drawing.Point(249, 57);
            this.Volt2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Volt2.Name = "Volt2";
            this.Volt2.Size = new System.Drawing.Size(13, 13);
            this.Volt2.TabIndex = 156;
            this.Volt2.Text = "--";
            // 
            // Load2
            // 
            this.Load2.AutoSize = true;
            this.Load2.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Load2.Location = new System.Drawing.Point(210, 57);
            this.Load2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Load2.Name = "Load2";
            this.Load2.Size = new System.Drawing.Size(13, 13);
            this.Load2.TabIndex = 155;
            this.Load2.Text = "--";
            // 
            // Vel2
            // 
            this.Vel2.AutoSize = true;
            this.Vel2.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Vel2.Location = new System.Drawing.Point(157, 57);
            this.Vel2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Vel2.Name = "Vel2";
            this.Vel2.Size = new System.Drawing.Size(13, 13);
            this.Vel2.TabIndex = 154;
            this.Vel2.Text = "--";
            // 
            // Pos2
            // 
            this.Pos2.AutoSize = true;
            this.Pos2.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Pos2.Location = new System.Drawing.Point(104, 57);
            this.Pos2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Pos2.Name = "Pos2";
            this.Pos2.Size = new System.Drawing.Size(13, 13);
            this.Pos2.TabIndex = 153;
            this.Pos2.Text = "--";
            // 
            // Temp1
            // 
            this.Temp1.AutoSize = true;
            this.Temp1.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Temp1.Location = new System.Drawing.Point(284, 34);
            this.Temp1.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Temp1.Name = "Temp1";
            this.Temp1.Size = new System.Drawing.Size(13, 13);
            this.Temp1.TabIndex = 151;
            this.Temp1.Text = "--";
            // 
            // Volt1
            // 
            this.Volt1.AutoSize = true;
            this.Volt1.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Volt1.Location = new System.Drawing.Point(249, 34);
            this.Volt1.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Volt1.Name = "Volt1";
            this.Volt1.Size = new System.Drawing.Size(13, 13);
            this.Volt1.TabIndex = 150;
            this.Volt1.Text = "--";
            // 
            // Load1
            // 
            this.Load1.AutoSize = true;
            this.Load1.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Load1.Location = new System.Drawing.Point(210, 34);
            this.Load1.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Load1.Name = "Load1";
            this.Load1.Size = new System.Drawing.Size(13, 13);
            this.Load1.TabIndex = 149;
            this.Load1.Text = "--";
            // 
            // Vel1
            // 
            this.Vel1.AutoSize = true;
            this.Vel1.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Vel1.Location = new System.Drawing.Point(157, 34);
            this.Vel1.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Vel1.Name = "Vel1";
            this.Vel1.Size = new System.Drawing.Size(13, 13);
            this.Vel1.TabIndex = 148;
            this.Vel1.Text = "--";
            // 
            // Pos1
            // 
            this.Pos1.AutoSize = true;
            this.Pos1.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Pos1.Location = new System.Drawing.Point(104, 34);
            this.Pos1.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Pos1.Name = "Pos1";
            this.Pos1.Size = new System.Drawing.Size(13, 13);
            this.Pos1.TabIndex = 147;
            this.Pos1.Text = "--";
            // 
            // label109
            // 
            this.label109.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label109.Location = new System.Drawing.Point(284, 14);
            this.label109.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label109.Name = "label109";
            this.label109.Size = new System.Drawing.Size(38, 20);
            this.label109.TabIndex = 145;
            this.label109.Text = "Temp:";
            // 
            // label108
            // 
            this.label108.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label108.Location = new System.Drawing.Point(249, 14);
            this.label108.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label108.Name = "label108";
            this.label108.Size = new System.Drawing.Size(27, 20);
            this.label108.TabIndex = 144;
            this.label108.Text = "Volt:";
            // 
            // label107
            // 
            this.label107.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label107.Location = new System.Drawing.Point(210, 14);
            this.label107.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label107.Name = "label107";
            this.label107.Size = new System.Drawing.Size(39, 20);
            this.label107.TabIndex = 143;
            this.label107.Text = "Load:";
            // 
            // Temp4
            // 
            this.Temp4.AutoSize = true;
            this.Temp4.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Temp4.Location = new System.Drawing.Point(284, 105);
            this.Temp4.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Temp4.Name = "Temp4";
            this.Temp4.Size = new System.Drawing.Size(13, 13);
            this.Temp4.TabIndex = 137;
            this.Temp4.Text = "--";
            // 
            // label106
            // 
            this.label106.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label106.Location = new System.Drawing.Point(157, 14);
            this.label106.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label106.Name = "label106";
            this.label106.Size = new System.Drawing.Size(46, 20);
            this.label106.TabIndex = 142;
            this.label106.Text = "Velocity:";
            // 
            // label200
            // 
            this.label200.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label200.Location = new System.Drawing.Point(104, 14);
            this.label200.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label200.Name = "label200";
            this.label200.Size = new System.Drawing.Size(46, 20);
            this.label200.TabIndex = 140;
            this.label200.Text = "Position:";
            // 
            // Volt4
            // 
            this.Volt4.AutoSize = true;
            this.Volt4.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Volt4.Location = new System.Drawing.Point(250, 105);
            this.Volt4.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Volt4.Name = "Volt4";
            this.Volt4.Size = new System.Drawing.Size(13, 13);
            this.Volt4.TabIndex = 132;
            this.Volt4.Text = "--";
            // 
            // Load4
            // 
            this.Load4.AutoSize = true;
            this.Load4.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Load4.Location = new System.Drawing.Point(210, 105);
            this.Load4.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Load4.Name = "Load4";
            this.Load4.Size = new System.Drawing.Size(13, 13);
            this.Load4.TabIndex = 127;
            this.Load4.Text = "--";
            // 
            // Vel4
            // 
            this.Vel4.AutoSize = true;
            this.Vel4.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Vel4.Location = new System.Drawing.Point(158, 105);
            this.Vel4.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Vel4.Name = "Vel4";
            this.Vel4.Size = new System.Drawing.Size(13, 13);
            this.Vel4.TabIndex = 122;
            this.Vel4.Text = "--";
            // 
            // Pos4
            // 
            this.Pos4.AutoSize = true;
            this.Pos4.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.Pos4.Location = new System.Drawing.Point(105, 105);
            this.Pos4.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Pos4.Name = "Pos4";
            this.Pos4.Size = new System.Drawing.Size(13, 13);
            this.Pos4.TabIndex = 117;
            this.Pos4.Text = "--";
            // 
            // cmbSerialPorts
            // 
            this.cmbSerialPorts.FormattingEnabled = true;
            this.cmbSerialPorts.Location = new System.Drawing.Point(67, 23);
            this.cmbSerialPorts.Name = "cmbSerialPorts";
            this.cmbSerialPorts.Size = new System.Drawing.Size(66, 21);
            this.cmbSerialPorts.TabIndex = 15;
            // 
            // checkGuide
            // 
            this.checkGuide.AutoSize = true;
            this.checkGuide.BackColor = System.Drawing.Color.Transparent;
            this.checkGuide.Enabled = false;
            this.checkGuide.Location = new System.Drawing.Point(90, 101);
            this.checkGuide.Name = "checkGuide";
            this.checkGuide.Size = new System.Drawing.Size(15, 14);
            this.checkGuide.TabIndex = 163;
            this.checkGuide.UseVisualStyleBackColor = false;
            // 
            // labelStickRightY
            // 
            this.labelStickRightY.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.labelStickRightY.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelStickRightY.Location = new System.Drawing.Point(189, 161);
            this.labelStickRightY.Name = "labelStickRightY";
            this.labelStickRightY.Size = new System.Drawing.Size(60, 15);
            this.labelStickRightY.TabIndex = 162;
            this.labelStickRightY.Text = "1.0";
            this.labelStickRightY.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // labelStickRightX
            // 
            this.labelStickRightX.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.labelStickRightX.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelStickRightX.Location = new System.Drawing.Point(189, 147);
            this.labelStickRightX.Name = "labelStickRightX";
            this.labelStickRightX.Size = new System.Drawing.Size(60, 15);
            this.labelStickRightX.TabIndex = 161;
            this.labelStickRightX.Text = "1.0";
            this.labelStickRightX.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // labelStickLeftY
            // 
            this.labelStickLeftY.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.labelStickLeftY.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelStickLeftY.Location = new System.Drawing.Point(189, 127);
            this.labelStickLeftY.Name = "labelStickLeftY";
            this.labelStickLeftY.Size = new System.Drawing.Size(60, 15);
            this.labelStickLeftY.TabIndex = 160;
            this.labelStickLeftY.Text = "1.0";
            this.labelStickLeftY.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // labelStickLeftX
            // 
            this.labelStickLeftX.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.labelStickLeftX.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelStickLeftX.Location = new System.Drawing.Point(189, 112);
            this.labelStickLeftX.Name = "labelStickLeftX";
            this.labelStickLeftX.Size = new System.Drawing.Size(60, 16);
            this.labelStickLeftX.TabIndex = 159;
            this.labelStickLeftX.Text = "1.0";
            this.labelStickLeftX.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // labelTriggerRight
            // 
            this.labelTriggerRight.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.labelTriggerRight.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelTriggerRight.Location = new System.Drawing.Point(189, 91);
            this.labelTriggerRight.Name = "labelTriggerRight";
            this.labelTriggerRight.Size = new System.Drawing.Size(60, 16);
            this.labelTriggerRight.TabIndex = 158;
            this.labelTriggerRight.Text = "1.0";
            this.labelTriggerRight.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // labelTriggerLeft
            // 
            this.labelTriggerLeft.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.labelTriggerLeft.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labelTriggerLeft.Location = new System.Drawing.Point(189, 77);
            this.labelTriggerLeft.Name = "labelTriggerLeft";
            this.labelTriggerLeft.Size = new System.Drawing.Size(60, 16);
            this.labelTriggerLeft.TabIndex = 157;
            this.labelTriggerLeft.Text = "1.0";
            this.labelTriggerLeft.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // checkDPadLeft
            // 
            this.checkDPadLeft.AutoSize = true;
            this.checkDPadLeft.BackColor = System.Drawing.Color.Transparent;
            this.checkDPadLeft.Enabled = false;
            this.checkDPadLeft.Location = new System.Drawing.Point(189, 59);
            this.checkDPadLeft.Name = "checkDPadLeft";
            this.checkDPadLeft.Size = new System.Drawing.Size(15, 14);
            this.checkDPadLeft.TabIndex = 156;
            this.checkDPadLeft.UseVisualStyleBackColor = false;
            // 
            // checkDPadDown
            // 
            this.checkDPadDown.AutoSize = true;
            this.checkDPadDown.BackColor = System.Drawing.Color.Transparent;
            this.checkDPadDown.Enabled = false;
            this.checkDPadDown.Location = new System.Drawing.Point(189, 43);
            this.checkDPadDown.Name = "checkDPadDown";
            this.checkDPadDown.Size = new System.Drawing.Size(15, 14);
            this.checkDPadDown.TabIndex = 155;
            this.checkDPadDown.UseVisualStyleBackColor = false;
            // 
            // checkDPadRight
            // 
            this.checkDPadRight.AutoSize = true;
            this.checkDPadRight.BackColor = System.Drawing.Color.Transparent;
            this.checkDPadRight.Enabled = false;
            this.checkDPadRight.Location = new System.Drawing.Point(189, 27);
            this.checkDPadRight.Name = "checkDPadRight";
            this.checkDPadRight.Size = new System.Drawing.Size(15, 14);
            this.checkDPadRight.TabIndex = 154;
            this.checkDPadRight.UseVisualStyleBackColor = false;
            // 
            // checkDPadUp
            // 
            this.checkDPadUp.AutoSize = true;
            this.checkDPadUp.BackColor = System.Drawing.Color.Transparent;
            this.checkDPadUp.Enabled = false;
            this.checkDPadUp.Location = new System.Drawing.Point(189, 12);
            this.checkDPadUp.Name = "checkDPadUp";
            this.checkDPadUp.Size = new System.Drawing.Size(15, 14);
            this.checkDPadUp.TabIndex = 153;
            this.checkDPadUp.UseVisualStyleBackColor = false;
            // 
            // checkStickLeft
            // 
            this.checkStickLeft.AutoSize = true;
            this.checkStickLeft.BackColor = System.Drawing.Color.Transparent;
            this.checkStickLeft.Enabled = false;
            this.checkStickLeft.Location = new System.Drawing.Point(90, 118);
            this.checkStickLeft.Name = "checkStickLeft";
            this.checkStickLeft.Size = new System.Drawing.Size(15, 14);
            this.checkStickLeft.TabIndex = 152;
            this.checkStickLeft.UseVisualStyleBackColor = false;
            // 
            // checkStickRight
            // 
            this.checkStickRight.AutoSize = true;
            this.checkStickRight.BackColor = System.Drawing.Color.Transparent;
            this.checkStickRight.Enabled = false;
            this.checkStickRight.Location = new System.Drawing.Point(90, 133);
            this.checkStickRight.Name = "checkStickRight";
            this.checkStickRight.Size = new System.Drawing.Size(15, 14);
            this.checkStickRight.TabIndex = 151;
            this.checkStickRight.UseVisualStyleBackColor = false;
            // 
            // checkBack
            // 
            this.checkBack.AutoSize = true;
            this.checkBack.BackColor = System.Drawing.Color.Transparent;
            this.checkBack.Enabled = false;
            this.checkBack.Location = new System.Drawing.Point(90, 86);
            this.checkBack.Name = "checkBack";
            this.checkBack.Size = new System.Drawing.Size(15, 14);
            this.checkBack.TabIndex = 150;
            this.checkBack.UseVisualStyleBackColor = false;
            // 
            // checkStart
            // 
            this.checkStart.AutoSize = true;
            this.checkStart.BackColor = System.Drawing.Color.Transparent;
            this.checkStart.Enabled = false;
            this.checkStart.Location = new System.Drawing.Point(90, 70);
            this.checkStart.Name = "checkStart";
            this.checkStart.Size = new System.Drawing.Size(15, 14);
            this.checkStart.TabIndex = 149;
            this.checkStart.UseVisualStyleBackColor = false;
            // 
            // checkA
            // 
            this.checkA.AutoSize = true;
            this.checkA.BackColor = System.Drawing.Color.Transparent;
            this.checkA.Enabled = false;
            this.checkA.Location = new System.Drawing.Point(90, 12);
            this.checkA.Name = "checkA";
            this.checkA.Size = new System.Drawing.Size(15, 14);
            this.checkA.TabIndex = 148;
            this.checkA.UseVisualStyleBackColor = false;
            // 
            // checkB
            // 
            this.checkB.AutoSize = true;
            this.checkB.BackColor = System.Drawing.Color.Transparent;
            this.checkB.Enabled = false;
            this.checkB.Location = new System.Drawing.Point(90, 26);
            this.checkB.Name = "checkB";
            this.checkB.Size = new System.Drawing.Size(15, 14);
            this.checkB.TabIndex = 147;
            this.checkB.UseVisualStyleBackColor = false;
            // 
            // checkX
            // 
            this.checkX.AutoSize = true;
            this.checkX.BackColor = System.Drawing.Color.Transparent;
            this.checkX.Enabled = false;
            this.checkX.Location = new System.Drawing.Point(90, 40);
            this.checkX.Name = "checkX";
            this.checkX.Size = new System.Drawing.Size(15, 14);
            this.checkX.TabIndex = 146;
            this.checkX.UseVisualStyleBackColor = false;
            // 
            // checkY
            // 
            this.checkY.AutoSize = true;
            this.checkY.BackColor = System.Drawing.Color.Transparent;
            this.checkY.Enabled = false;
            this.checkY.Location = new System.Drawing.Point(90, 55);
            this.checkY.Name = "checkY";
            this.checkY.Size = new System.Drawing.Size(15, 14);
            this.checkY.TabIndex = 145;
            this.checkY.UseVisualStyleBackColor = false;
            // 
            // checkShoulderRight
            // 
            this.checkShoulderRight.AutoSize = true;
            this.checkShoulderRight.BackColor = System.Drawing.Color.Transparent;
            this.checkShoulderRight.Enabled = false;
            this.checkShoulderRight.Location = new System.Drawing.Point(90, 164);
            this.checkShoulderRight.Name = "checkShoulderRight";
            this.checkShoulderRight.Size = new System.Drawing.Size(15, 14);
            this.checkShoulderRight.TabIndex = 144;
            this.checkShoulderRight.UseVisualStyleBackColor = false;
            // 
            // label59
            // 
            this.label59.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label59.Location = new System.Drawing.Point(69, 10);
            this.label59.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label59.Name = "label59";
            this.label59.Size = new System.Drawing.Size(21, 15);
            this.label59.TabIndex = 150;
            this.label59.Text = "A:";
            this.label59.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label61
            // 
            this.label61.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label61.Location = new System.Drawing.Point(69, 25);
            this.label61.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label61.Name = "label61";
            this.label61.Size = new System.Drawing.Size(21, 15);
            this.label61.TabIndex = 164;
            this.label61.Text = "B:";
            this.label61.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label62
            // 
            this.label62.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label62.Location = new System.Drawing.Point(69, 38);
            this.label62.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label62.Name = "label62";
            this.label62.Size = new System.Drawing.Size(21, 15);
            this.label62.TabIndex = 165;
            this.label62.Text = "X:";
            this.label62.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label69
            // 
            this.label69.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label69.Location = new System.Drawing.Point(69, 54);
            this.label69.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label69.Name = "label69";
            this.label69.Size = new System.Drawing.Size(21, 15);
            this.label69.TabIndex = 166;
            this.label69.Text = "Y:";
            this.label69.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label71
            // 
            this.label71.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label71.Location = new System.Drawing.Point(34, 116);
            this.label71.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label71.Name = "label71";
            this.label71.Size = new System.Drawing.Size(56, 15);
            this.label71.TabIndex = 167;
            this.label71.Text = "StickLeft:";
            this.label71.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label73
            // 
            this.label73.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label73.Location = new System.Drawing.Point(24, 132);
            this.label73.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label73.Name = "label73";
            this.label73.Size = new System.Drawing.Size(66, 15);
            this.label73.TabIndex = 168;
            this.label73.Text = "StickRight:";
            this.label73.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label76
            // 
            this.label76.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label76.Location = new System.Drawing.Point(20, 147);
            this.label76.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label76.Name = "label76";
            this.label76.Size = new System.Drawing.Size(70, 15);
            this.label76.TabIndex = 169;
            this.label76.Text = "ShoulderLeft:";
            this.label76.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label80
            // 
            this.label80.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label80.Location = new System.Drawing.Point(10, 163);
            this.label80.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label80.Name = "label80";
            this.label80.Size = new System.Drawing.Size(80, 15);
            this.label80.TabIndex = 170;
            this.label80.Text = "ShoulderRight:";
            this.label80.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label81
            // 
            this.label81.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label81.Location = new System.Drawing.Point(109, 10);
            this.label81.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label81.Name = "label81";
            this.label81.Size = new System.Drawing.Size(80, 15);
            this.label81.TabIndex = 171;
            this.label81.Text = "DPadUp:";
            this.label81.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label82
            // 
            this.label82.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label82.Location = new System.Drawing.Point(109, 25);
            this.label82.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label82.Name = "label82";
            this.label82.Size = new System.Drawing.Size(80, 15);
            this.label82.TabIndex = 172;
            this.label82.Text = "DPadRight:";
            this.label82.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label84
            // 
            this.label84.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label84.Location = new System.Drawing.Point(109, 41);
            this.label84.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label84.Name = "label84";
            this.label84.Size = new System.Drawing.Size(80, 15);
            this.label84.TabIndex = 173;
            this.label84.Text = "DPadDown:";
            this.label84.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label85
            // 
            this.label85.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label85.Location = new System.Drawing.Point(109, 57);
            this.label85.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label85.Name = "label85";
            this.label85.Size = new System.Drawing.Size(80, 15);
            this.label85.TabIndex = 174;
            this.label85.Text = "DPadLeft:";
            this.label85.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label86
            // 
            this.label86.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label86.Location = new System.Drawing.Point(20, 101);
            this.label86.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label86.Name = "label86";
            this.label86.Size = new System.Drawing.Size(70, 15);
            this.label86.TabIndex = 177;
            this.label86.Text = "Guide:";
            this.label86.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label87
            // 
            this.label87.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label87.Location = new System.Drawing.Point(24, 86);
            this.label87.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label87.Name = "label87";
            this.label87.Size = new System.Drawing.Size(66, 15);
            this.label87.TabIndex = 176;
            this.label87.Text = "Back:";
            this.label87.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label88
            // 
            this.label88.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label88.Location = new System.Drawing.Point(34, 70);
            this.label88.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label88.Name = "label88";
            this.label88.Size = new System.Drawing.Size(56, 15);
            this.label88.TabIndex = 175;
            this.label88.Text = "Start:";
            this.label88.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // checkShoulderLeft
            // 
            this.checkShoulderLeft.AutoSize = true;
            this.checkShoulderLeft.BackColor = System.Drawing.Color.Transparent;
            this.checkShoulderLeft.CheckAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.checkShoulderLeft.Enabled = false;
            this.checkShoulderLeft.Location = new System.Drawing.Point(89, 149);
            this.checkShoulderLeft.Name = "checkShoulderLeft";
            this.checkShoulderLeft.Size = new System.Drawing.Size(15, 14);
            this.checkShoulderLeft.TabIndex = 178;
            this.checkShoulderLeft.UseVisualStyleBackColor = false;
            // 
            // label105
            // 
            this.label105.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label105.Location = new System.Drawing.Point(108, 127);
            this.label105.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label105.Name = "label105";
            this.label105.Size = new System.Drawing.Size(80, 15);
            this.label105.TabIndex = 182;
            this.label105.Text = "StickLeftY:";
            this.label105.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label111
            // 
            this.label111.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label111.Location = new System.Drawing.Point(109, 112);
            this.label111.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label111.Name = "label111";
            this.label111.Size = new System.Drawing.Size(80, 15);
            this.label111.TabIndex = 181;
            this.label111.Text = "StickLeftX:";
            this.label111.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label112
            // 
            this.label112.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label112.Location = new System.Drawing.Point(109, 91);
            this.label112.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label112.Name = "label112";
            this.label112.Size = new System.Drawing.Size(80, 15);
            this.label112.TabIndex = 180;
            this.label112.Text = "TriggerRight:";
            this.label112.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label113
            // 
            this.label113.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label113.Location = new System.Drawing.Point(109, 76);
            this.label113.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label113.Name = "label113";
            this.label113.Size = new System.Drawing.Size(80, 15);
            this.label113.TabIndex = 179;
            this.label113.Text = "TriggerLeft:";
            this.label113.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label114
            // 
            this.label114.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label114.Location = new System.Drawing.Point(108, 160);
            this.label114.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label114.Name = "label114";
            this.label114.Size = new System.Drawing.Size(80, 15);
            this.label114.TabIndex = 184;
            this.label114.Text = "StickRightY:";
            this.label114.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label115
            // 
            this.label115.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label115.Location = new System.Drawing.Point(108, 145);
            this.label115.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label115.Name = "label115";
            this.label115.Size = new System.Drawing.Size(80, 15);
            this.label115.TabIndex = 183;
            this.label115.Text = "StickRightX:";
            this.label115.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // pollingWorker
            // 
            this.pollingWorker.WorkerSupportsCancellation = true;
            this.pollingWorker.DoWork += new System.ComponentModel.DoWorkEventHandler(this.pollingWorker_DoWork);
            // 
            // dynaConnect
            // 
            this.dynaConnect.Location = new System.Drawing.Point(5, 50);
            this.dynaConnect.Margin = new System.Windows.Forms.Padding(2);
            this.dynaConnect.Name = "dynaConnect";
            this.dynaConnect.Size = new System.Drawing.Size(56, 19);
            this.dynaConnect.TabIndex = 185;
            this.dynaConnect.Text = "Connect";
            this.dynaConnect.UseVisualStyleBackColor = true;
            this.dynaConnect.Click += new System.EventHandler(this.dynaConnect_Click);
            // 
            // dynaDisconnect
            // 
            this.dynaDisconnect.Enabled = false;
            this.dynaDisconnect.Location = new System.Drawing.Point(65, 50);
            this.dynaDisconnect.Margin = new System.Windows.Forms.Padding(2);
            this.dynaDisconnect.Name = "dynaDisconnect";
            this.dynaDisconnect.Size = new System.Drawing.Size(74, 19);
            this.dynaDisconnect.TabIndex = 186;
            this.dynaDisconnect.Text = "Disconnect";
            this.dynaDisconnect.UseVisualStyleBackColor = true;
            this.dynaDisconnect.Click += new System.EventHandler(this.dynaDisconnect_Click);
            // 
            // TorqueOn
            // 
            this.TorqueOn.Location = new System.Drawing.Point(8, 20);
            this.TorqueOn.Margin = new System.Windows.Forms.Padding(2);
            this.TorqueOn.Name = "TorqueOn";
            this.TorqueOn.Size = new System.Drawing.Size(79, 21);
            this.TorqueOn.TabIndex = 187;
            this.TorqueOn.Text = "Torque On";
            this.TorqueOn.UseVisualStyleBackColor = true;
            this.TorqueOn.Click += new System.EventHandler(this.TorqueOn_Click);
            // 
            // TorqueOff
            // 
            this.TorqueOff.Enabled = false;
            this.TorqueOff.Location = new System.Drawing.Point(91, 20);
            this.TorqueOff.Margin = new System.Windows.Forms.Padding(2);
            this.TorqueOff.Name = "TorqueOff";
            this.TorqueOff.Size = new System.Drawing.Size(86, 21);
            this.TorqueOff.TabIndex = 188;
            this.TorqueOff.Text = "Torque Off";
            this.TorqueOff.UseVisualStyleBackColor = true;
            this.TorqueOff.Click += new System.EventHandler(this.TorqueOff_Click);
            // 
            // LEDon
            // 
            this.LEDon.Location = new System.Drawing.Point(5, 15);
            this.LEDon.Margin = new System.Windows.Forms.Padding(2);
            this.LEDon.Name = "LEDon";
            this.LEDon.Size = new System.Drawing.Size(56, 19);
            this.LEDon.TabIndex = 189;
            this.LEDon.Text = "LED On";
            this.LEDon.UseVisualStyleBackColor = true;
            this.LEDon.Click += new System.EventHandler(this.LEDon_Click);
            // 
            // LEDoff
            // 
            this.LEDoff.Location = new System.Drawing.Point(65, 15);
            this.LEDoff.Margin = new System.Windows.Forms.Padding(2);
            this.LEDoff.Name = "LEDoff";
            this.LEDoff.Size = new System.Drawing.Size(56, 19);
            this.LEDoff.TabIndex = 190;
            this.LEDoff.Text = "LED Off";
            this.LEDoff.UseVisualStyleBackColor = true;
            this.LEDoff.Click += new System.EventHandler(this.LEDoff_Click);
            // 
            // moveCW
            // 
            this.moveCW.Enabled = false;
            this.moveCW.Location = new System.Drawing.Point(147, 15);
            this.moveCW.Margin = new System.Windows.Forms.Padding(2);
            this.moveCW.Name = "moveCW";
            this.moveCW.Size = new System.Drawing.Size(80, 19);
            this.moveCW.TabIndex = 191;
            this.moveCW.Text = "Close Hand";
            this.moveCW.UseVisualStyleBackColor = true;
            this.moveCW.Click += new System.EventHandler(this.moveCW_Click);
            // 
            // moveCCW
            // 
            this.moveCCW.Enabled = false;
            this.moveCCW.Location = new System.Drawing.Point(231, 15);
            this.moveCCW.Margin = new System.Windows.Forms.Padding(2);
            this.moveCCW.Name = "moveCCW";
            this.moveCCW.Size = new System.Drawing.Size(80, 19);
            this.moveCCW.TabIndex = 192;
            this.moveCCW.Text = "Open Hand";
            this.moveCCW.UseVisualStyleBackColor = true;
            this.moveCCW.Click += new System.EventHandler(this.moveCCW_Click);
            // 
            // label116
            // 
            this.label116.AutoSize = true;
            this.label116.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label116.Location = new System.Drawing.Point(7, 26);
            this.label116.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label116.Name = "label116";
            this.label116.Size = new System.Drawing.Size(56, 13);
            this.label116.TabIndex = 65;
            this.label116.Text = "COM Port:";
            // 
            // label117
            // 
            this.label117.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label117.Location = new System.Drawing.Point(5, 24);
            this.label117.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label117.Name = "label117";
            this.label117.Size = new System.Drawing.Size(76, 15);
            this.label117.TabIndex = 193;
            this.label117.Text = "DELAY (ms):";
            this.label117.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // delay
            // 
            this.delay.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.delay.Location = new System.Drawing.Point(85, 24);
            this.delay.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.delay.Name = "delay";
            this.delay.Size = new System.Drawing.Size(39, 15);
            this.delay.TabIndex = 194;
            this.delay.Text = "--";
            // 
            // label118
            // 
            this.label118.AutoSize = true;
            this.label118.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label118.Location = new System.Drawing.Point(213, 16);
            this.label118.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label118.Name = "label118";
            this.label118.Size = new System.Drawing.Size(72, 13);
            this.label118.TabIndex = 195;
            this.label118.Text = "Comm Result:";
            this.label118.TextAlign = System.Drawing.ContentAlignment.TopRight;
            this.label118.Visible = false;
            // 
            // dynaCommResult
            // 
            this.dynaCommResult.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.dynaCommResult.Location = new System.Drawing.Point(282, 16);
            this.dynaCommResult.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.dynaCommResult.Name = "dynaCommResult";
            this.dynaCommResult.Size = new System.Drawing.Size(39, 15);
            this.dynaCommResult.TabIndex = 196;
            this.dynaCommResult.Text = "--";
            this.dynaCommResult.Visible = false;
            // 
            // label120
            // 
            this.label120.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label120.Location = new System.Drawing.Point(246, 32);
            this.label120.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label120.Name = "label120";
            this.label120.Size = new System.Drawing.Size(39, 15);
            this.label120.TabIndex = 198;
            this.label120.Text = "Error:";
            this.label120.TextAlign = System.Drawing.ContentAlignment.TopRight;
            this.label120.Visible = false;
            // 
            // dynaError
            // 
            this.dynaError.AutoSize = true;
            this.dynaError.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.dynaError.Location = new System.Drawing.Point(282, 33);
            this.dynaError.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.dynaError.Name = "dynaError";
            this.dynaError.Size = new System.Drawing.Size(13, 13);
            this.dynaError.TabIndex = 197;
            this.dynaError.Text = "--";
            this.dynaError.TextAlign = System.Drawing.ContentAlignment.TopRight;
            this.dynaError.Visible = false;
            // 
            // delay_max
            // 
            this.delay_max.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.delay_max.Location = new System.Drawing.Point(224, 24);
            this.delay_max.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.delay_max.Name = "delay_max";
            this.delay_max.Size = new System.Drawing.Size(39, 15);
            this.delay_max.TabIndex = 203;
            this.delay_max.Text = "0";
            // 
            // label121
            // 
            this.label121.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label121.Location = new System.Drawing.Point(125, 24);
            this.label121.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label121.Name = "label121";
            this.label121.Size = new System.Drawing.Size(95, 15);
            this.label121.TabIndex = 202;
            this.label121.Text = "MAX DELAY (ms):";
            this.label121.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // label119
            // 
            this.label119.AutoSize = true;
            this.label119.Location = new System.Drawing.Point(200, 26);
            this.label119.Name = "label119";
            this.label119.Size = new System.Drawing.Size(40, 13);
            this.label119.TabIndex = 204;
            this.label119.Text = "Status:";
            this.label119.Visible = false;
            // 
            // dynaStatus
            // 
            this.dynaStatus.AutoSize = true;
            this.dynaStatus.Location = new System.Drawing.Point(237, 26);
            this.dynaStatus.Name = "dynaStatus";
            this.dynaStatus.Size = new System.Drawing.Size(73, 13);
            this.dynaStatus.TabIndex = 205;
            this.dynaStatus.Text = "Disconnected";
            this.dynaStatus.Visible = false;
            // 
            // cmbSerialRefresh
            // 
            this.cmbSerialRefresh.Location = new System.Drawing.Point(139, 23);
            this.cmbSerialRefresh.Margin = new System.Windows.Forms.Padding(2);
            this.cmbSerialRefresh.Name = "cmbSerialRefresh";
            this.cmbSerialRefresh.Size = new System.Drawing.Size(56, 19);
            this.cmbSerialRefresh.TabIndex = 206;
            this.cmbSerialRefresh.Text = "Refresh";
            this.cmbSerialRefresh.UseVisualStyleBackColor = true;
            this.cmbSerialRefresh.Click += new System.EventHandler(this.cmbSerialRefresh_Click);
            // 
            // BentoGroupBox
            // 
            this.BentoGroupBox.Controls.Add(this.groupBox1);
            this.BentoGroupBox.Controls.Add(this.TorqueOn);
            this.BentoGroupBox.Controls.Add(this.TorqueOff);
            this.BentoGroupBox.Controls.Add(this.RobotFeedbackBox);
            this.BentoGroupBox.Controls.Add(this.label118);
            this.BentoGroupBox.Controls.Add(this.dynaError);
            this.BentoGroupBox.Controls.Add(this.dynaCommResult);
            this.BentoGroupBox.Controls.Add(this.label120);
            this.BentoGroupBox.Enabled = false;
            this.BentoGroupBox.Location = new System.Drawing.Point(280, 139);
            this.BentoGroupBox.Name = "BentoGroupBox";
            this.BentoGroupBox.Size = new System.Drawing.Size(341, 251);
            this.BentoGroupBox.TabIndex = 207;
            this.BentoGroupBox.TabStop = false;
            this.BentoGroupBox.Text = "Bento Arm";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.LEDon);
            this.groupBox1.Controls.Add(this.moveCCW);
            this.groupBox1.Controls.Add(this.moveCW);
            this.groupBox1.Controls.Add(this.LEDoff);
            this.groupBox1.Location = new System.Drawing.Point(6, 46);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(326, 39);
            this.groupBox1.TabIndex = 193;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Test Commands";
            // 
            // xBoxGroupBox
            // 
            this.xBoxGroupBox.Controls.Add(this.label80);
            this.xBoxGroupBox.Controls.Add(this.checkShoulderRight);
            this.xBoxGroupBox.Controls.Add(this.checkY);
            this.xBoxGroupBox.Controls.Add(this.checkX);
            this.xBoxGroupBox.Controls.Add(this.checkB);
            this.xBoxGroupBox.Controls.Add(this.checkA);
            this.xBoxGroupBox.Controls.Add(this.checkStart);
            this.xBoxGroupBox.Controls.Add(this.checkBack);
            this.xBoxGroupBox.Controls.Add(this.checkStickRight);
            this.xBoxGroupBox.Controls.Add(this.checkStickLeft);
            this.xBoxGroupBox.Controls.Add(this.checkDPadUp);
            this.xBoxGroupBox.Controls.Add(this.checkDPadRight);
            this.xBoxGroupBox.Controls.Add(this.checkDPadDown);
            this.xBoxGroupBox.Controls.Add(this.checkDPadLeft);
            this.xBoxGroupBox.Controls.Add(this.labelTriggerLeft);
            this.xBoxGroupBox.Controls.Add(this.labelTriggerRight);
            this.xBoxGroupBox.Controls.Add(this.labelStickLeftX);
            this.xBoxGroupBox.Controls.Add(this.labelStickLeftY);
            this.xBoxGroupBox.Controls.Add(this.labelStickRightX);
            this.xBoxGroupBox.Controls.Add(this.label114);
            this.xBoxGroupBox.Controls.Add(this.labelStickRightY);
            this.xBoxGroupBox.Controls.Add(this.label115);
            this.xBoxGroupBox.Controls.Add(this.checkGuide);
            this.xBoxGroupBox.Controls.Add(this.label105);
            this.xBoxGroupBox.Controls.Add(this.label59);
            this.xBoxGroupBox.Controls.Add(this.label111);
            this.xBoxGroupBox.Controls.Add(this.label61);
            this.xBoxGroupBox.Controls.Add(this.label112);
            this.xBoxGroupBox.Controls.Add(this.label62);
            this.xBoxGroupBox.Controls.Add(this.label113);
            this.xBoxGroupBox.Controls.Add(this.label69);
            this.xBoxGroupBox.Controls.Add(this.checkShoulderLeft);
            this.xBoxGroupBox.Controls.Add(this.label71);
            this.xBoxGroupBox.Controls.Add(this.label86);
            this.xBoxGroupBox.Controls.Add(this.label73);
            this.xBoxGroupBox.Controls.Add(this.label87);
            this.xBoxGroupBox.Controls.Add(this.label76);
            this.xBoxGroupBox.Controls.Add(this.label88);
            this.xBoxGroupBox.Controls.Add(this.label81);
            this.xBoxGroupBox.Controls.Add(this.label85);
            this.xBoxGroupBox.Controls.Add(this.label82);
            this.xBoxGroupBox.Controls.Add(this.label84);
            this.xBoxGroupBox.Enabled = false;
            this.xBoxGroupBox.Location = new System.Drawing.Point(9, 139);
            this.xBoxGroupBox.Name = "xBoxGroupBox";
            this.xBoxGroupBox.Size = new System.Drawing.Size(265, 187);
            this.xBoxGroupBox.TabIndex = 208;
            this.xBoxGroupBox.TabStop = false;
            this.xBoxGroupBox.Text = "Xbox";
            // 
            // XboxDisconnect
            // 
            this.XboxDisconnect.Enabled = false;
            this.XboxDisconnect.Location = new System.Drawing.Point(65, 18);
            this.XboxDisconnect.Margin = new System.Windows.Forms.Padding(2);
            this.XboxDisconnect.Name = "XboxDisconnect";
            this.XboxDisconnect.Size = new System.Drawing.Size(74, 19);
            this.XboxDisconnect.TabIndex = 210;
            this.XboxDisconnect.Text = "Disconnect";
            this.XboxDisconnect.UseVisualStyleBackColor = true;
            this.XboxDisconnect.Click += new System.EventHandler(this.XboxDisconnect_Click);
            // 
            // XboxConnect
            // 
            this.XboxConnect.Location = new System.Drawing.Point(5, 18);
            this.XboxConnect.Margin = new System.Windows.Forms.Padding(2);
            this.XboxConnect.Name = "XboxConnect";
            this.XboxConnect.Size = new System.Drawing.Size(56, 19);
            this.XboxConnect.TabIndex = 209;
            this.XboxConnect.Text = "Connect";
            this.XboxConnect.UseVisualStyleBackColor = true;
            this.XboxConnect.Click += new System.EventHandler(this.XboxConnect_Click);
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.groupBox6);
            this.groupBox4.Controls.Add(this.groupBox5);
            this.groupBox4.Location = new System.Drawing.Point(9, 27);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(612, 106);
            this.groupBox4.TabIndex = 211;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "I/O Setup";
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.dynaDisconnect);
            this.groupBox6.Controls.Add(this.cmbSerialPorts);
            this.groupBox6.Controls.Add(this.dynaConnect);
            this.groupBox6.Controls.Add(this.dynaStatus);
            this.groupBox6.Controls.Add(this.cmbSerialRefresh);
            this.groupBox6.Controls.Add(this.label119);
            this.groupBox6.Controls.Add(this.label116);
            this.groupBox6.Location = new System.Drawing.Point(271, 19);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Size = new System.Drawing.Size(332, 77);
            this.groupBox6.TabIndex = 1;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "Bento Arm - Setup";
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.XboxConnect);
            this.groupBox5.Controls.Add(this.XboxDisconnect);
            this.groupBox5.Location = new System.Drawing.Point(6, 19);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(259, 77);
            this.groupBox5.TabIndex = 0;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "Xbox - Setup";
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.label117);
            this.groupBox3.Controls.Add(this.delay);
            this.groupBox3.Controls.Add(this.label121);
            this.groupBox3.Controls.Add(this.delay_max);
            this.groupBox3.Location = new System.Drawing.Point(9, 329);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(265, 61);
            this.groupBox3.TabIndex = 212;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Loop Delay";
            // 
            // mainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(628, 395);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.xBoxGroupBox);
            this.Controls.Add(this.BentoGroupBox);
            this.Controls.Add(this.MenuStrip1);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "mainForm";
            this.Text = "brachI/Oplexus - V0.6";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.mainForm_FormClosing);
            this.Load += new System.EventHandler(this.mainForm_Load);
            this.MenuStrip1.ResumeLayout(false);
            this.MenuStrip1.PerformLayout();
            this.RobotFeedbackBox.ResumeLayout(false);
            this.RobotFeedbackBox.PerformLayout();
            this.BentoGroupBox.ResumeLayout(false);
            this.BentoGroupBox.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.xBoxGroupBox.ResumeLayout(false);
            this.xBoxGroupBox.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            this.groupBox5.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        internal System.Windows.Forms.MenuStrip MenuStrip1;
        internal System.Windows.Forms.ToolStripMenuItem FileToolStripMenuItem;
        internal System.Windows.Forms.ToolStripSeparator toolStripSeparator;
        internal System.Windows.Forms.ToolStripSeparator toolStripSeparator2;
        internal System.Windows.Forms.ToolStripMenuItem ExitToolStripMenuItem;
        internal System.Windows.Forms.ToolStripMenuItem HelpToolStripMenuItem;
        internal System.Windows.Forms.ToolStripMenuItem ContentsToolStripMenuItem;
        internal System.Windows.Forms.ToolStripSeparator toolStripSeparator5;
        internal System.Windows.Forms.ToolStripMenuItem AboutToolStripMenuItem;
        private System.IO.Ports.SerialPort serialPort1;
        internal System.Windows.Forms.Label label7;
        internal System.Windows.Forms.Label Label18;
        internal System.Windows.Forms.Label Label20;
        internal System.Windows.Forms.Label Label21;
        internal System.Windows.Forms.Label Label19;
        private System.Windows.Forms.GroupBox RobotFeedbackBox;
        internal System.Windows.Forms.Label Temp4;
        internal System.Windows.Forms.Label Volt4;
        internal System.Windows.Forms.Label Load4;
        internal System.Windows.Forms.Label Vel4;
        internal System.Windows.Forms.Label Pos4;
        internal System.Windows.Forms.Label Temp5;
        internal System.Windows.Forms.Label Volt5;
        internal System.Windows.Forms.Label Load5;
        internal System.Windows.Forms.Label Vel5;
        internal System.Windows.Forms.Label Pos5;
        internal System.Windows.Forms.Label Temp3;
        internal System.Windows.Forms.Label Volt3;
        internal System.Windows.Forms.Label Load3;
        internal System.Windows.Forms.Label Vel3;
        internal System.Windows.Forms.Label Pos3;
        internal System.Windows.Forms.Label Temp2;
        internal System.Windows.Forms.Label Volt2;
        internal System.Windows.Forms.Label Load2;
        internal System.Windows.Forms.Label Vel2;
        internal System.Windows.Forms.Label Pos2;
        internal System.Windows.Forms.Label Temp1;
        internal System.Windows.Forms.Label Volt1;
        internal System.Windows.Forms.Label Load1;
        internal System.Windows.Forms.Label Vel1;
        internal System.Windows.Forms.Label Pos1;
        internal System.Windows.Forms.Label label109;
        internal System.Windows.Forms.Label label108;
        internal System.Windows.Forms.Label label107;
        internal System.Windows.Forms.Label label106;
        internal System.Windows.Forms.Label label200;
        internal System.Windows.Forms.HelpProvider HelpProvider1;
        private System.Windows.Forms.ComboBox cmbSerialPorts;
        internal System.Windows.Forms.Label label114;
        internal System.Windows.Forms.Label label115;
        internal System.Windows.Forms.Label label105;
        internal System.Windows.Forms.Label label111;
        internal System.Windows.Forms.Label label112;
        internal System.Windows.Forms.Label label113;
        private System.Windows.Forms.CheckBox checkShoulderLeft;
        internal System.Windows.Forms.Label label86;
        internal System.Windows.Forms.Label label87;
        internal System.Windows.Forms.Label label88;
        internal System.Windows.Forms.Label label85;
        internal System.Windows.Forms.Label label84;
        internal System.Windows.Forms.Label label82;
        internal System.Windows.Forms.Label label81;
        internal System.Windows.Forms.Label label80;
        internal System.Windows.Forms.Label label76;
        internal System.Windows.Forms.Label label73;
        internal System.Windows.Forms.Label label71;
        internal System.Windows.Forms.Label label69;
        internal System.Windows.Forms.Label label62;
        internal System.Windows.Forms.Label label61;
        internal System.Windows.Forms.Label label59;
        private System.Windows.Forms.CheckBox checkGuide;
        private System.Windows.Forms.Label labelStickRightY;
        private System.Windows.Forms.Label labelStickRightX;
        private System.Windows.Forms.Label labelStickLeftY;
        private System.Windows.Forms.Label labelStickLeftX;
        private System.Windows.Forms.Label labelTriggerRight;
        private System.Windows.Forms.Label labelTriggerLeft;
        private System.Windows.Forms.CheckBox checkDPadLeft;
        private System.Windows.Forms.CheckBox checkDPadDown;
        private System.Windows.Forms.CheckBox checkDPadRight;
        private System.Windows.Forms.CheckBox checkDPadUp;
        private System.Windows.Forms.CheckBox checkStickLeft;
        private System.Windows.Forms.CheckBox checkStickRight;
        private System.Windows.Forms.CheckBox checkBack;
        private System.Windows.Forms.CheckBox checkStart;
        private System.Windows.Forms.CheckBox checkA;
        private System.Windows.Forms.CheckBox checkB;
        private System.Windows.Forms.CheckBox checkX;
        private System.Windows.Forms.CheckBox checkY;
        private System.Windows.Forms.CheckBox checkShoulderRight;
        private System.ComponentModel.BackgroundWorker pollingWorker;
        private System.Windows.Forms.Button dynaDisconnect;
        private System.Windows.Forms.Button dynaConnect;
        private System.Windows.Forms.Button LEDoff;
        private System.Windows.Forms.Button LEDon;
        private System.Windows.Forms.Button TorqueOff;
        private System.Windows.Forms.Button TorqueOn;
        private System.Windows.Forms.Button moveCCW;
        private System.Windows.Forms.Button moveCW;
        internal System.Windows.Forms.Label label116;
        internal System.Windows.Forms.Label delay;
        internal System.Windows.Forms.Label label117;
        internal System.Windows.Forms.Label label118;
        internal System.Windows.Forms.Label label120;
        internal System.Windows.Forms.Label dynaError;
        internal System.Windows.Forms.Label dynaCommResult;
        internal System.Windows.Forms.Label delay_max;
        internal System.Windows.Forms.Label label121;
        private System.Windows.Forms.Label dynaStatus;
        private System.Windows.Forms.Label label119;
        private System.Windows.Forms.Button cmbSerialRefresh;
        private System.Windows.Forms.GroupBox BentoGroupBox;
        private System.Windows.Forms.GroupBox xBoxGroupBox;
        private System.Windows.Forms.Button XboxDisconnect;
        private System.Windows.Forms.Button XboxConnect;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.ToolStripMenuItem mappingGraphicToolStripMenuItem;
    }
}

