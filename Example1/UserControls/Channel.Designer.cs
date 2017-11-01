namespace brachIOplexus
{
    partial class Channel
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

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.inputBox = new System.Windows.Forms.ComboBox();
            this.label145 = new System.Windows.Forms.Label();
            this.outputBox = new System.Windows.Forms.ComboBox();
            this.sminLabel = new System.Windows.Forms.Label();
            this.sminTick = new System.Windows.Forms.Label();
            this.smaxTick = new System.Windows.Forms.Label();
            this.smaxCtrl = new System.Windows.Forms.NumericUpDown();
            this.sminCtrl = new System.Windows.Forms.NumericUpDown();
            this.signalBar = new System.Windows.Forms.ProgressBar();
            this.mappingBox = new System.Windows.Forms.ComboBox();
            this.label161 = new System.Windows.Forms.Label();
            this.gainCtrl = new System.Windows.Forms.NumericUpDown();
            this.smaxLabel = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.smaxCtrl)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.sminCtrl)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.gainCtrl)).BeginInit();
            this.SuspendLayout();
            // 
            // inputBox
            // 
            this.inputBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.inputBox.FormattingEnabled = true;
            this.inputBox.Items.AddRange(new object[] {
            "Off"});
            this.inputBox.Location = new System.Drawing.Point(0, 0);
            this.inputBox.Margin = new System.Windows.Forms.Padding(2);
            this.inputBox.Name = "inputBox";
            this.inputBox.Size = new System.Drawing.Size(99, 21);
            this.inputBox.TabIndex = 157;
            this.inputBox.SelectedIndexChanged += new System.EventHandler(this.inputBox_SelectedIndexChanged);
            this.inputBox.Enter += new System.EventHandler(this.inputBox_Enter);
            // 
            // label145
            // 
            this.label145.AutoSize = true;
            this.label145.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label145.Location = new System.Drawing.Point(223, 3);
            this.label145.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label145.Name = "label145";
            this.label145.Size = new System.Drawing.Size(18, 13);
            this.label145.TabIndex = 155;
            this.label145.Text = "by";
            // 
            // outputBox
            // 
            this.outputBox.DisplayMember = "1";
            this.outputBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.outputBox.FormattingEnabled = true;
            this.outputBox.Items.AddRange(new object[] {
            "Off"});
            this.outputBox.Location = new System.Drawing.Point(121, 0);
            this.outputBox.Margin = new System.Windows.Forms.Padding(2);
            this.outputBox.Name = "outputBox";
            this.outputBox.Size = new System.Drawing.Size(99, 21);
            this.outputBox.TabIndex = 156;
            this.outputBox.SelectedIndexChanged += new System.EventHandler(this.outputBox_SelectedIndexChanged);
            // 
            // sminLabel
            // 
            this.sminLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F);
            this.sminLabel.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.sminLabel.Location = new System.Drawing.Point(351, 24);
            this.sminLabel.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.sminLabel.Name = "sminLabel";
            this.sminLabel.Size = new System.Drawing.Size(26, 15);
            this.sminLabel.TabIndex = 153;
            this.sminLabel.Text = "Smin";
            this.sminLabel.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // sminTick
            // 
            this.sminTick.BackColor = System.Drawing.Color.MediumPurple;
            this.sminTick.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.sminTick.Location = new System.Drawing.Point(363, 0);
            this.sminTick.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.sminTick.Name = "sminTick";
            this.sminTick.Size = new System.Drawing.Size(2, 24);
            this.sminTick.TabIndex = 152;
            // 
            // smaxTick
            // 
            this.smaxTick.BackColor = System.Drawing.Color.MediumPurple;
            this.smaxTick.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.smaxTick.Location = new System.Drawing.Point(496, 0);
            this.smaxTick.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.smaxTick.Name = "smaxTick";
            this.smaxTick.Size = new System.Drawing.Size(2, 24);
            this.smaxTick.TabIndex = 151;
            // 
            // smaxCtrl
            // 
            this.smaxCtrl.DecimalPlaces = 1;
            this.smaxCtrl.Increment = new decimal(new int[] {
            2,
            0,
            0,
            65536});
            this.smaxCtrl.Location = new System.Drawing.Point(601, 0);
            this.smaxCtrl.Margin = new System.Windows.Forms.Padding(2);
            this.smaxCtrl.Maximum = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.smaxCtrl.Name = "smaxCtrl";
            this.smaxCtrl.Size = new System.Drawing.Size(40, 20);
            this.smaxCtrl.TabIndex = 150;
            this.smaxCtrl.Value = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.smaxCtrl.ValueChanged += new System.EventHandler(this.smaxCtrl_ValueChanged);
            // 
            // sminCtrl
            // 
            this.sminCtrl.DecimalPlaces = 1;
            this.sminCtrl.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.sminCtrl.Increment = new decimal(new int[] {
            2,
            0,
            0,
            65536});
            this.sminCtrl.Location = new System.Drawing.Point(557, 0);
            this.sminCtrl.Margin = new System.Windows.Forms.Padding(2);
            this.sminCtrl.Maximum = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.sminCtrl.Name = "sminCtrl";
            this.sminCtrl.Size = new System.Drawing.Size(40, 20);
            this.sminCtrl.TabIndex = 149;
            this.sminCtrl.ValueChanged += new System.EventHandler(this.sminCtrl_ValueChanged);
            // 
            // signalBar
            // 
            this.signalBar.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.signalBar.Location = new System.Drawing.Point(363, 0);
            this.signalBar.Margin = new System.Windows.Forms.Padding(2);
            this.signalBar.MarqueeAnimationSpeed = 30;
            this.signalBar.Maximum = 500;
            this.signalBar.Name = "signalBar";
            this.signalBar.Size = new System.Drawing.Size(134, 22);
            this.signalBar.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
            this.signalBar.TabIndex = 146;
            // 
            // mappingBox
            // 
            this.mappingBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.mappingBox.FormattingEnabled = true;
            this.mappingBox.Items.AddRange(new object[] {
            "First to Smin"});
            this.mappingBox.Location = new System.Drawing.Point(242, 0);
            this.mappingBox.Margin = new System.Windows.Forms.Padding(2);
            this.mappingBox.Name = "mappingBox";
            this.mappingBox.Size = new System.Drawing.Size(99, 21);
            this.mappingBox.TabIndex = 148;
            // 
            // label161
            // 
            this.label161.AutoSize = true;
            this.label161.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.label161.Location = new System.Drawing.Point(102, 3);
            this.label161.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label161.Name = "label161";
            this.label161.Size = new System.Drawing.Size(19, 13);
            this.label161.TabIndex = 147;
            this.label161.Text = ">>";
            // 
            // gainCtrl
            // 
            this.gainCtrl.Location = new System.Drawing.Point(511, 0);
            this.gainCtrl.Margin = new System.Windows.Forms.Padding(2);
            this.gainCtrl.Maximum = new decimal(new int[] {
            5000,
            0,
            0,
            0});
            this.gainCtrl.Name = "gainCtrl";
            this.gainCtrl.Size = new System.Drawing.Size(42, 20);
            this.gainCtrl.TabIndex = 145;
            this.gainCtrl.Value = new decimal(new int[] {
            100,
            0,
            0,
            0});
            // 
            // smaxLabel
            // 
            this.smaxLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F);
            this.smaxLabel.ImeMode = System.Windows.Forms.ImeMode.NoControl;
            this.smaxLabel.Location = new System.Drawing.Point(482, 24);
            this.smaxLabel.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.smaxLabel.Name = "smaxLabel";
            this.smaxLabel.Size = new System.Drawing.Size(30, 15);
            this.smaxLabel.TabIndex = 154;
            this.smaxLabel.Text = "Smax";
            this.smaxLabel.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            // 
            // Channel
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.inputBox);
            this.Controls.Add(this.label145);
            this.Controls.Add(this.outputBox);
            this.Controls.Add(this.smaxLabel);
            this.Controls.Add(this.sminLabel);
            this.Controls.Add(this.sminTick);
            this.Controls.Add(this.smaxTick);
            this.Controls.Add(this.smaxCtrl);
            this.Controls.Add(this.sminCtrl);
            this.Controls.Add(this.signalBar);
            this.Controls.Add(this.mappingBox);
            this.Controls.Add(this.label161);
            this.Controls.Add(this.gainCtrl);
            this.Name = "Channel";
            this.Size = new System.Drawing.Size(640, 39);
            ((System.ComponentModel.ISupportInitialize)(this.smaxCtrl)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.sminCtrl)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.gainCtrl)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        public System.Windows.Forms.ComboBox inputBox;
        public System.Windows.Forms.ComboBox outputBox;
        public System.Windows.Forms.ProgressBar signalBar;
        public System.Windows.Forms.ComboBox mappingBox;
        private System.Windows.Forms.Label sminLabel;
        private System.Windows.Forms.Label sminTick;
        private System.Windows.Forms.Label smaxTick;
        public System.Windows.Forms.NumericUpDown smaxCtrl;
        public System.Windows.Forms.NumericUpDown sminCtrl;
        public System.Windows.Forms.NumericUpDown gainCtrl;
        private System.Windows.Forms.Label label145;
        private System.Windows.Forms.Label label161;
        private System.Windows.Forms.Label smaxLabel;
    }
}
