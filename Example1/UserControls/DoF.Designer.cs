namespace brachIOplexus
{
    partial class DoF
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
            this.DoFBox = new System.Windows.Forms.GroupBox();
            this.channel2 = new brachIOplexus.Channel();
            this.channel1 = new brachIOplexus.Channel();
            this.DoFBox.SuspendLayout();
            this.SuspendLayout();
            // 
            // DoFBox
            // 
            this.DoFBox.Controls.Add(this.channel2);
            this.DoFBox.Controls.Add(this.channel1);
            this.DoFBox.Location = new System.Drawing.Point(0, -2);
            this.DoFBox.Name = "DoFBox";
            this.DoFBox.Size = new System.Drawing.Size(650, 96);
            this.DoFBox.TabIndex = 0;
            this.DoFBox.TabStop = false;
            this.DoFBox.Text = "Degree of Freedom";
            this.DoFBox.MouseClick += new System.Windows.Forms.MouseEventHandler(this.channel2_MouseClick);     // Delegate mouse click event to hide/show the individual degrees of freedom (DOF)
            // 
            // channel2
            // 
            this.channel2.Location = new System.Drawing.Point(5, 55);
            this.channel2.Name = "channel2";
            this.channel2.Size = new System.Drawing.Size(640, 39);
            this.channel2.TabIndex = 1;
            this.channel2.MouseClick += new System.Windows.Forms.MouseEventHandler(this.channel2_MouseClick);   // Delegate mouse click event to hide/show the individual degrees of freedom (DOF)

            // 
            // channel1
            // 
            this.channel1.Location = new System.Drawing.Point(5, 17);
            this.channel1.Name = "channel1";
            this.channel1.Size = new System.Drawing.Size(640, 39);
            this.channel1.TabIndex = 0;
            this.channel1.MouseClick += new System.Windows.Forms.MouseEventHandler(this.channel1_MouseClick);   // Delegate mouse click event to hide/show the individual degrees of freedom (DOF)
            // 
            // DoF
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.DoFBox);
            this.Name = "DoF";
            this.Size = new System.Drawing.Size(652, 96);
            this.MouseClick += new System.Windows.Forms.MouseEventHandler(this.DoF_MouseClick);
            this.DoFBox.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion
        public System.Windows.Forms.GroupBox DoFBox;
        public Channel channel1;
        public Channel channel2;
    }
}
