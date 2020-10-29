using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace brachIOplexus
{
    public partial class DoF : UserControl
    {
        public DoF()
        {
            InitializeComponent();

            // Delegate signalbar mouse click event to hide/show the individual degrees of freedom (DOF)
            // The delegation needs to happen hear instead of InitializeComponent() otherwise the designer view does not open properly
            this.channel2.signalBar.MouseClick += new System.Windows.Forms.MouseEventHandler(this.channel2_MouseClick);
            this.channel1.signalBar.MouseClick += new System.Windows.Forms.MouseEventHandler(this.channel1_MouseClick);
        }
        
        // Check mouse click events to hide/show the individual degrees of freedom (DOF)
        // Any mouse click will make the entire DoF visible
        private void DoF_MouseClick(object sender, MouseEventArgs e)
        {
            if (DoFBox.Visible == false)
            {
                DoFBox.Visible = true;
            }
        }

        // Check mouse click events to hide/show the individual degrees of freedom (DOF)
        // A right click will hide the degree of freedom
        private void channel1_MouseClick(object sender, MouseEventArgs e)
        {
            if (e.Button == System.Windows.Forms.MouseButtons.Right && DoFBox.Visible == true)
            {
                DoFBox.Visible = false;
            }
        }

        // Check mouse click events to hide/show the individual degrees of freedom (DOF)
        // A right click will hide the degree of freedom
        private void channel2_MouseClick(object sender, MouseEventArgs e)
        {
            if (e.Button == System.Windows.Forms.MouseButtons.Right && DoFBox.Visible == true)
            {
                DoFBox.Visible = false;
            }
        }
    }
}
