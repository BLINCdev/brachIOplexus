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
    public partial class Channel : UserControl
    {
        public Channel()
        {
            InitializeComponent();
        }

        private void sminCtrl_ValueChanged(object sender, EventArgs e)
        {
            // Adjust the position of the smin tick and label to reflect changes to the smin value
            sminTick.Location = new Point(tick_position(Convert.ToDouble(sminCtrl.Value)), sminTick.Location.Y);
            sminLabel.Location = new Point(tick_position(Convert.ToDouble(sminCtrl.Value)) - sminLabel.Width/2 + sminTick.Width/2, sminLabel.Location.Y);
        }


        private void smaxCtrl_ValueChanged(object sender, EventArgs e)
        {
            // Adjust the position of the smin tick and label to reflect changes to the smin value
            smaxTick.Location = new Point(tick_position(Convert.ToDouble(smaxCtrl.Value)), smaxTick.Location.Y);
            smaxLabel.Location = new Point(tick_position(Convert.ToDouble(smaxCtrl.Value)) - smaxLabel.Width / 2 + smaxTick.Width / 2, smaxLabel.Location.Y);
        }

        //Helper function to control position of threshold ticks and labels
        public int tick_position(double x)
        {
            //return Convert.ToInt32(35.4 * voltage + 52);
            return Convert.ToInt32(signalBar.Width / Convert.ToDouble(sminCtrl.Maximum) * x + signalBar.Location.X);
        }

        // When index changes on output box trigger method in parent form using event handler
        public event EventHandler OutputIndexChanged;

        protected virtual void OnOutputIndexChanged(EventArgs e)
        {
            var handler = OutputIndexChanged;
            if (handler != null)
                handler(this, e);
        }

        private void outputBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            OnOutputIndexChanged(e);
        }

        // When index changes on input box trigger method in parent form using event handler
        public event EventHandler InputIndexChanged;

        protected virtual void OnInputIndexChanged(EventArgs e)
        {
            var handler = InputIndexChanged;
            if (handler != null)
                handler(this, e);
        }

        private void inputBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            OnInputIndexChanged(e);
        }

        // When inputbox comes into focus (either by clicking on or tabbing to it) on input box trigger method in parent form using event handler
        public event EventHandler InputEnter;

        protected virtual void OnInputEnter(EventArgs e)
        {
            var handler = InputEnter;
            if (handler != null)
                handler(this, e);
        }

        private void inputBox_Enter(object sender, EventArgs e)
        {
            OnInputEnter(e);
        }
    }
}
