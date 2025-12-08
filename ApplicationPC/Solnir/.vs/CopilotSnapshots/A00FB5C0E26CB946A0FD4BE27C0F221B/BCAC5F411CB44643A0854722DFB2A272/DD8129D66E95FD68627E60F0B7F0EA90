using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;

namespace Solnir
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            try
            {
                RefreshPorts();

                if (cbBaudRate.Items.Count > 0)
                    cbBaudRate.SelectedIndex = 0; // default to first (9600)

                UpdateControlsForClosedPort();
            }
            catch (Exception ex)
            {
                AppendLog($"Erreur au démarrage: {ex.Message}");
            }
        }

        private void RefreshPorts()
        {
            var ports = SerialPort.GetPortNames().OrderBy(p => p).ToArray();
            cbPorts.Items.Clear();
            cbPorts.Items.AddRange(ports);
            if (cbPorts.Items.Count > 0)
                cbPorts.SelectedIndex = 0;
        }

        private void btnRefresh_Click(object sender, EventArgs e)
        {
            RefreshPorts();
            AppendLog("Liste des ports rafraîchie.");
        }

        private void btnOpenClose_Click(object sender, EventArgs e)
        {
            try
            {
                if (serialPort1.IsOpen)
                {
                    serialPort1.Close();
                    AppendLog($"Port {serialPort1.PortName} fermé.");
                    UpdateControlsForClosedPort();
                }
                else
                {
                    if (cbPorts.SelectedItem == null)
                    {
                        AppendLog("Aucun port sélectionné.");
                        return;
                    }

                    serialPort1.PortName = cbPorts.SelectedItem.ToString();

                    int baud = 9600;
                    if (cbBaudRate.SelectedItem != null && int.TryParse(cbBaudRate.SelectedItem.ToString(), out int b))
                        baud = b;
                    serialPort1.BaudRate = baud;

                    serialPort1.NewLine = "\r\n";
                    serialPort1.Open();
                    AppendLog($"Port {serialPort1.PortName} ouvert à {serialPort1.BaudRate} bps.");
                    UpdateControlsForOpenPort();
                }
            }
            catch (Exception ex)
            {
                AppendLog($"Erreur port série: {ex.Message}");
            }
        }

        private void btnSend_Click(object sender, EventArgs e)
        {
            try
            {
                if (!serialPort1.IsOpen)
                {
                    AppendLog("Le port n'est pas ouvert.");
                    return;
                }

                var cmd = txtCommand.Text ?? string.Empty;
                if (string.IsNullOrWhiteSpace(cmd))
                {
                    AppendLog("Commande vide.");
                    return;
                }

                serialPort1.WriteLine(cmd);
                AppendLog($"TX: {cmd}");
            }
            catch (Exception ex)
            {
                AppendLog($"Erreur en envoi: {ex.Message}");
            }
        }

        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                var sp = (SerialPort)sender;
                string data = sp.ReadExisting();
                if (string.IsNullOrEmpty(data))
                    return;

                // Marshal to UI thread
                AppendLogThreadSafe($"RX: {data}");
            }
            catch (Exception ex)
            {
                AppendLogThreadSafe($"Erreur reception: {ex.Message}");
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            try
            {
                if (serialPort1 != null && serialPort1.IsOpen)
                    serialPort1.Close();
            }
            catch
            {
                // ignore on closing
            }
        }

        private void UpdateControlsForOpenPort()
        {
            btnOpenClose.Text = "Fermer";
            cbPorts.Enabled = false;
            cbBaudRate.Enabled = false;
            btnRefresh.Enabled = false;
        }

        private void UpdateControlsForClosedPort()
        {
            btnOpenClose.Text = "Ouvrir";
            cbPorts.Enabled = true;
            cbBaudRate.Enabled = true;
            btnRefresh.Enabled = true;
        }

        private void AppendLog(string text)
        {
            if (txtLog.InvokeRequired)
            {
                txtLog.BeginInvoke(new Action(() => AppendLog(text)));
                return;
            }

            var ts = DateTime.Now.ToString("HH:mm:ss");
            txtLog.AppendText($"[{ts}] {text}{Environment.NewLine}");
        }

        private void AppendLogThreadSafe(string text)
        {
            if (txtLog.InvokeRequired)
            {
                txtLog.BeginInvoke(new Action(() => AppendLog(text)));
                return;
            }

            AppendLog(text);
        }
    }
}
