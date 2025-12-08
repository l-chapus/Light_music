using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Drawing.Drawing2D;

namespace Solnir
{
    public class HomeForm : Form
    {
        private Panel panelContent;
        private Button btnAddElement;
        private PictureBox pictureBoxLogo;
        private Form1 embeddedMain;

        public HomeForm()
        {
            InitializeComponent();

            // Make the form background match the panel so no white bands show
            if (panelContent != null)
                this.BackColor = panelContent.BackColor;

            // Reduce flicker during resize
            this.SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer, true);
            this.UpdateStyles();

            EnableDoubleBuffer(panelContent);
        }

        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(HomeForm));
            this.panelContent = new System.Windows.Forms.Panel();
            this.btnAddElement = new System.Windows.Forms.Button();
            this.pictureBoxLogo = new System.Windows.Forms.PictureBox();
            this.panelContent.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxLogo)).BeginInit();
            this.SuspendLayout();
            // 
            // panelContent
            // 
            this.panelContent.Anchor = System.Windows.Forms.AnchorStyles.Top;
            this.panelContent.AutoSize = true;
            this.panelContent.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(17)))), ((int)(((byte)(17)))), ((int)(((byte)(17)))));
            this.panelContent.Controls.Add(this.btnAddElement);
            this.panelContent.Controls.Add(this.pictureBoxLogo);
            this.panelContent.Location = new System.Drawing.Point(-1, -2);
            this.panelContent.Name = "panelContent";
            this.panelContent.Size = new System.Drawing.Size(1347, 592);
            this.panelContent.TabIndex = 2;
            this.panelContent.Paint += new System.Windows.Forms.PaintEventHandler(this.panelContent_Paint);
            // 
            // btnAddElement
            // 
            this.btnAddElement.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.btnAddElement.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(42)))), ((int)(((byte)(42)))), ((int)(((byte)(42)))));
            this.btnAddElement.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btnAddElement.BackgroundImage")));
            this.btnAddElement.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.btnAddElement.Cursor = System.Windows.Forms.Cursors.Hand;
            this.btnAddElement.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(85)))), ((int)(((byte)(186)))), ((int)(((byte)(224)))));
            this.btnAddElement.FlatAppearance.BorderSize = 0;
            this.btnAddElement.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnAddElement.ForeColor = System.Drawing.SystemColors.Menu;
            this.btnAddElement.Location = new System.Drawing.Point(851, 308);
            this.btnAddElement.Name = "btnAddElement";
            this.btnAddElement.Size = new System.Drawing.Size(160, 160);
            this.btnAddElement.TabIndex = 1;
            this.btnAddElement.UseVisualStyleBackColor = false;
            this.btnAddElement.Click += new System.EventHandler(this.btnAddElement_Click);
            // 
            // pictureBoxLogo
            // 
            this.pictureBoxLogo.Image = global::Solnir.Properties.Resources.Logo_Foreground;
            this.pictureBoxLogo.Location = new System.Drawing.Point(395, 48);
            this.pictureBoxLogo.Name = "pictureBoxLogo";
            this.pictureBoxLogo.Size = new System.Drawing.Size(590, 173);
            this.pictureBoxLogo.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pictureBoxLogo.TabIndex = 2;
            this.pictureBoxLogo.TabStop = false;
            // 
            // HomeForm
            // 
            this.ClientSize = new System.Drawing.Size(1345, 588);
            this.Controls.Add(this.panelContent);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "HomeForm";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Solnir";
            this.Load += new System.EventHandler(this.HomeForm_Load);
            this.panelContent.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBoxLogo)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        private void HomeForm_Load(object sender, EventArgs e)
        {
            // éviter l'exécution en design-time
            if (LicenseManager.UsageMode == LicenseUsageMode.Designtime)
                return;

            // Apply rounded corners and ensure they stay after resize
            SetRoundedRegion(btnAddElement, 16);
            btnAddElement.SizeChanged += (s, ev) => { SetRoundedRegion(btnAddElement, 16); btnAddElement.Invalidate(); };
            btnAddElement.Parent.SizeChanged += (s, ev) => { SetRoundedRegion(btnAddElement, 16); btnAddElement.Invalidate(); };
            this.Resize += (s, ev) => { SetRoundedRegion(btnAddElement, 16); btnAddElement.Invalidate(); panelContent.Invalidate(); };

            // Draw a custom rounded border in Paint (default border removed)
            btnAddElement.Paint += BtnAddElement_Paint;
        }

        protected override void OnPaintBackground(PaintEventArgs e)
        {
            // Ensure the background is always painted with the panel color to avoid white bands
            if (panelContent != null)
            {
                using (var brush = new SolidBrush(panelContent.BackColor))
                {
                    e.Graphics.FillRectangle(brush, this.ClientRectangle);
                }
            }
            else
            {
                base.OnPaintBackground(e);
            }
        }

        private void BtnAddElement_Paint(object sender, PaintEventArgs e)
        {
            var btn = sender as Button;
            if (btn == null)
                return;

            var rect = new Rectangle(1, 1, btn.Width - 2, btn.Height - 2);
            int radius = 16;

            e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;

            using (var path = CreateRoundedPath(rect, radius))
            using (var pen = new Pen(Color.FromArgb(128, 203, 232), 2))
            {
                e.Graphics.DrawPath(pen, path);
            }
        }

        private GraphicsPath CreateRoundedPath(Rectangle rect, int radius)
        {
            var path = new GraphicsPath();
            int r = radius * 2;
            path.AddArc(rect.X, rect.Y, r, r, 180, 90);
            path.AddArc(rect.Right - r, rect.Y, r, r, 270, 90);
            path.AddArc(rect.Right - r, rect.Bottom - r, r, r, 0, 90);
            path.AddArc(rect.X, rect.Bottom - r, r, r, 90, 90);
            path.CloseFigure();
            return path;
        }



        private void btnOpenMain_Click(object sender, EventArgs e)
        {
            // Hide home button
            btnAddElement.Visible = false;

            // Create Form1 and embed it inside panelContent
            if (embeddedMain == null)
            {
                embeddedMain = new Form1();
                embeddedMain.TopLevel = false;
                embeddedMain.FormBorderStyle = FormBorderStyle.None;
                embeddedMain.Dock = DockStyle.Fill;
                // allow Form1 to call back to HomeForm
                embeddedMain.Tag = this;
                this.panelContent.Controls.Add(embeddedMain);
                embeddedMain.Show();
                embeddedMain.BringToFront();
            }
        }

        public void ReturnToHome()
        {
            // Remove the embedded main form and show the home button again
            if (embeddedMain != null)
            {
                try
                {
                    embeddedMain.Close();
                    embeddedMain.Dispose();
                }
                catch { }
                this.panelContent.Controls.Remove(embeddedMain);
                embeddedMain = null;
            }

            btnAddElement.Visible = true;
        }

        private void btnAddElement_Click(object sender, EventArgs e)
        {
            // Hide home button
            btnAddElement.Visible = false;

            // Create Form1 and embed it inside panelContent
            if (embeddedMain == null)
            {
                embeddedMain = new Form1();
                embeddedMain.TopLevel = false;
                embeddedMain.FormBorderStyle = FormBorderStyle.None;
                embeddedMain.Dock = DockStyle.Fill;
                // allow Form1 to call back to HomeForm
                embeddedMain.Tag = this;
                this.panelContent.Controls.Add(embeddedMain);
                embeddedMain.Show();
                embeddedMain.BringToFront();
            }

        }

        private void SetRoundedRegion(Control ctl, int radius)
        {
            if (ctl == null || ctl.Width <= 0 || ctl.Height <= 0)
                return;

            var rect = new Rectangle(0, 0, ctl.Width, ctl.Height);
            using (var path = new GraphicsPath())
            {
                int r = radius * 2;
                path.AddArc(rect.X, rect.Y, r, r, 180, 90);
                path.AddArc(rect.Right - r, rect.Y, r, r, 270, 90);
                path.AddArc(rect.Right - r, rect.Bottom - r, r, r, 0, 90);
                path.AddArc(rect.X, rect.Bottom - r, r, r, 90, 90);
                path.CloseFigure();
                ctl.Region = new Region(path);
            }
        }

        private void EnableDoubleBuffer(Control ctrl)
        {
            if (ctrl == null) return;
            try
            {
                var dgvType = ctrl.GetType();
                var prop = dgvType.GetProperty("DoubleBuffered", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public);
                if (prop != null)
                    prop.SetValue(ctrl, true, null);
            }
            catch
            {
                // ignore
            }
        }

        private void panelContent_Paint(object sender, PaintEventArgs e)
        {

        }
    }
}
