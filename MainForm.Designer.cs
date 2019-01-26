namespace GCS
{
    partial class GCS
    {
        /// <summary>
        /// 設計工具所需的變數。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清除任何使用中的資源。
        /// </summary>
        /// <param name="disposing">如果應該處置受控資源則為 true，否則為 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form 設計工具產生的程式碼

        /// <summary>
        /// 此為設計工具支援所需的方法 - 請勿使用程式碼編輯器修改
        /// 這個方法的內容。
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.CMB_comport = new System.Windows.Forms.ComboBox();
            this.CMB_baudrate = new System.Windows.Forms.ComboBox();
            this.button_connect = new System.Windows.Forms.Button();
            this.button_armdisarm = new System.Windows.Forms.Button();
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.Setmode_button = new System.Windows.Forms.Button();
            this.disconnect = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // CMB_comport
            // 
            this.CMB_comport.FormattingEnabled = true;
            this.CMB_comport.Location = new System.Drawing.Point(16, 15);
            this.CMB_comport.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.CMB_comport.Name = "CMB_comport";
            this.CMB_comport.Size = new System.Drawing.Size(160, 23);
            this.CMB_comport.TabIndex = 0;
            this.CMB_comport.Click += new System.EventHandler(this.CMB_comport_Click);
            // 
            // CMB_baudrate
            // 
            this.CMB_baudrate.FormattingEnabled = true;
            this.CMB_baudrate.Items.AddRange(new object[] {
            "9600",
            "14400",
            "19200",
            "28800",
            "38400",
            "57600",
            "115200"});
            this.CMB_baudrate.Location = new System.Drawing.Point(205, 15);
            this.CMB_baudrate.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.CMB_baudrate.Name = "CMB_baudrate";
            this.CMB_baudrate.Size = new System.Drawing.Size(160, 23);
            this.CMB_baudrate.TabIndex = 1;
            // 
            // button_connect
            // 
            this.button_connect.Location = new System.Drawing.Point(417, 16);
            this.button_connect.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.button_connect.Name = "button_connect";
            this.button_connect.Size = new System.Drawing.Size(100, 29);
            this.button_connect.TabIndex = 2;
            this.button_connect.Text = "Connect";
            this.button_connect.UseVisualStyleBackColor = true;
            this.button_connect.Click += new System.EventHandler(this.button_connect_Click);
            // 
            // button_armdisarm
            // 
            this.button_armdisarm.Location = new System.Drawing.Point(588, 16);
            this.button_armdisarm.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.button_armdisarm.Name = "button_armdisarm";
            this.button_armdisarm.Size = new System.Drawing.Size(100, 29);
            this.button_armdisarm.TabIndex = 3;
            this.button_armdisarm.Text = "Arm/Disarm";
            this.button_armdisarm.UseVisualStyleBackColor = true;
            this.button_armdisarm.Click += new System.EventHandler(this.button_armdisarm_Click);
            // 
            // Setmode_button
            // 
            this.Setmode_button.Location = new System.Drawing.Point(587, 306);
            this.Setmode_button.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.Setmode_button.Name = "Setmode_button";
            this.Setmode_button.Size = new System.Drawing.Size(100, 29);
            this.Setmode_button.TabIndex = 4;
            this.Setmode_button.Text = "Setmode";
            this.Setmode_button.UseVisualStyleBackColor = true;
            this.Setmode_button.Click += new System.EventHandler(this.Setmode_button_Click_1);
            // 
            // disconnect
            // 
            this.disconnect.Location = new System.Drawing.Point(417, 71);
            this.disconnect.Margin = new System.Windows.Forms.Padding(4);
            this.disconnect.Name = "disconnect";
            this.disconnect.Size = new System.Drawing.Size(100, 29);
            this.disconnect.TabIndex = 5;
            this.disconnect.Text = "disConnect";
            this.disconnect.UseVisualStyleBackColor = true;
            this.disconnect.Click += new System.EventHandler(this.disconnect_Click);
            // 
            // GCS
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 15F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(916, 430);
            this.Controls.Add(this.disconnect);
            this.Controls.Add(this.Setmode_button);
            this.Controls.Add(this.button_armdisarm);
            this.Controls.Add(this.button_connect);
            this.Controls.Add(this.CMB_baudrate);
            this.Controls.Add(this.CMB_comport);
            this.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.Name = "GCS";
            this.Text = "GCS";
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ComboBox CMB_comport;
        private System.Windows.Forms.ComboBox CMB_baudrate;
        private System.Windows.Forms.Button button_connect;
        private System.Windows.Forms.Button button_armdisarm;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.Button Setmode_button;
        private System.Windows.Forms.Button disconnect;
    }
}

