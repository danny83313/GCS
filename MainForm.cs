using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.ComponentModel;
using System.Data;
using System.Drawing;
//using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using GCS.Utilities;
using log4net;
using GCS.Controls;
using GCS.Comms;
using System.Threading;

namespace GCS
{
    public partial class GCS : Form
    {
        MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
        ManualResetEvent SerialThreadrunner = new ManualResetEvent(false);
        public static GCS instance = null;
        bool armed = false;
        /// <summary>
        /// controls the main serial reader thread
        /// </summary>
        bool serialThread = false;
        
        /// <summary>
        /// mono detection
        /// </summary>
        public static bool MONO = false;
        
        // locking to prevent multiple reads on serial port
        object readlock = new object();
        
        // our target sysid
        byte sysid;
        
        // our target compid
        byte compid;
        
        /// <summary>
        /// other planes in the area from adsb
        /// </summary>
        public object adsblock = new object();
       
        /// <summary>
        /// track the last heartbeat sent
        /// </summary>
        private DateTime heatbeatSend = DateTime.Now;

        public ConcurrentDictionary<string, adsb.PointLatLngAltHdg> adsbPlanes = new ConcurrentDictionary<string, adsb.PointLatLngAltHdg>();
        private static readonly ILog log =
           LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
       
        /// <summary>
        /// store the time we first connect
        /// </summary>
        DateTime connecttime = DateTime.Now;
        Thread serialreaderthread;
        string titlebar;
        public GCS()
        {
            InitializeComponent();
            Comports.Add(comPort);
        }
        protected override void OnLoad(EventArgs e)
        {
            serialreaderthread = new Thread(SerialReader)
            {
                IsBackground = true,
                Name = "Main Serial reader",
                Priority = ThreadPriority.AboveNormal
            };
            serialreaderthread.Start();

            BinaryLog.onFlightMode += (firmware, modeno) =>
            {
                try
                {
                    var modes = Common.getModesList((GCS.Firmwares)Enum.Parse(typeof(GCS.Firmwares), firmware));
                    string currentmode = null;

                    foreach (var mode in modes)
                    {
                        if (mode.Key == modeno)
                        {
                            currentmode = mode.Value;
                            break;
                        }
                    }

                    return currentmode;
                }
                catch
                {
                    return null;
                }
            };

        }
        public enum Firmwares
        {
            ArduPlane,
            ArduCopter2,
            ArduRover,
            ArduSub,
            Ateryx,
            ArduTracker,
            Gymbal,
            PX4
        }
        /// <summary>
        /// passive comports
        /// </summary>
        public static List<MAVLinkInterface> Comports = new List<MAVLinkInterface>();
        static MAVLinkInterface _comPort = new MAVLinkInterface();
        public static MAVLinkInterface comPort
        {
            get
            {
                return _comPort;
            }
            set
            {
                if (_comPort == value)
                    return;
                _comPort = value;
                /*_comPort.MavChanged -= instance.comPort_MavChanged;
                _comPort.MavChanged += instance.comPort_MavChanged;
                instance.comPort_MavChanged(null, null);*/
            }
        }
        public void doConnect(MAVLinkInterface comPort, string portname, string baud)
        {
            bool skipconnectcheck = false;
            log.Info("We are connecting to " + portname + " " + baud);
            switch (portname)
            {
                case "preset":
                    skipconnectcheck = true;
                    if (comPort.BaseStream is TcpSerial)
                        //_connectionControl.CMB_serialport.Text = "TCP";
                    if (comPort.BaseStream is UdpSerial)
                        //_connectionControl.CMB_serialport.Text = "UDP";
                    //if (comPort.BaseStream is UdpSerialConnect)
                        //_connectionControl.CMB_serialport.Text = "UDPCl";
                    if (comPort.BaseStream is SerialPort)
                    {
                         CMB_comport.Text = comPort.BaseStream.PortName;
                         CMB_baudrate.Text = comPort.BaseStream.BaudRate.ToString();
                    }
                    break;
                case "TCP":
                    comPort.BaseStream = new TcpSerial();
                   // _connectionControl.CMB_serialport.Text = "TCP";
                    break;
                case "UDP":
                    comPort.BaseStream = new UdpSerial();
                    //_connectionControl.CMB_serialport.Text = "UDP";
                    break;
                case "UDPCl":
                    //comPort.BaseStream = new UdpSerialConnect();
                    //_connectionControl.CMB_serialport.Text = "UDPCl";
                    break;
                case "AUTO":
                    // do autoscan
                    //Comms.CommsSerialScan.Scan(true);
                    DateTime deadline = DateTime.Now.AddSeconds(50);
                   /* while (Comms.CommsSerialScan.foundport == false)
                    {
                        System.Threading.Thread.Sleep(100);

                        if (DateTime.Now > deadline || Comms.CommsSerialScan.run == 0)
                        {
                            CustomMessageBox.Show(Strings.Timeout);
                            _connectionControl.IsConnected(false);
                            return;
                        }
                    }*/
                    return;
                default:
                    comPort.BaseStream = new SerialPort();
                    break;
            }

            // Tell the connection UI that we are now connected.
            //_connectionControl.IsConnected(true);

            // Here we want to reset the connection stats counter etc.
            //this.ResetConnectionStats();

            comPort.MAV.cs.ResetInternals();

            //cleanup any log being played
            comPort.logreadmode = false;
            if (comPort.logplaybackfile != null)
                comPort.logplaybackfile.Close();
            comPort.logplaybackfile = null;

            try
            {
                log.Info("Set Portname");
                // set port, then options
                if (portname.ToLower() != "preset")
                    comPort.BaseStream.PortName = portname;

                log.Info("Set Baudrate");
                try
                {
                    if (baud != "" && baud != "0")
                        comPort.BaseStream.BaudRate = int.Parse(baud);
                }
                catch (Exception exp)
                {
                    log.Error(exp);
                }

                // prevent serialreader from doing anything
                comPort.giveComport = true;

                log.Info("About to do dtr if needed");
                // reset on connect logic.
                if (Settings.Instance.GetBoolean("CHK_resetapmonconnect") == true)
                {
                    log.Info("set dtr rts to false");
                    comPort.BaseStream.DtrEnable = false;
                    comPort.BaseStream.RtsEnable = false;

                    comPort.BaseStream.toggleDTR();
                }

                comPort.giveComport = false;

                // setup to record new logs
               /* try
                {
                    Directory.CreateDirectory(Settings.Instance.LogDir);
                    lock (this)
                    {
                        // create log names
                        var dt = DateTime.Now.ToString("yyyy-MM-dd HH-mm-ss");
                        var tlog = Settings.Instance.LogDir + Path.DirectorySeparatorChar +
                                   dt + ".tlog";
                        var rlog = Settings.Instance.LogDir + Path.DirectorySeparatorChar +
                                   dt + ".rlog";

                        // check if this logname already exists
                        int a = 1;
                        while (File.Exists(tlog))
                        {
                            Thread.Sleep(1000);
                            // create new names with a as an index
                            dt = DateTime.Now.ToString("yyyy-MM-dd HH-mm-ss") + "-" + a.ToString();
                            tlog = Settings.Instance.LogDir + Path.DirectorySeparatorChar +
                                   dt + ".tlog";
                            rlog = Settings.Instance.LogDir + Path.DirectorySeparatorChar +
                                   dt + ".rlog";
                        }

                        //open the logs for writing
                        comPort.logfile =
                            new BufferedStream(File.Open(tlog, FileMode.CreateNew, FileAccess.ReadWrite, FileShare.None));
                        comPort.rawlogfile =
                            new BufferedStream(File.Open(rlog, FileMode.CreateNew, FileAccess.ReadWrite, FileShare.None));
                        log.Info("creating logfile " + dt + ".tlog");
                    }
                }
                catch (Exception exp2)
                {
                    log.Error(exp2);
                    CustomMessageBox.Show(Strings.Failclog);
                } // soft fail*/

                // reset connect time - for timeout functions
                connecttime = DateTime.Now;

                // do the connect
                comPort.Open(false, skipconnectcheck);

                if (!comPort.BaseStream.IsOpen)
                {
                    log.Info("comport is closed. existing connect");
                    try
                    {
                        //_connectionControl.IsConnected(false);
                        //UpdateConnectIcon();
                        comPort.Close();
                    }
                    catch
                    {
                    }
                    return;
                }

                comPort.getParamList();

                //_connectionControl.UpdateSysIDS();

                // detect firmware we are conected to.
                /*if (comPort.MAV.cs.firmware == Firmwares.ArduCopter2)
                {
                    _connectionControl.TOOL_APMFirmware.SelectedIndex =
                        _connectionControl.TOOL_APMFirmware.Items.IndexOf(Firmwares.ArduCopter2);
                }
                else if (comPort.MAV.cs.firmware == Firmwares.Ateryx)
                {
                    _connectionControl.TOOL_APMFirmware.SelectedIndex =
                        _connectionControl.TOOL_APMFirmware.Items.IndexOf(Firmwares.Ateryx);
                }
                else if (comPort.MAV.cs.firmware == Firmwares.ArduRover)
                {
                    _connectionControl.TOOL_APMFirmware.SelectedIndex =
                        _connectionControl.TOOL_APMFirmware.Items.IndexOf(Firmwares.ArduRover);
                }
                else if (comPort.MAV.cs.firmware == Firmwares.ArduSub)
                {
                    _connectionControl.TOOL_APMFirmware.SelectedIndex =
                        _connectionControl.TOOL_APMFirmware.Items.IndexOf(Firmwares.ArduSub);
                }
                else if (comPort.MAV.cs.firmware == Firmwares.ArduPlane)
                {
                    _connectionControl.TOOL_APMFirmware.SelectedIndex =
                        _connectionControl.TOOL_APMFirmware.Items.IndexOf(Firmwares.ArduPlane);
                }*/

                // check for newer firmware
                /*var softwares = Firmware.LoadSoftwares();

                if (softwares.Count > 0)
                {
                    try
                    {
                        string[] fields1 = comPort.MAV.VersionString.Split(' ');

                        foreach (Firmware.software item in softwares)
                        {
                            string[] fields2 = item.name.Split(' ');

                            // check primare firmware type. ie arudplane, arducopter
                            if (fields1[0] == fields2[0])
                            {
                                Version ver1 = VersionDetection.GetVersion(comPort.MAV.VersionString);
                                Version ver2 = VersionDetection.GetVersion(item.name);

                                if (ver2 > ver1)
                                {
                                    Common.MessageShowAgain(Strings.NewFirmware + "-" + item.name,
                                        Strings.NewFirmwareA + item.name + Strings.Pleaseup);
                                    break;
                                }

                                // check the first hit only
                                break;
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        log.Error(ex);
                    }
                }*/

                //FlightData.CheckBatteryShow();

                Utilities.Tracking.AddEvent("Connect", "Connect", comPort.MAV.cs.firmware.ToString(),
                    comPort.MAV.param.Count.ToString());
                Utilities.Tracking.AddTiming("Connect", "Connect Time",
                    (DateTime.Now - connecttime).TotalMilliseconds, "");

                Utilities.Tracking.AddEvent("Connect", "Baud", comPort.BaseStream.BaudRate.ToString(), "");

                if (comPort.MAV.param.ContainsKey("SPRAY_ENABLE"))
                    Utilities.Tracking.AddEvent("Param", "Value", "SPRAY_ENABLE", comPort.MAV.param["SPRAY_ENABLE"].ToString());

                if (comPort.MAV.param.ContainsKey("CHUTE_ENABLE"))
                    Utilities.Tracking.AddEvent("Param", "Value", "CHUTE_ENABLE", comPort.MAV.param["CHUTE_ENABLE"].ToString());

                if (comPort.MAV.param.ContainsKey("TERRAIN_ENABLE"))
                    Utilities.Tracking.AddEvent("Param", "Value", "TERRAIN_ENABLE", comPort.MAV.param["TERRAIN_ENABLE"].ToString());

                if (comPort.MAV.param.ContainsKey("ADSB_ENABLE"))
                    Utilities.Tracking.AddEvent("Param", "Value", "ADSB_ENABLE", comPort.MAV.param["ADSB_ENABLE"].ToString());

                if (comPort.MAV.param.ContainsKey("AVD_ENABLE"))
                    Utilities.Tracking.AddEvent("Param", "Value", "AVD_ENABLE", comPort.MAV.param["AVD_ENABLE"].ToString());

                // save the baudrate for this port
                Settings.Instance[CMB_comport.Text + "_BAUD"] = CMB_baudrate.Text;

                this.Text = titlebar + " " + comPort.MAV.VersionString;

                // refresh config window if needed
                /*if (MyView.current != null)
                {
                    if (MyView.current.Name == "HWConfig")
                        MyView.ShowScreen("HWConfig");
                    if (MyView.current.Name == "SWConfig")
                        MyView.ShowScreen("SWConfig");
                }*/

                // load wps on connect option.
                /*if (Settings.Instance.GetBoolean("loadwpsonconnect") == true)
                {
                    // only do it if we are connected.
                    if (comPort.BaseStream.IsOpen)
                    {
                        MenuFlightPlanner_Click(null, null);
                        FlightPlanner.BUT_read_Click(null, null);
                    }
                }*/

                // get any rallypoints
                if (GCS.comPort.MAV.param.ContainsKey("RALLY_TOTAL") &&
                    int.Parse(GCS.comPort.MAV.param["RALLY_TOTAL"].ToString()) > 0)
                {
                    //FlightPlanner.getRallyPointsToolStripMenuItem_Click(null, null);

                    double maxdist = 0;

                    foreach (var rally in comPort.MAV.rallypoints)
                    {
                        foreach (var rally1 in comPort.MAV.rallypoints)
                        {
                            var pnt1 = new PointLatLngAlt(rally.Value.lat / 10000000.0f, rally.Value.lng / 10000000.0f);
                            var pnt2 = new PointLatLngAlt(rally1.Value.lat / 10000000.0f, rally1.Value.lng / 10000000.0f);

                            var dist = pnt1.GetDistance(pnt2);

                            maxdist = Math.Max(maxdist, dist);
                        }
                    }

                    if (comPort.MAV.param.ContainsKey("RALLY_LIMIT_KM") &&
                        (maxdist / 1000.0) > (float)comPort.MAV.param["RALLY_LIMIT_KM"])
                    {
                        CustomMessageBox.Show(Strings.Warningrallypointdistance + " " +
                                              (maxdist / 1000.0).ToString("0.00") + " > " +
                                              (float)comPort.MAV.param["RALLY_LIMIT_KM"]);
                    }
                }

                // get any fences
                if (GCS.comPort.MAV.param.ContainsKey("FENCE_TOTAL") &&
                    int.Parse(GCS.comPort.MAV.param["FENCE_TOTAL"].ToString()) > 1 &&
                    GCS.comPort.MAV.param.ContainsKey("FENCE_ACTION"))
                {
                    //FlightPlanner.GeoFencedownloadToolStripMenuItem_Click(null, null);
                }

                // set connected icon
                //this.MenuConnect.Image = displayicons.disconnect;
            }
            catch (Exception ex)
            {
                log.Warn(ex);
                try
                {
                    //_connectionControl.IsConnected(false);
                    //UpdateConnectIcon();
                    comPort.Close();
                }
                catch (Exception ex2)
                {
                    log.Warn(ex2);
                }
                CustomMessageBox.Show("Can not establish a connection\n\n" + ex.Message);
                return;
            }
        }
    
        private void button_connect_Click(object sender, EventArgs e)
        {
            // if the port is open close it
            //if (serialPort1.IsOpen)
            if(comPort.BaseStream.IsOpen)
            {
                //serialPort1.Close();
                comPort.Close();
                return;
            }

            // set the comport options
            //serialPort1.PortName = CMB_comport.Text;
            //serialPort1.BaudRate = int.Parse(CMB_baudrate.Text);
            comPort.BaseStream.PortName = CMB_comport.Text;
            comPort.BaseStream.BaudRate = int.Parse(CMB_baudrate.Text);

            doConnect(comPort, CMB_comport.Text, CMB_baudrate.Text);
            // open the comport
            //serialPort1.Open();
            //comPort.Open();
            // set timeout to 2 seconds
            //serialPort1.ReadTimeout = 2000;
            
            BackgroundWorker bgw = new BackgroundWorker();

            bgw.DoWork += bgw_DoWork;

            bgw.RunWorkerAsync();
        }
        void bgw_DoWork(object sender, DoWorkEventArgs e)
        {
            //while (serialPort1.IsOpen)
            while (comPort.BaseStream.IsOpen)
            {
                try
                {
                    MAVLink.MAVLinkMessage packet;
                    lock (readlock)
                    {
                        // read any valid packet from the port
                        //packet = mavlink.ReadPacket(serialPort1.BaseStream);
                        packet = mavlink.ReadPacket(comPort.BaseStream.BaseStream);
                        // check its valid
                        if (packet == null || packet.data == null)
                            continue;
                    }

                    // check to see if its a hb packet from the comport
                    if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                    {
                        var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                        // save the sysid and compid of the seen MAV
                        sysid = packet.sysid;
                        compid = packet.compid;

                        // request streams at 2 hz
                        mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                            new MAVLink.mavlink_request_data_stream_t()
                            {
                                req_message_rate = 2,
                                req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                start_stop = 1,
                                target_component = compid,
                                target_system = sysid
                            });
                    }

                    // from here we should check the the message is addressed to us
                    if (sysid != packet.sysid || compid != packet.compid)
                        continue;

                    if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                    //or
                    //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                    {
                        var att = (MAVLink.mavlink_attitude_t)packet.data;
                        Console.WriteLine(att.pitch * 57.2958 + " " + att.roll * 57.2958);
                        Console.WriteLine(comPort.MAV.cs.pitch * 57.2958 + " " + comPort.MAV.cs.roll * 57.2958);
                        Console.WriteLine(comPort.MAV.cs.mode);
                    }
                   
                }
                catch
                {
                }

                System.Threading.Thread.Sleep(1);
            }
        }

            T readsomedata<T>(byte sysid, byte compid, int timeout = 2000)
            {
                DateTime deadline = DateTime.Now.AddMilliseconds(timeout);

                lock (readlock)
                {
                    // read the current buffered bytes
                    while (DateTime.Now < deadline)
                    {
                        //var packet = mavlink.ReadPacket(serialPort1.BaseStream);
                        var packet = mavlink.ReadPacket(comPort.BaseStream.BaseStream);
                        // check its not null, and its addressed to us
                        if (packet == null || sysid != packet.sysid || compid != packet.compid)
                            continue;

                        Console.WriteLine(packet);

                        if (packet.data.GetType() == typeof(T))
                        {
                            return (T)packet.data;
                        }
                    }
                }

                throw new Exception("No packet match found");
            }

        

        private void button_armdisarm_Click(object sender, EventArgs e)
        {
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();

            req.target_system = 1;
            req.target_component = 1;

            req.command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM;

            req.param1 = armed ? 0 : 1;
            armed = !armed;
            /*
            req.param2 = p2;
            req.param3 = p3;
            req.param4 = p4;
            req.param5 = p5;
            req.param6 = p6;
            req.param7 = p7;
            */

            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

            //serialPort1.Write(packet, 0, packet.Length);
            comPort.BaseStream.Write(packet, 0, packet.Length);
            try
            {
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
                {

                }
            }
            catch
            {
            }
        }
        private void SerialReader()
        {
            if (serialThread == true)
                return;
            serialThread = true;

            SerialThreadrunner.Reset();

            int minbytes = 0;

            int altwarningmax = 0;

            bool armedstatus = false;

            string lastmessagehigh = "";

            DateTime speechcustomtime = DateTime.Now;

            DateTime speechlowspeedtime = DateTime.Now;

            DateTime linkqualitytime = DateTime.Now;

            while (serialThread)
            {
                try
                {
                    Thread.Sleep(1); // was 5

                    /*try
                    {
                        if (GCSViews.Terminal.comPort is MAVLinkSerialPort)
                        {
                        }
                        else
                        {
                            if (GCSViews.Terminal.comPort != null && GCSViews.Terminal.comPort.IsOpen)
                                continue;
                        }
                    }
                    catch (Exception ex)
                    {
                        log.Error(ex);
                    }*/

                    // update connect/disconnect button and info stats
                    /*try
                    {
                        UpdateConnectIcon();
                    }
                    catch (Exception ex)
                    {
                        log.Error(ex);
                    }*/

                    // 30 seconds interval speech options
                    /*if (speechEnable && speechEngine != null && (DateTime.Now - speechcustomtime).TotalSeconds > 30 &&
                        (GCS.comPort.logreadmode || comPort.BaseStream.IsOpen))
                    {
                        if (GCS.speechEngine.IsReady)
                        {
                            if (Settings.Instance.GetBoolean("speechcustomenabled"))
                            {
                                GCS.speechEngine.SpeakAsync(Common.speechConversion("" + Settings.Instance["speechcustom"]));
                            }

                            speechcustomtime = DateTime.Now;
                        }

                        // speech for battery alerts
                        //speechbatteryvolt
                        float warnvolt = Settings.Instance.GetFloat("speechbatteryvolt");
                        float warnpercent = Settings.Instance.GetFloat("speechbatterypercent");

                        if (Settings.Instance.GetBoolean("speechbatteryenabled") == true &&
                            GCS.comPort.MAV.cs.battery_voltage <= warnvolt &&
                            GCS.comPort.MAV.cs.battery_voltage >= 5.0)
                        {
                            if (GCS.speechEngine.IsReady)
                            {
                                GCS.speechEngine.SpeakAsync(Common.speechConversion("" + Settings.Instance["speechbattery"]));
                            }
                        }
                        else if (Settings.Instance.GetBoolean("speechbatteryenabled") == true &&
                                 (GCS.comPort.MAV.cs.battery_remaining) < warnpercent &&
                                 GCS.comPort.MAV.cs.battery_voltage >= 5.0 &&
                                 GCS.comPort.MAV.cs.battery_remaining != 0.0)
                        {
                            if (GCS.speechEngine.IsReady)
                            {
                                GCS.speechEngine.SpeakAsync(
                                    Common.speechConversion("" + Settings.Instance["speechbattery"]));
                            }
                        }
                    }*/

                    // speech for airspeed alerts
                    /*if (speechEnable && speechEngine != null && (DateTime.Now - speechlowspeedtime).TotalSeconds > 10 &&
                        (GCS.comPort.logreadmode || comPort.BaseStream.IsOpen))
                    {
                        if (Settings.Instance.GetBoolean("speechlowspeedenabled") == true && GCS.comPort.MAV.cs.armed)
                        {
                            float warngroundspeed = Settings.Instance.GetFloat("speechlowgroundspeedtrigger");
                            float warnairspeed = Settings.Instance.GetFloat("speechlowairspeedtrigger");

                            if (GCS.comPort.MAV.cs.airspeed < warnairspeed)
                            {
                                if (GCS.speechEngine.IsReady)
                                {
                                    GCS.speechEngine.SpeakAsync(
                                        Common.speechConversion("" + Settings.Instance["speechlowairspeed"]));
                                    speechlowspeedtime = DateTime.Now;
                                }
                            }
                            else if (GCS.comPort.MAV.cs.groundspeed < warngroundspeed)
                            {
                                if (GCS.speechEngine.IsReady)
                                {
                                    GCS.speechEngine.SpeakAsync(
                                        Common.speechConversion("" + Settings.Instance["speechlowgroundspeed"]));
                                    speechlowspeedtime = DateTime.Now;
                                }
                            }
                            else
                            {
                                speechlowspeedtime = DateTime.Now;
                            }
                        }
                    }*/

                    // speech altitude warning - message high warning
                    /*if (speechEnable && speechEngine != null &&
                        (GCS.comPort.logreadmode || comPort.BaseStream.IsOpen))
                    {
                        float warnalt = float.MaxValue;
                        if (Settings.Instance.ContainsKey("speechaltheight"))
                        {
                            warnalt = Settings.Instance.GetFloat("speechaltheight");
                        }
                        try
                        {
                            altwarningmax = (int)Math.Max(GCS.comPort.MAV.cs.alt, altwarningmax);

                            if (Settings.Instance.GetBoolean("speechaltenabled") == true && GCS.comPort.MAV.cs.alt != 0.00 &&
                                (GCS.comPort.MAV.cs.alt <= warnalt) && GCS.comPort.MAV.cs.armed)
                            {
                                if (altwarningmax > warnalt)
                                {
                                    if (GCS.speechEngine.IsReady)
                                        GCS.speechEngine.SpeakAsync(
                                            Common.speechConversion("" + Settings.Instance["speechalt"]));
                                }
                            }
                        }
                        catch
                        {
                        } // silent fail


                        try
                        {
                            // say the latest high priority message
                            if (GCS.speechEngine.IsReady &&
                                lastmessagehigh != GCS.comPort.MAV.cs.messageHigh && GCS.comPort.MAV.cs.messageHigh != null)
                            {
                                if (!GCS.comPort.MAV.cs.messageHigh.StartsWith("PX4v2 "))
                                {
                                    GCS.speechEngine.SpeakAsync(GCS.comPort.MAV.cs.messageHigh);
                                    lastmessagehigh = GCS.comPort.MAV.cs.messageHigh;
                                }
                            }
                        }
                        catch
                        {
                        }
                    }*/

                    // not doing anything
                    if (!GCS.comPort.logreadmode && !comPort.BaseStream.IsOpen)
                    {
                        altwarningmax = 0;
                    }

                    // attenuate the link qualty over time
                    if ((DateTime.Now - GCS.comPort.MAV.lastvalidpacket).TotalSeconds >= 1)
                    {
                        if (linkqualitytime.Second != DateTime.Now.Second)
                        {
                            GCS.comPort.MAV.cs.linkqualitygcs = (ushort)(GCS.comPort.MAV.cs.linkqualitygcs * 0.8f);
                            linkqualitytime = DateTime.Now;

                            // force redraw if there are no other packets are being read
                            //GCSViews.FlightData.myhud.Invalidate();
                        }
                    }

                    // data loss warning - wait min of 10 seconds, ignore first 30 seconds of connect, repeat at 5 seconds interval
                    /*if ((DateTime.Now - GCS.comPort.MAV.lastvalidpacket).TotalSeconds > 10
                        && (DateTime.Now - connecttime).TotalSeconds > 30
                        && (DateTime.Now - nodatawarning).TotalSeconds > 5
                        && (GCS.comPort.logreadmode || comPort.BaseStream.IsOpen)
                        && GCS.comPort.MAV.cs.armed)
                    {
                        if (speechEnable && speechEngine != null)
                        {
                            if (GCS.speechEngine.IsReady)
                            {
                                GCS.speechEngine.SpeakAsync("WARNING No Data for " +
                                                               (int)
                                                                   (DateTime.Now - GCS.comPort.MAV.lastvalidpacket)
                                                                       .TotalSeconds + " Seconds");
                                nodatawarning = DateTime.Now;
                            }
                        }
                    }*/

                    // get home point on armed status change.
                    if (armedstatus != GCS.comPort.MAV.cs.armed && comPort.BaseStream.IsOpen)
                    {
                        armedstatus = GCS.comPort.MAV.cs.armed;
                        // status just changed to armed
                        if (GCS.comPort.MAV.cs.armed == true && GCS.comPort.MAV.aptype != MAVLink.MAV_TYPE.GIMBAL)
                        {
                            System.Threading.ThreadPool.QueueUserWorkItem(state =>
                            {
                                try
                                {
                                    //GCS.comPort.getHomePosition();
                                    GCS.comPort.MAV.cs.HomeLocation = new PointLatLngAlt(GCS.comPort.getWP(0));
                                   /* if (MyView.current != null && MyView.current.Name == "FlightPlanner")
                                    {
                                        // update home if we are on flight data tab
                                        FlightPlanner.updateHome();
                                    }*/

                                }
                                catch
                                {
                                    // dont hang this loop
                                    this.BeginInvoke(
                                        (MethodInvoker)
                                            delegate
                                            {
                                                CustomMessageBox.Show("Failed to update home location (" +
                                                                      GCS.comPort.MAV.sysid + ")");
                                            });
                                }
                            });
                        }

                        /*if (speechEnable && speechEngine != null)
                        {
                            if (Settings.Instance.GetBoolean("speecharmenabled"))
                            {
                                string speech = armedstatus ? Settings.Instance["speecharm"] : Settings.Instance["speechdisarm"];
                                if (!string.IsNullOrEmpty(speech))
                                {
                                    GCS.speechEngine.SpeakAsync(Common.speechConversion(speech));
                                }
                            }
                        }*/
                    }

                    // send a hb every seconds from gcs to ap
                    if (heatbeatSend.Second != DateTime.Now.Second)
                    {
                        MAVLink.mavlink_heartbeat_t htb = new MAVLink.mavlink_heartbeat_t()
                        {
                            type = (byte)MAVLink.MAV_TYPE.GCS,
                            autopilot = (byte)MAVLink.MAV_AUTOPILOT.INVALID,
                            mavlink_version = 3 // MAVLink.MAVLINK_VERSION
                        };

                        // enumerate each link
                        foreach (var port in Comports)
                        {
                            if (!port.BaseStream.IsOpen)
                                continue;

                            // poll for params at heartbeat interval - primary mav on this port only
                            if (!port.giveComport)
                            {
                                try
                                {
                                    // poll only when not armed
                                    if (!port.MAV.cs.armed)
                                    {
                                        port.getParamPoll();
                                        port.getParamPoll();
                                    }
                                }
                                catch
                                {
                                }
                            }

                            // there are 3 hb types we can send, mavlink1, mavlink2 signed and unsigned
                            bool sentsigned = false;
                            bool sentmavlink1 = false;
                            bool sentmavlink2 = false;

                            // enumerate each mav
                            foreach (var MAV in port.MAVlist)
                            {
                                try
                                {
                                    // are we talking to a mavlink2 device
                                    if (MAV.mavlinkv2)
                                    {
                                        // is signing enabled
                                        if (MAV.signing)
                                        {
                                            // check if we have already sent
                                            if (sentsigned)
                                                continue;
                                            sentsigned = true;
                                        }
                                        else
                                        {
                                            // check if we have already sent
                                            if (sentmavlink2)
                                                continue;
                                            sentmavlink2 = true;
                                        }
                                    }
                                    else
                                    {
                                        // check if we have already sent
                                        if (sentmavlink1)
                                            continue;
                                        sentmavlink1 = true;
                                    }

                                    port.sendPacket(htb, MAV.sysid, MAV.compid);
                                }
                                catch (Exception ex)
                                {
                                    log.Error(ex);
                                    // close the bad port
                                    try
                                    {
                                        port.Close();
                                    }
                                    catch
                                    {
                                    }
                                    // refresh the screen if needed
                                    if (port == GCS.comPort)
                                    {
                                        // refresh config window if needed
                                        /*if (MyView.current != null)
                                        {
                                            this.BeginInvoke((MethodInvoker)delegate ()
                                            {
                                                if (MyView.current.Name == "HWConfig")
                                                    MyView.ShowScreen("HWConfig");
                                                if (MyView.current.Name == "SWConfig")
                                                    MyView.ShowScreen("SWConfig");
                                            });
                                        }*/
                                    }
                                }
                            }
                        }

                        heatbeatSend = DateTime.Now;
                    }

                    // if not connected or busy, sleep and loop
                    if (!comPort.BaseStream.IsOpen || comPort.giveComport == true)
                    {
                        if (!comPort.BaseStream.IsOpen)
                        {
                            // check if other ports are still open
                            foreach (var port in Comports)
                            {
                                if (port.BaseStream.IsOpen)
                                {
                                    Console.WriteLine("Main comport shut, swapping to other mav");
                                    comPort = port;
                                    break;
                                }
                            }
                        }

                        System.Threading.Thread.Sleep(100);
                    }

                    // read the interfaces
                    foreach (var port in Comports.ToArray())
                    {
                        if (!port.BaseStream.IsOpen)
                        {
                            // skip primary interface
                            if (port == comPort)
                                continue;

                            // modify array and drop out
                            Comports.Remove(port);
                            port.Dispose();
                            break;
                        }

                        while (port.BaseStream.IsOpen && port.BaseStream.BytesToRead > minbytes &&
                               port.giveComport == false && serialThread)
                        {
                            try
                            {
                                port.readPacket();
                            }
                            catch (Exception ex)
                            {
                                log.Error(ex);
                            }
                        }
                        // update currentstate of sysids on the port
                        foreach (var MAV in port.MAVlist)
                        {
                            try
                            {
                                MAV.cs.UpdateCurrentSettings(null, false, port, MAV);
                            }
                            catch (Exception ex)
                            {
                                log.Error(ex);
                            }
                        }
                    }
                }
                catch (Exception e)
                {
                    Tracking.AddException(e);
                    log.Error("Serial Reader fail :" + e.ToString());
                    try
                    {
                        comPort.Close();
                    }
                    catch (Exception ex)
                    {
                        log.Error(ex);
                    }
                }
            }

            Console.WriteLine("SerialReader Done");
            SerialThreadrunner.Set();
        }
        private void CMB_comport_Click(object sender, EventArgs e)
        {
            CMB_comport.DataSource = System.IO.Ports.SerialPort.GetPortNames();  //原先是用System.IO，加入MAVInterface後有多一個GCS.Comms.SerialPort，暫時選用System
        }

        private void Setmode_button_Click_1(object sender, EventArgs e)
        {
            comPort.setMode("GUIDED");
            Console.WriteLine("Set mode to GUIDED");
        }

        private void disconnect_Click(object sender, EventArgs e)
        {
            log.Info("We are disconnecting");
            try
            {
             

                comPort.BaseStream.DtrEnable = false;
                comPort.Close();
            }
            catch (Exception ex)
            {
                log.Error(ex);
            }
          
        }
    }
}
