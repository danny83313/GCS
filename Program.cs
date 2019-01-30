using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Threading;

namespace GCS
{
    static class Program
    {
        internal static Thread Thread;
        /// <summary>
        /// 應用程式的主要進入點。
        /// </summary>
        [STAThread]
        static void Main()
        {
            Thread = Thread.CurrentThread;

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            /*執行程式前先定義Heartbeat封包*/
            MAVLink.MavlinkParse tmp = new MAVLink.MavlinkParse();
            MAVLink.mavlink_heartbeat_t hb = new MAVLink.mavlink_heartbeat_t()
            {
                autopilot = 1,
                base_mode = 2,
                custom_mode = 3,
                mavlink_version = 2,
                system_status = 6,
                type = 7
            };
            var t1 = tmp.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);
            var t2 = tmp.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);
            tmp.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);
            tmp.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);

            tmp.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb, true);
            tmp.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb, true);
            Thread.CurrentThread.Name = "Base Thread";

            Application.Run(new GCS());
            ////1
        }
    }
}
