using Newtonsoft.Json;

namespace ROSBridge.SensorMsgs
{

    public struct NavSatStatus
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "sensor_msgs/msg/NavSatStatus";

        [JsonIgnore]
        public static readonly sbyte STATUS_NO_FIX = -1;
        [JsonIgnore]
        public static readonly sbyte STATUS_FIX = 0;
        [JsonIgnore]
        public static readonly sbyte STATUS_SBAS_FIX = 1;
        [JsonIgnore]
        public static readonly sbyte STATUS_GBAS_FIX = 2;

        [JsonIgnore]
        public static readonly ushort SERVICE_GPS = 1;
        [JsonIgnore]
        public static readonly ushort SERVICE_GLONASS = 2;
        [JsonIgnore]
        public static readonly ushort SERVICE_COMPASS = 4;
        [JsonIgnore]
        public static readonly ushort SERVICE_GALILEO = 8;

        [JsonProperty("status")]
        public sbyte Status { get; set; }

        [JsonProperty("service")]
        public ushort Service { get; set; }
    }

}
