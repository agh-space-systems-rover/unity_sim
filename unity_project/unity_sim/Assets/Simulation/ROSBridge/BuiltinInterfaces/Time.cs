using Newtonsoft.Json;
using System;

namespace ROSBridge.BuiltinInterfaces
{
    public struct Time
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "builtin_interfaces/msg/Time";

        [JsonProperty("sec")]
        public int Sec { get; set; }

        [JsonProperty("nanosec")]
        public uint Nanosec { get; set; }

        public static Time Realtime(double offsetSeconds = 0.0)
        {
            // If this is the first time this method is called, initialize the
            // stopwatch.
            if (secondsSinceEpochAtInit == 0)
            {
                DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
                TimeSpan span = (DateTime.UtcNow - epochStart);
                secondsSinceEpochAtInit = (long)span.TotalSeconds;
                additionalNanosAtInit = (long)span.Milliseconds * 1000000L;
                gameTimeAtInit = UnityEngine.Time.unscaledTimeAsDouble;
            }

            // Integrate stopwatch time with the initial time.
            long gameNanosSinceInit = (long)((UnityEngine.Time.unscaledTimeAsDouble - gameTimeAtInit) * 1000000000.0);
            long nanoOffset = additionalNanosAtInit + gameNanosSinceInit;

            // Add the offset.
            nanoOffset += (long)(offsetSeconds * 1000000000.0);

            return new Time
            {
                Sec = (int)(secondsSinceEpochAtInit + nanoOffset / 1000000000L),
                Nanosec = (uint)(nanoOffset % 1000000000L)
            };
        }

        public static Time Simulated()
        {
            return new Time
            {
                Sec = (int)UnityEngine.Time.time,
                Nanosec = (uint)(UnityEngine.Time.time * 1000000000.0 % 1000000000.0)
            };
        }

        private static long secondsSinceEpochAtInit = 0;
        private static long additionalNanosAtInit = 0;
        private static double gameTimeAtInit = 0;
    }
}