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

        [UnityEngine.RuntimeInitializeOnLoadMethod(
            UnityEngine.RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetSimulationClock()
        {
            // Enter Play Mode can preserve static fields when domain reload is
            // disabled. Re-anchor every run so old simulation time cannot make
            // new messages appear seconds in the future.
            unixNanosAtSimulationStart = 0;
        }

        public static Time Realtime(double offsetSeconds = 0.0)
        {
            return FromSimulationTime(
                UnityEngine.Time.unscaledTimeAsDouble + offsetSeconds
            );
        }

        // Converts a Unity simulation timestamp to Unix epoch time. Passing an
        // explicit physics time lets high-rate sensors retain evenly spaced
        // timestamps when several FixedUpdate calls run in one rendered frame.
        public static Time FromSimulationTime(double simulationTimeSeconds)
        {
            if (unixNanosAtSimulationStart == 0)
            {
                long unixTicks =
                    DateTime.UtcNow.Ticks - UnixEpochTicks;
                long unixNanos = unixTicks * 100L;
                unixNanosAtSimulationStart =
                    unixNanos
                    - (long)(
                        UnityEngine.Time.unscaledTimeAsDouble * 1000000000.0
                    );
            }

            long totalNanos =
                unixNanosAtSimulationStart
                + (long)(simulationTimeSeconds * 1000000000.0);

            return new Time
            {
                Sec = (int)(totalNanos / 1000000000L),
                Nanosec = (uint)(totalNanos % 1000000000L)
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

        private const long UnixEpochTicks = 621355968000000000L;
        private static long unixNanosAtSimulationStart = 0;
    }
}