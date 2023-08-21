using System.Threading.Tasks;
using Newtonsoft.Json;
using System.Collections.Concurrent;
using System.Reflection;
using System.Text;
using System.Net.WebSockets;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Threading;


namespace ROSBridge
{

    public class Publisher<T>
    {
        private readonly ROS ros;
        private readonly string topic;
        // private readonly BufferPool bufferPool;

        public Publisher(ROS ros, string topic)
        {
            this.ros = ros;
            this.topic = topic;
            // bufferPool = new BufferPool();
        }

        public async Task Publish(T msg, bool largeData = false)
        {
            // ROSBridge fragmentation is currently too unstable to use it.
            // if (largeData)
            // {
            //     // Wait for the connection to be established.
            //     await ros.connectionEstablished.Task;

            //     // If connection is still open.
            //     if (ros.socket.State != WebSocketState.Open)
            //     {
            //         return;
            //     }

            //     byte[] tempBuf = bufferPool.Rent(10 * 1024 * 1024);
            //     byte[] tempBuf2 = bufferPool.Rent(10 * 1024 * 1024);
            //     try
            //     {
            //         // extract .Data from msg using reflection
            //         var dataProp = msg.GetType().GetProperty("Data", BindingFlags.Public | BindingFlags.Instance);
            //         // Debug.Log(msg.GetType().GetProperties()[0].Name);
            //         var largeDataBytes = (byte[])dataProp.GetValue(msg);

            //         // debug: truncate data to 10 bytes
            //         largeDataBytes = largeDataBytes[..10];

            //         // create a custom msg object without data for json serialization
            //         // for each field not named "data", copy the value from msg to msg2
            //         var msg2 = new Dictionary<string, object>();
            //         foreach (var field in msg.GetType().GetProperties())
            //         {
            //             if (field.Name != "Data")
            //             {
            //                 // Get proper name from JsonProperty attribute if it exists.
            //                 var jsonProp = field.GetCustomAttribute<JsonPropertyAttribute>();
            //                 var fieldName = jsonProp != null ? jsonProp.PropertyName : field.Name;

            //                 // Copy the value from msg to msg2.
            //                 msg2[fieldName] = field.GetValue(msg);
            //             }
            //         }
            //         // Serialize msg2 publish request
            //         var msgStr = JsonConvert.SerializeObject(new
            //         {
            //             op = "publish",
            //             topic = topic,
            //             msg = msg2
            //         });

            //         // Encode data as Base64 into tempBuf2
            //         int base64Size = CustomBase64Encode(largeDataBytes, 0, largeDataBytes.Length, tempBuf2, 0);

            //         // Define the maximum size for each fragment.
            //         int maxDataFragSize = 2;

            //         // Calculate the total number of fragments.
            //         int totalFragments = (int)Math.Ceiling((double)base64Size / maxDataFragSize) + 2;
            //         // + 2 for the first and last frags

            //         // Create a unique ID for this fragmented message.
            //         var id = Guid.NewGuid().ToString();

            //         // Send the first fragment message
            //         // msgStr is sent without the two last curly braces and with "data": " instead.
            //         await ros.socket.SendAsync(Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(new
            //         {
            //             op = "fragment",
            //             id = id, // Use the same ID for all fragments.
            //             num = 0,
            //             total = totalFragments,
            //             data = msgStr.Substring(0, msgStr.Length - 2) + ", \"data\": \""
            //         })), WebSocketMessageType.Text, true, CancellationToken.None);

            //         // debug
            //         Debug.Log("Sent init: " + JsonConvert.SerializeObject(new
            //         {
            //             op = "fragment",
            //             id = id, // Use the same ID for all fragments.
            //             num = 0,
            //             total = totalFragments,
            //             data = msgStr.Substring(0, msgStr.Length - 2) + ", \"data\": \""
            //         }));

            //         // Split largeData into fragments
            //         for (int dataFragNum = 0; dataFragNum < totalFragments - 2; dataFragNum++)
            //         {
            //             // Calculate the start and end indexes for this fragment.
            //             int startIndex = dataFragNum * maxDataFragSize;
            //             int endIndex = Math.Min(startIndex + maxDataFragSize, base64Size);
            //             int fragmentSize = endIndex - startIndex;

            //             // Create the first part of the fragment message.
            //             var fragmentMessage = new
            //             {
            //                 op = "fragment",
            //                 id = id, // Use the same ID for all fragments.
            //                 num = dataFragNum + 1,
            //                 total = totalFragments
            //             };

            //             // Add it to tempBuf without the last curly brace.
            //             var fragStr = JsonConvert.SerializeObject(fragmentMessage);
            //             Array.Copy(Encoding.UTF8.GetBytes(fragStr), tempBuf, fragStr.Length - 1);

            //             // Add ", data: \"" to tempBuf.
            //             Array.Copy(Encoding.UTF8.GetBytes(", data: \""), 0, tempBuf, fragStr.Length - 1, 9);

            //             // Add the fragment data to tempBuf.
            //             Array.Copy(tempBuf2, startIndex, tempBuf, fragStr.Length + 8, fragmentSize);

            //             // Add the last curly brace to tempBuf.
            //             Array.Copy(Encoding.UTF8.GetBytes("\"}"), 0, tempBuf, fragStr.Length + 8 + fragmentSize, 2);

            //             // debug: Debug.Log as string
            //             var fragStr2 = Encoding.UTF8.GetString(tempBuf, 0, fragStr.Length + 11 + fragmentSize + 2);
            //             Debug.Log("Sending: " + fragStr2);

            //             // Send the fragment.
            //             await ros.socket.SendAsync(tempBuf, WebSocketMessageType.Text, true, CancellationToken.None);
            //         }

            //         // Send last fragment
            //         await ros.socket.SendAsync(Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(new
            //         {
            //             op = "fragment",
            //             id = id, // Use the same ID for all fragments.
            //             num = totalFragments - 1,
            //             total = totalFragments,
            //             data = "\"}"
            //         })), WebSocketMessageType.Text, true, CancellationToken.None);

            //         Debug.Log("Sent all data fragments");
            //     }
            //     finally
            //     {
            //         bufferPool.Return(tempBuf);
            //         bufferPool.Return(tempBuf2);
            //     }
            // }
            // else
            // {
            await ros.Send(new
            {
                op = "publish",
                topic = topic,
                msg = msg
            });
            // }
        }

        // private static int CustomBase64Encode(byte[] input, int inputOffset, int inputLength, byte[] output, int outputOffset)
        // {
        //     const string base64Chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

        //     int i = inputOffset;
        //     int j = outputOffset;

        //     while (inputLength >= 3)
        //     {
        //         int a = input[i++];
        //         int b = input[i++];
        //         int c = input[i++];

        //         output[j++] = (byte)base64Chars[(a >> 2) & 0x3F];
        //         output[j++] = (byte)base64Chars[((a << 4) | (b >> 4)) & 0x3F];
        //         output[j++] = (byte)base64Chars[((b << 2) | (c >> 6)) & 0x3F];
        //         output[j++] = (byte)base64Chars[c & 0x3F];

        //         inputLength -= 3;
        //     }

        //     // Handle the remaining bytes (if any)
        //     if (inputLength > 0)
        //     {
        //         int a = input[i++];
        //         int b = (inputLength > 1) ? input[i++] : 0;
        //         int c = 0;

        //         output[j++] = (byte)base64Chars[(a >> 2) & 0x3F];
        //         output[j++] = (byte)base64Chars[((a << 4) | (b >> 4)) & 0x3F];
        //         output[j++] = (byte)((inputLength > 1) ? base64Chars[((b << 2) | (c >> 6)) & 0x3F] : '=');
        //         output[j++] = (byte)'=';
        //     }

        //     return j - outputOffset;
        // }

        // private class BufferPool
        // {
        //     private readonly ConcurrentQueue<byte[]> pool = new ConcurrentQueue<byte[]>();

        //     public byte[] Rent(int size)
        //     {
        //         if (pool.TryDequeue(out byte[] buffer))
        //         {
        //             if (buffer.Length >= size)
        //                 return buffer;

        //             // If the rented buffer is too small, discard it and create a new one.
        //             Return(buffer);
        //         }

        //         return new byte[size];
        //     }

        //     public void Return(byte[] buffer)
        //     {
        //         // Clear the buffer before returning it to the pool to ensure no data leaks.
        //         // Array.Clear(buffer, 0, buffer.Length);
        //         pool.Enqueue(buffer);
        //     }
        // }
    }
}