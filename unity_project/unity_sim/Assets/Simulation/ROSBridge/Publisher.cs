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
            // internal readonly List<AdvertisementStatus> activePublishers = new List<AdvertisementStatus>();
            // Abort if publisher is not yet advertised
            var i = ros.activePublishers.FindIndex(x => x.name == topic);
            if (i == -1 || !ros.activePublishers[i].advertised)
            {
                return;
            }

            await ROS.Send(new
            {
                op = "publish",
                topic = topic,
                msg = msg
            });
        }
    }
}