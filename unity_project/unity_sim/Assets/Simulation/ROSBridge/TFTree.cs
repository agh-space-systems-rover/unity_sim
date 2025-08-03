using System.Collections.Generic;
using UnityEngine;

namespace ROSBridge
{
    public class TFTree
    {
        public class TransformMissingException : System.Exception
        {
            public TransformMissingException(string message) : base(message) { }
        }

        public TFTree(ROS ros, string topicPrefix = "", bool subscribeDynamicTf = false)
        {
            tfStaticTopic = topicPrefix + tfStaticTopic;
            tfTopic = topicPrefix + tfTopic;

            // Subscribe to tf_static.
            ros.CreateSubscription<ROSBridge.TF2Msgs.TFMessage>(tfStaticTopic, (msg) =>
            {
                // TODO: Sometimes this message is not received.
                // For each transform.
                foreach (ROSBridge.GeometryMsgs.TransformStamped transform in msg.Transforms)
                {
                    // Add the transform to the tree.
                    AddTransform(transform);
                }
            });

            // Subscribe to tf.
            if (subscribeDynamicTf)
            {
                ros.CreateSubscription<ROSBridge.TF2Msgs.TFMessage>(tfTopic, (msg) =>
                {
                    // For each transform.
                    foreach (ROSBridge.GeometryMsgs.TransformStamped transform in msg.Transforms)
                    {
                        // Add the transform to the tree.
                        AddTransform(transform);
                    }
                });
            }
        }

        public Matrix4x4 LatestTransform(string targetFrame, string sourceFrame)
        {
            // First try to find for parent = targetFrame and child = sourceFrame.
            try
            {
                return SearchDFS(targetFrame, sourceFrame);
            }
            catch (TransformMissingException)
            {
                // Then find for parent = sourceFrame and child = targetFrame.
                return SearchDFS(sourceFrame, targetFrame).inverse;
            }
        }

        private class TFNode
        {
            public string name;
            public Dictionary<string, TFNode> children = new Dictionary<string, TFNode>();
            public Matrix4x4 transform;

            public TFNode(string name)
            {
                this.name = name;
            }

            public void AddChild(TFNode child, Matrix4x4 transform)
            {
                // Add the child.
                // If the child already exists, then it will be overwritten.
                if (children.ContainsKey(child.name))
                {
                    children[child.name] = child;
                }
                else
                {
                    children.Add(child.name, child);
                }

                // Set the transform.
                child.transform = transform;
            }
        }

        private string tfStaticTopic = "/tf_static/republished_for_unity";
        private string tfTopic = "/tf";
        private Dictionary<string, TFNode> nodes = new Dictionary<string, TFNode>();

        private void AddTransform(ROSBridge.GeometryMsgs.TransformStamped msg)
        {
            // Add the transform to the tree.
            AddTransform(msg.Transform.Translation, msg.Transform.Rotation, msg.Header.FrameId, msg.ChildFrameId);
        }

        private void AddTransform(ROSBridge.GeometryMsgs.Vector3 translation, ROSBridge.GeometryMsgs.Quaternion rotation, string parent, string child)
        {
            // Get the parent node.
            TFNode parentNode = GetOrCreateNode(parent);

            // Get the child node.
            TFNode childNode = GetOrCreateNode(child);

            // Convert the translation and rotation to a matrix.
            Vector3 t = new Vector3((float)translation.X, (float)translation.Y, (float)translation.Z);
            Quaternion r = new Quaternion((float)rotation.X, (float)rotation.Y, (float)rotation.Z, (float)rotation.W);
            Vector3 s = new Vector3(1, 1, 1);
            Matrix4x4 transform = Matrix4x4.TRS(t, r, s);

            // Add the transform.
            parentNode.AddChild(childNode, transform);
        }

        private TFNode GetOrCreateNode(string name)
        {
            // If the node already exists.
            if (nodes.ContainsKey(name))
            {
                // Return the node.
                return nodes[name];
            }
            // Otherwise.
            else
            {
                // Create the node.
                TFNode node = new TFNode(name);

                // Add the node to the tree.
                nodes.Add(name, node);

                // Return the node.
                return node;
            }
        }

        private Matrix4x4 SearchDFS(string parent, string child)
        {
            // If the parent node exists.
            if (nodes.ContainsKey(parent))
            {
                // Get the parent node.
                TFNode parentNode = nodes[parent];

                // Look through the children.
                foreach (KeyValuePair<string, TFNode> childNode in parentNode.children)
                {
                    // If the child node is the one we're looking for.
                    if (childNode.Key == child)
                    {
                        // Return the transform.
                        return childNode.Value.transform;
                    }
                    // Otherwise...
                    else
                    {
                        // Recursively search for the child node.
                        try
                        {
                            Matrix4x4 transform = SearchDFS(childNode.Key, child);
                            return childNode.Value.transform * transform;
                        }
                        catch (TransformMissingException)
                        {
                            // Do nothing. Eventually the current call will throw its own exception.
                        }
                    }
                }
            }

            // Throw an exception.
            throw new TransformMissingException($"Transform from {parent} to {child} does not exist.");
        }
    }
}