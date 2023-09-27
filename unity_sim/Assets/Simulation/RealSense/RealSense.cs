using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;
using Unity.Collections;
using ROSBridge;
using OperationCanceledException = System.OperationCanceledException;
using System.Threading.Tasks;
using System;
using System.Net.WebSockets;
using System.Threading;
using System.Text;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;

public class RealSense : MonoBehaviour
{
    [SerializeField]
    private int width = 640;
    [SerializeField]
    private int height = 360;
    [SerializeField]
    private int fps = 30;
    [SerializeField]
    private float vFov = 60.0F;
    [SerializeField]
    private float depthMin = 0.5F;
    [SerializeField]
    private float depthMax = 8.0F;
    [SerializeField]
    private string id = "camera";

    private Camera cam;
    bool frameAvailable = false; // Feels a bit smoother than cam.Render()
    RenderTexture asyncRt = null;
    Material depthMaterial = null;

    [DllImport("UnityRSPublisherPlugin")]
    private static extern void OpenBridgeConnection(string id, int width, int height, float vFov, float depthMin, float depthMax);
    [DllImport("UnityRSPublisherPlugin")]
    private static extern void CloseBridgeConnection(string id);
    [DllImport("UnityRSPublisherPlugin")]
    private static unsafe extern void TryPushFrame(string id, void* ptr);

    private async void Start()
    {
        // Matrix4x4 rosToUnity = Matrix4x4.identity;
        // // 0, -1, 0 // Ros +Y is Unity -X (left) (second column)
        // // 0,  0, 1 // Ros +Z is Unity +Y (up) (third column)
        // // 1,  0, 0 // Ros +X is Unity +Z (forward) (first column)
        // rosToUnity.SetColumn(0, new Vector4(0, 0, 1, 0));
        // rosToUnity.SetColumn(1, new Vector4(-1, 0, 0, 0));
        // rosToUnity.SetColumn(2, new Vector4(0, 1, 0, 0));
        // Matrix4x4 unityToRos = rosToUnity.inverse;

        // camera settings
        cam = GetComponent<Camera>();
        cam.enabled = true;
        cam.depthTextureMode = DepthTextureMode.Depth;
        cam.targetTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32);
        cam.nearClipPlane = 0.01F;
        cam.farClipPlane = 3000;
        cam.fieldOfView = vFov;

        // --------
        // Matrices
        // --------

        Matrix4x4 rosOpticalToRosView = Matrix4x4.identity;
        rosOpticalToRosView.SetColumn(0, new Vector4(0, -1, 0, 0));
        rosOpticalToRosView.SetColumn(1, new Vector4(0, 0, -1, 0));
        rosOpticalToRosView.SetColumn(2, new Vector4(1, 0, 0, 0));

        Matrix4x4 rosViewToGlView = Matrix4x4.identity;
        //  0, -1, 0 // Ros +Y is GL -X (left) (second column)
        //  0,  0, 1 // Ros +Z is GL +Y (up) (third column)
        // -1,  0, 0 // Ros +X is GL -Z (forward) (first column)
        rosViewToGlView.SetColumn(0, new Vector4(0, 0, -1, 0));
        rosViewToGlView.SetColumn(1, new Vector4(-1, 0, 0, 0));
        rosViewToGlView.SetColumn(2, new Vector4(0, 1, 0, 0));

        Matrix4x4 glViewToGlNdc = cam.nonJitteredProjectionMatrix;
        Matrix4x4 depth_GlViewToGlNdc = Matrix4x4.Perspective(vFov, width / (float)height, depthMin, depthMax);
        Matrix4x4 glNDCToGlUv = Matrix4x4.identity; // the standard "* 0.5 + 0.5" transform
        glNDCToGlUv.SetRow(0, new Vector4(0.5F, 0, 0, 0.5F));
        glNDCToGlUv.SetRow(1, new Vector4(0, 0.5F, 0, 0.5F));
        glNDCToGlUv.SetRow(2, new Vector4(0, 0, 0.5F, 0.5F));

        Matrix4x4 glUvToPixel = Matrix4x4.identity; // scale by screen size, flip Y
        glUvToPixel.SetRow(0, new Vector4(width, 0, 0, 0));
        glUvToPixel.SetRow(1, new Vector4(0, -height, 0, height));

        Matrix4x4 rosOpticalToPixel = glUvToPixel * glNDCToGlUv * glViewToGlNdc * rosViewToGlView * rosOpticalToRosView;
        Matrix4x4 depth_RosOpticalToPixel = glUvToPixel * glNDCToGlUv * depth_GlViewToGlNdc * rosViewToGlView * rosOpticalToRosView;

        // depth shader
        Shader depthShader = Shader.Find("Custom/DepthTextureShader");
        depthMaterial = new Material(depthShader);
        // depthMaterial.SetFloat("_DepthMin", depthMin);
        // depthMaterial.SetFloat("_DepthMax", depthMax);
        depthMaterial.SetMatrix("_InvProj", glViewToGlNdc.inverse);
        depthMaterial.SetMatrix("_DepthProj", depth_GlViewToGlNdc);

        // async capture
        asyncRt = new RenderTexture(width, height, 0);
        var syncRt = new RenderTexture(width, height, 0);
        var buffer = new NativeArray<byte>(width * height * 4, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

        float lastCaptureTime = 0.0F;
        // // debug fps counter
        // float frameTime = 0.016F;
        // float lastFpsCaptureTime = 0.0F;
        // float lastFpsPrintTime = 0.0F;

        // ROS
        var ros = new ROS();
        Publisher<ROSBridge.SensorMsgs.CameraInfo> colorInfoPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.CameraInfo>("/" + id + "/color/camera_info");
        Publisher<ROSBridge.SensorMsgs.CameraInfo> depthInfoPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.CameraInfo>("/" + id + "/depth/camera_info");
        OpenBridgeConnection(id, width, height, vFov, depthMin, depthMax);

        // Use a custom update loop to guarantee resource destruction.
        try
        {
            var cancel = destroyCancellationToken;
            while (true)
            {
                // Wait for the end of the frame.
                await Awaitable.EndOfFrameAsync(cancel);
                if (cancel.IsCancellationRequested)
                {
                    break;
                }

                // If a frame is available
                if (frameAvailable)
                {
                    // Limit the framerate.
                    if (Time.time - lastCaptureTime < 1.0F / fps)
                    {
                        continue;
                    }
                    lastCaptureTime = Time.time;

                    // // Debug: mesaure framerate
                    // float delta = Time.time - lastFpsCaptureTime;
                    // frameTime = frameTime * 0.9F + delta * 0.1F;
                    // lastFpsCaptureTime = Time.time;
                    // if (Time.time - lastFpsPrintTime > 1.0F)
                    // {
                    //     Debug.Log("FPS: " + (1.0F / frameTime).ToString("F2"));
                    //     lastFpsPrintTime = Time.time;
                    // }

                    // Copy from asyncRt to syncRt.
                    Graphics.Blit(asyncRt, syncRt);

                    // Re-enable camera after the blit.
                    cam.enabled = true;
                    frameAvailable = false;

                    // Copy the frame to the CPU.
                    try
                    {
                        await AsyncGPUReadback.RequestIntoNativeArrayAsync(ref buffer, syncRt, 0, TextureFormat.RGBA32);
                    }
                    catch (MissingReferenceException)
                    {
                        // TODO: This is a hacky fix.
                        continue;
                    }

                    // Send frame to C++ for further processing.
                    unsafe
                    {
                        TryPushFrame(id, NativeArrayUnsafeUtility.GetUnsafePtr(buffer));
                    }

                    // Publish color info
                    var stamp = ROSBridge.BuiltinInterfaces.Time.Current();
                    await colorInfoPublisher.Publish(
                        new ROSBridge.SensorMsgs.CameraInfo
                        {
                            Header = new ROSBridge.StdMsgs.Header
                            {
                                Stamp = stamp,
                                FrameId = id + "_color_optical_frame"
                            },
                            Height = (uint)height,
                            Width = (uint)width,
                            DistortionModel = "plumb_bob",
                            D = new double[] { 0, 0, 0, 0, 0 },
                            K = new double[] {
                                rosOpticalToPixel[0, 0], rosOpticalToPixel[0, 1], rosOpticalToPixel[0, 2],
                                rosOpticalToPixel[1, 0], rosOpticalToPixel[1, 1], rosOpticalToPixel[1, 2],
                                rosOpticalToPixel[2, 0], rosOpticalToPixel[2, 1], rosOpticalToPixel[2, 2]
                            },
                            R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                            P = new double[] {
                                rosOpticalToPixel[0, 0], rosOpticalToPixel[0, 1], rosOpticalToPixel[0, 2], rosOpticalToPixel[0, 3],
                                rosOpticalToPixel[1, 0], rosOpticalToPixel[1, 1], rosOpticalToPixel[1, 2], rosOpticalToPixel[1, 3],
                                rosOpticalToPixel[2, 0], rosOpticalToPixel[2, 1], rosOpticalToPixel[2, 2], rosOpticalToPixel[2, 3]
                            },
                            BinningX = 0,
                            BinningY = 0,
                            ROI = new ROSBridge.SensorMsgs.RegionOfInterest
                            {
                                XOffset = 0,
                                YOffset = 0,
                                Height = 0,
                                Width = 0,
                                DoRectify = false
                            }
                        }
                    );

                    // Publish depth info
                    await depthInfoPublisher.Publish(
                        new ROSBridge.SensorMsgs.CameraInfo
                        {
                            Header = new ROSBridge.StdMsgs.Header
                            {
                                Stamp = stamp,
                                FrameId = id + "_depth_optical_frame"
                            },
                            Height = (uint)height,
                            Width = (uint)width,
                            DistortionModel = "plumb_bob",
                            D = new double[] { 0, 0, 0, 0, 0 },
                            K = new double[] {
                                depth_RosOpticalToPixel[0, 0], depth_RosOpticalToPixel[0, 1], depth_RosOpticalToPixel[0, 2],
                                depth_RosOpticalToPixel[1, 0], depth_RosOpticalToPixel[1, 1], depth_RosOpticalToPixel[1, 2],
                                depth_RosOpticalToPixel[2, 0], depth_RosOpticalToPixel[2, 1], depth_RosOpticalToPixel[2, 2]
                            },
                            R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                            P = new double[] {
                                depth_RosOpticalToPixel[0, 0], depth_RosOpticalToPixel[0, 1], depth_RosOpticalToPixel[0, 2], depth_RosOpticalToPixel[0, 3],
                                depth_RosOpticalToPixel[1, 0], depth_RosOpticalToPixel[1, 1], depth_RosOpticalToPixel[1, 2], depth_RosOpticalToPixel[1, 3],
                                depth_RosOpticalToPixel[2, 0], depth_RosOpticalToPixel[2, 1], depth_RosOpticalToPixel[2, 2], depth_RosOpticalToPixel[2, 3]
                            },
                            BinningX = 0,
                            BinningY = 0,
                            ROI = new ROSBridge.SensorMsgs.RegionOfInterest
                            {
                                XOffset = 0,
                                YOffset = 0,
                                Height = 0,
                                Width = 0,
                                DoRectify = false
                            }
                        }
                    );
                }
            }
        }
        catch (OperationCanceledException)
        {
        }
        finally
        {
            if (buffer.IsCreated) buffer.Dispose();
            await ros.Close();
            CloseBridgeConnection(id);
        }
    }

    private void OnEnable()
    {
        RenderPipelineManager.endCameraRendering += RenderPipelineManager_endCameraRendering;
    }

    private void OnDisable()
    {
        RenderPipelineManager.endCameraRendering -= RenderPipelineManager_endCameraRendering;
    }

    private void RenderPipelineManager_endCameraRendering(ScriptableRenderContext context, Camera camera)
    {
        if (camera == cam && asyncRt != null && depthMaterial != null)
        {
            cam.enabled = false;

            // Blit through the depth shader to the async gbuffer.
            depthMaterial.SetTexture("_ColorTex", cam.targetTexture);
            Graphics.Blit(cam.targetTexture, asyncRt, depthMaterial);


            frameAvailable = true;
        }
    }
}
