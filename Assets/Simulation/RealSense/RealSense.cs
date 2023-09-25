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

    [DllImport("RealSenseNativePlugin")]
    private static extern void OpenBridgeConnection(string id, int width, int height, float vFov, float depthMin, float depthMax);
    [DllImport("RealSenseNativePlugin")]
    private static extern void CloseBridgeConnection(string id);
    [DllImport("RealSenseNativePlugin")]
    private static unsafe extern void TryPushFrame(string id, void* ptr);

    private async void Start()
    {
        // camera settings
        cam = GetComponent<Camera>();
        cam.enabled = true;
        cam.depthTextureMode = DepthTextureMode.Depth;
        cam.targetTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32);
        cam.nearClipPlane = 0.01F;
        cam.farClipPlane = 3000;
        cam.fieldOfView = vFov;

        // depth shader
        Shader depthShader = Shader.Find("Custom/DepthTextureShader");
        depthMaterial = new Material(depthShader);
        depthMaterial.SetFloat("_DepthMin", depthMin);
        depthMaterial.SetFloat("_DepthMax", depthMax);

        // async capture
        asyncRt = new RenderTexture(width, height, 0);
        var syncRt = new RenderTexture(width, height, 0);
        var buffer = new NativeArray<byte>(width * height * 4, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

        float lastCaptureTime = 0.0F;
        // debug fps counter
        float frameTime = 0.016F;
        float lastFpsCaptureTime = 0.0F;
        float lastFpsPrintTime = 0.0F;

        // ROS
        var ros = new ROS();
        // Publisher<ROSBridge.SensorMsgs.CameraInfo> colorInfoPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.CameraInfo>("/" + id + "/color/camera_info");
        // Publisher<ROSBridge.SensorMsgs.CameraInfo> depthInfoPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.CameraInfo>("/" + id + "/depth/camera_info");
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

                    // Debug: mesaure framerate
                    float delta = Time.time - lastFpsCaptureTime;
                    frameTime = frameTime * 0.9F + delta * 0.1F;
                    lastFpsCaptureTime = Time.time;
                    if (Time.time - lastFpsPrintTime > 1.0F)
                    {
                        Debug.Log("FPS: " + (1.0F / frameTime).ToString("F2"));
                        lastFpsPrintTime = Time.time;
                    }

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

                    // NOTE: camera_info messages are copied from gazebo and most probably are invalid.

                    // Publish color info
                    // TODO: placeholder values
                    // var stamp = new ROSBridge.BuiltinInterfaces.Time().Current();
                    // await colorInfoPublisher.Publish(
                    //     new ROSBridge.SensorMsgs.CameraInfo
                    //     {
                    //         Header = new ROSBridge.StdMsgs.Header
                    //         {
                    //             Stamp = stamp,
                    //             FrameId = id + "_color_optical_frame"
                    //         },
                    //         Height = (uint)height,
                    //         Width = (uint)width,
                    //         DistortionModel = "plumb_bob",
                    //         D = new double[] { 0, 0, 0, 0, 0 },
                    //         K = new double[] { 283.119, 0, 320.5, 0, 283.119, 240.5, 0, 0, 1 },
                    //         R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                    //         P = new double[] { 283.119, 0, 320.5, -19.818, 0, 283.119, 240.5, 0, 0, 0, 1, 0 },
                    //         BinningX = 0,
                    //         BinningY = 0,
                    //         ROI = new ROSBridge.SensorMsgs.RegionOfInterest
                    //         {
                    //             XOffset = 0,
                    //             YOffset = 0,
                    //             Height = 0,
                    //             Width = 0,
                    //             DoRectify = false
                    //         }
                    //     }
                    // );

                    // Publish depth info
                    // TODO: placeholder values
                    // await colorInfoPublisher.Publish(
                    //     new ROSBridge.SensorMsgs.CameraInfo
                    //     {
                    //         Header = new ROSBridge.StdMsgs.Header
                    //         {
                    //             Stamp = stamp,
                    //             FrameId = id + "_depth_optical_frame"
                    //         },
                    //         Height = (uint)height,
                    //         Width = (uint)width,
                    //         DistortionModel = "plumb_bob",
                    //         D = new double[] { 0, 0, 0, 0, 0 },
                    //         K = new double[] { 283.119, 0, 320.5, 0, 283.119, 240.5, 0, 0, 1 },
                    //         R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                    //         P = new double[] { 283.119, 0, 320.5, 0, 0, 283.119, 240.5, 0, 0, 0, 1, 0 },
                    //         BinningX = 0,
                    //         BinningY = 0,
                    //         ROI = new ROSBridge.SensorMsgs.RegionOfInterest
                    //         {
                    //             XOffset = 0,
                    //             YOffset = 0,
                    //             Height = 0,
                    //             Width = 0,
                    //             DoRectify = false
                    //         }
                    //     }
                    // );
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
