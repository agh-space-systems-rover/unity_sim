using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Sensor.LiDAR;
using UnitySensors.Utils.Noise;

using Random = Unity.Mathematics.Random;

namespace Kalman.UnitySimulation.Livox
{
    /// <summary>
    /// Runs the full MID-360 raycast pattern without blocking Unity's main
    /// thread on JobHandle.Complete(). If rendering falls behind while a scan
    /// is in flight, the missed scan is dropped instead of queued.
    /// </summary>
    public class AsyncRaycastLivoxSensor : LiDARSensor
    {
        private Transform sensorTransform;
        private JobHandle scanHandle;
        private IUpdateRaycastCommandsJob updateCommandsJob;
        private IUpdateGaussianNoisesJob updateNoisesJob;
        private IRaycastHitsToPointsJob hitsToPointsJob;

        private NativeArray<float3> directions;
        private NativeArray<RaycastCommand> raycastCommands;
        private NativeArray<RaycastHit> raycastHits;
        private NativeArray<float> noises;
        private NativeArray<PointXYZI> completedPoints;

        private bool scanInFlight;
        private double pendingSampleTime;
        private double nextScanTime;

        public double sampleTime { get; private set; }

        protected override void Init()
        {
            base.Init();

            sensorTransform = transform;
            directions = new NativeArray<float3>(
                scanPattern.size * 2,
                Allocator.Persistent
            );
            for (int i = 0; i < scanPattern.size; i++)
            {
                directions[i] = directions[i + scanPattern.size] =
                    scanPattern.scans[i];
            }

            raycastCommands = new NativeArray<RaycastCommand>(
                pointsNum,
                Allocator.Persistent
            );
            raycastHits = new NativeArray<RaycastHit>(
                pointsNum,
                Allocator.Persistent
            );
            noises = new NativeArray<float>(pointsNum, Allocator.Persistent);
            completedPoints = new NativeArray<PointXYZI>(
                pointsNum,
                Allocator.Persistent
            );

            updateCommandsJob = new IUpdateRaycastCommandsJob
            {
                origin = sensorTransform.position,
                localToWorldMatrix = sensorTransform.localToWorldMatrix,
                maxRange = maxRange,
                directions = directions,
                indexOffset = 0,
                raycastCommands = raycastCommands
            };
            updateNoisesJob = new IUpdateGaussianNoisesJob
            {
                sigma = gaussianNoiseSigma,
                random = new Random((uint)Environment.TickCount),
                noises = noises
            };
            hitsToPointsJob = new IRaycastHitsToPointsJob
            {
                minRange = minRange,
                sqrMinRange = minRange * minRange,
                maxRange = maxRange,
                maxIntensity = maxIntensity,
                directions = directions,
                indexOffset = 0,
                raycastHits = raycastHits,
                noises = noises,
                points = completedPoints
            };
            nextScanTime = Time.fixedUnscaledTimeAsDouble + dt;
        }

        protected override void Update()
        {
            CompleteScanIfReady();

            double now = Time.fixedUnscaledTimeAsDouble;
            if (now + 1e-9 < nextScanTime)
            {
                return;
            }

            // Schedule relative to the current physics time, not the previous
            // deadline. This deliberately drops scans missed during a hitch
            // instead of emitting a high-rate catch-up burst afterwards.
            nextScanTime = now + dt;
            UpdateSensor();
        }

        protected override void UpdateSensor()
        {
            // UnitySensor may call this repeatedly after a long rendered-frame
            // hitch. Do not build a queue that would prolong the hitch.
            if (scanInFlight)
            {
                return;
            }

            updateCommandsJob.origin = sensorTransform.position;
            updateCommandsJob.localToWorldMatrix =
                sensorTransform.localToWorldMatrix;
            pendingSampleTime = Time.fixedUnscaledTimeAsDouble;

            JobHandle commandsHandle = updateCommandsJob.Schedule(pointsNum, 64);
            JobHandle noisesHandle = updateNoisesJob.Schedule(
                pointsNum,
                64,
                commandsHandle
            );
            JobHandle raycastsHandle = RaycastCommand.ScheduleBatch(
                raycastCommands,
                raycastHits,
                256,
                noisesHandle
            );
            scanHandle = hitsToPointsJob.Schedule(
                pointsNum,
                64,
                raycastsHandle
            );
            scanInFlight = true;
            JobHandle.ScheduleBatchedJobs();
        }

        private void CompleteScanIfReady()
        {
            if (!scanInFlight || !scanHandle.IsCompleted)
            {
                return;
            }

            scanHandle.Complete();
            NativeArray<PointXYZI>.Copy(completedPoints, pointCloud.points);
            sampleTime = pendingSampleTime;
            scanInFlight = false;

            updateCommandsJob.indexOffset =
                (updateCommandsJob.indexOffset + pointsNum) % scanPattern.size;
            hitsToPointsJob.indexOffset =
                (hitsToPointsJob.indexOffset + pointsNum) % scanPattern.size;

            onSensorUpdated?.Invoke();
        }

        protected override void OnSensorDestroy()
        {
            if (scanInFlight)
            {
                scanHandle.Complete();
            }

            completedPoints.Dispose();
            noises.Dispose();
            raycastHits.Dispose();
            raycastCommands.Dispose();
            directions.Dispose();
            base.OnSensorDestroy();
        }
    }
}
