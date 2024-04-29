using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using UnityEngine;
using ROSBridge;

public class GPS : MonoBehaviour
{
    public struct Coords
    {
        public double latitude;
        public double longitude;
        public float altitude;
    }

    [SerializeField]
    private string frameId = "gps_link";
    
    [SerializeField]
    private string topic = "/gps/fix";

    [SerializeField]
    private float frequency = 5;

    [SerializeField]
    private bool enableAltitude = false;

    [SerializeField]
    private float noiseStdDev = 1.0F;
    [SerializeField]
    private float driftUpdateRate = 0.2F;

    private ROS ros;
    private Publisher<ROSBridge.SensorMsgs.NavSatFix> publisher = null;
    private double lastPublishTime = 0.0;
    private GPSBaseStation[] baseStations;
    private Planet earth;
    private Vector<double> centerLatLon;
    internal Matrix<double> procrustesTransform;
    private bool initialized = false;
    internal Coords lastCoords;
    internal bool hasFix = false;
    internal bool lastCoordsWerePublished = false;
    private Vector3 targetDrift = Vector3.zero;
    private Vector3 currentDrift = Vector3.zero;
    private double lastDriftUpdateTime = 0.0;

    private async void Start()
    {
        ros = new ROS();
        publisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.NavSatFix>(topic);

        ScanBaseStations();
    }

    private async void OnApplicationQuit()
    {
        if (ros != null)
        {
            await ros.Close();
        }
    }

    internal void ScanBaseStations()
    {
        baseStations = FindObjectsByType<GPSBaseStation>(FindObjectsSortMode.None);
        if (baseStations.Length < 2)
        {
            Debug.LogWarning("There are not enough base stations to compute GPS coordinates. Please spawn at least 2 base stations.");
            return;
        } else {
            Debug.Log("Found " + baseStations.Length + " base stations: " + string.Join(", ", Array.ConvertAll(baseStations, x => x.name)));
        }

        earth = new Planet(6371000);

        // Find the center of all base stations for the sphere<->plane projection.
        Vector3D centerNormalSum = new Vector3D(0, 0, 0);
        for (int i = 0; i < baseStations.Length; i++)
        {
            double lambda = baseStations[i].longitude * Math.PI / 180.0;
            double phi = baseStations[i].latitude * Math.PI / 180.0;
            centerNormalSum += new Vector3D(Math.Cos(phi) * Math.Cos(lambda), Math.Cos(phi) * Math.Sin(lambda), Math.Sin(phi));
        }
        UnitVector3D centerNormal = centerNormalSum.Normalize();
        centerLatLon = Vector<double>.Build.DenseOfArray(new[] {
            Math.Asin(centerNormal.Z) * 180.0 / Math.PI,
            Math.Atan2(centerNormal.Y, centerNormal.X) * 180.0 / Math.PI
        });

        // Pre-compute the procrustes transformation.
        var xyPositions = new List<Vector<double>>();
        var projPositions = new List<Vector<double>>();
        foreach (var station in baseStations)
        {
            xyPositions.Add(Vector<double>.Build.DenseOfArray(new double[] { station.transform.position.x, station.transform.position.z }));
            projPositions.Add(earth.ProjOntoPlane(centerLatLon, Vector<double>.Build.DenseOfArray(new double[] { station.latitude, station.longitude })));
        }
        procrustesTransform = Procrustes.MeanProcrustesTransform(xyPositions, projPositions);
        
        initialized = true;
    }

    internal void FixedUpdate()
    {
        if (!initialized)
        {
            return;
        }

        // Update drift target every once in a while.
        double now = Time.unscaledTimeAsDouble;
        if (now - lastDriftUpdateTime > 1.0 / driftUpdateRate)
        {
            var noiseVec = Vector<double>.Build.Random(3, new MathNet.Numerics.Distributions.Normal(0, noiseStdDev));
            targetDrift = new Vector3((float)noiseVec[0], (float)noiseVec[1], (float)noiseVec[2]);
            lastDriftUpdateTime = now;
        }
        // Drift towards the target.
        currentDrift = Vector3.Lerp(currentDrift, targetDrift, Time.fixedDeltaTime * 0.1F);

        // Get the robot's 2D position. Append a 1 to multiply with a 2D transformation matrix.
        var robotPos = Vector<double>.Build.DenseOfArray(new double[] { transform.position.x, transform.position.z, 1 });

        // Add noise to the robot's position.
        robotPos += Vector<double>.Build.DenseOfArray(new double[] { currentDrift.x, currentDrift.z, 0 });

        // Compute the latitude and longitude.
        var robotPosProj = procrustesTransform.Multiply(robotPos);
        var robotLatLon = earth.LatLonFromProj(centerLatLon, robotPosProj);
        double latitude = robotLatLon[0];
        double longitude = robotLatLon[1];

        // Compute the average altitude at Y=0.
        float altitudeAtZero = 0;
        int altitudeSamples = 0;
        for (int i = 0; i < baseStations.Length; i++)
        {
            if (baseStations[i].altitude == 0)
            {
                continue;
            }
            altitudeAtZero += baseStations[i].altitude - baseStations[i].transform.position.y;
            altitudeSamples++;
        }
        altitudeAtZero /= altitudeSamples;

        // Compute the altitude at the robot's Y.
        float altitude = altitudeAtZero + transform.position.y;
        altitude += currentDrift.z;

        lastCoords = new Coords
        {
            latitude = latitude,
            longitude = longitude,
            altitude = altitude
        };
        hasFix = true;
        lastCoordsWerePublished = false;
    }

    private async void Update()
    {
        // publish to ROS
        if (publisher == null)
        {
            return;
        }

        double currentTime = Time.unscaledTimeAsDouble;
        if (currentTime - lastPublishTime < 1.0 / frequency || lastCoordsWerePublished)
        {
            return;
        }
        lastPublishTime = currentTime;

        double variance = Math.Pow(noiseStdDev, 2);
        await publisher.Publish(new ROSBridge.SensorMsgs.NavSatFix
        {
            Header = new ROSBridge.StdMsgs.Header
            {
                Stamp = ROSBridge.BuiltinInterfaces.Time.Realtime(),
                FrameId = frameId
            },
            Status = new ROSBridge.SensorMsgs.NavSatStatus
            {
                Status = hasFix ? ROSBridge.SensorMsgs.NavSatStatus.STATUS_FIX : ROSBridge.SensorMsgs.NavSatStatus.STATUS_NO_FIX,
                Service = ROSBridge.SensorMsgs.NavSatStatus.SERVICE_GPS
            },
            Latitude = hasFix ? lastCoords.latitude : 0,
            Longitude = hasFix ? lastCoords.longitude : 0,
            Altitude = hasFix && enableAltitude ? lastCoords.altitude : 0,
            PositionCovariance = new double[9] { variance, 0, 0, 0, variance, 0, 0, 0, variance },
            PositionCovarianceType = ROSBridge.SensorMsgs.NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            // NOTE: COVARIANCE_TYPE_UNKNOWN does not necessarily mean that the covariance will be automatically computed by navsat_transform_node.
        });
        lastCoordsWerePublished = true;
    }
}
