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
    private float frequency = 1;

    [SerializeField]
    private bool enableAltitude = false;

    [SerializeField]
    private float noiseStdDev = 0;

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

        // Compute a random noise vector.
        // double noiseRadius = MathNet.Numerics.Distributions.Normal.Sample(0, noiseStdDev);
        // double noiseTheta = MathNet.Numerics.Distributions.ContinuousUniform.Sample(0, 2 * Math.PI);
        // double noisePhi = MathNet.Numerics.Distributions.ContinuousUniform.Sample(0, Math.PI);
        // var noise = new Vector3D(
        //     noiseRadius * Math.Cos(noiseTheta) * Math.Sin(noisePhi),
        //     noiseRadius * Math.Sin(noiseTheta) * Math.Sin(noisePhi),
        //     noiseRadius * Math.Cos(noisePhi)
        // );
        var noiseVec = Vector<double>.Build.Random(3, new MathNet.Numerics.Distributions.Normal(0, noiseStdDev));
        var noise = new Vector3D(noiseVec[0], noiseVec[1], noiseVec[2]);

        // Get the robot's 2D position. Append a 1 to multiply with a 2D transformation matrix.
        var robotPos = Vector<double>.Build.DenseOfArray(new double[] { transform.position.x, transform.position.z, 1 });

        // Add noise to the robot's position.
        robotPos += Vector<double>.Build.DenseOfArray(new double[] { noise.X, noise.Y, 0 });

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
        altitude += (float)noise.Z;

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
