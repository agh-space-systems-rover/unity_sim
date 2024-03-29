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
    private float frequency = 10;

    // [SerializeField]
    // private float noise = 0.0f;

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

    // private Vector3d Barycentric2D(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p)
    // {
    //     double detT = (p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y);

    //     double alpha = ((p2.y - p3.y) * (p.x - p3.x) + (p3.x - p2.x) * (p.y - p3.y)) / detT;
    //     double beta = ((p3.y - p1.y) * (p.x - p3.x) + (p1.x - p3.x) * (p.y - p3.y)) / detT;
    //     double gamma = 1 - alpha - beta;

    //     return new Vector3d(alpha, beta, gamma);
    // }

    // private Vector2d ComputeLatLongPlanarFit() {
    //     // Exit in case there are not enough base stations.
    //     if (baseStations == null || baseStations.Length < 2)
    //     {
    //         return Vector2d.zero;
    //     }

    //     // For each pair of base stations, compute the transform from sim vectors to GPS vectors.
    //     // Average the transforms to get a more precise one.
    //     Vector2d rotationSum = Vector2d.zero;
    //     double scaleSum = 0;
    //     double samples = 0;
    //     for (int i = 0; i < baseStations.Length; i++)
    //     {
    //         for (int j = i + 1; j < baseStations.Length; j++)
    //         {
    //             // Get the simulated world and GPS vectors from baseI to baseJ.
    //             Vector2d vSimIJ = new Vector2d(
    //                 (double)baseStations[j].transform.position.x - (double)baseStations[i].transform.position.x,
    //                 (double)baseStations[j].transform.position.z - (double)baseStations[i].transform.position.z
    //             );
    //             Vector2d vGpsIJ = new Vector2d(
    //                 baseStations[j].longitude - baseStations[i].longitude,
    //                 baseStations[j].latitude - baseStations[i].latitude
    //             );

    //             // Find the transform from vSim to vGps and accumulate.
    //             double weight = vSimIJ.magnitude; // weight by distance
    //             double rot = System.Math.Atan2(vGpsIJ.y, vGpsIJ.x) - System.Math.Atan2(vSimIJ.y, vSimIJ.x);
    //             rotationSum += new Vector2d(System.Math.Cos(rot), System.Math.Sin(rot)) * weight;
    //             scaleSum += vGpsIJ.magnitude / vSimIJ.magnitude * weight;
    //             samples += weight;
    //         }
    //     }
    //     double rotation = System.Math.Atan2(rotationSum.y, rotationSum.x);
    //     double scale = scaleSum / samples;

    //     // Compute the average base station position and lat/long.
    //     Vector2d avgBasePos = Vector2d.zero;
    //     Vector2d avgLongLat = Vector2d.zero;
    //     for (int i = 0; i < baseStations.Length; i++)
    //     {
    //         avgBasePos += new Vector2d((double)baseStations[i].transform.position.x, (double)baseStations[i].transform.position.z);
    //         avgLongLat += new Vector2d(baseStations[i].longitude, baseStations[i].latitude);
    //     }
    //     avgBasePos /= baseStations.Length;
    //     avgLongLat /= baseStations.Length;

    //     // TODO: Move above computation to Start() and only update the following in FixedUpdate().

    //     // Get the sim vectors from the average base station to the robot.
    //     Vector2d vSim = new Vector2d(
    //         (double)transform.position.x - (double)avgBasePos.x,
    //         (double)transform.position.z - (double)avgBasePos.y
    //     );

    //     // Apply the sim->GPS transform.
    //     Vector2d vGps = new Vector2d(
    //         System.Math.Cos(rotation) * vSim.x - System.Math.Sin(rotation) * vSim.y,
    //         System.Math.Sin(rotation) * vSim.x + System.Math.Cos(rotation) * vSim.y
    //     ) * scale;
        
    //     // Compute the proper GPS coords.
    //     double longitude = avgLongLat.x + vGps.x;
    //     double latitude = avgLongLat.y + vGps.y;

    //     return new Vector2d(latitude, longitude);
    // }

    // private Vector2d ComputeLatLongBarycentricInterpolation() {
    //     // Exit in case there are not enough base stations.
    //     if (baseStations == null || baseStations.Length < 3)
    //     {
    //         return Vector2d.zero;
    //     }

    //     // Compute base station distances
    //     float[] distances = new float[baseStations.Length];
    //     for (int i = 0; i < baseStations.Length; i++)
    //     {
    //         distances[i] = Vector3.Distance(transform.position, baseStations[i].transform.position);
    //     }
    //     // Sort base stations by distance
    //     GPSBaseStation[] sortedBaseStations = new GPSBaseStation[baseStations.Length];
    //     Array.Copy(baseStations, sortedBaseStations, baseStations.Length);
    //     System.Array.Sort(distances, sortedBaseStations);
    //     // Get the 3 closest base stations
    //     ArraySegment<GPSBaseStation> closestBaseStations = new ArraySegment<GPSBaseStation>(sortedBaseStations, 0, 3);

    //     // Compute the barycentric coordinates of the robot with respect to the 3 closest base stations.
    //     Vector3d barycentric = Barycentric2D(
    //         new Vector2d((double)closestBaseStations[0].transform.position.x, (double)closestBaseStations[0].transform.position.z),
    //         new Vector2d((double)closestBaseStations[1].transform.position.x, (double)closestBaseStations[1].transform.position.z),
    //         new Vector2d((double)closestBaseStations[2].transform.position.x, (double)closestBaseStations[2].transform.position.z),
    //         new Vector2d((double)transform.position.x, (double)transform.position.z)
    //     );

    //     // Compute the GPS coordinates.
    //     double latitude = barycentric.x * closestBaseStations[0].latitude + barycentric.y * closestBaseStations[1].latitude + barycentric.z * closestBaseStations[2].latitude;
    //     double longitude = barycentric.x * closestBaseStations[0].longitude + (double)barycentric.y * closestBaseStations[1].longitude + barycentric.z * closestBaseStations[2].longitude;
    //     // NOTE: Simple coordinate interpolation will break at the poles.
    //     // To do this properly, we would need to slerp between three
    //     // quaternions and then compute coords from the resulting quaternion.
    //     // This is not critical for our purposes, so we will leave it as is.

    //     return new Vector2d(latitude, longitude);
    // }

    internal void FixedUpdate()
    {
        if (!initialized)
        {
            return;
        }

        // Compute the GPS coordinates.
        // Vector2d latLong = ComputeLatLongPlanarFit();
        // Vector2d latLong = ComputeLatLongBarycentricInterpolation();
        // if (latLong == Vector2d.zero)
        // {
        //     hasFix = false;
        //     return;
        // }
        // double latitude = latLong.x;
        // double longitude = latLong.y;

        var robotPos = Vector<double>.Build.DenseOfArray(new double[] { transform.position.x, transform.position.z, 1 });
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

        lastCoords = new Coords
        {
            latitude = latitude,
            longitude = longitude,
            altitude = altitude
        };
        hasFix = true;
    }

    private async void Update()
    {
        // publish to ROS
        if (publisher == null)
        {
            return;
        }

        double currentTime = Time.unscaledTimeAsDouble;
        if (currentTime - lastPublishTime < 1.0 / frequency)
        {
            return;
        }
        lastPublishTime = currentTime;

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
            Altitude = hasFix ? lastCoords.altitude : 0,
            PositionCovariance = new double[9] { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            PositionCovarianceType = ROSBridge.SensorMsgs.NavSatFix.COVARIANCE_TYPE_UNKNOWN
        });
    }
}
