using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(GPSProbe))]
public class GPSProbeGUI : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        GPSProbe probe = (GPSProbe)target;

        // Add a button to the inspector
        if (GUILayout.Button("Debug.Log GPS Coordinates"))
        {
            // Spawn a GPS device using Assets/Simulation/GPS/GPS.prefab
            GameObject gpsObj = (GameObject)Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Simulation/GPS/GPS.prefab"));
            GPS gps = gpsObj.GetComponent<GPS>();

            // Move the GPS device to the position of the probe.
            gpsObj.transform.parent = probe.transform;
            gpsObj.transform.localPosition = Vector3.zero;
            
            // Scan for base stations.
            gps.ScanBaseStations();

            // Update GPS.
            gps.FixedUpdate();

            // Read GPS data.
            GPS.Coords lastCoords = gps.lastCoords;
            bool hasFix = gps.hasFix;
            Matrix<double> procrustesTransform = gps.procrustesTransform;

            // Read the yaw from procrustes transformation.
            double yaw = -Math.Atan2(procrustesTransform[1, 0], procrustesTransform[0, 0]);
            yaw -= Math.PI / 2; // Correct to east=0, north=pi/2
            yaw = Math.Atan2(Math.Sin(yaw), Math.Cos(yaw)); // Normalize to (-pi, pi)
            
            if (hasFix)
            {
                Debug.Log("Lat, Long: " + lastCoords.latitude + ", " + lastCoords.longitude + ", Altitude: " + lastCoords.altitude + ", World Yaw: " + yaw + " rad");
            }
            else
            {
                Debug.Log("No GPS fix.");
            }
            

            // Destroy the GPS device.
            DestroyImmediate(gps);
        }
    }
}
