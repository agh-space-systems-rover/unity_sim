using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(GPSProbe))]
public class GPSProbeGUI : Editor
{
    private class Waypoint {
        public string name;
        public double x;
        public double z;
    }

    private Waypoint[] ParseWaypoints(string waypointListStr) {
        string[] lines = waypointListStr.Split(new string[] { "\r\n", "\n" }, StringSplitOptions.None);
        Waypoint[] waypoints = new Waypoint[lines.Length];
        for (int i = 0; i < lines.Length; i++) {
            string line = lines[i];
            string[] parts = line.Split(' ');
            if (parts.Length == 3) {
                Waypoint wp = new Waypoint();
                wp.name = parts[0];
                wp.x = double.Parse(parts[1]);
                wp.z = double.Parse(parts[2]);
                waypoints[i] = wp;
            }
        }
        return waypoints;
    }

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        GPSProbe probe = (GPSProbe)target;

        if (GUILayout.Button("Localize Waypoints"))
        {
            string waypointListStr = probe.waypointsToLocalize;
            Waypoint[] waypoints = ParseWaypoints(waypointListStr);

            // Spawn a GPS device using Assets/Simulation/GPS/GPS.prefab
            GameObject gpsObj = (GameObject)Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Simulation/GPS/GPS.prefab"));
            GPS gps = gpsObj.GetComponent<GPS>();

            // Scan for base stations.
            gps.ScanBaseStations();

            // For each waypoint, localize it.
            string waypointResults = "";
            foreach (Waypoint wp in waypoints) {
                // Move the GPS device to the position of the waypoint.
                gpsObj.transform.position = new Vector3((float)wp.x, 0, (float)wp.z);

                // Update GPS.
                gps.FixedUpdate();

                // Read GPS data.
                GPS.Coords lastCoords = gps.lastCoords;
                bool hasFix = gps.hasFix;
                
                // Add the result to the output string.
                if (hasFix)
                {
                    waypointResults += wp.name + " " + lastCoords.latitude + " " + lastCoords.longitude + "\n";
                }
                else
                {
                    waypointResults += wp.name + ": No GPS fix.\n";
                }
            }

            // Print the results.
            Debug.Log(waypointResults);

            // Destroy the GPS device.
            DestroyImmediate(gpsObj);
        }

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
            DestroyImmediate(gpsObj);
        }
    }
}
