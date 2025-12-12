using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridge;

using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnityEditor;

public class GPSProbe : MonoBehaviour
{
    // See GPSProbeGUI.cs
    [TextArea(3,10)]
    public string waypointsToLocalize;
    public bool localizeInENUFrame = false;

    private const string GPS_PATH = "Assets/Simulation/GPS/GPS.prefab";

    public double calculateMagnitude(){
        GameObject gpsObj = (GameObject)Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(GPS_PATH));
        GPS gps = gpsObj.GetComponent<GPS>();

        // Move the GPS device to the position of the probe.
        gpsObj.transform.parent = transform;
        gpsObj.transform.localPosition = Vector3.zero;
        
        // Scan for base stations.
        gps.ScanBaseStations();

        // Update GPS.
        gps.FixedUpdate();

        // Read GPS data.
        Matrix<double> procrustesTransform = gps.procrustesTransform;

        // Read the yaw from procrustes transformation.
        double yaw = -Math.Atan2(procrustesTransform[1, 0], procrustesTransform[0, 0]);
        yaw -= Math.PI / 2; // Correct to east=0, north=pi/2
        yaw = Math.Atan2(Math.Sin(yaw), Math.Cos(yaw)); // Normalize to (-pi, pi)

        return yaw;
    }
}
