using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridge;

public class GPSProbe : MonoBehaviour
{
    // See GPSProbeGUI.cs
    [TextArea(3,10)]
    public string waypointsToLocalize;
    public bool localizeInENUFrame = false;
}
