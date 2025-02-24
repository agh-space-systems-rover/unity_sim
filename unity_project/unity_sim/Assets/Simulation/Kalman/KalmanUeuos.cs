using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;

using UnityEngine;
using UnityEngine.Serialization;
using UnityEditor;

using ROSBridge;
using ROSBridge.KalmanInterfaces;

public class KalmanUeuos : MonoBehaviour
{
    private static Func<float, Color> bootColorFunction = time => {
        const int numCycles = 10;
        const float cycleSpeed = 20;
        const float cycleSteepness = 4;
        float scaledX = time * cycleSpeed;
        if (scaledX < 2*Mathf.PI * numCycles)
        {
            // Only light up for the first 5 cycles.
            float sine = Mathf.Sin(scaledX - Mathf.PI/2);
            float sCurve = 1 / (1 + Mathf.Exp(-sine * cycleSteepness));
            return Color.red * sCurve;
        } else {
            return Color.black;
        }
    };

    [SerializeField]
    private string autonomyMaterialNameRegex = "autonomy";
    [SerializeField]
    private float hdrIntensity = 5.0F;
    [SerializeField]
    private string setColorService = "ueuos/set_color";
    [SerializeField]
    private string setStateService = "ueuos/set_state";
    [SerializeField]
    private string setEffectService = "ueuos/set_effect";

    private ROS ros;
    private Material autonomyMaterial;
    private Func<float, Color> colorFunction;
    private float colorFunctionTimer;

    private void Start()
    {
        // Get the autonomy material and clone it.
        var renderer = GetComponent<Renderer>();
        Material[] materials = renderer.materials;
        for (int i = 0; i < materials.Length; i++)
        {
            if (Regex.IsMatch(materials[i].name, autonomyMaterialNameRegex))
            {
                autonomyMaterial = materials[i];
                break;
            }
        }

        // Set default color.
        colorFunction = bootColorFunction;

        // Initialize ROSBridge connection.
        ros = new ROS();

        // Create services.
        ros.CreateService<SetUeuosColor, SetUeuosColor.Request, SetUeuosColor.Response>(setColorService, SetColor);
        ros.CreateService<SetUeuosState, SetUeuosState.Request, SetUeuosState.Response>(setStateService, SetState);
        ros.CreateService<SetUeuosEffect, SetUeuosEffect.Request, SetUeuosEffect.Response>(setEffectService, SetEffect);
    }

    private void OnApplicationQuit()
    {
        if (ros != null)
        {
            ros.Close();
        }
    }

    private void SetColor(SetUeuosColor.Request req, SetUeuosColor.Response res)
    {
        colorFunction = _ => new Color(req.Color.R, req.Color.G, req.Color.B);
    }

    private void SetState(SetUeuosState.Request req, SetUeuosState.Response res)
    {
        switch (req.State) {
            case SetUeuosState.Request.OFF:
                colorFunction = _ => Color.black;
                break;
            case SetUeuosState.Request.AUTONOMY:
                colorFunction = _ => Color.red;
                break;
            case SetUeuosState.Request.TELEOP:
                colorFunction = _ => Color.blue;
                break;
            case SetUeuosState.Request.FINISHED:
                colorFunction = time => {
                    float strength = Mathf.Sin(time * 2) * 0.5F + 0.5F;
                    return Color.green * strength;
                };
                colorFunctionTimer = 0;
                break;
            default:
                break;
        }
    }

    private void SetEffect(SetUeuosEffect.Request req, SetUeuosEffect.Response res)
    {
        switch (req.Effect) {
            case SetUeuosEffect.Request.BOOT:
                colorFunction = bootColorFunction;
                colorFunctionTimer = 0;
                break;
            case SetUeuosEffect.Request.RAINBOW:
                colorFunction = time => {
                    float hue = (0.2F * time) % 1;
                    return Color.HSVToRGB(hue, 1, 1);
                };
                colorFunctionTimer = 0;
                break;
            default:
                break;
        }
    }

    private void Update()
    {
        // Add to the timer.
        colorFunctionTimer += Time.deltaTime;

        // Update the color.
        if (colorFunction != null)
        {
            // Shader is URP/Lit.
            autonomyMaterial.SetColor("_EmissionColor", colorFunction(colorFunctionTimer) * Mathf.Pow(2, hdrIntensity));
        }
    }
}
