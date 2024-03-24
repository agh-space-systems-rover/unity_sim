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
        const float cycleSteepness = 3;
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
    private float maxIntensity = 5.0F;
    [SerializeField]
    private string setColorService = "ueuos/set_color";
    [SerializeField]
    private string setModeService = "ueuos/set_mode";
    [SerializeField]
    private string setEffectService = "ueuos/set_effect";

    private ROS ros;
    private Material autonomyMaterial;
    private Func<float, Color> colorFunction;
    private float colorFunctionTimer;

    private async void Start()
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
        await ros.CreateService<SetUeuosColor, SetUeuosColor.Request, SetUeuosColor.Response>(setColorService, SetColor);
        await ros.CreateService<SetUeuosMode, SetUeuosMode.Request, SetUeuosMode.Response>(setModeService, SetMode);
        await ros.CreateService<SetUeuosEffect, SetUeuosEffect.Request, SetUeuosEffect.Response>(setEffectService, SetEffect);
    }

    private async void OnApplicationQuit()
    {
        if (ros != null)
        {
            await ros.Close();
        }
    }

    private void SetColor(SetUeuosColor.Request req, SetUeuosColor.Response res)
    {
        colorFunction = _ => new Color(req.Color.R, req.Color.G, req.Color.B);
    }

    private void SetMode(SetUeuosMode.Request req, SetUeuosMode.Response res)
    {
        switch (req.Mode) {
            case SetUeuosMode.Request.OFF:
                colorFunction = _ => Color.black;
                break;
            case SetUeuosMode.Request.AUTONOMY:
                colorFunction = _ => Color.red;
                break;
            case SetUeuosMode.Request.TELEOP:
                colorFunction = _ => Color.blue;
                break;
            case SetUeuosMode.Request.FINISHED:
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
            autonomyMaterial.SetColor("_EmissionColor", colorFunction(colorFunctionTimer) * maxIntensity);
        }
    }
}
