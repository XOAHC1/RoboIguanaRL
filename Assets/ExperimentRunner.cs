using UnityEngine;
using System;
using SmarcGUI.KeyboardControllers;

[RequireComponent(typeof(RoboIguanaIKController))]
public class ExperimentRunner : MonoBehaviour
{
    private RoboIguanaIKController controller;

    float simulationTime;
    float timer = 0f;

    void Awake()
    {
        controller = GetComponent<RoboIguanaIKController>();

        // ==============================
        // Read parameters from command line
        // ==============================

        float freq = GetArg("-frequency", 0.4f);
        float phaseShift = GetArg("-phaseShift", 0f);
        float spineAmp = GetArg("-spineAmp", 0f);
        float spineStiff = GetArg("-spineStiff", 8.64f);

        // ==============================
        // Apply parameters to controller
        // ==============================

        controller.frequencyHz = freq;
        controller.PhaseShift = phaseShift;
        controller.spineAmplitudeDeg = spineAmp;
        controller.spineStiffness = spineStiff;

        // ==============================
        // Simulation duration
        // ==============================

        simulationTime = 50f / freq+5;

        Debug.Log("===== Experiment Parameters =====");
        Debug.Log("Frequency       = " + freq);
        Debug.Log("Phase Shift     = " + phaseShift);
        Debug.Log("Spine Amplitude = " + spineAmp);
        Debug.Log("Spine Stiffness = " + spineStiff);
        Debug.Log("Simulation time = " + simulationTime);
    }

    void Update()
    {
        timer += Time.deltaTime;

        if (timer > simulationTime)
        {
            Debug.Log("Simulation finished");
            Application.Quit();
        }
    }

    float GetArg(string name, float defaultValue)
    {
        string[] args = Environment.GetCommandLineArgs();

        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] == name && i + 1 < args.Length)
            {
                float value;

                if (float.TryParse(args[i + 1], out value))
                    return value;
            }
        }

        return defaultValue;
    }
}