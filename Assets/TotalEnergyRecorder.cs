using UnityEngine;
using System.IO;
using SmarcGUI.KeyboardControllers;

public class TotalEnergyRecorder : MonoBehaviour
{
    private ServoPowerEstimator[] estimators;
    private StreamWriter writer;
    private bool hasStartedWriting = false;
    private bool fileInitialized = false;

    public RoboIguanaIKController controller;

    [Header("Logging settings")]
    public float startLoggingTime = 5f;     // start writing after 5 s
    public float logInterval = 0.02f;       // 50 Hz logging
    private float nextLogTime = 0f;

    private string filePath;

    void Start()
    {
        estimators = GetComponentsInChildren<ServoPowerEstimator>();

        Debug.Log("Energy recorder started.");
        Debug.Log("Found " + estimators.Length + " servo estimators");

        if (controller == null)
            controller = GetComponent<RoboIguanaIKController>();
    }

    void Update()
    {
        if (Time.time < startLoggingTime)
            return;

        if (!fileInitialized)
        {
            InitializeFile();
        }

        if (Time.time >= nextLogTime)
        {
            WriteRealtimeEnergy();
            nextLogTime = Time.time + logInterval;
        }
    }

    void InitializeFile()
    {
        float freq = 0f;
        float phaseShift = 0f;
        float spineAmp = 0f;
        float spineStiff = 0f;

        if (controller != null)
        {
            freq = controller.frequencyHz;
            phaseShift = controller.PhaseShift;
            spineAmp = controller.spineAmplitudeDeg;
            spineStiff = controller.spineStiffness;
        }

        string stiffLabel = "stiff_unknown";

        if (controller != null && controller.spine_Link != null)
        {
            if (controller.spine_Link.jointType == ArticulationJointType.FixedJoint)
            {
                stiffLabel = "stiff_rigid";
            }
            else
            {
                stiffLabel = "stiff_" + spineStiff.ToString("F2");
            }
        }

        string fileName =
            "energy_f" + freq.ToString("F2") +
            "_phase" + phaseShift.ToString("F1") +
            "_amp" + spineAmp.ToString("F1") +
            "_" + stiffLabel +
            ".csv";

        filePath = Path.Combine(Application.dataPath, fileName);

        writer = new StreamWriter(filePath, false);
        writer.WriteLine("Time_s,TotalEnergy_J");
        writer.Flush();

        fileInitialized = true;
        hasStartedWriting = true;

        Debug.Log("Realtime energy logging started: " + filePath);
    }

    void WriteRealtimeEnergy()
    {
        if (writer == null) return;

        float totalEnergy = 0f;

        foreach (var est in estimators)
        {
            totalEnergy += est.mechanicalEnergyJ;
        }

        writer.WriteLine(Time.time.ToString("F4") + "," + totalEnergy.ToString("F6"));
        writer.Flush();
    }

    void OnApplicationQuit()
    {
        CloseFile();
    }

    void OnDestroy()
    {
        CloseFile();
    }

    void CloseFile()
    {
        if (writer != null)
        {
            Debug.Log("Closing energy log file.");
            writer.Flush();
            writer.Close();
            writer = null;
        }
    }
}