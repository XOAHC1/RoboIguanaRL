using UnityEngine;
using System.IO;
using System.Collections;
using SmarcGUI.KeyboardControllers;

public class RecordPosition : MonoBehaviour
{
    public Transform legBase;
    public Transform footTip;

    public RoboIguanaIKController controller;

    private StreamWriter writer;
    private bool isRecording = false;

    void Start()
    {
        // Automatically find controller if not assigned
        if (controller == null)
            controller = GetComponent<RoboIguanaIKController>();

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

        // ===================== NEW LOGIC =====================
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
        // ====================================================

        // Construct filename from experiment parameters
        string fileName =
            "position_f" + freq.ToString("F2") +
            "_phase" + phaseShift.ToString("F1") +
            "_amp" + spineAmp.ToString("F1") +
            "_" + stiffLabel +
            ".csv";

        string path = Application.dataPath + "/" + fileName;

        writer = new StreamWriter(path);
        writer.WriteLine("time,x_rel,y_rel,z_rel");

        Debug.Log("Recording to: " + path);

        // Start recording automatically after delay
        StartCoroutine(StartRecordingAfterDelay(5f));
    }

    IEnumerator StartRecordingAfterDelay(float delay)
    {
        Debug.Log("Waiting " + delay + " seconds before recording...");
        yield return new WaitForSeconds(delay);

        isRecording = true;
        Debug.Log("Recording started");
    }

    void FixedUpdate()
    {
        if (!isRecording)
            return;

        if (legBase == null || footTip == null)
        {
            Debug.LogWarning("Transforms not assigned!");
            return;
        }

        Vector3 pRel = legBase.InverseTransformPoint(footTip.position);

        writer.WriteLine($"{Time.fixedTime},{pRel.x},{pRel.y},{pRel.z}");
        writer.Flush();
    }

    void OnDestroy()
    {
        writer?.Close();
    }
}