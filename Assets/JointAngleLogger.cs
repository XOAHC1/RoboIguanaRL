using UnityEngine;
using System.IO;

[RequireComponent(typeof(ArticulationBody))]
public class JointAngleLogger : MonoBehaviour
{
    private ArticulationBody joint;
    private StreamWriter writer;

    [Header("Logging")]
    public string fileName = "joint_angle_log.csv";
    public bool logInDegrees = true;

    void Start()
    {
        joint = GetComponent<ArticulationBody>();

        string path = Path.Combine(Application.dataPath, fileName);

        writer = new StreamWriter(path);
        writer.WriteLine("Time,TargetAngle,ActualAngle");

        Debug.Log("Logging joint angles to: " + path);
    }

    void FixedUpdate()
    {
        if (writer == null) return;

        // Actual joint angle
        float actualAngle = joint.jointPosition[0];

        // Target angle from drive
        float targetAngle = joint.xDrive.target;

        if (logInDegrees)
        {
            actualAngle *= Mathf.Rad2Deg;
        }

        float time = Time.time;

        writer.WriteLine($"{time},{targetAngle},{actualAngle}");
    }

    void OnDestroy()
    {
        if (writer != null)
        {
            writer.Flush();
            writer.Close();
        }
    }
}