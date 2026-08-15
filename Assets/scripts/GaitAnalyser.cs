using System.Collections.Generic;
using System.IO;

public class GaitAnalyser
{
    public bool active;

    public Dictionary<string, float> AnalysisState;
    private List<Dictionary<string, float>> AnalysisParameters;

    private string LogPath;
    private string run_id;
    private bool first = true;

    public GaitAnalyser(bool active, string run_id = "test")
    {
        this.run_id = run_id;
        this.active = active;

        AnalysisParameters = new List<Dictionary<string, float>>();
        AnalysisState = new Dictionary<string, float>();

        LogPath = Path.Combine("Analysis", "Gait_data", $"{this.run_id}_GaitAnalysis.csv");
    }

    public void DoUpdate()
    {
        AnalysisParameters.Add(new Dictionary<string, float>(AnalysisState));
    }

    private void WriteHead() 
    {
        var directory = Path.GetDirectoryName(LogPath);
        if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
        {
            Directory.CreateDirectory(directory);
        }

        if (!File.Exists(LogPath))
        {
            // Prepare file for logging rewards
            using (var writer = new StreamWriter(LogPath, false))
            {
                writer.Write("Episode");
                foreach(var k in AnalysisParameters[0].Keys)
                {
                    writer.Write($",{k}");
                }
                writer.WriteLine();
            }
        }

    }

    public void LogEpisode()
    {
        if (first) {WriteHead(); first=false;}

        using (var writer = new StreamWriter(LogPath, true))
        {
            writer.WriteLine("New episode");
            foreach(var d in AnalysisParameters)
            {
                writer.Write("");
                foreach(var k in d.Keys)
                {
                    writer.Write($",{d[k]}");
                }
                writer.WriteLine();
            }
        }
    }


}
