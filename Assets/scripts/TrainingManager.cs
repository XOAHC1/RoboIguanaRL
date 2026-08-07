using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Newtonsoft.Json;

namespace RoboIguanaRL
{
    /// <summary>
    /// Class to handle import of reward weights and logging of reward development throughout training.
    /// </summary>
    public class TrainingManager
    {
        /// <summary>
        /// Contains configuration settings for the training.
        /// </summary>
        public Dictionary<string, bool> Config;

        /// <summary>
        /// Reward factors as measured by the agent.
        /// </summary>
        public Dictionary<string, float> 
            ExpRewards = new Dictionary<string, float>(),
            QuadPenalties = new Dictionary<string, float>(),
            LinRewards = new Dictionary<string, float>();

        /// <summary>
        /// Weighted rewards of the ongoing episode.
        /// </summary>
        public Dictionary<string, List<float>> Rewards = new Dictionary<string, List<float>>();

        /// <summary>
        /// Total reward of past episodes by parameter.
        /// </summary>
        public Dictionary<string, List<float>> RewardHistory = new Dictionary<string, List<float>>();

        /// <summary>
        /// Weights of different reward aspects.
        /// </summary>
        public Dictionary<string, float> RewardWeights = new Dictionary<string, float>();

        /// <summary>
        /// All reward parameters.
        /// </summary>
        public List<string> keys;

        /// <summary>
        /// Reward factors by category.
        /// </summary>
        public List<string> 
            expKeys = new List<string>(),
            quadKeys = new List<string>(),
            linKeys = new List<string>();

        /// <summary>
        /// File path of reward weights.
        /// </summary>
        private string WeightFile = Path.Combine(Application.streamingAssetsPath, "LearningWeights.json");

        /// <summary>
        /// File path of confg file.
        /// </summary>
        private string ConfigFile = Path.Combine(Application.streamingAssetsPath, "LearningConfig.json");

        /// <summary>
        /// Path to which Rewards are logged.
        /// </summary>
        private string LogPath;

        /// <summary>
        /// Run id parameter of the mlagents learning process.
        /// </summary>
        private string run_id;

        public bool LogHistory;

        /// <summary>
        /// Class to handle import of reward weights and logging of reward development throughout training.
        /// </summary>
        public TrainingManager()
        {
            Debug.Log("Loading Reward Weights");

            ReadConfig();
            LoadWeights();


            if (LogHistory)
            {
                LogPath = Path.Combine("results", run_id, "RewardHistory.csv");
                WriteHead();
                
            }
        }

        /// <summary>
        /// Reset current rewards. Log Rewards if wanted.
        /// </summary>
        public void NewEpisode()
        {
            // Log 
            foreach (var key in keys)
            {
                RewardHistory[key].Add(Rewards[key].Sum());
                Rewards[key].Clear();
            }

            LinRewards["crash"] = 0;
        }

        /// <summary>
        /// Read config file and set <c>LogHistory</c>.
        /// </summary>
        private void ReadConfig()
        {
            if (!File.Exists(ConfigFile))
            {
                Debug.LogWarning($"Config file not found: {ConfigFile}");
                return;
            }

            string json = File.ReadAllText(ConfigFile);
            if (string.IsNullOrWhiteSpace(json))
            {
                Debug.Log("Reward config file is empty!");
                return;
            }

            string configString = File.ReadAllText(ConfigFile);

            if (string.IsNullOrWhiteSpace(configString))
            {
                Debug.Log("Reward weight file is empty!");
                return;
            }

            Config = JsonConvert.DeserializeObject<Dictionary<string, bool>>(configString)!;

            run_id = Config.Keys.First();
            LogHistory = Config.Values.First();
        }

        /// <summary>
        /// Load weights from specfied Json file.
        /// </summary>
        public void LoadWeights()
        {
            if (!File.Exists(WeightFile))
            {
                Debug.LogWarning($"Weight file not found: {WeightFile}");
                return;
            }

            string json = File.ReadAllText(WeightFile);
            if (string.IsNullOrWhiteSpace(json))
            {
                Debug.Log("Reward weight file is empty!");
                return;
            }

            var input = JsonConvert.DeserializeObject<Dictionary<string, float>>(json)!;

            foreach (var k in input.Keys.ToArray())
            {
                // seperate identifier
                var id = k[0];
                var key = k[2..];
                          
                // sort into respectice category
                if (id == 'e') {expKeys.Add(key); ExpRewards[key] = 0;}
                else if (id == 'q') {quadKeys.Add(key); QuadPenalties[key] = 0;}
                else if (id == 'l') {linKeys.Add(key); LinRewards[key] = 0;}

                // initialize reward lists
                RewardWeights[key] = input[k];      
                Rewards[key] = new List<float>();
                RewardHistory[key] = new List<float>();
            }

            // collect reward parameters
            keys = RewardWeights.Keys.ToList();

            // set default values
            LinRewards["baseReward"] = 1f;
        }

        /// <summary>
        /// Calculates weighted rewards from current <c>ExpRewards</c>.
        /// </summary>
        /// <remarks>
        /// All raw reward factors are high => bad.
        /// </remarks>
        /// <returns>Current step reward</returns>
        public float GetReward()
        {
            var stepReward = 0f;
            foreach (var key in expKeys)
            {
                var val = 
                    Mathf.Pow(Mathf.Abs(ExpRewards[key]), 2);
                var partialReward = 
                    Mathf.Exp(-(val / 0.2f)) * RewardWeights[key] * Time.fixedDeltaTime;

                Rewards[key].Add(partialReward);
                stepReward += partialReward;
            }
            foreach (var key in linKeys) 
            {
                var partialReward = 
                LinRewards[key] * RewardWeights[key] * Time.fixedDeltaTime;
                Rewards[key].Add(partialReward);
                stepReward += partialReward;
            }
            foreach (var key in quadKeys) 
            {
                var partialReward = 
                - Mathf.Pow(QuadPenalties[key], 2) * RewardWeights[key] * Time.fixedDeltaTime;
                Rewards[key].Add(partialReward);
                stepReward += partialReward;
            }

            return stepReward;            
        }

        /// <summary>
        /// Writes reward weights and config in log file.
        /// </summary>
        private void WriteHead() 
        {
            var directory = Path.GetDirectoryName(LogPath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }

            // Prepare file for logging rewards
            using (var writer = new StreamWriter(LogPath, false))
            {
                // write reward weights
                writer.Write("Episode ");
                foreach (var k in keys) writer.Write($",{k}");
                writer.WriteLine();
                writer.Write("Weights: ");
                foreach (var w in RewardWeights.Values) writer.Write($",{w}");
                writer.WriteLine();
                writer.WriteLine();

                // write config
                writer.WriteLine("Config:");
                foreach(var c in Config) writer.WriteLine($"{c.Key}, \"{c.Value}\"");
                writer.WriteLine();

                writer.WriteLine("Values:");
            }

        }

        /// <summary>
        /// Writes cumulated rewards of last episode by reward parameter into log file.
        /// </summary>
        public void LogEpisode()
        {
            using (var writer = new StreamWriter(LogPath, true))
            {
                // Log last episode's rewards
                writer.Write(RewardHistory.Values.FirstOrDefault()?.Count ?? 0);

                foreach (var key in keys)
                {
                    writer.Write($",{RewardHistory[key].Last()}");

                }

                writer.WriteLine();
            }   
        }
    }
}