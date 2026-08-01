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
        public Dictionary<string, bool> Config;

        /// <summary>
        /// Reward factors as measured by the agent.
        /// </summary>
        public Dictionary<string, float> RawRewards = new Dictionary<string, float>();

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
        public Dictionary<string, float> RewardWeights;

        /// <summary>
        /// Wether the Robot has crashed.
        /// </summary>
        public bool Crashed;

        /// <summary>
        /// Linear reward.
        /// </summary>
        private float CrashPenalty, BaseReward;

        /// <summary>
        /// Different reward parameters.
        /// </summary>
        public string[] keys;

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
        private string LogPath = Path.Combine("results", "Rewards");

        /// <summary>
        /// Wether to log reward history.
        /// </summary>
        private bool LogHistory;

        /// <summary>
        /// Envronment parameters.
        /// </summary>
        public bool RandomDirection, RandomVelocity, Swimming, Transition;

        /// <summary>
        /// Class to handle import of reward weights and logging of reward development throughout training.
        /// </summary>
        public TrainingManager()
        {
            Debug.Log("Loading Reward Weights");

            ReadConfig();
            LoadWeights();

            if (LogHistory) {
                // Prepare file for logging rewards
                string filePath = Path.Combine(LogPath, "RewardHistory.csv");

                using (var writer = new StreamWriter(filePath, false))
                {
                    writer.Write("Episode");
                    foreach (var key in keys)
                    {
                        writer.Write($",{key}");
                    }
                    writer.Write(",Crashed");
                    writer.WriteLine();
                }
            }
        }

        /// <summary>
        /// Reset current rewards. Log Rewards if wanted.
        /// </summary>
        public void NewEpisode()
        {
            if (LogHistory)
            {
                string filePath = Path.Combine(LogPath, "RewardHistory.csv");

                using (var writer = new StreamWriter(filePath, true))
                {
                    // Log last episode's rewards
                    writer.Write(RewardHistory.Values.FirstOrDefault()?.Count ?? 0);

                    foreach (var key in keys)
                    {
                        float episodeReward = Rewards[key].Sum();

                        RewardHistory[key].Add(episodeReward);

                        writer.Write($",{episodeReward}");

                        Rewards[key].Clear();
                    }

                    writer.Write($",{(Crashed? 0: 1)}");
                    writer.WriteLine();
                }   
            } 
            else 
            {
                foreach (var key in keys)
                {
                    RewardHistory[key].Add(Rewards[key].Sum());
                    Rewards[key].Clear();
                    RawRewards[key] = 0f;
                }
            }

            Crashed = false;

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

            LogHistory = Config["LogRewardHistory"];

            Config.Remove("LogHistory");

        }

        /// <summary>
        /// Load weights from specfied Json file.
        /// </summary>
        public void LoadWeights()
        {
            RewardWeights = new Dictionary<string, float>();

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

            RewardWeights = JsonConvert.DeserializeObject<Dictionary<string, float>>(json)!;

            // extract linear rewards
            Crashed = false;
            CrashPenalty = RewardWeights["CrashPenalty"];
            BaseReward = RewardWeights["BaseReward"];
            RewardWeights.Remove("CrashPenalty");
            RewardWeights.Remove("BaseReward");

            keys = RewardWeights.Keys.ToArray();

            foreach (var key in keys)
            {
                Rewards[key] = new List<float>();
                RewardHistory[key] = new List<float>();
            }
        }

        /// <summary>
        /// Calculates weighted rewards from current <c>RawRewards</c>.
        /// </summary>
        /// <remarks>
        /// All raw rewards are high => bad.
        /// </remarks>
        /// <returns>Current step reward</returns>
        public float GetReward()
        {
            var stepReward = 0f;
            foreach (var key in keys)
            {
                var partialReward = Mathf.Exp(-(RawRewards[key]/0.25f)) * RewardWeights[key] * Time.fixedDeltaTime;
                Rewards[key].Add(partialReward);
                stepReward += partialReward;
            }

            stepReward += BaseReward * Time.fixedDeltaTime + (Crashed? CrashPenalty: 0);
            return stepReward;            
        }

    }
}