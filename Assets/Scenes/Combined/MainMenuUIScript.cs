using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MainMenuUIScript : MonoBehaviour
{

    [SerializeField] private Button RunningAgent;
    [SerializeField] private Button ImitationAgent;
    [SerializeField] private Button TerrainAgent;

    private void Awake() {
        RunningAgent.onClick.AddListener(()=> {
            SceneLoader.Load(SceneLoader.Scene.StraightWalker);
        });

        ImitationAgent.onClick.AddListener(()=> {
            SceneLoader.Load(SceneLoader.Scene.RagdollRunningAgent);
        });

        TerrainAgent.onClick.AddListener(()=> {
            SceneLoader.Load(SceneLoader.Scene.TerrainWalker);
        });

        Time.timeScale = 1f;
    }
}
