using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class SceneLoader {

    public enum Scene {
        MainMenuScene,
        StraightWalker,
        TerrainWalker,
        RagdollRunningAgent
    }

    public static void Load(Scene targetScene) {
        SceneManager.LoadScene(targetScene.ToString());
    }
}
