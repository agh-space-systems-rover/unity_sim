using System;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

[InitializeOnLoad]
public static class AutoPlayOnStartup
{
    private static bool armed;
    private static int settleFrames;
    private static bool maximizeAfterPlay;

    static AutoPlayOnStartup()
    {
        var args = Environment.GetCommandLineArgs();
        if (!args.Contains("-autoplay"))
            return;

        armed = true;
        maximizeAfterPlay = true;

        EditorSceneManager.sceneOpened += OnSceneOpened;
        EditorApplication.update += OnEditorUpdate;
        EditorApplication.playModeStateChanged += OnPlayModeChanged;
    }

    private static void OnSceneOpened(Scene scene, OpenSceneMode mode)
    {
        if (!armed)
            return;

        settleFrames = 30;
    }

    private static void OnEditorUpdate()
    {
        if (!armed || settleFrames <= 0)
            return;

        var scene = SceneManager.GetActiveScene();
        if (!scene.IsValid() || !scene.isLoaded)
            return;

        if (EditorApplication.isUpdating)
            return;

        if (EditorApplication.isCompiling)
            return;

        if (Lightmapping.isRunning)
            return;

        settleFrames--;

        if (settleFrames > 0)
            return;

        armed = false;
        EditorSceneManager.sceneOpened -= OnSceneOpened;
        EditorApplication.update -= OnEditorUpdate;

        if (!EditorApplication.isPlayingOrWillChangePlaymode)
            EditorApplication.isPlaying = true;
    }

    private static void OnPlayModeChanged(PlayModeStateChange state)
    {
        if (!maximizeAfterPlay)
            return;

        if (state != PlayModeStateChange.EnteredPlayMode)
            return;

        maximizeAfterPlay = false;

        // Wait one more tick so the Game view exists and has focusable UI state.
        EditorApplication.delayCall += MaximizeGameView;
    }

    private static void MaximizeGameView()
    {
        var gameViewType = Type.GetType("UnityEditor.GameView,UnityEditor");
        if (gameViewType == null)
            return;

        var gameView = EditorWindow.GetWindow(gameViewType);
        if (gameView == null)
            return;

        gameView.Focus();
        gameView.maximized = true;
    }
}
