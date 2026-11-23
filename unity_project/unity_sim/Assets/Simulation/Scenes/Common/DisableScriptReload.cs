using UnityEngine;

public class DisableScriptReload : MonoBehaviour
{
#if UNITY_EDITOR
    void OnEnable()
    {
        Debug.Log("Disabling automatic script reload during play mode.");
        UnityEditor.EditorApplication.LockReloadAssemblies();
    }

    void OnDisable()
    {
        Debug.Log("Re-enabling automatic script reload since play mode was exited.");
        UnityEditor.EditorApplication.UnlockReloadAssemblies();
    }
#endif
}
