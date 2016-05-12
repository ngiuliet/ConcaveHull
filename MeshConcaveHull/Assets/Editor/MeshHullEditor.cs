using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MeshHull))]
public class MeshHullEditor : Editor
{

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        MeshHull myTarget = (MeshHull)target;

        if (GUILayout.Button("Build Hull Collider"))
        {
            myTarget.BuildHullCollider();
        }
    }
}
