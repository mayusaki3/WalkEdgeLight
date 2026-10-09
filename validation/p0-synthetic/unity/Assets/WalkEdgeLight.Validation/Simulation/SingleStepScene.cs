using UnityEngine;

namespace WalkEdgeLight.Validation.UnitySimulation
{
public sealed class SingleStepScene : MonoBehaviour
{
    [SerializeField] private float stepHeightMetres = 0.020f;
    [SerializeField] private float edgeZMetres = 1.0f;
    [SerializeField] private float widthMetres = 4.0f;
    [SerializeField] private float nearDepthMetres = 2.0f;
    [SerializeField] private float farDepthMetres = 3.0f;

    public float StepHeightMetres => stepHeightMetres;
    public float EdgeZMetres => edgeZMetres;

    public void Build()
    {
        BuildAtEdge(edgeZMetres);
    }

    public void BuildAtEdge(float edgeZ)
    {
        ClearChildren();

        // Camera approaches along +Z. The near surface is the reference floor.
        CreateBox("NearFloor",
            new Vector3(0f, -0.05f, edgeZ * 0.5f),
            new Vector3(widthMetres, 0.1f, edgeZ));

        // P0-A default: a 20 mm downward step after the edge.
        CreateBox("FarFloor",
            new Vector3(0f, -stepHeightMetres - 0.05f, edgeZ + farDepthMetres * 0.5f),
            new Vector3(widthMetres, 0.1f, farDepthMetres));

        // Fill the area behind the camera so the generated geometry is easy to inspect.
        CreateBox("NearExtension",
            new Vector3(0f, -0.05f, -nearDepthMetres * 0.5f),
            new Vector3(widthMetres, 0.1f, nearDepthMetres));
    }

    public void BuildSlanted(float slope)
    {
        ClearChildren();
        // A common yawed frame gives both floor boxes exactly the same edge.
        // Its local X axis follows z = EdgeZMetres + slope * x.
        var frame = new GameObject("SlantedFloorFrame");
        frame.transform.SetParent(transform, false);
        frame.transform.localPosition = new Vector3(0f, 0f, edgeZMetres);
        frame.transform.localRotation = Quaternion.Euler(0f, -Mathf.Atan(slope) * Mathf.Rad2Deg, 0f);

        CreateSlantedBox(frame.transform, "NearFloor",
            new Vector3(0f, -0.05f, -1f),
            new Vector3(widthMetres, 0.1f, 2f));
        CreateSlantedBox(frame.transform, "FarFloor",
            new Vector3(0f, -stepHeightMetres - 0.05f, farDepthMetres * 0.5f),
            new Vector3(widthMetres, 0.1f, farDepthMetres));
    }

    private static void CreateSlantedBox(Transform parent, string name, Vector3 position, Vector3 scale)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name;
        go.transform.SetParent(parent, false);
        go.transform.localPosition = position;
        go.transform.localScale = scale;
    }

    private void CreateBox(string objectName, Vector3 position, Vector3 scale)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = objectName;
        go.transform.SetParent(transform, false);
        go.transform.localPosition = position;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale = scale;
    }

    private void ClearChildren()
    {
        for (var i = transform.childCount - 1; i >= 0; --i)
        {
            var child = transform.GetChild(i).gameObject;
            if (Application.isPlaying) Destroy(child);
            else DestroyImmediate(child);
        }
    }
}
}
