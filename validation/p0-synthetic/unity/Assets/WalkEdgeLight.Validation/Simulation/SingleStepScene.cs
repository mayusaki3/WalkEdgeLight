using UnityEngine;

namespace WalkEdgeLight.Validation.UnitySimulation;

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
        ClearChildren();

        // Camera approaches along +Z. The near surface is the reference floor.
        CreateBox("NearFloor",
            new Vector3(0f, -0.05f, edgeZMetres * 0.5f),
            new Vector3(widthMetres, 0.1f, edgeZMetres));

        // P0-A default: a 20 mm downward step after the edge.
        CreateBox("FarFloor",
            new Vector3(0f, -stepHeightMetres - 0.05f, edgeZMetres + farDepthMetres * 0.5f),
            new Vector3(widthMetres, 0.1f, farDepthMetres));

        // Fill the area behind the camera so the generated geometry is easy to inspect.
        CreateBox("NearExtension",
            new Vector3(0f, -0.05f, -nearDepthMetres * 0.5f),
            new Vector3(widthMetres, 0.1f, nearDepthMetres));
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
