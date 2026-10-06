using UnityEngine;

namespace WalkEdgeLight.Validation.UnitySimulation
{
public sealed class CpuRaycastPerfectDepthGenerator
{
    public GeneratedDepth Generate(
        Camera camera,
        int width,
        int height,
        float maxRangeMetres = 10f,
        int layerMask = Physics.DefaultRaycastLayers)
    {
        if (camera == null) throw new System.ArgumentNullException(nameof(camera));
        if (width <= 0 || height <= 0) throw new System.ArgumentOutOfRangeException(nameof(width));

        var count = checked(width * height);
        var depth = new float[count];
        var valid = new bool[count];

        for (var v = 0; v < height; ++v)
        {
            for (var u = 0; u < width; ++u)
            {
                // Viewport origin is bottom-left; WalkEdgeLight image origin is top-left.
                var viewportX = (u + 0.5f) / width;
                var viewportY = 1f - (v + 0.5f) / height;
                var ray = camera.ViewportPointToRay(new Vector3(viewportX, viewportY, 0f));
                var index = v * width + u;

                if (!Physics.Raycast(ray, out var hit, maxRangeMetres, layerMask, QueryTriggerInteraction.Ignore))
                    continue;

                // RaycastHit.distance is RANGE, not Z_DEPTH. Convert the world-space
                // intersection back into camera space and store its +Z component.
                var cameraPoint = camera.transform.InverseTransformPoint(hit.point);
                if (cameraPoint.z <= 0f || !(!float.IsNaN(cameraPoint.z) && !float.IsInfinity(cameraPoint.z)))
                    continue;

                depth[index] = cameraPoint.z;
                valid[index] = true;
            }
        }

        return new GeneratedDepth(width, height, depth, valid);
    }
}

public readonly struct GeneratedDepth
{
    public GeneratedDepth(int width, int height, float[] depthMetres, bool[] valid)
    {
        Width = width;
        Height = height;
        DepthMetres = depthMetres;
        Valid = valid;
    }

    public int Width { get; }
    public int Height { get; }
    public float[] DepthMetres { get; }
    public bool[] Valid { get; }
}
}
