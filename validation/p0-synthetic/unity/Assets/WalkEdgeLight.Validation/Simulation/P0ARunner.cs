using System;
using System.Numerics;
using UnityEngine;
using WalkEdgeLight.Validation.Reconstruction;

namespace WalkEdgeLight.Validation.UnitySimulation;

public sealed class P0ARunner : MonoBehaviour
{
    [SerializeField] private SingleStepScene sceneDefinition;
    [SerializeField] private Camera sensorCamera;
    [SerializeField] private int depthWidth = 160;
    [SerializeField] private int depthHeight = 120;
    [SerializeField] private float maxRangeMetres = 10f;

    [ContextMenu("Run P0-A")]
    public void Run()
    {
        if (sceneDefinition == null) throw new InvalidOperationException("SingleStepScene is not assigned.");
        if (sensorCamera == null) throw new InvalidOperationException("Sensor Camera is not assigned.");

        sceneDefinition.Build();
        Physics.SyncTransforms();

        var truth = SingleStepGroundTruth.FromSceneDefinition(sceneDefinition);
        var generator = new CpuRaycastPerfectDepthGenerator();
        var generated = generator.Generate(sensorCamera, depthWidth, depthHeight, maxRangeMetres);
        var frame = P0ASensorFrameAdapter.Create(0, Time.realtimeSinceStartupAsDouble, sensorCamera, generated);

        var validCount = 0;
        var reconstructionResidualSum = 0.0;
        var reconstructionResidualMax = 0.0;

        // P0-A checks round-trip consistency:
        // generated Z-depth -> common SensorFrame -> common point reconstruction.
        for (var v = 0; v < depthHeight; ++v)
        {
            for (var u = 0; u < depthWidth; ++u)
            {
                var index = v * depthWidth + u;
                if (!generated.Valid[index]) continue;

                var world = PointReconstructor.ReconstructWorldPoint(frame, u, v);
                if (world is null) continue;

                var unityPoint = new UnityEngine.Vector3(world.Value.X, world.Value.Y, world.Value.Z);

                // Recast the same pixel ray and compare with the reconstructed point.
                var viewportX = (u + 0.5f) / depthWidth;
                var viewportY = 1f - (v + 0.5f) / depthHeight;
                var ray = sensorCamera.ViewportPointToRay(new UnityEngine.Vector3(viewportX, viewportY, 0f));
                if (!Physics.Raycast(ray, out var hit, maxRangeMetres, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
                    continue;

                var residual = UnityEngine.Vector3.Distance(unityPoint, hit.point);
                reconstructionResidualSum += residual;
                reconstructionResidualMax = Math.Max(reconstructionResidualMax, residual);
                ++validCount;
            }
        }

        var meanResidual = validCount == 0 ? double.NaN : reconstructionResidualSum / validCount;

        Debug.Log(
            $"[WalkEdgeLight P0-A] " +
            $"stepTruth={truth.HeightDifferenceMetres * 1000f:F3} mm, " +
            $"edgeZ={truth.EdgeZMetres:F3} m, " +
            $"validSamples={validCount}, " +
            $"meanReconstructionResidual={meanResidual * 1000.0:F6} mm, " +
            $"maxReconstructionResidual={reconstructionResidualMax * 1000.0:F6} mm");
    }
}
