using System;
using UnityEngine;
using WalkEdgeLight.Validation.Reconstruction;

namespace WalkEdgeLight.Validation.UnitySimulation
{
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

        // Height measurement deliberately uses only reconstructed points. Ground truth
        // is used after measurement solely for comparison.
        const float edgeExclusionMetres = 0.10f;
        const float sampleBandMetres = 0.60f;
        double nearYSum = 0.0;
        double farYSum = 0.0;
        var nearCount = 0;
        var farCount = 0;

        for (var v = 0; v < depthHeight; ++v)
        {
            for (var u = 0; u < depthWidth; ++u)
            {
                var index = v * depthWidth + u;
                if (!generated.Valid[index]) continue;

                var world = PointReconstructor.ReconstructWorldPoint(frame, u, v);
                if (!world.HasValue) continue;

                var unityPoint = new UnityEngine.Vector3(world.Value.X, world.Value.Y, world.Value.Z);

                var viewportX = (u + 0.5f) / depthWidth;
                var viewportY = 1f - (v + 0.5f) / depthHeight;
                var ray = sensorCamera.ViewportPointToRay(new UnityEngine.Vector3(viewportX, viewportY, 0f));
                RaycastHit hit;
                if (!Physics.Raycast(ray, out hit, maxRangeMetres, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
                    continue;

                var residual = UnityEngine.Vector3.Distance(unityPoint, hit.point);
                reconstructionResidualSum += residual;
                reconstructionResidualMax = Math.Max(reconstructionResidualMax, residual);
                ++validCount;

                var z = unityPoint.z;
                if (z >= truth.EdgeZMetres - sampleBandMetres &&
                    z <= truth.EdgeZMetres - edgeExclusionMetres)
                {
                    nearYSum += unityPoint.y;
                    ++nearCount;
                }
                else if (z >= truth.EdgeZMetres + edgeExclusionMetres &&
                         z <= truth.EdgeZMetres + sampleBandMetres)
                {
                    farYSum += unityPoint.y;
                    ++farCount;
                }
            }
        }

        var meanResidual = validCount == 0 ? double.NaN : reconstructionResidualSum / validCount;
        var nearY = nearCount == 0 ? double.NaN : nearYSum / nearCount;
        var farY = farCount == 0 ? double.NaN : farYSum / farCount;
        var measuredStep = farY - nearY;
        var heightError = measuredStep - truth.HeightDifferenceMetres;

        Debug.Log(
            $"[WalkEdgeLight P0-A] " +
            $"stepTruth={truth.HeightDifferenceMetres * 1000f:F3} mm, " +
            $"stepMeasured={measuredStep * 1000.0:F6} mm, " +
            $"stepError={heightError * 1000.0:F6} mm, " +
            $"nearSamples={nearCount}, farSamples={farCount}, " +
            $"validSamples={validCount}, " +
            $"meanReconstructionResidual={meanResidual * 1000.0:F6} mm, " +
            $"maxReconstructionResidual={reconstructionResidualMax * 1000.0:F6} mm");
    }
}
}
