using UnityEngine;

namespace WalkEdgeLight.Validation.UnitySimulation
{
public readonly struct SingleStepGroundTruth
{
    public SingleStepGroundTruth(float edgeZMetres, float heightDifferenceMetres)
    {
        EdgeZMetres = edgeZMetres;
        HeightDifferenceMetres = heightDifferenceMetres;
        EdgeDirectionWorld = Vector3.right;
        ReferenceSurfaceNormalWorld = Vector3.up;
    }

    public float EdgeZMetres { get; }
    public float HeightDifferenceMetres { get; }
    public Vector3 EdgeDirectionWorld { get; }
    public Vector3 ReferenceSurfaceNormalWorld { get; }

    public static SingleStepGroundTruth FromSceneDefinition(SingleStepScene scene)
        => new SingleStepGroundTruth(scene.EdgeZMetres, -scene.StepHeightMetres);
}
}
