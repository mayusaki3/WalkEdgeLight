using System.Numerics;
using UnityEngine;
using WalkEdgeLight.Validation.Sensors;
using NumericsQuaternion = System.Numerics.Quaternion;
using NumericsVector3 = System.Numerics.Vector3;

namespace WalkEdgeLight.Validation.UnitySimulation
{
public static class P0ASensorFrameAdapter
{
    public static SensorFrame Create(
        long frameId,
        double timestampSeconds,
        Camera camera,
        GeneratedDepth generated)
    {
        if (camera == null) throw new System.ArgumentNullException(nameof(camera));

        var intrinsics = CreateIntrinsics(camera, generated.Width, generated.Height);
        var position = camera.transform.position;
        var rotation = camera.transform.rotation;

        return new SensorFrame(
            frameId,
            timestampSeconds,
            intrinsics,
            new Pose3(
                new NumericsVector3(position.x, position.y, position.z),
                new NumericsQuaternion(rotation.x, rotation.y, rotation.z, rotation.w)),
            new DepthFrame(
                generated.Width,
                generated.Height,
                generated.DepthMetres,
                generated.Valid));
    }

    private static CameraIntrinsics CreateIntrinsics(Camera camera, int width, int height)
    {
        // Unity vertical FOV -> pinhole intrinsics at the synthetic depth resolution.
        // The synthetic image aspect may differ from the Camera/Game View aspect.
        // ViewportPointToRay uses camera.aspect, so fx must compensate for it.
        var fy = 0.5 * height / System.Math.Tan(camera.fieldOfView * Mathf.Deg2Rad * 0.5);
        var fx = fy * ((double)width / height) / camera.aspect;

        // Pixel-center convention: image center lies between the central pixels.
        var cx = width * 0.5;
        var cy = height * 0.5;

        return new CameraIntrinsics(fx, fy, cx, cy, width, height);
    }
}
}
