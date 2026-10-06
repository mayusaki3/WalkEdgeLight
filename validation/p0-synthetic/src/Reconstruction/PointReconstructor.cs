using System;
using System.Numerics;
using WalkEdgeLight.Validation.Sensors;
namespace WalkEdgeLight.Validation.Reconstruction
{
    public static class PointReconstructor
    {
        public static Vector3? ReconstructCameraPoint(SensorFrame frame,int u,int v)
        {
            var i=frame.Intrinsics;
            if((uint)u>=(uint)i.Width||(uint)v>=(uint)i.Height) throw new ArgumentOutOfRangeException();
            if(frame.Depth.Width!=i.Width||frame.Depth.Height!=i.Height) throw new InvalidOperationException("Depth dimensions must match camera intrinsics in P0-A.");
            var index=v*i.Width+u; if(!frame.Depth.Valid[index]) return null;
            var z=frame.Depth.DepthMetres[index]; if(float.IsNaN(z)||float.IsInfinity(z)||z<=0) return null;
            var x=((u+0.5)-i.Cx)/i.Fx*z; var y=-((v+0.5)-i.Cy)/i.Fy*z;
            return new Vector3((float)x,(float)y,z);
        }
        public static Vector3? ReconstructWorldPoint(SensorFrame frame,int u,int v)
        {
            var p=ReconstructCameraPoint(frame,u,v); return p.HasValue?frame.CameraToWorld.TransformPoint(p.Value):(Vector3?)null;
        }
    }
}
