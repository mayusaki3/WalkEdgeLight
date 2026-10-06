using System;
namespace WalkEdgeLight.Validation.Sensors
{
    public sealed class DepthFrame
    {
        public DepthFrame(int width,int height,float[] depthMetres,bool[] valid)
        {
            if(width<=0||height<=0) throw new ArgumentOutOfRangeException(nameof(width));
            var count=checked(width*height);
            if(depthMetres.Length!=count||valid.Length!=count) throw new ArgumentException("Depth and validity arrays must match width * height.");
            Width=width; Height=height; DepthMetres=depthMetres; Valid=valid;
        }
        public int Width { get; } public int Height { get; } public float[] DepthMetres { get; } public bool[] Valid { get; }
    }
}
