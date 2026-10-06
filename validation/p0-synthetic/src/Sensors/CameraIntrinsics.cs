namespace WalkEdgeLight.Validation.Sensors
{
    public readonly struct CameraIntrinsics
    {
        public CameraIntrinsics(double fx,double fy,double cx,double cy,int width,int height)
        { Fx=fx; Fy=fy; Cx=cx; Cy=cy; Width=width; Height=height; }
        public double Fx { get; } public double Fy { get; } public double Cx { get; } public double Cy { get; }
        public int Width { get; } public int Height { get; }
    }
}
