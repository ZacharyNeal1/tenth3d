using System.Collections.Generic;
using System;

namespace tenth3D
{

    public class Fragment
    {
        public static int count = 0;
        public int ID { get; set; } = 0;
        public Vector4[] RenderData { get; set; } = new Vector4[0];
        public int[] RenderData1 { get; set; } = new int[0];
        public virtual void GenerateWorldPoints(Vector3 camRot) // ment to be overridden
        {

        }
        public virtual void GenerateRenderData(int stride, int index)// ment to be overridden
        {

        }
        public static (Vector3, Vector3) Forward(Vector3 rot)
        {
            var x = Matrix.RotationX(rot.X);
            var y = Matrix.RotationY(rot.Y);
            var z = Matrix.RotationZ(rot.Z);
            Matrix fullMatrix = z * x * y;

            var forward = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitZ, fullMatrix));
            var right = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitX, fullMatrix));
            return (forward, right);
        }
    }
    public class FragmentPoint : Fragment
    {
        public Vector3 point { get; set; } = Vector3.Zero;
        public int radialCount { get; set; } = 10;
        public float radius { get; set; } = 0.2f;
        public Vector4 Color { get; set; } = Vector4.Zero;
        public Vector3[] WorldPoints { get; set; } = new Vector3[0];
        public int[] WorldInts { get; set; } = new int[0];

        public FragmentPoint(Vector3 p)
        {
            ID = count++;
            point = p;
            var rand = new Random(ID + Environment.TickCount);
            Color = rand.NextColor().ToVector4();
        }
        public FragmentPoint(Vector3 p, Vector4 color)
        {
            ID = count++;
            point = p;
            Color = color;
        }
        public override void GenerateWorldPoints(Vector3 camRot)
        {
            var wl = new List<Vector3>();
            var wi = new List<int>();

            (var forward, var right) = Forward(camRot);
            var up = Vector3.Normalize(Vector3.Cross(forward, right));

            wl.Add(point);

            var standard = (Math.PI * 2) / radialCount;
            for (int i = 0; i < radialCount; i++)
            {
                var current = i * standard;
                var x = (float)Math.Cos(current) * radius;
                var y = (float)Math.Sin(current) * radius;

                wl.Add(point + (right * x) + (up * y));
            }
            for (int i = 1; i < radialCount; i++)
            {
                var next = 0;
                //if (i + 1 == radialCount) next = 1;
                //else
                next = i + 1;
                wi.Add(0);
                wi.Add(next);
                wi.Add(i);
            }

            wi.Add(0);
            wi.Add(1);
            wi.Add(radialCount);

            WorldPoints = wl.ToArray();
            WorldInts = wi.ToArray();

        }
        public override void GenerateRenderData(int stride, int index)
        {
            if (RenderData.Length != WorldPoints.Length * stride)
            {
                RenderData = new Vector4[WorldPoints.Length * stride];
                for (int i = 0; i < WorldPoints.Length; i++)
                {
                    RenderData[i * stride] = new Vector4(WorldPoints[i], 1.0f);
                    RenderData[i * stride + 1] = Color;
                    RenderData[i * stride + 2] = Vector4.Zero;
                }
            }
            else
            {
                for (int i = 0; i < WorldPoints.Length; i++)
                {
                    RenderData[i * stride] = new Vector4(WorldPoints[i], 1.0f);
                }
            }
            if (RenderData1.Length != WorldInts.Length) RenderData1 = new int[WorldInts.Length];
            //RenderData1 = WorldInts
            for (int i = 0; i < WorldInts.Length; i++)
            {
                RenderData1[i] = (WorldInts[i] * stride) + index;
            }
        }

    }
    public class FragmentLine : Fragment
    {

        public Vector3[] points { get; set; } = new Vector3[0];
        public Vector3[] WorldPoints { get; set; } = new Vector3[0];
        public int[] WorldInts { get; set; } = new int[0];

        public Vector4 Color { get; set; } = Vector4.Zero;
        public float LineThickness { get; set; } = 0.05f;


        public FragmentLine(Vector3 a, Vector3 b)
        {
            ID = count++;
            points = new Vector3[] { a, b };
            var rand = new Random(ID + Environment.TickCount);
            Color = rand.NextColor().ToVector4();
        }
        public FragmentLine(Vector3 a, Vector3 b, Vector4 color)
        {
            ID = count++;
            points = new Vector3[] { a, b };
            Color = color;
        }
        public override void GenerateWorldPoints(Vector3 camRot)
        {
            if (points.Length < 2) return;

            if (WorldPoints.Length != points.Length * 2) WorldPoints = new Vector3[points.Length * 2];

            var wl = new List<Vector3>();
            var wi = new List<int>();

            var forward = Forward(camRot).Item1;

            for (int i = 0; i < points.Length; i++)
            {
                var last = i - 1; if (i - 1 < 0) last = i + 1;
                var dir = points[last] - points[i];
                var normal = Vector3.Normalize(Vector3.Cross(forward, dir));

                wl.Add(normal * LineThickness + points[i]);
                wl.Add(-normal * LineThickness + points[i]);
            }
            WorldPoints = wl.ToArray();
            for (int i = 0; i < points.Length - 1; i++)
            {

                wi.Add(i * 2);
                wi.Add((i + 1) * 2);
                wi.Add(i * 2 + 1);

                wi.Add(i * 2);
                wi.Add((i + 1) * 2 + 1);
                wi.Add((i + 1) * 2);
            }
            WorldInts = wi.ToArray();
        }
        public override void GenerateRenderData(int stride, int index)
        {
            if (RenderData.Length != WorldPoints.Length * stride)
            {
                RenderData = new Vector4[WorldPoints.Length * stride];
                for (int i = 0; i < WorldPoints.Length; i++)
                {
                    RenderData[i * stride] = new Vector4(WorldPoints[i], 1.0f);
                    RenderData[i * stride + 1] = Color;
                    RenderData[i * stride + 2] = Vector4.Zero;
                }
            }
            else
            {
                for (int i = 0; i < WorldPoints.Length; i++)
                {
                    RenderData[i * stride] = new Vector4(WorldPoints[i], 1.0f);
                }
            }
            if (RenderData1.Length != WorldInts.Length) RenderData1 = new int[WorldInts.Length];
            //RenderData1 = WorldInts
            for (int i = 0; i < WorldInts.Length; i++)
            {
                RenderData1[i] = (WorldInts[i] * stride) + index;
            }

        }
    }
    public class FragmentCircle : Fragment
    {
        public Vector3 point { get; set; } = Vector3.Zero;
        public Vector3[] WorldPoints { get; set; } = new Vector3[0];
        public int[] WorldInts { get; set; } = new int[0];

        public Vector4 Color { get; set; } = Vector4.Zero;
        public int radialCount { get; set; } = 10;
        public float Radius { get; set; } = 0.3f;
        public float LineThickness { get; set; } = 0.3f;
        public float EndingAngle { get; set; } = 2f * (float)Math.PI;
        public float StartingAngle { get; set; } = 0;
        public FragmentLine[] lines { get; set; } = new FragmentLine[0];
        public Vector3 Up { get; set; } = Vector3.Zero;
        public Vector3 Right { get; set; } = Vector3.Zero;

        public FragmentCircle(Vector3 center, Vector3 up, Vector3 right, float starting, float ending, Vector4 color)
        {
            ID = count++;
            point = center;
            Up = up;
            Right = right;
            StartingAngle = starting;
            EndingAngle = ending;
            Color = color;
            GenerateLines();
        }
        public void GenerateLines()
        {
            //wl.Add(point);
            var points = new List<Vector3>();
            var standard = (Math.PI * 2) / radialCount;
            for (int i = 0; i < radialCount + 1; i++)
            {
                var current = i * standard;
                if (current < StartingAngle) current = StartingAngle;
                if (current > EndingAngle) current = EndingAngle;
                var x = (float)Math.Cos(current) * Radius;
                var y = (float)Math.Sin(current) * Radius;

                points.Add(point + ((Right * x) + (Up * y)) * Radius);
            }
            lines = new FragmentLine[radialCount];
            for (int i = 0; i < radialCount; i++)
            {
                lines[i] = new FragmentLine(points[i], points[i + 1], Color) { LineThickness = LineThickness };
            }

        }
        public override void GenerateWorldPoints(Vector3 camRot)
        {
            foreach (FragmentLine fl in lines)
            {
                fl.GenerateWorldPoints(camRot);
            }
        }
        public override void GenerateRenderData(int stride, int index)
        {

            int size1 = 0;
            int size2 = 0;
            foreach (FragmentLine fl in lines)
            {
                fl.GenerateRenderData(stride, index);
                size1 += fl.RenderData.Length;
                size2 += fl.RenderData1.Length;
            }
            RenderData = new Vector4[size1];
            RenderData1 = new int[size2];
            int index1 = 0, index2 = 0;
            foreach (FragmentLine fl in lines)
            {
                Array.Copy(fl.RenderData, 0, RenderData, index1, fl.RenderData.Length);
                index1 += fl.RenderData.Length;
                Array.Copy(fl.RenderData1, 0, RenderData1, index2, fl.RenderData1.Length);
                index2 += fl.RenderData1.Length;
            }

        }



    }
    public class FragmentText
    {

    }
}