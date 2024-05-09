using SharpDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using static System.Windows.Forms.AxHost;
using System.Windows.Media.Imaging;
using System.Timers;
using System.Windows.Input;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TextBox;
using System.Security.Cryptography;
using System.Windows.Media.Media3D;
using SharpDX;
using Vector3 = SharpDX.Vector3;
using Vector2 = SharpDX.Vector2;
using System.IO;
using Vector4 = SharpDX.Vector4;
using System.Windows.Forms;
using SharpDX.Windows;
using Timer = System.Timers.Timer;
using System.Windows.Ink;
using System.Windows.Media;
using Matrix = SharpDX.Matrix;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Rebar;

namespace tenth3d
{

    public class ConstantUpdate
    {
        Timer ConstantTimer = new Timer(1000 / 50.0) { AutoReset = true };
        List<BasicObject> WorldObjects = new List<BasicObject>();
        public Vector3 camPos = Vector3.Zero, camRot = Vector3.Zero;
        public float fov = (float)(Math.PI / 2.0f);
        ObjFile data = new ObjFile(@"C:\Users\nealz\Downloads\simple.obj");

        Matrix proj = Matrix.Identity;
        public Matrix worldViewProj = Matrix.Identity;

        public ConstantUpdate()
        {
            ConstantTimer.Elapsed += ConstantUpdateFrame;

            var ob1 = new BasicObject(data);
            WorldObjects.Add(ob1);
            var rand = new Random();
            for (int i = 0; i < 50; i++)
            {
                var o2 = new BasicObject(data);
                o2.Position = new Vector3((float)rand.NextDouble(), (float)rand.NextDouble(), (float)rand.NextDouble()) * 1000f;
                WorldObjects.Add(o2);

            }

            ConstantTimer.Start();
        }
        private void ConstantUpdateFrame(object sender, ElapsedEventArgs e)
        {
            WorldObjects[0].Position = new Vector3(0f, 0f, 70f);
            var rand = new Random();

            for (int i = 0; i < WorldObjects.Count; i++)
            {
               // WorldObjects[i].Position += new Vector3((float)rand.NextDouble(), (float)rand.NextDouble(), (float)rand.NextDouble());
                WorldObjects[i].MeshData.Transform();
            }



        }
        public Matrix GetWorldViewProj(Form form, bool resized)
        {
            if (resized)
                proj = Matrix.PerspectiveFovLH((float)Math.PI / 4.0f, form.ClientSize.Width / (float)form.ClientSize.Height, 0.1f, 10000.0f);
            (var forward, _) = Forward(camRot);
            var view = Matrix.LookAtLH(camPos -forward*5, camPos, Vector3.UnitY);
            var viewProj = Matrix.Multiply(view, proj);
            viewProj.Transpose();
            return viewProj;

        }
        public void Draw(Device device, DeviceContext dc, int stride)
        {
            int len1 = 0;
            int len2 = 0;
            foreach(BasicObject bo in WorldObjects)
            {
                bo.MeshData.GenerateRenderData(stride, len1);
                len1 += bo.MeshData.RenderData.Length;
                len2 += bo.MeshData.RenderData1.Length;
            }
            var full1 = new Vector4[len1];
            var full2 = new int[len2];
            int index1 = 0, index2 = 0;
            for (int i = 0; i < WorldObjects.Count; i++)
            {
                var current = WorldObjects[i].MeshData;
                Array.Copy(current.RenderData, 0, full1, index1, current.RenderData.Length);
                index1 += current.RenderData.Length;
                Array.Copy(current.RenderData1, 0, full2, index2, current.RenderData1.Length);
                index2 += current.RenderData1.Length;

            }
            Program.Draw(full1, full2, device, dc);

            
        }

        /// <summary>
        /// Gets the forward and right vectors of a object based on its rotation (assuming the base forward is unit z)
        /// </summary>
        /// <param name="rot"></param>
        /// <returns>(forward, right)</returns>
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
    //public class PhysicsData
    //{
    //    public Vector3 OldVelocity { get; set; } = Vector3.Zero;
    //    public Vector3 Velocity { get; set; } = Vector3.Zero;
    //    public Vector3 AngularVelocity { get; set; } = Vector3.Zero;
    //    public float VelocityDecay { get; set; } = 0.985f;
    //    public float AngularVelocityDecay { get; set; } = 0.99f;
    //    public float Weight { get; set; } = 100f;
    //    public float Drag { get; set; } = 0.3f;
    //    public float GravityScale { get; set; } = 1f;
    //    public float Bouncyness { get; set; } = 0.2f;
    //    public PhysicsData()
    //    {
    //    }
    //}
    public class MeshData
    {
        public List<Vector3> points { get; set; } = new List<Vector3>();
        public Vector4[] WorldPoints { get; set; } = new Vector4[0];
        public Vector4[] Colors { get; set; } = new Vector4[0];
        public List<int> faces { get; set; } = new List<int>();
        public List<int> facesTex { get; set; } = new List<int>();
        public List<Vector2> texCoord { get; set; } = new List<Vector2>();
        public Vector3 center { get; set; } = Vector3.Zero;
        public BasicObject parent { get; set; }
        public Vector4[] RenderData { get; set; } = new Vector4[0];
        public int[] RenderData1 { get; set; } = new int[0];
        Vector3 Center()
        {
            Vector3 full = Vector3.Zero;
            foreach (Vector3 a in points)
                full += a;
            full /= points.Count;
            return full;
        }
        public MeshData(ObjFile data)
        {

            points = data.points;
            faces = data.faces;
            facesTex = data.facesTex;
            texCoord = data.texCoord;
            center = Center();
            var rand = new Random();
            Colors = new Vector4[data.points.Count];
            for (int i = 0; i < Colors.Length; i++)
            {
                Colors[i] = new Vector4((float)rand.NextDouble(), 0.0f, (float)rand.NextDouble(), 1.0f);
            }
        }
        public void Transform()
        {
            Vector3 rot = parent.Rotation;
            Vector3 scale = parent.Scale;
            Vector3 pos = parent.Position;

            Matrix m;
            Matrix.RotationYawPitchRoll(rot.Y, rot.X, rot.Z, out m);

            if (WorldPoints.Length != points.Count)
                WorldPoints = new Vector4[points.Count];

            for (int i = 0; i < points.Count; i++)
                WorldPoints[i] = new Vector4((Vector3)Vector3.Transform(points[i] - center, m) * scale + pos,1.0f);
        }
        public void GenerateRenderData(int stride, int index)
        {

            if (RenderData.Length != points.Count * stride)
            {
                var pl = new List<Vector4>();

                for (int i = 0; i < WorldPoints.Length; i++)
                {
                    pl.Add(WorldPoints[i]);
                    pl.Add(Colors[i]);
                    if (facesTex[i] == -1)
                        pl.Add(new Vector4(0.0f, 0.0f, 0.0f, 1.0f));
                    else
                    {
                        var vect = texCoord[facesTex[i]];
                        pl.Add(new Vector4(vect.X, vect.Y, 1.0f, 1.0f));
                    }
                }

                RenderData = pl.ToArray();
            }
            else
            { //if render data is already set no need to change the color and tex coord 
                for (int i = 0; i < WorldPoints.Length; i++)
                {
                    RenderData[i * stride] = WorldPoints[i];
                }
            }


            //if (RenderData1.Length != faces.Count)
            //{
                var ti = new List<int>();

                for (int i = 0; i < faces.Count; i++)
                    ti.Add((faces[i] * stride )+ index);

                RenderData1 = ti.ToArray();
            //}


        }
    }
    public class BasicObject
    {
        public static int count = 0;
        public int ID { get; set; }

        // basic object vars
        public Vector3 Position { get; set; } = Vector3.Zero;
        public Vector3 Rotation { get; set; } = Vector3.Zero;
        public Vector3 Scale { get; set; } = Vector3.One;

        public MeshData MeshData { get; set; }
        public BasicObject(ObjFile data)
        {
            ID = count++;
            MeshData = new MeshData(data);
            MeshData.parent = this;
        }
    }
    public class ObjFile
    {
        public List<Vector3> points { get; set; } = new List<Vector3>();
        public List<int> faces { get; set; } = new List<int>();
        public List<int> facesTex { get; set; } = new List<int>();
        public List<Vector2> texCoord { get; set; } = new List<Vector2>();
        string[] lines { get; set; }
        public ObjFile(string path)
        {

            lines = File.ReadAllLines(path);
            //var sections = new List<string[]>();
            string input = "";
            foreach (string s in File.ReadAllLines(path))
            {
                input += s + "\n";
            }

            for (int i = 0; i < lines.Length; i++)
            {
                var str = lines[i];

                if (str.StartsWith("vt "))
                {
                    var val = str.Substring(3);
                    var split = val.Split(' ');
                    texCoord.Add(new Vector2(float.Parse(split[0]), float.Parse(split[1])));
                }
                if (str.StartsWith("v "))
                {
                    var val = str.Substring(2);
                    var split = val.Split(' ');
                    points.Add(new Vector3(float.Parse(split[0]), float.Parse(split[1]), float.Parse(split[2])));
                }
                if (str.StartsWith("f "))
                {

                    var val = str.Substring(2);
                    var split = val.Split(' ');

                    if (!str.Contains("/"))
                    {
                        var f = new int[] { int.Parse(split[0]) - 1, int.Parse(split[1]) - 1, int.Parse(split[2]) - 1 };
                        faces.AddRange(f);
                        facesTex.Add(-1);
                        facesTex.Add(-1);
                        facesTex.Add(-1);
                    }
                    else
                    {
                        var f = new int[]
                        {
                               int.Parse(split[0].Split('/')[0])-1,
                               int.Parse(split[1].Split('/')[0])-1,
                               int.Parse(split[2].Split('/')[0])-1,
                        };
                        faces.AddRange(f);
                        var b = split[0].Split('/');
                        facesTex[int.Parse(b[0]) - 1] = int.Parse(b[1]) - 1;
                        b = split[1].Split('/');
                        facesTex[int.Parse(b[0]) - 1] = int.Parse(b[1]) - 1;
                        b = split[2].Split('/');
                        facesTex[int.Parse(b[0]) - 1] = int.Parse(b[1]) - 1;

                    }
                }

            }

        }


    } //the inital for mesh data and a easy way to decompile obj files made by me :)
}
