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
using System.ComponentModel;
using SharpDX.Direct2D1;
using System.Windows;
using Device = SharpDX.Direct3D11.Device;
using DeviceContext = SharpDX.Direct3D11.DeviceContext;
using System.Diagnostics;
using KeyEventArgs = System.Windows.Forms.KeyEventArgs;
using System.Drawing.Drawing2D;
using System.Reflection;
using System.Threading;
using static System.Net.Mime.MediaTypeNames;
using Quaternion = SharpDX.Quaternion;

namespace tenth3d
{

    public class ConstantUpdate
    {
        Timer ConstantTimer = new Timer(1000 / 120.0) { AutoReset = true };
        Stopwatch deltaTimer = new Stopwatch();

        public List<BasicObject> WorldObjects = new List<BasicObject>();
        public List<Fragment> Fragments = new List<Fragment>();

        public Vector3 camPos = Vector3.Zero;
        public Vector3 camRot = Vector3.Zero;

        Collision collision;
        public Input input;

        Matrix proj = Matrix.Identity;

        bool RunCollisions = true;

        public ConstantUpdate() //on start
        {
            ConstantTimer.Elapsed += ConstantUpdateFrame;

            input = new Input();


            { // initalized the physical part of the world
                collision = new Collision(WorldObjects, this);


                string objPath = Path.Combine(Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location), @"Resources/simple3.obj");
                string objPath1 = Path.Combine(Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location), @"Resources/simple2.obj");
                ObjFile data = new ObjFile(objPath);
                ObjFile data1 = new ObjFile(objPath1);

                var ob1 = new BasicObject(data);
                var rand = new Random();
                for (int i = 0; i < 2; i++)
                {
                    var o2 = new BasicObject(data);
                    //o2.Position = new Vector3((float)rand.NextDouble(), (float)rand.NextDouble(), (float)rand.NextDouble()) * 10f;
                    o2.Position = new Vector3(0,i*10f,0);
                    //o2.Rotation = new Vector3((float)rand.NextDouble(), (float)rand.NextDouble(), (float)rand.NextDouble());
                    o2.Scale = new Vector3(0.05f);
                    var p2 = new PhysicsData(data, o2);
                    o2.PhysicsData = p2;
                    p2.Weight = p2.volume;
                    p2.Weight = 50f;
                    WorldObjects.Add(o2);

                }
                for (int i = 0; i < 0; i++)
                {
                    var o2 = new BasicObject(data);
                    o2.Position = new Vector3((float)rand.NextDouble(), (float)rand.NextDouble(), (float)rand.NextDouble()) * 1000f;
                    o2.Rotation = new Vector3((float)rand.NextDouble(), (float)rand.NextDouble(), (float)rand.NextDouble());
                    o2.Scale = new Vector3(0.05f);
                    WorldObjects.Add(o2);

                }
                var o3 = new BasicObject(data1);
                o3.Scale = new Vector3(0.2f, 0.5f, 0.2f);
                o3.Position = new Vector3(0, -20, 0);
                o3.Rotation = new Vector3(90 * ((float)Math.PI / 180), 0, 0);
                var p3 = new PhysicsData(data1, o3);
                o3.PhysicsData = p3;
                p3.Weight = 100f;
                p3.locked = true;
                WorldObjects.Add(o3);
            }

            ConstantTimer.Start();
            deltaTimer.Start();
        }

        public void RenderCheckKeys(float scaledTime)
        {
            (var pos, var rot) = CheckKeys();

            camPos += pos * scaledTime;
            camRot += rot * scaledTime;
        }

        public (Vector3, Vector3) CheckKeys()
        {
            var ks = input.keyStates;

            (var forward, var right) = Forward(camRot);

            var tempPos = Vector3.Zero;

            var tempRot = Vector3.Zero;

            float posSpeed = 30f;
            float rotSpeed = 1f;

            if (ks[0]) tempPos += XYVector(forward);
            if (ks[1]) tempPos += XYVector(-right);
            if (ks[2]) tempPos -= XYVector(forward);
            if (ks[3]) tempPos += XYVector(right);

            if (ks[4]) tempPos.Y += 1;
            if (ks[5]) tempPos.Y -= 1;

            if (ks[6]) tempRot.X += 1;
            if (ks[7]) tempRot.Y -= 1;
            if (ks[8]) tempRot.X -= 1;
            if (ks[9]) tempRot.Y += 1;


            if (ks[10]) RunCollisions = true;
            if (ks[11]) RunCollisions = false;

            return (posSpeed * Vector3.Normalize(tempPos), rotSpeed * tempRot);
        }
        public void KeyUpCallBack(Keys input)
        {
            if (input == Keys.R)
            {

            }
        }

        private void ConstantUpdateFrame(object sender, ElapsedEventArgs e)
        {
            var rand = new Random();
            //KeyCheck();

            //CheckKeys();
            if (RunCollisions)
            {
                CollisionUpdate();
            }
            for (int i = 0; i < WorldObjects.Count; i++)
                WorldObjects[i].MeshData.Transform();

        } // on collision update
        public void CollisionUpdate()
        {
            collision.CollisionDetection((float)ConstantTimer.Interval / 1000f);
        }

        public Matrix GetWorldViewProj(Form form, bool resized)
        {


            if (resized)
                proj = Matrix.PerspectiveFovLH((float)Math.PI / 2.0f, form.ClientSize.Width / (float)form.ClientSize.Height, 0.1f, 10000.0f);
            (var forward, _) = Forward(camRot);
            var view = Matrix.LookAtLH(camPos - forward, camPos, Vector3.UnitY);
            var viewProj = Matrix.Multiply(view, proj);
            viewProj.Transpose();
            return viewProj;

        } // on draw frame
        public (Vector4[], int[]) DrawFragments(int stride, Fragment[] frgs)
        {
            int len1 = 0;
            int len2 = 0;

            foreach (Fragment f in frgs)
            {
                if (f != null)
                {
                    f.GenerateWorldPoints(camRot);
                    f.GenerateRenderData(stride, len1);
                    len1 += f.RenderData.Length;
                    len2 += f.RenderData1.Length;
                }
            }
            var full1 = new Vector4[len1];
            var full2 = new int[len2];
            int index1 = 0, index2 = 0;
            foreach (Fragment f in frgs)
            {
                var current = f;

                if (current != null)
                {
                    Array.Copy(current.RenderData, 0, full1, index1, current.RenderData.Length);
                    index1 += current.RenderData.Length;
                    Array.Copy(current.RenderData1, 0, full2, index2, current.RenderData1.Length);
                    index2 += current.RenderData1.Length;
                }

            }
            return (full1, full2);
        }
        public void Draw(Device device, DeviceContext dc, RenderTargetView rt, int stride)
        {
            (var obs, var frgs) = UpdateLists();
            if (WorldObjects.Count < 1) return;

            int len1 = 0;
            int len2 = 0;
            foreach (BasicObject bo in obs)
            {
                bo.MeshData.GenerateRenderData(stride, len1);
                len1 += bo.MeshData.RenderData.Length;
                len2 += bo.MeshData.RenderData1.Length;
            }

            var full1 = new Vector4[len1];
            var full2 = new int[len2];
            int index1 = 0, index2 = 0;
            foreach (BasicObject bo in obs)
            {
                var current = bo.MeshData;
                Array.Copy(current.RenderData, 0, full1, index1, current.RenderData.Length);
                index1 += current.RenderData.Length;
                Array.Copy(current.RenderData1, 0, full2, index2, current.RenderData1.Length);
                index2 += current.RenderData1.Length;
            }

            Program.Draw(full1, full2, device, dc, rt);

            if (true)
                if (frgs.Length > 0)
                {
                    var tuple = DrawFragments(stride, frgs);
                    Program.Draw(tuple.Item1, tuple.Item2, device, dc, rt, true);
                }
            //Fragments.AddRange(PendingFragments);
            //WorldObjects.AddRange(PendingWorldObjects);
            //PendingFragments.Clear();
            //PendingWorldObjects.Clear();



            RenderCheckKeys((float)deltaTimer.ElapsedMilliseconds / 1000f);
            deltaTimer.Restart();


        } // on draw frame
        (BasicObject[], Fragment[]) UpdateLists()
        {
            return (WorldObjects.ToArray(), Fragments.ToArray());
        }


        //functions
        static (Vector3, Vector3) Forward(Vector3 rot)
        {
            var x = Matrix.RotationX(rot.X);
            var y = Matrix.RotationY(rot.Y);
            var z = Matrix.RotationZ(rot.Z);
            Matrix fullMatrix = z * x * y;

            var forward = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitZ, fullMatrix));
            var right = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitX, fullMatrix));
            return (forward, right);
        }
        public static Vector3 XYVector(Vector3 vect)
        {
            return Vector3.Normalize(new Vector3(vect.X, 0, vect.Z));
        }
    }
    public class Fragment
    {
        public static int count = 0;
        public int ID { get; set; } = 0;
        public Vector4[] RenderData { get; set; } = new Vector4[0];
        public int[] RenderData1 { get; set; } = new int[0];
        public virtual void GenerateWorldPoints(Vector3 camRot)
        {

        }
        public virtual void GenerateRenderData(int stride, int index)
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
    public class FragmentText {
    
    }

    public class Input
    {
        Key[] keys;
        public bool[] keyStates;

        public Input()
        {
            keys = new Key[]
            {
                Key.W,
                Key.A,
                Key.S,
                Key.D,

                Key.E,
                Key.Q,

                Key.Up,
                Key.Left,
                Key.Down,
                Key.Right,

                Key.D1,
                Key.D2,
            };

            keyStates = new bool[keys.Length];
        }

        public int len()
        {
            return keys.Length;
        }

        public Key[] GetKeys()
        {
            return keys;
        }

        public void SetStates(bool[] ks)
        {
            keyStates = ks;
        }
    }

    public class Collision
    {
        public float gravity { get; set; } = 9.8f;
        public List<BasicObject> objs { get; set; }
        public List<PhysicsData> pds { get; set; }

        public bool Running { get; set; } = false;

        public ConstantUpdate cu { get; set; }
        public Collision(List<BasicObject> ObjectList, ConstantUpdate c)
        {

            cu = c;
            objs = ObjectList;

            pds = new List<PhysicsData>();
            foreach (BasicObject obj in objs)
            {
                if ((obj != null) && (obj.PhysicsData != null))
                {
                    pds.Add(obj.PhysicsData);
                }
            }
        }

        public void UpdateList()
        {
            pds.Clear();
            foreach (BasicObject obj in objs)
            {
                if ((obj != null) && (obj.PhysicsData != null))
                {
                    pds.Add(obj.PhysicsData);
                    obj.PhysicsData.GenerateWorldPoints();
                }
            }
        }
        public void ApplyGravity(float oneOver, float scaledTime)
        {
            for (int i = 0; i < pds.Count; i++)
            {
                var pd0 = pds[i];

                if (!pd0.locked)
                {
                    pd0.Velocity += new Vector3(0, -gravity, 0) * scaledTime * oneOver;

                }
            } //apply forces
        }
        public void ChangePositions(float oneOver, float scaledTime)
        {

            for (int i = 0; i < pds.Count; i++)
            {
                var pd0 = pds[i];
                if (pd0.locked == true)
                {
                    pd0.Velocity = Vector3.Zero;
                    pd0.AngularVelocity = Vector3.Zero;

                }
                else
                {
                    var p0 = pd0.parent;

                    p0.Position += pd0.Velocity * scaledTime * oneOver;
                    pd0.Velocity *= 1 - ((1 - pd0.VelocityDecay) * scaledTime * oneOver);

                    p0.Rotation += pd0.AngularVelocity * scaledTime;
                    pd0.AngularVelocity *= 1 - ((1 - pd0.AngularVelocityDecay) * scaledTime * oneOver);

                    p0.Rotation = ScaledRotation(p0.Rotation);

                    pd0.GenerateWorldPoints();
                }
            } //update positions and points and veloicties
        }
        public void ApplyCollisionForces()
        {
            for (int i = 0; i < pds.Count; i++)
            {
                var pd0 = pds[i];
                pd0.Velocity += pd0.NextVelocity;
                pd0.AngularVelocity += pd0.NextAngularVelocity;
                pd0.NextVelocity = Vector3.Zero;
                pd0.NextAngularVelocity = Vector3.Zero;
            } //apply collision forces
        }
        public void CollisionDetection(float scaledTime)
        {
            if (Running) return;
            Running = true;
            UpdateList();

            List<Fragment> tempFrags = new List<Fragment>();

            for (int c = 0; c < 10; c++)
            {
                var oneOver = 1f / 10f;

                ApplyGravity(oneOver, scaledTime);

                ChangePositions(oneOver, scaledTime);







                for (int a = 0; a < pds.Count; a++)
                {
                    var pd0 = pds[a];
                    var p0 = pd0.parent;
                    for (int i = 0; i < pds.Count; i++)
                    {
                        var pd1 = pds[i];
                        var p1 = pd1.parent;

                        if (a == i || p0.ID == p1.ID) continue;

                        Vector3 contact, normal;
                        float depth;

                        bool areColliding = GJK_EPA_BCP.CheckIntersection(pd0.worldMesh, pd1.worldMesh, out contact, out depth, out normal);

                        if (!areColliding) continue;



                        normal = -Vector3.Normalize(normal);

                        float imp = Impulse(normal, pd0, pd1, contact);
                        Vector3 force = imp * normal; //base impulse caculation

                        var jnM = force / pd0.Weight;
                        var col = normal * depth;

                        var rajn = Vector3.Cross(contact - pd0.worldCenter, force);
                        var im1 = Vector3.Transform(rajn, Matrix3x3.Invert(pd0.Tensor)); //angular impulse caculation


                        pd0.Velocity += jnM + col; //impulse
                        if (!pd0.locked) p0.Position += col; //disallow interpentraion

                        //imp = Impulse(normal, pd0, pd1, contact);
                        //force = imp * normal;


                        pd0.AngularVelocity += im1 * depth;//impulse * disallow overlap
                        //random rotation is happening due to the upwards velocity

                        //var rel = VelocityAtPoint(pd0, contact) - VelocityAtPoint(pd1, contact);
                        //rel = pd0.Velocity - pd1.Velocity;
                        //var normalForce = Vector3.Dot(rel, normal) * normal;
                        //var tangentForce = rel - normalForce;

                        //if (tangentForce.Length() < normalForce.Length() * pd0.StaticFriction)
                        //    pd0.NextVelocity -= tangentForce;
                        //else
                        //    pd0.NextVelocity -= tangentForce * pd0.KeneticFriction;

                        //var angRel = pd0.AngularVelocity - pd1.AngularVelocity;
                        //var normalAngle = Vector3.Dot(angRel, normal);
                        //var tangentAngle = angRel - (normalAngle * normal);
                        //var angT = tangentAngle.Normalize();
                        if (!pd0.locked)
                        {
                            tempFrags.Add(new FragmentPoint(contact, Vector4.One));
                            tempFrags.Add(new FragmentLine(pd0.worldCenter, pd0.worldCenter + (Vector3)Vector3.Transform(pd0.AngularVelocity,pd0.RotationMatrix), new Vector4(0, 1, 0, 1)));
                            tempFrags.Add(new FragmentLine(pd0.worldCenter, pd0.worldCenter + pd0.Velocity, new Vector4(1, 0, 0, 1)));
                        }
                    }
                } // collision forces

                ApplyCollisionForces();
            }

            //Thread.MemoryBarrier();
            cu.Fragments = tempFrags;

            Running = false;
        }
        public static Vector3[] Add(Vector3[] vs, Vector3 v)
        {
            for (int i = 0; i < vs.Length; i++) vs[i] += v;
            return vs;
        }
        static Vector3 ScaledRotation(Vector3 rotation)
        {
            var s = (float)Math.PI * 2f;
            return new Vector3(rotation.X % s, rotation.Y % s, rotation.Z % s);
        }
        Vector3 VelocityAtPoint(PhysicsData a, Vector3 b)
        {
            return a.Velocity + Vector3.Cross(a.AngularVelocity, b - a.worldCenter);
        }
        float Impulse(Vector3 normal, PhysicsData a, PhysicsData b, Vector3 point)
        {
            Vector3 padot = VelocityAtPoint(a, point),
                    pbdot = VelocityAtPoint(b, point),
                    ra = point - a.worldCenter,
                    rb = point - b.worldCenter;

            if (b.locked) pbdot = Vector3.Zero;
            if (a.locked) padot = Vector3.Zero;

            Matrix3x3 InvTensorA = Matrix3x3.Invert(a.Tensor);
            if (a.locked) InvTensorA = Matrix3x3.Zero;

            Matrix3x3 InvTensorB = Matrix3x3.Invert(b.Tensor);
            if (b.locked) InvTensorB = Matrix3x3.Zero;

            float vrel = Vector3.Dot(normal, (padot - pbdot)),
                  num = -(1 + a.Elasticity) * vrel;




            float term1 = 1 / a.Weight,
                  term2 = 1 / b.Weight,
                  term3 = Vector3.Dot(normal, Vector3.Cross(Vector3.Transform(Vector3.Cross(ra, normal), InvTensorA), ra)),
                  term4 = Vector3.Dot(normal, Vector3.Cross(Vector3.Transform(Vector3.Cross(rb, normal), InvTensorB), rb));
            if (a.locked) term1 = 0;
            if (b.locked) term2 = 0;

            float j = (num) / (term1 + term2 + term3 + term4);

            return j;

        }
    }
    public class PhysicsData // collisions are NOT rigid bodies
    {
        public static int count = 0;
        public int ID { get; }
        public Vector3 NextVelocity { get; set; } = Vector3.Zero;
        public Vector3 NextAngularVelocity { get; set; } = Vector3.Zero;
        public Vector3 Velocity { get; set; } = Vector3.Zero;
        public Vector3 AngularVelocity { get; set; } = Vector3.Zero;
        public float VelocityDecay { get; set; } = 0.985f;
        public float AngularVelocityDecay { get; set; } = 0.98f;
        public float Elasticity { get; set; } = 0.3f;
        public float StaticFriction { get; set; } = 0.6f;
        public float KeneticFriction { get; set; } = 0.3f;
        public float RollingFriction { get; set; } = 0.3f;
        public float SpinningFriction { get; set; } = 0.6f;


        public bool locked { get; set; } = false;


        public BasicObject parent { get; set; }

        /// <summary>
        /// the farest distance for this object to collide with another (the distance of the farest point from the center)
        /// </summary>
        public float initalDistance { get; set; } = 0;

        public Matrix3x3 Tensor { get; set; } = Matrix3x3.Zero;
        public Matrix RotationMatrix { get; set; } = Matrix.Identity;
      
        public float volume { get; set; } = 0;

        public Vector3 localCenter { get; set; } = Vector3.Zero;
        public Vector3 worldCenter { get; set; } = Vector3.Zero;
        public Vector3[] baseMesh { get; set; } = new Vector3[0];
        public int[] intMesh { get; set; } = new int[0];
        public Vector3[] worldMesh { get; set; } = new Vector3[0];

        public Vector3 forward { get; set; } = Vector3.Zero;
        public Vector3 right { get; set; } = Vector3.Zero;

        public float Weight { get; set; } = 1;

        public PhysicsData(ObjFile of, BasicObject parent1)
        {
            ID = count++;
            baseMesh = of.points.ToArray();
            intMesh = of.faces.ToArray();
            parent = parent1;

            Intialize();
        }
        public void Intialize()
        {
            GenerateLocalCenter();
            GenerateTensor();
            GenerateWorldPoints();
            GenerateVolume();

        }
        public void GenerateVolume()
        {
            volume = 0;
            for (int i = 0; i < intMesh.Length / 3; i++)
            {
                var a = worldMesh[intMesh[i * 3]];
                var b = worldMesh[intMesh[i * 3 + 1]];
                var c = worldMesh[intMesh[i * 3 + 2]];
                var d = worldCenter;

                volume += Math.Abs(Vector3.Dot(a - d, Vector3.Cross((b - d), (c - d)))) / 6f;
            }
        }
        public void GenerateTensor()
        {


            var mk = Weight / (float)baseMesh.Length;

            var ixx = 0f;
            var iyy = 0f;
            var izz = 0f;

            var ps = baseMesh;
            GenerateLocalCenter();

            for (int i = 0; i < ps.Length; i++)
                ixx += mk * (P(ps[i].Y) + P(ps[i].Z));
            for (int i = 0; i < ps.Length; i++)
                iyy += mk * (P(ps[i].X) + P(ps[i].Z));
            for (int i = 0; i < ps.Length; i++)
                izz += mk * (P(ps[i].X) + P(ps[i].Y));

            var ixy = 0f;
            var ixz = 0f;
            var iyz = 0f;

            for (int i = 0; i < ps.Length; i++)
                ixy += mk * ps[i].X * ps[i].Y;
            for (int i = 0; i < ps.Length; i++)
                ixz += mk * ps[i].X * ps[i].Z;
            for (int i = 0; i < ps.Length; i++)
                iyz += mk * ps[i].Y * ps[i].Z;

            ixy = -ixy;
            ixz = -ixz;
            iyz = -iyz;

            var length = ixx + iyy + izz + ixy + ixy + ixz + ixz + iyz + iyz;

            ixx /= length;
            iyy /= length;
            izz /= length;
            ixy /= length;
            ixz /= length;
            iyz /= length;

            int round = 2;

            ixx = (float)Math.Round(ixx, round);
            iyy = (float)Math.Round(iyy, round);
            izz = (float)Math.Round(izz, round);
            ixy = (float)Math.Round(ixy, round);
            ixz = (float)Math.Round(ixz, round);
            iyz = (float)Math.Round(iyz, round);

            Tensor = new Matrix3x3(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz);

        }
        float P(float p)
        {
            return p * p;
        }
        public void GenerateWorldCenter()
        {
            Vector3 full = Vector3.Zero;
            foreach (Vector3 v in worldMesh) full += v;
            full = full / worldMesh.Length;
            worldCenter = full;
        }
        public void GenerateLocalCenter()
        {
            Vector3 full = Vector3.Zero;
            foreach (Vector3 v in baseMesh) full += v;
            full /= baseMesh.Length;
            localCenter = full;
        }
        public void GenerateWorldPoints()
        {
            Vector3 rot = parent.Rotation;
            Vector3 scale = parent.Scale;
            Vector3 pos = parent.Position;

            Matrix m;
            Matrix.RotationYawPitchRoll(rot.Y, rot.X, rot.Z, out m);

            RotationMatrix = m;

            if (worldMesh.Length != baseMesh.Length)
                worldMesh = new Vector3[baseMesh.Length];

            for (int i = 0; i < baseMesh.Length; i++)
            {
                worldMesh[i] = ((Vector3)Vector3.Transform(baseMesh[i] - localCenter, m) * scale) + pos;
                initalDistance = Math.Max(Vector3.Distance(worldMesh[i], worldCenter), initalDistance);
            }
            forward = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitZ, m));
            right = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitX, m));

            GenerateWorldCenter();

        }
    }
    public class MeshData
    {
        public static int count = 0;
        public int ID { get; set; } = 0;
        public List<Vector3> points { get; set; } = new List<Vector3>();
        public Vector4[] WorldPoints { get; set; } = new Vector4[0];
        public Vector4[] Colors { get; set; } = new Vector4[0];
        public List<int> faces { get; set; } = new List<int>();
        public List<int> facesTex { get; set; } = new List<int>();
        public List<Vector2> texCoord { get; set; } = new List<Vector2>();
        /// <summary>
        /// local space
        /// </summary>
        public Vector3 center { get; set; } = Vector3.Zero;
        public BasicObject parent { get; set; }
        public Vector4[] RenderData { get; set; } = new Vector4[0];
        public int[] RenderData1 { get; set; } = new int[0];


        public bool changed { get; set; } = true;
        public bool JustPosition { get; set; } = false;
        Vector3 Center()
        {
            Vector3 full = Vector3.Zero;
            foreach (Vector3 a in points)
                full += a;
            full /= points.Count;
            return full;
        }
        public void SetColor(Vector4 col)
        {
            changed = true;
            for (int i = 0; i < Colors.Length; i++)
            {
                Colors[i] = col;
            }
        }
        public MeshData(ObjFile data)
        {
            ID = count++;
            points = data.points;
            faces = data.faces;
            facesTex = data.facesTex;
            texCoord = data.texCoord;
            center = Center();
            var rand = new Random(ID + Environment.TickCount);
            Colors = new Vector4[data.points.Count];
            for (int i = 0; i < Colors.Length; i++)
            {
                Colors[i] = new Vector4((float)rand.NextDouble(), (float)rand.NextDouble(), (float)rand.NextDouble(), 1f);
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
                WorldPoints[i] = new Vector4(((Vector3)Vector3.Transform(points[i] - center, m) * scale) + pos, 1.0f);
            JustPosition = true;
        }
        public void GenerateRenderData(int stride, int index)
        {

            if (changed) // if things such as color and texcoord was changed
            {
                if (RenderData.Length != points.Count * stride) // if the array isnt big enough
                {
                    var pl = new List<Vector4>(RenderData.Length * stride); // start

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

                    RenderData = pl.ToArray(); // end
                }
                else
                {
                    for (int i = 0; i < WorldPoints.Length; i++) //start
                    {
                        RenderData[i * stride] = WorldPoints[i];
                        RenderData[i * stride + 1] = Colors[i];
                        if (facesTex[i] == -1)
                            RenderData[i * stride + 2] = new Vector4(0.0f, 0.0f, 0.0f, 1.0f);
                        else
                        {
                            var vect = texCoord[facesTex[i]];
                            RenderData[i * stride + 2] = new Vector4(vect.X, vect.Y, 1.0f, 1.0f);
                        }
                    } // end
                }
            }
            else
            {
                if (RenderData.Length != points.Count * stride)
                    RenderData = new Vector4[points.Count * stride];
                if (JustPosition)
                {
                    for (int i = 0; i < WorldPoints.Length; i++) //start
                        RenderData[i * stride] = WorldPoints[i];//end
                }
            }

            if (RenderData1.Length != faces.Count)
            {
                var ti = new List<int>(faces.Count);

                for (int i = 0; i < faces.Count; i++)
                    ti.Add((faces[i] * stride) + index);

                RenderData1 = ti.ToArray();
            }
            else
            {
                if (changed)
                    for (int i = 0; i < faces.Count; i++)
                        RenderData1[i] = (faces[i] * stride) + index;
            }
            changed = false;


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
        public PhysicsData PhysicsData { get; set; }

        public BasicObject(ObjFile data)
        {
            ID = count++;
            MeshData = new MeshData(data);
            MeshData.parent = this;
        }
    } //super basic class (hence the name) where everything else is stored
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
