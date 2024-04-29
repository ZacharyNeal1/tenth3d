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

namespace tenth3d
{
    public class ConstantUpdate
    {
        Timer ConstantTimer = new Timer(1000/50.0);

        ConstantUpdate()
        {
            ConstantTimer.Elapsed += ConstantUpdateFrame;
        }

        private void ConstantUpdateFrame(object sender, ElapsedEventArgs e)
        {
            
        }
        
    }
    public class MainGame
    {



    }
    //public class Collider //will be used for raycasts
    //{
    //    public static int count = 0;
    //    public int ID { get; set; }
    //    public Vector3 Offset { get; set; }
    //    public Vector3 Scale { get; set; }
    //    public Vector3 Rotation { get; set; }


    //    public float initalDistance { get; set; }//if the distance between two objects is greater than this it ignores the collison between
    //    public Vector3[] points { get; set; }

    //    public List<Touch> touchingColliders { get; set; }
    //    public Vector3[] basePoints { get; set; }
    //    public float staticFriction { get; set; } = 0.5f;
    //    public float keneticFriction { get; set; } = 0.3f;
    //    public Vector3[] TransformAll(Matrix4x4 m)
    //    {
    //        Vector3[] transformed = points;
    //        for (int i = 0; i < points.Length; i++)
    //        {
    //            transformed[i] = Vector3.Transform(points[i], m);
    //        }
    //        return transformed;
    //    }

    //    public void GeneratePoints()
    //    {
    //        points = new Vector3[]
    //        {
    //            new Vector3(size.X, size.Y, size.Z) ,
    //            new Vector3(-size.X, size.Y, size.Z) ,
    //            new Vector3(-size.X, size.Y, -size.Z),
    //            new Vector3(size.X, size.Y, -size.Z) ,
    //            new Vector3(size.X, -size.Y, size.Z) ,
    //            new Vector3(-size.X, -size.Y, size.Z),
    //            new Vector3(-size.X, -size.Y, -size.Z),
    //            new Vector3(size.X, -size.Y, -size.Z)
    //        };
    //        basePoints = points;
    //        for (int i = 0; i < points.Length; i++)
    //        {
    //            points[i] *= scale;
    //            points[i] += position;
    //        }
    //        Matrix4x4 m = Matrix4x4.CreateFromYawPitchRoll(rotation.Y, rotation.X, rotation.Z);
    //        for (int i = 0; i < points.Length; i++)
    //        {
    //            Vector3 clone = points[i];
    //            points[i] = Vector3.TransformNormal(clone - position, m) * scale + position;
    //        }
    //        initalDistance = Vector3.Distance(position, points[0]);
    //    }
    //    public Collider(Vector3 position, Vector3 scale, Vector3 size)
    //    {
    //        ID = count++;
    //        this.position = position;
    //        this.scale = scale;
    //        this.size = size;
    //        initalDistance = Vector3.Distance(position, new Vector3(size.X, size.Y, size.Z) * scale);
    //        touchingColliders = new List<Touch>();


    //    }
    //}
    //public class Touch
    //{
    //    public Collider bodyA { get; set; }
    //    public Collider bodyB { get; set; }
    //    public Vector3 point { get; set; }
    //    public Vector3 normal { get; set; }
    //    public Vector3 secondPoint { get; set; }
    //    public Touch(Collider a, Collider b, Vector3 c, Vector3 d)
    //    {
    //        bodyA = a;
    //        bodyB = b;
    //        point = c;
    //        normal = d;
    //    }
    //    public Touch(Collider a, Collider b, Vector3 pointOfContact, Vector3 normalVector, Vector3 point2)
    //    {
    //        bodyA = a;
    //        bodyB = b;
    //        point = pointOfContact;
    //        normal = normalVector;
    //        secondPoint = point2;
    //    }
    //    public Touch(Collider a, Collider b)
    //    {
    //        bodyA = a;
    //        bodyB = b;
    //    }
    //    public Touch(Touch t)
    //    {
    //        this.bodyA = t.bodyA;
    //        this.bodyB = t.bodyB;
    //        this.point = t.point;
    //        this.normal = t.normal;
    //    }
    //    public override bool Equals(object? a)
    //    {
    //        if (a != null)
    //        {
    //            var b = a as Touch;
    //            if ((bodyA.ID == b.bodyA.ID && bodyB.ID == b.bodyB.ID) || (bodyA.ID == b.bodyB.ID && bodyB.ID == b.bodyA.ID))
    //            {
    //                return true;
    //            }
    //            else return false;
    //        }
    //        return false;
    //    }
    //}
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
    public class BasicObject
    {
        public static int count = 0;
        public int ID { get; set; }

        // basic object vars
        public Vector3 Position { get; set; } = Vector3.Zero;
        public Vector3 Rotation { get; set; } = Vector3.Zero;
        public Vector3 Scale { get; set; } = Vector3.One;
        public Vector3 Center { get; set; } = Vector3.Zero;
        public Vector3[] WorldPoints { get; set; }



        // the below vars are used for the behind the scenes of rotating and such
        private Vector3[] BasePoints { get; set; }
        private int[] FaceData { get; set; }


        // the below vars are used for extra infomation that can be given durring runtime
        //public PhysicsData PhysicsData { get; set; }
        /// <summary>
        ///  makes a basic object with mesh data
        /// </summary>
        /// <param name="basePoints"></param>
        /// <param name="faceData"></param>
        public BasicObject(Vector3[] basePoints, int[] faceData)
        {
            ID = count++;

            GenerateNonNulls();

            BasePoints = basePoints;
            FaceData = faceData;


            GenerateCenter();
            Rotate();

        }
        void GenerateNonNulls()
        {
            WorldPoints = new Vector3[BasePoints.Length];
        }
        void GenerateCenter()
        {
            Vector3 full = Vector3.Zero;
            for (int i = 0; i < BasePoints.Length; i++)
            {
                full += BasePoints[i];
            }
            full /= BasePoints.Length;
            Center = full;
        }
        public void Rotate()
        {
            Matrix4x4 m = Matrix4x4.CreateFromYawPitchRoll(Rotation.Y, Rotation.X, Rotation.Z);
            for (int i = 0; i < BasePoints.Length; i++)
            {
                WorldPoints[i] = Vector3.TransformNormal(BasePoints[i] - Center, m) * Scale + Position;
            }
        }
    }
}
