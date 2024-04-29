using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing.Drawing2D;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Runtime.Remoting.Channels;
using System.Windows.Forms;
using System.Windows.Input;
using SharpDX;
using SharpDX.D3DCompiler;
using SharpDX.Direct3D;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using SharpDX.Windows;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TextBox;
using Buffer = SharpDX.Direct3D11.Buffer;
using Device = SharpDX.Direct3D11.Device;
using Matrix = SharpDX.Matrix;
using Vector3 = SharpDX.Vector3;
using Vector4 = SharpDX.Vector4;

namespace tenth3d
{
    class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            var form = new RenderForm("SharpDX - MiniCube Direct3D11 Sample");

            // SwapChain description
            var desc = new SwapChainDescription()
            {
                BufferCount = 1,
                ModeDescription =
                    new ModeDescription(form.ClientSize.Width, form.ClientSize.Height,
                                        new Rational(60, 1), Format.R8G8B8A8_UNorm),
                IsWindowed = true,
                OutputHandle = form.Handle,
                SampleDescription = new SampleDescription(1, 0),
                SwapEffect = SwapEffect.Discard,
                Usage = Usage.RenderTargetOutput
            };

            Device device;
            SwapChain swapChain;
            Device.CreateWithSwapChain(DriverType.Hardware, DeviceCreationFlags.None, desc, out device, out swapChain);
            var context = device.ImmediateContext;

            // Ignore all windows events
            var factory = swapChain.GetParent<Factory>();
            factory.MakeWindowAssociation(form.Handle, WindowAssociationFlags.IgnoreAll);

            // Compile Vertex and Pixel shaders
            var vertexShaderByteCode = ShaderBytecode.CompileFromFile("RenderShader.fx", "VS", "vs_4_0");
            var vertexShader = new VertexShader(device, vertexShaderByteCode);

            var pixelShaderByteCode = ShaderBytecode.CompileFromFile("RenderShader.fx", "PS", "ps_4_0");
            var pixelShader = new PixelShader(device, pixelShaderByteCode);

            var signature = ShaderSignature.GetInputSignature(vertexShaderByteCode);
            var layout = new InputLayout(device, signature, new[]
             {
                        new InputElement("POSITION", 0, Format.R32G32B32A32_Float, 0, 0),
                        new InputElement("COLOR", 0, Format.R32G32B32A32_Float, 16, 0)
                    });

            var data = new ObjFile(@"C:\Users\nealz\Downloads\simple.obj");

            var ar = new List<Vector4>();

            for (int b = 0; b < data.faces.Count; b++)
            {
                ar.Add(new Vector4(data.points[data.faces[b]], 1f));
                ar.Add(new Vector4(0.5f, 0.5f, 0.0f, 1.0f));
            }
            var pL = ar.ToArray();



            var points = new Vector4[0];
            var faces = new int[0];

            (points, faces) = AddToLists(data, points.ToList(), faces.ToList(), Vector3.Zero);


            var vertices = Buffer.Create(device, BindFlags.VertexBuffer, ar.ToArray());

            var contantBuffer = new Buffer(device, Utilities.SizeOf<Matrix>(), ResourceUsage.Default, BindFlags.ConstantBuffer, CpuAccessFlags.None, ResourceOptionFlags.None, 0);

            context.InputAssembler.InputLayout = layout;
            context.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
            context.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(vertices, Utilities.SizeOf<Vector4>() * 2, 0));
            context.VertexShader.SetConstantBuffer(0, contantBuffer);
            context.VertexShader.Set(vertexShader);
            context.PixelShader.Set(pixelShader);

            //device.

            Matrix proj = Matrix.Identity;
            Vector3 camPos = new Vector3(0, 0, 0);
            Vector3 camRot = new Vector3(0, 0, 0);

            bool userResized = true;
            Texture2D backBuffer = null;
            RenderTargetView renderView = null;
            
            Texture2D depthBuffer = null;
            DepthStencilView depthView = null;

            form.UserResized += (sender, args) => userResized = true;
            // Setup full screen mode change F5 (Full) F4 (Window)
            form.KeyUp += (sender, args) =>
            {
                if (args.KeyCode == Keys.F5)
                    swapChain.SetFullscreenState(true, null);
                else if (args.KeyCode == Keys.F4)
                    swapChain.SetFullscreenState(false, null);
                else if (args.KeyCode == Keys.Escape)
                    form.Close();

                if (args.KeyCode == Keys.R)
                {
                    //(points, faces) = AddToLists(data, points.ToList(), faces.ToList(), camPos);
                    //pL = ar.ToArray();
                }
            };


            bool i(Key k)
            {
                return Keyboard.IsKeyDown(k);
            }

            Key[] ins = {
                Key.Right,
                Key.Left,
                Key.Up,
                Key.Down,

                Key.W,
                Key.S,
                Key.D,
                Key.A,

                Key.Q,
                Key.E

            };
            Key[] keyUp =
            {
                Key.R,
                Key.T,
            };
            bool[] pressed = new bool[keyUp.Length];


            (var forward, var right) = Forward(camRot);

            var walk = 0.6f;
            var speed = 0.005f;

            var watch = new Stopwatch();
            RenderLoop.Run(form, () =>
            {
                watch.Start();
                (forward, right) = Forward(camRot);
                var view = Matrix.LookAtLH(camPos - forward * 5, camPos, Vector3.UnitY);
                // If Form resized
                if (userResized)
                {
                    Utilities.Dispose(ref backBuffer);
                    Utilities.Dispose(ref renderView);
                    Utilities.Dispose(ref depthBuffer);
                    Utilities.Dispose(ref depthView);

                    swapChain.ResizeBuffers(desc.BufferCount, form.ClientSize.Width, form.ClientSize.Height, Format.Unknown, SwapChainFlags.None);
                    backBuffer = Texture2D.FromSwapChain<Texture2D>(swapChain, 0);

                    renderView = new RenderTargetView(device, backBuffer);

                    depthBuffer = new Texture2D(device, new Texture2DDescription()
                    {
                        Format = Format.D32_Float_S8X24_UInt,
                        ArraySize = 1,
                        MipLevels = 1,
                        Width = form.ClientSize.Width,
                        Height = form.ClientSize.Height,
                        SampleDescription = new SampleDescription(1, 0),
                        Usage = ResourceUsage.Default,
                        BindFlags = BindFlags.DepthStencil,
                        CpuAccessFlags = CpuAccessFlags.None,
                        OptionFlags = ResourceOptionFlags.None
                    });
                    depthView = new DepthStencilView(device, depthBuffer);

                    context.Rasterizer.SetViewport(new Viewport(0, 0, form.ClientSize.Width, form.ClientSize.Height, 0.0f, 1.0f));
                    context.OutputMerger.SetTargets(renderView);
                    proj = Matrix.PerspectiveFovLH((float)Math.PI / 2.0f, form.ClientSize.Width / (float)form.ClientSize.Height, 0.1f, 10000.0f);
                    userResized = false;
                }

                var viewProj = Matrix.Multiply(view, proj);

                context.ClearDepthStencilView(depthView, DepthStencilClearFlags.Depth, 1.0f, 0);
                context.ClearRenderTargetView(renderView, Color.Black);

                // Update WorldViewProj Matrix
                var worldViewProj = /*Matrix.RotationZ(camRot.Z) * Matrix.RotationY(camRot.Y)* Matrix.RotationX(camRot.X) **/ viewProj;
                worldViewProj.Transpose();
                //context.InputAssembler.SetVertexBuffers(0, GenerateVBB(pL, device));
                context.UpdateSubresource(ref worldViewProj, contantBuffer);
                Draw(points, faces, device, context, worldViewProj);
                //renderingContext.OutputMerger.SetTargets(depthView, renderView);
                context.OutputMerger.SetTargets(depthView, renderView);
                swapChain.Present(0, PresentFlags.None);

                if (i(ins[0]))
                    camRot.Y += speed;
                if (i(ins[1]))
                    camRot.Y -= speed;
                if (i(ins[2]))
                    camRot.X += speed;
                if (i(ins[3]))
                    camRot.X -= speed;
                if (i(ins[4]))
                    camPos += XYVector(forward) * walk;
                if (i(ins[5]))
                    camPos -= XYVector(forward) * walk;
                if (i(ins[6]))
                    camPos += XYVector(right) * walk;
                if (i(ins[7]))
                    camPos -= XYVector(right) * walk;
                if (i(ins[8]))
                    camPos.Y += walk;
                if (i(ins[9]))
                    camPos.Y -= walk;
                if (i(keyUp[0]))pressed[0] = true;else{if (pressed[0]) {
                        (points, faces) =  AddToLists(data, points.ToList(), faces.ToList(), camPos);
                    } pressed[0] = false; }


                //if (watch.Elapsed.TotalMilliseconds * 1000 != 0)
                form.Text = "fps:" +
                Math.Truncate(1 / (watch.Elapsed.TotalMilliseconds * 0.001)).ToString("#0000") +
                "  interval:" +
                watch.ElapsedMilliseconds.ToString("#000") +
                "  points:" +
                points.Length +
                "  indecies:" +
                faces.Length
                ;

                watch.Reset();
            });
        }
        public static (Vector4[], int[]) AddToLists(ObjFile data, List<Vector4> currentPoints, List<int> currentInts, Vector3 pos)
        {
            var rand = new Random();
            int initalLength = currentPoints.Count;
            var tp = currentPoints;//.ToList();
            var ti = currentInts;//.ToList();
            //tp.Capacity = data.points.Count * 2 + initalLength;
            //ti.Capacity = data.faces.Count +ti.Count;

            for (int i = 0; i < data.points.Count; i++)
            {
                tp.Add(new Vector4(data.points[i] + pos, 1.0f));
                //tp.Add(new Vector4(data.points[i], 1.0f));
                tp.Add(new Vector4((float)rand.NextDouble(), 0.0f, (float)rand.NextDouble(), 1.0f));
            }
            for (int i = 0; i < data.faces.Count; i++)
                ti.Add(data.faces[i]*2 + initalLength);
            return (tp.ToArray(), ti.ToArray());
        }
        public static void Draw(Vector4[] verts, int[] indices, Device d, DeviceContext dc, Matrix mat)
        {
            var vbd = new BufferDescription(
                Utilities.SizeOf<Vector4>() * verts.Length,
                ResourceUsage.Immutable,
                BindFlags.VertexBuffer,
                CpuAccessFlags.None,
                ResourceOptionFlags.None,
                0);

            var ibd = new BufferDescription(
               sizeof(int) * indices.Length,
               ResourceUsage.Immutable,
               BindFlags.IndexBuffer,
               CpuAccessFlags.None,
               ResourceOptionFlags.None,
               0);

            var _vertexBuffer = SharpDX.Direct3D11.Buffer.Create<Vector4>(d, verts, vbd);
            var _vertexBufferBinding = new VertexBufferBinding(_vertexBuffer, Utilities.SizeOf<Vector4>(), 0);
            var _indexBuffer = SharpDX.Direct3D11.Buffer.Create<int>(d, indices, ibd);

            dc.InputAssembler.SetVertexBuffers(0, _vertexBufferBinding);
            dc.InputAssembler.SetIndexBuffer(_indexBuffer, Format.R32_UInt, 0);

            dc.UpdateSubresource(ref mat, _vertexBuffer);
            //dc.UpdateSubresource(ref mat, );

            dc.DrawIndexed(indices.Length, 0, 0);
        }
        public static VertexBufferBinding GenerateVBB(Vector4[] ps, Device device)
        {
            var b = Buffer.Create(device, BindFlags.VertexBuffer, ps);
            return new VertexBufferBinding(b, Utilities.SizeOf<Vector4>() * 2, 0);
        }
        public static Vector3 XYVector(Vector3 vect)
        {
            return Vector3.Normalize(new Vector3(vect.X, 0, vect.Z));
        }
        public static (Vector3, Vector3) Forward(Vector3 rot)
        {
            var x = Matrix.RotationX(rot.X);
            var y = Matrix.RotationY(rot.Y);
            var z = Matrix.RotationZ(rot.Z);
            //var pos = Matrix.Translation(-camPos);
            Matrix fullMatrix = z * x * y;

            var forward = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitZ, fullMatrix));
            var right = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitX, fullMatrix));
            return (forward, right);
        }

        static byte[] GenerateTextureData(int TextureWidth, int TexturePixelSize, int TextureHeight)
        {
            int rowPitch = TextureWidth * TexturePixelSize;
            int cellPitch = rowPitch >> 3;       // The width of a cell in the checkboard texture.
            int cellHeight = TextureWidth >> 3;  // The height of a cell in the checkerboard texture.
            int textureSize = rowPitch * TextureHeight;
            byte[] data = new byte[textureSize];

            for (int n = 0; n < textureSize; n += TexturePixelSize)
            {
                int x = n % rowPitch;
                int y = n / rowPitch;
                int i = x / cellPitch;
                int j = y / cellHeight;

                if (i % 2 == j % 2)
                {
                    data[n] = 0x00;     // R
                    data[n + 1] = 0x00; // G
                    data[n + 2] = 0x00; // B
                    data[n + 3] = 0xff; // A
                }
                else
                {
                    data[n] = 0xff;     // R
                    data[n + 1] = 0xff; // G
                    data[n + 2] = 0xff; // B
                    data[n + 3] = 0xff; // A
                }
            }

            return data;
        }
        public class ObjFile
        {
            public List<Vector3> points { get; set; } = new List<Vector3>();
            public List<int> faces { get; set; } = new List<int>();
            public string[] lines { get; set; }
            public ObjFile(string path)
            {

                lines = File.ReadAllLines(path);
                //var sections = new List<string[]>();

                for (int i = 0; i < lines.Length; i++)
                {
                    var str = lines[i];

                    if (str.StartsWith("vt"))
                    {
                        lines = lines.Skip(i - 1).ToArray();
                        break;
                    }
                    if (str.StartsWith("v"))
                    {
                        var val = str.Substring(2);
                        var split = val.Split(' ');
                        points.Add(new Vector3(float.Parse(split[0]), float.Parse(split[1]), float.Parse(split[2])));
                    }
                    if (str.StartsWith("f"))
                    {
                        var val = str.Substring(2);
                        var split = val.Split(' ');
                        var f = new int[] { int.Parse(split[0]) - 1, int.Parse(split[1]) - 1, int.Parse(split[2]) - 1 };
                        faces.AddRange(f);
                        //faces.AddRange(f.Reverse());
                    }

                }

            }


        }
    }
}
