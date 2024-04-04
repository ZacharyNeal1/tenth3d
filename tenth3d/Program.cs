using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing.Drawing2D;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using System.Windows.Input;
using SharpDX;
using SharpDX.D3DCompiler;
using SharpDX.Direct3D;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using SharpDX.Windows;
using Buffer = SharpDX.Direct3D11.Buffer;
using Device = SharpDX.Direct3D11.Device;
using Matrix = SharpDX.Matrix;
using Vector3 = SharpDX.Vector3;
using Vector4 = SharpDX.Vector4;

namespace tenth3d
{
    internal static class Program
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
                        new InputElement("POSITION", 0, Format.R32G32B32_Float, 0, 0),
                        new InputElement("COLOR", 0, Format.R32G32B32A32_Float, 16, 0)
                    });

            var data = new ObjFile(@"C:\Users\nealz\Downloads\model.obj");

            var ar = new List<Vector4>();

            for (int i = 0; i < data.faces.Count; i ++)
            {
                ar.Add(new Vector4(data.points[data.faces[i]], 1f));
                ar.Add(new Vector4(1.0f, 1.0f, 0.0f, 1.0f));
            }

            var vertices = Buffer.Create(device, BindFlags.VertexBuffer, ar.ToArray());

            var contantBuffer = new Buffer(device, Utilities.SizeOf<Matrix>(), ResourceUsage.Default, BindFlags.ConstantBuffer, CpuAccessFlags.None, ResourceOptionFlags.None, 0);



            context.InputAssembler.InputLayout = layout;
            context.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
            context.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(vertices, Utilities.SizeOf<Vector4>() * 2, 0));
            context.VertexShader.SetConstantBuffer(0, contantBuffer);
            context.VertexShader.Set(vertexShader);
            context.PixelShader.Set(pixelShader);


            Matrix proj = Matrix.Identity;


            bool userResized = true;
            Texture2D backBuffer = null;
            RenderTargetView renderView = null;
            //Texture2D depthBuffer = null;
            //DepthStencilView depthView = null;

            // Setup handler on resize form
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
            };


            Vector3 camPos = new Vector3(0, 0, 0);
            Vector3 camRot = new Vector3(0, 0, 0);

            var x = Matrix.RotationX(camRot.X);
            var y = Matrix.RotationY(camRot.Y);
            var z = Matrix.RotationZ(camRot.Z);
            //var pos = Matrix.Translation(-camPos);
            Matrix fullMatrix = z * x * y;

            var forward = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitZ, fullMatrix));
            var right = Vector3.Normalize((Vector3)Vector3.Transform(Vector3.UnitX, fullMatrix));

            var walk = 0.3f;
            var speed = 0.01f;

            RenderLoop.Run(form, () =>
            {
                var view = Matrix.LookAtLH(camPos - forward* 5, camPos, Vector3.UnitY);
                // If Form resized
                if (userResized)
                {
                    // Dispose all previous allocated resources
                    Utilities.Dispose(ref backBuffer);
                    Utilities.Dispose(ref renderView);
                    //Utilities.Dispose(ref depthBuffer);
                    //Utilities.Dispose(ref depthView);

                    // Resize the backbuffer
                    swapChain.ResizeBuffers(desc.BufferCount, form.ClientSize.Width, form.ClientSize.Height, Format.Unknown, SwapChainFlags.None);

                    // Get the backbuffer from the swapchain
                    backBuffer = Texture2D.FromSwapChain<Texture2D>(swapChain, 0);

                    // Renderview on the backbuffer
                    renderView = new RenderTargetView(device, backBuffer);

                    // Create the depth buffer
                    //depthBuffer = new Texture2D(device, new Texture2DDescription()
                    //{
                    //    Format = Format.D32_Float_S8X24_UInt,
                    //    ArraySize = 1,
                    //    MipLevels = 1,
                    //    Width = form.ClientSize.Width,
                    //    Height = form.ClientSize.Height,
                    //    SampleDescription = new SampleDescription(1, 0),
                    //    Usage = ResourceUsage.Default,
                    //    BindFlags = BindFlags.DepthStencil,
                    //    CpuAccessFlags = CpuAccessFlags.None,
                    //    OptionFlags = ResourceOptionFlags.None
                    //});

                    // Create the depth buffer view
                    //depthView = new DepthStencilView(device, depthBuffer);

                    // Setup targets and viewport for rendering
                    context.Rasterizer.SetViewport(new Viewport(0, 0, form.ClientSize.Width, form.ClientSize.Height, 0.0f, 1.0f));
                    //context.OutputMerger.SetTargets(depthView, renderView);
                    context.OutputMerger.SetTargets(renderView);

                    // Setup new projection matrix with correct aspect ratio
                    proj = Matrix.PerspectiveFovLH((float)Math.PI / 2.0f, form.ClientSize.Width / (float)form.ClientSize.Height, 0.1f, 10000.0f);

                    // We are done resizing
                    userResized = false;
                }

                var viewProj = Matrix.Multiply(view, proj);

                // Clear views
                //context.ClearDepthStencilView(depthView, DepthStencilClearFlags.Depth, 1.0f, 0);
                context.ClearRenderTargetView(renderView, Color.White);

                // Update WorldViewProj Matrix
                var worldViewProj = Matrix.RotationZ(camRot.Z) * Matrix.RotationY(camRot.Y)* Matrix.RotationX(camRot.X) * viewProj;
                worldViewProj.Transpose();
                context.UpdateSubresource(ref worldViewProj, contantBuffer);

                // Draw the cube
                context.Draw(ar.Count/2, 0);
                //context.DrawIndexed(ar.Count, 0, 0);

                // Present!
                swapChain.Present(0, PresentFlags.None);


                if (Keyboard.IsKeyDown(Key.Right))
                    camRot.Y += speed;
                if (Keyboard.IsKeyDown(Key.Left))
                    camRot.Y -= speed;
                if (Keyboard.IsKeyDown(Key.Up))
                    camRot.X += speed;
                if (Keyboard.IsKeyDown(Key.Down))
                    camRot.X -= speed;
                if (Keyboard.IsKeyDown(Key.W))
                    camPos += forward * walk;
                if (Keyboard.IsKeyDown(Key.S))
                    camPos -= forward * walk;
                if (Keyboard.IsKeyDown(Key.D))
                    camPos += right * walk;
                if (Keyboard.IsKeyDown(Key.A))
                    camPos -= right * walk;
            });
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
                
                for (int i = 0; i < lines.Length; i ++)
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

//        public class PointFaceObject
//        {
//            public Vector3[] verts { get; set; }
//            public int[] indices { get; set; }

//            private Buffer _indexBuffer;
//            private Buffer _vertexBuffer;

//            private VertexBufferBinding _vertexBufferBinding;

//            BufferDescription vb;
//            BufferDescription ibd;
//            public PointFaceObject(Vector3[] points, int[] ints)
//            {
//                verts = points;

//                vb = new BufferDescription(
//   Utilities.SizeOf<Vector4>() * verts.Length,
//     ResourceUsage.Immutable,
//  BindFlags.VertexBuffer,
//  CpuAccessFlags.None,
// ResourceOptionFlags.None,
//0);
//                indices = ints;

//                ibd = new BufferDescription(
//                   sizeof(int) * indices.Length,
//                   ResourceUsage.Immutable,
//                   BindFlags.IndexBuffer,
//                   CpuAccessFlags.None,
//                   ResourceOptionFlags.None,
//                   0);
//            }
//            public Buffer MakeBuffer(Device d)
//            {
//                _vertexBuffer = Buffer.Create(d, verts, vb);
//                _vertexBufferBinding = new VertexBufferBinding(_vertexBuffer, 32, 0);
//                _indexBuffer = Buffer.Create(d, indices, ibd);
//                return _vertexBuffer;
//            }
//            public void SelectBuffers(DeviceContext c)
//            {
//                c.InputAssembler.SetVertexBuffers(0, _vertexBufferBinding);
//                c.InputAssembler.SetIndexBuffer(_indexBuffer, Format.R32_UInt, 0);
//            }
//            public void Draw(DeviceContext c)
//            {
//                c.DrawIndexed(indices.Length, 0, 0);
//            }

//        }


        [StructLayout(LayoutKind.Sequential)]
        public struct InputPoint
        {
            Vector3 point { get; set; }
            Color4 color { get; set; }

            public InputPoint(Vector3 p)
            {
                point = p;
                color = Color4.Black;
            }
            public InputPoint(Vector3 p, Color4 c)
            {
                point = p;
                color = c;
            }

        }
    }
}
