using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Runtime.Remoting.Channels;
using System.Windows.Forms;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using SharpDX;
using SharpDX.D3DCompiler;
using SharpDX.Direct2D1;
using SharpDX.Direct3D;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using SharpDX.WIC;
using SharpDX.Windows;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TaskbarClock;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TextBox;
using BitmapDecoder = SharpDX.WIC.BitmapDecoder;
using Buffer = SharpDX.Direct3D11.Buffer;
using Device = SharpDX.Direct3D11.Device;
using DeviceContext = SharpDX.Direct3D11.DeviceContext;
using Factory = SharpDX.DXGI.Factory;
using InputElement = SharpDX.Direct3D11.InputElement;
using Matrix = SharpDX.Matrix;
using PixelFormat = SharpDX.WIC.PixelFormat;
using Vector2 = System.Numerics.Vector2;
using Vector3 = SharpDX.Vector3;
using Vector4 = SharpDX.Vector4;

namespace tenth3d
{
    public class Program
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

            var factory = swapChain.GetParent<Factory>();
            factory.MakeWindowAssociation(form.Handle, WindowAssociationFlags.IgnoreAll);

            var vertexShaderByteCode = ShaderBytecode.CompileFromFile("RenderShader.fx", "VS", "vs_4_0");
            var vertexShader = new VertexShader(device, vertexShaderByteCode);

            var pixelShaderByteCode = ShaderBytecode.CompileFromFile("RenderShader.fx", "PS", "ps_4_0");
            var pixelShader = new PixelShader(device, pixelShaderByteCode);

            var signature = ShaderSignature.GetInputSignature(vertexShaderByteCode);
            var layout = new InputLayout(device, signature, new[]
             {
                        new InputElement("POSITION", 0, Format.R32G32B32A32_Float, 0, 0),
                        new InputElement("COLOR", 0, Format.R32G32B32A32_Float, 16, 0),
                        new InputElement("TEXCOORD", 0, Format.R32G32B32A32_Float, 32,0),
                    });

            var vertices = Buffer.Create(device, BindFlags.VertexBuffer, new Vector4[3]);

            var contantBuffer = new Buffer(device, Utilities.SizeOf<Matrix>(), ResourceUsage.Default, BindFlags.ConstantBuffer, CpuAccessFlags.None, ResourceOptionFlags.None, 0);

            context.InputAssembler.InputLayout = layout;
            context.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
            context.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(vertices, Utilities.SizeOf<Vector4>() * 3, 0));
            context.VertexShader.SetConstantBuffer(0, contantBuffer);
            context.VertexShader.Set(vertexShader);
            context.PixelShader.Set(pixelShader);

            var srv = FileToTexture2D(device);
            context.PixelShader.SetShaderResource(0, srv);

            Vector3 camPos = new Vector3(0, 0, 0);
            Vector3 camRot = new Vector3(0, 0, 0);

            bool userResized = true;
            Texture2D backBuffer = null;
            RenderTargetView renderView = null;

            Texture2D depthBuffer = null;
            DepthStencilView depthView = null;


            (var forward, var right) = Forward(camRot);

            var walk = 0.3f;
            var speed = 0.005f;

            var constant = new ConstantUpdate();
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
                else if (args.KeyCode ==Keys.D3)
                {
                    constant.CollisionUpdate();
                }

                constant.KeyUpCallBack(args.KeyCode);

                //constant.KeyUpCallBack(sender, args);
            };

            Key[] keys = new Key[0];
            bool[] ks = new bool[0];

            var view = Matrix.LookAtLH(new Vector3(0, 0, -5), new Vector3(0, 0, 0), Vector3.UnitY);
            Matrix proj = Matrix.Identity;

            var watch = new Stopwatch();
            RenderLoop.Run(form, () =>
            {
                watch.Start();

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
                    context.OutputMerger.SetTargets(depthView, renderView);

                    // Setup new projection matrix with correct aspect ratio
                    proj = Matrix.PerspectiveFovLH((float)Math.PI / 4.0f, form.ClientSize.Width / (float)form.ClientSize.Height, 0.1f, 100.0f);

                    
                }

                { //draw faces

                    context.ClearDepthStencilView(depthView, DepthStencilClearFlags.Depth, 1f, 0);
                    context.ClearRenderTargetView(renderView, SharpDX.Color.Black);

                    var mat = constant.GetWorldViewProj(form, userResized);
                    userResized = false;

                    context.UpdateSubresource(ref mat, contantBuffer);
                    context.OutputMerger.SetTargets(depthView, renderView);
                    constant.Draw(device, context, renderView, 3); 


                    swapChain.Present(0, PresentFlags.None);
                } //draw faces


                {//input
                    if (keys.Length != constant.input.len())
                        keys = constant.input.GetKeys();

                    if (ks.Length != keys.Length)
                        ks = new bool[keys.Length];

                    for (int i = 0; i < keys.Length; i ++)
                        ks[i] = Keyboard.IsKeyDown(keys[i]);

                    constant.input.SetStates(ks);
                }

                form.Text = watch.ElapsedMilliseconds.ToString();
                watch.Reset();
            });
        }
        public static ShaderResourceView FileToTexture2D(Device device)
        {
            var imfactory = new ImagingFactory();
            
            string imagePath = Path.Combine(Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location), @"Resources/simpleBmp2.bmp");
            var decoder = new BitmapDecoder(imfactory, imagePath, DecodeOptions.CacheOnDemand);
            var result = new FormatConverter(imfactory);
            result.Initialize(decoder.GetFrame(0), PixelFormat.Format32bppPRGBA, BitmapDitherType.None, null, 0.0, BitmapPaletteType.Custom);
            var bitmap = result;
            Texture2D texArray = new Texture2D(device, new Texture2DDescription()
            {
                Width = bitmap.Size.Width,
                Height = bitmap.Size.Height,
                ArraySize = 1,
                BindFlags = BindFlags.ShaderResource | BindFlags.RenderTarget,
                Usage = SharpDX.Direct3D11.ResourceUsage.Default,
                CpuAccessFlags = SharpDX.Direct3D11.CpuAccessFlags.None,
                Format = SharpDX.DXGI.Format.R8G8B8A8_UNorm,
                MipLevels = 1,
                OptionFlags = SharpDX.Direct3D11.ResourceOptionFlags.GenerateMipMaps,
                SampleDescription = new SharpDX.DXGI.SampleDescription(1, 0),
            });
            var stride = bitmap.Size.Width * 4;
            var buffer = new DataStream(stride * bitmap.Size.Height, true, true);
            bitmap.CopyPixels(stride, buffer);
            DataBox box = new DataBox(buffer.DataPointer, stride, 1);
            device.ImmediateContext.UpdateSubresource(box, texArray);
            return new ShaderResourceView(device, texArray);
        }
        public static void Draw(Vector4[] verts, int[] indices, Device d, DeviceContext dc,RenderTargetView rt, bool drawOnTop = false)
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

            //dc.UpdateSubresource(ref mat, _vertexBuffer);
            //dc.PixelShader.SetShaderResource(0, srv);
            if (drawOnTop)
            {
                dc.OutputMerger.SetRenderTargets(rt);
            } 
            dc.DrawIndexed(indices.Length, 0, 0);

            _vertexBuffer.Dispose();
            _indexBuffer.Dispose();
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
    }
}
