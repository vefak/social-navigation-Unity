
using System;
using System.Collections;                  // IEnumerator / coroutine
using Unity.Collections;                   // NativeArray<T>
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;      // ROSConnection
using Unity.Robotics.Core;                 // SimTime.Now()
using RosMessageTypes.Sensor;              // ImageMsg, CameraInfoMsg
using RosMessageTypes.Std;                 // HeaderMsg
using RosMessageTypes.BuiltinInterfaces;   // TimeMsg

[RequireComponent(typeof(ROSConnection))]
public class RGBDCameraPublisher : MonoBehaviour
{
    // ─────────────── Inspector‑exposed fields ───────────────
    [Header("Cameras")] public Camera rgbCamera; public Camera depthCamera;

    [Header("Frame IDs (prefer optical frames)")]
    public string rgbFrameId = "rgb_camera_optical_frame";
    public string depthFrameId = "depth_camera_optical_frame";

    [Header("ROS Topics")]
    public string rgbTopic       = "/camera/color/image_raw";
    public string depthTopic     = "/camera/depth/image_raw";
    public string rgbInfoTopic   = "/camera/color/camera_info";
    public string depthInfoTopic = "/camera/depth/camera_info";

    [Header("Intrinsics / Geometry")]
    public Vector2Int resolution = new Vector2Int(640, 480);
    public float minRange = 0.1f;  // m
    public float maxRange = 10.0f; // m

    [Header("Synthetic Noise (σ)")]
    [Tooltip("8‑bit RGB noise std‑dev, DN. 0 = no noise")] public float rgbNoiseSigma = 0f;
    [Tooltip("Depth noise coefficient k in σ(z)=k·z². 0 = no noise")] public float depthNoiseK = 0f;

    [Header("Publishing")]
    [Range(1, 60)] public float publishRateHz = 15f;
    [Tooltip("Tick if colours look swapped in RViz")] public bool publishBGR = false;
    [Tooltip("Clamp depth outside [min,max] to 0")] public bool clampDepthToRange = true;

    [Header("Depth Linearisation Material (BIRP)")]
    [Tooltip("Material from 'Hidden/ROS/DepthToMeters_BIRP' — outputs metres in R")] public Material depthMaterial;

    // ─────────────── Internals ───────────────
    ROSConnection ros;
    RenderTexture rtRGB, rtDepth;
    Texture2D texRGB, texDepth;

    ImageMsg rgbMsg, depthMsg;  CameraInfoMsg rgbInfo, depthInfo;
    byte[] rgbBuffer;           float[] depthFloatBuffer; byte[] depthByteBuffer;

    System.Random rng = new System.Random();
    bool ready;

    // ─────────────────────────────────────────
    void Awake() => Application.runInBackground = true;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(rgbTopic);
        ros.RegisterPublisher<ImageMsg>(depthTopic);
        ros.RegisterPublisher<CameraInfoMsg>(rgbInfoTopic);
        ros.RegisterPublisher<CameraInfoMsg>(depthInfoTopic);

        QualitySettings.vSyncCount = 0;      // uncap FPS
        Application.targetFrameRate = 120;

        if (!Validate()) return;
        ConfigureCameras();
        AllocateBuffers();
        PrepareIntrinsics();
        StartCoroutine(PublishLoop());
        ready = true;
    }

    bool Validate()
    {
        if (rgbCamera == null || depthCamera == null)
        { Debug.LogError("[RGBDCameraPublisher] Assign rgbCamera & depthCamera"); return false; }
        if (depthMaterial == null)
        { Debug.LogError("[RGBDCameraPublisher] Assign depthMaterial (DepthToMeters_BIRP)"); return false; }
        return true;
    }

    void ConfigureCameras()
    {
        depthCamera.depthTextureMode = DepthTextureMode.Depth;
        depthCamera.nearClipPlane = minRange; depthCamera.farClipPlane = maxRange;
        rgbCamera.forceIntoRenderTexture = depthCamera.forceIntoRenderTexture = true;
        rgbCamera.allowHDR = depthCamera.allowHDR = false;
        rgbCamera.allowMSAA = depthCamera.allowMSAA = false;
    }

    void AllocateBuffers()
    {
        int w = resolution.x, h = resolution.y;
        rtRGB = new RenderTexture(w, h, 24, RenderTextureFormat.ARGB32);
        texRGB = new Texture2D(w, h, TextureFormat.RGB24, false);
        rgbCamera.targetTexture = rtRGB;

        var rtFmt = SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.RFloat) ? RenderTextureFormat.RFloat : RenderTextureFormat.ARGBFloat;
        rtDepth = new RenderTexture(w, h, 24, rtFmt);
        var texFmt = (rtFmt == RenderTextureFormat.RFloat) ? TextureFormat.RFloat : TextureFormat.RGBAFloat;
        texDepth = new Texture2D(w, h, texFmt, false);
        depthCamera.targetTexture = rtDepth;

        rgbBuffer = new byte[w * h * 3]; depthFloatBuffer = new float[w * h]; depthByteBuffer = new byte[depthFloatBuffer.Length * 4];
        rgbMsg = new ImageMsg(); depthMsg = new ImageMsg(); rgbInfo = new CameraInfoMsg(); depthInfo = new CameraInfoMsg();
    }

    void PrepareIntrinsics()
    {
        BuildCameraInfo(rgbInfo, resolution.x, resolution.y, rgbCamera.fieldOfView);
        BuildCameraInfo(depthInfo, resolution.x, resolution.y, depthCamera.fieldOfView);
    }

    // ─────────────── Coroutine timer ───────────────
    IEnumerator PublishLoop()
    {
        var wait = new WaitForSecondsRealtime(1f / publishRateHz);
        while (true)
        {
            if (ready)
            {
                var stamp = SimTime.Now();
                PublishRGB(stamp); PublishDepth(stamp); PublishInfos(stamp);
            }
            yield return wait;
        }
    }

    // ─────────────── Publishers ───────────────
    void PublishRGB(TimeMsg stamp)
    {
        rgbCamera.Render();
        var prev = RenderTexture.active; RenderTexture.active = rtRGB;
        texRGB.ReadPixels(new Rect(0,0,resolution.x,resolution.y),0,0); texRGB.Apply(false);
        RenderTexture.active = prev;
        FlipTexture(texRGB);

        texRGB.GetRawTextureData<byte>().CopyTo(rgbBuffer);
        if (rgbNoiseSigma > 0f) AddRgbNoise();

        rgbMsg.header = new HeaderMsg(stamp, rgbFrameId);
        rgbMsg.height = (uint)resolution.y; rgbMsg.width = (uint)resolution.x;
        rgbMsg.encoding = publishBGR ? "bgr8" : "rgb8"; rgbMsg.is_bigendian = 0; rgbMsg.step = (uint)(resolution.x*3);
        rgbMsg.data = rgbBuffer;
        ros.Publish(rgbTopic, rgbMsg);
    }

    void PublishDepth(TimeMsg stamp)
    {
        depthCamera.Render();
        depthMaterial.SetFloat("_NearClip", depthCamera.nearClipPlane);
        depthMaterial.SetFloat("_FarClip",  depthCamera.farClipPlane);
        var prev = RenderTexture.active; Graphics.Blit(null, rtDepth, depthMaterial);
        RenderTexture.active = rtDepth; texDepth.ReadPixels(new Rect(0,0,resolution.x,resolution.y),0,0); texDepth.Apply(false); RenderTexture.active = prev;
        static bool IsFinite(float v) => !(float.IsNaN(v) || float.IsInfinity(v));
        
        var px = texDepth.GetPixels(); int w = resolution.x, h = resolution.y;
        for (int y=0;y<h;y++)
        {
            int yFlip = h-1-y; int src=y*w; int dst=yFlip*w;
            for (int x=0;x<w;x++)
            {
                float d = px[src+x].r;
                //if (clampDepthToRange && (!float.IsFinite(d)||d<minRange||d>maxRange)) d=0f;
                if (clampDepthToRange && (!IsFinite(d) || d < minRange || d > maxRange)) d = 0f;

                depthFloatBuffer[dst+x]=d;
            }
        }
        if (depthNoiseK>0f) AddDepthNoise();
        Buffer.BlockCopy(depthFloatBuffer,0,depthByteBuffer,0,depthByteBuffer.Length);

        depthMsg.header = new HeaderMsg(stamp, depthFrameId);
        depthMsg.height=(uint)h; depthMsg.width=(uint)w; depthMsg.encoding="32FC1"; depthMsg.is_bigendian=0; depthMsg.step=(uint)(w*4);
        depthMsg.data = depthByteBuffer;
        ros.Publish(depthTopic, depthMsg);
    }

    void PublishInfos(TimeMsg stamp)
    {
        rgbInfo.header = new HeaderMsg(stamp,rgbFrameId);
        depthInfo.header = new HeaderMsg(stamp,depthFrameId);
        ros.Publish(rgbInfoTopic,rgbInfo); ros.Publish(depthInfoTopic,depthInfo);
    }

    // ─────────────── Helpers ───────────────
    void AddRgbNoise()
    {
        double sd = rgbNoiseSigma;
        for (int i=0;i<rgbBuffer.Length;i++)
        {
            double u1=1.0-rng.NextDouble(), u2=1.0-rng.NextDouble();
            double n=Math.Sqrt(-2.0*Math.Log(u1))*Math.Sin(2*Math.PI*u2);
            int val=rgbBuffer[i]+(int)(n*sd); rgbBuffer[i]=(byte)Mathf.Clamp(val,0,255);
        }
    }

    void AddDepthNoise()
    {
        double k = depthNoiseK;
        for(int i=0;i<depthFloatBuffer.Length;i++)
        {
            float d=depthFloatBuffer[i]; if(d==0f) continue;
            double sd=k*d*d; double u1=1.0-rng.NextDouble(),u2=1.0-rng.NextDouble();
            double n=Math.Sqrt(-2.0*Math.Log(u1))*Math.Sin(2*Math.PI*u2);
            depthFloatBuffer[i]=(float)(d+n*sd);
        }
    }

    static void BuildCameraInfo(CameraInfoMsg msg,int w,int h,float fovYdeg)
    {
        float fy=0.5f*h/Mathf.Tan(fovYdeg*Mathf.Deg2Rad*0.5f); float fx=fy; float cx=w*0.5f, cy=h*0.5f;
        msg.height=(uint)h; msg.width=(uint)w; msg.distortion_model="plumb_bob";
        msg.d=new double[5]{0,0,0,0,0};
        msg.k=new double[9]{fx,0,cx, 0,fy,cy, 0,0,1};
        msg.r=new double[9]{1,0,0, 0,1,0, 0,0,1};
        msg.p=new double[12]{fx,0,cx,0, 0,fy,cy,0, 0,0,1,0};
    }

    static void FlipTexture(Texture2D tex)
    {
        int w=tex.width,h=tex.height; var px=tex.GetPixels();
        for(int y=0;y<h/2;y++){int top=y*w,bot=(h-1-y)*w; for(int x=0;x<w;x++){var c=px[top+x]; px[top+x]=px[bot+x]; px[bot+x]=c;}}
        tex.SetPixels(px); tex.Apply(false);
    }

    void OnDestroy(){ Release(rtRGB); Release(rtDepth); }
    static void Release(RenderTexture rt){ if(rt!=null&&rt.IsCreated()) rt.Release(); }
}
