/*
Brennan Miller-Klugman

depthAPIImage is a modified version of ImageCollection that is used to collect both images from ARCameraManager and depth images from googles depth api and publishes them to ROS

Sources:
    https://github.com/Unity-Technologies/ROS-TCP-Connector/issues/223
    https://docs.unity3d.com/ScriptReference/ImageConversion.EncodeToJPG.html
    https://docs.unity.cn/Packages/com.unity.xr.arfoundation@4.2/manual/cpu-camera-image.html
*/

using System;
using System.Collections;
using UnityEngine;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;

using Unity.Robotics.ROSTCPConnector;
using img = RosMessageTypes.Sensor.CompressedImageMsg;
using RosMessageTypes.Std;
public class depthAPIImage : MonoBehaviour
{

    ROSConnection ros;
    Texture2D depth_texture, camera_texture;

    // Adjustable parameters
    public string DepthtopicName = "image/depth/compressed";
    public string CameraTopicName = "image/compressed";
    public int output_width = 640;
    public int output_height = 480;
    private ARCameraManager cameraManager;
    private AROcclusionManager depthManager; 
    private void Start()
    {
        // Initialize ROS and register publisher
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<img>(DepthtopicName);
        ros.RegisterPublisher<img>(CameraTopicName);

        // Initialize ARCameraManager and AROcclusionManager
        depthManager = FindObjectOfType<AROcclusionManager>();
        cameraManager = FindObjectOfType<ARCameraManager>();
        cameraManager.frameReceived += OnCameraFrameReceived;


    }


    void OnCameraFrameReceived(ARCameraFrameEventArgs eventArgs)
    {   
        // When a depth frame is recieved, try to aquire and publish the image
        if (depthManager.TryAcquireEnvironmentDepthCpuImage(out XRCpuImage depth_image) && cameraManager.TryAcquireLatestCpuImage(out XRCpuImage camera_image))
        {

            StartCoroutine(ProcessImage(depth_image, camera_image));

            depth_image.Dispose();
            camera_image.Dispose();
        }
    }

    IEnumerator ProcessImage(XRCpuImage depth_image, XRCpuImage camera_image)
    {
        var depth_width = depth_image.width;
        var depth_height = depth_image.height;

        // Convert the image & depth image to an RGB texture

        var depth_request = depth_image.ConvertAsync(new XRCpuImage.ConversionParams
        {
            inputRect = new RectInt(0, 0, depth_image.width, depth_image.height),

            outputDimensions = new Vector2Int(depth_image.width, depth_image.height),

            outputFormat = TextureFormat.R16,

            transformation = XRCpuImage.Transformation.MirrorX
        });

        var camera_request = camera_image.ConvertAsync(new XRCpuImage.ConversionParams
        {
            inputRect = new RectInt(0, 0, camera_image.width, camera_image.height),

            outputDimensions = new Vector2Int(output_width, output_height),

            outputFormat = TextureFormat.RGB24,

            transformation = XRCpuImage.Transformation.MirrorX

        });

        // Wait for the conversion to complete.
        while (!(depth_request.status.IsDone() && camera_request.status.IsDone()))
            yield return null;

        // Check status to see if the conversion completed successfully.
        if (depth_request.status != XRCpuImage.AsyncConversionStatus.Ready || camera_request.status != XRCpuImage.AsyncConversionStatus.Ready)
        {
            Debug.LogErrorFormat("Request failed with status {0}", depth_request.status);

            depth_request.Dispose();
            camera_request.Dispose();
            yield break;
        }

        var rawDepthData = depth_request.GetData<byte>();


        if (depth_texture == null)
        {
            depth_texture = new Texture2D(
                depth_width,
                depth_height,
                depth_request.conversionParams.outputFormat,
                false);
        }

        if (camera_texture == null)
        {
            camera_texture = new Texture2D(
                depth_width,
                depth_height,
                camera_request.conversionParams.outputFormat,
                false);
        }

        depth_texture.LoadRawTextureData(rawDepthData);
        depth_texture.Apply();

        camera_texture.LoadRawTextureData(camera_request.GetData<byte>());
        camera_texture.Apply();

        // Encode texture into JPG
        byte[] depth_bytes = ImageConversion.EncodeToJPG(depth_texture);
        byte[] camera_bytes = ImageConversion.EncodeToJPG(camera_texture);

        Destroy(depth_texture);
        Destroy(camera_texture);

        // Publish the imageS
        img depth = new img(
            header: new HeaderMsg(),
            format: "jpeg",
            data: depth_bytes
        );

        img camera = new img(
            header: new HeaderMsg(),
            format: "jpeg",
            data: camera_bytes
        );

        ros.Publish(DepthtopicName, depth);
        ros.Publish(CameraTopicName, camera);

        depth_request.Dispose();
        camera_request.Dispose();
    }
}