/*
Brennan Miller-Klugman

Sources:
    https://github.com/Unity-Technologies/ROS-TCP-Connector/issues/223
    https://docs.unity.cn/Packages/com.unity.xr.arfoundation@4.2/manual/cpu-camera-image.html
    https://github.com/Unity-Technologies/ROS-TCP-Connector/issues/223
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
public class depthImage : MonoBehaviour
{

    ROSConnection ros;
    Texture2D depth_texture, camera_texture;

    public string DepthtopicName = "image/depth/compressed";
    public string CameraTopicName = "image/compressed";

    // Limit the number of points to bound the performance cost of rendering the point cloud.

    private ARCameraManager cameraManager;
    private AROcclusionManager depthManager; 
    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<img>(DepthtopicName);
        ros.RegisterPublisher<img>(CameraTopicName);

        depthManager = FindObjectOfType<AROcclusionManager>();
        cameraManager = FindObjectOfType<ARCameraManager>();
        cameraManager.frameReceived += OnCameraFrameReceived;


    }


    void OnCameraFrameReceived(ARCameraFrameEventArgs eventArgs)
    {   
        // Get information about the device camera image.
        if (depthManager.TryAcquireEnvironmentDepthCpuImage(out XRCpuImage depth_image) && cameraManager.TryAcquireLatestCpuImage(out XRCpuImage camera_image))
        {
            // If successful, launch a coroutine that waits for the image
            // to be ready, then apply it to a texture.
            StartCoroutine(ProcessImage(depth_image, camera_image));

            // It's safe to dispose the image before the async operation completes.
            depth_image.Dispose();
            camera_image.Dispose();
        }
    }

    IEnumerator ProcessImage(XRCpuImage depth_image, XRCpuImage camera_image)
    {
        var depth_width = depth_image.width;
        var depth_height = depth_image.height;

        // Create the async conversion request.

        var depth_request = depth_image.ConvertAsync(new XRCpuImage.ConversionParams
        {
            // Use the full image.
            inputRect = new RectInt(0, 0, depth_image.width, depth_image.height),

            // Downsample by 2.
            outputDimensions = new Vector2Int(depth_image.width, depth_image.height),

            // Color image format.
            outputFormat = TextureFormat.R16,

            transformation = XRCpuImage.Transformation.MirrorX
        });

        var camera_request = camera_image.ConvertAsync(new XRCpuImage.ConversionParams
        {
            // Use the full image.
            inputRect = new RectInt(0, 0, camera_image.width, camera_image.height),

            // Downsample by 2.
            outputDimensions = new Vector2Int(depth_width, depth_height),

            // Color image format.
            outputFormat = TextureFormat.RGB24,

            transformation = XRCpuImage.Transformation.MirrorX

        });

        // Wait for the conversion to complete.
        while (!(depth_request.status.IsDone() && camera_request.status.IsDone()))
            yield return null;

        // Check status to see if the conversion completed successfully.
        if (depth_request.status != XRCpuImage.AsyncConversionStatus.Ready || camera_request.status != XRCpuImage.AsyncConversionStatus.Ready)
        {
            // Something went wrong.
            Debug.LogErrorFormat("Request failed with status {0}", depth_request.status);

            // Dispose even if there is an error.
            depth_request.Dispose();
            camera_request.Dispose();
            yield break;
        }

        var rawDepthData = depth_request.GetData<byte>();


        // Create a texture if necessary.
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

        byte[] depth_bytes = ImageConversion.EncodeToJPG(depth_texture);
        byte[] camera_bytes = ImageConversion.EncodeToJPG(camera_texture);

        Destroy(depth_texture);
        Destroy(camera_texture);

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

        // Need to dispose the request to delete resources associated
        // with the request, including the raw data.
        depth_request.Dispose();
        camera_request.Dispose();
    }
}