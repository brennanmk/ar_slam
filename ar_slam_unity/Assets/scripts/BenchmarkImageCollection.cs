/*
Brennan Miller-Klugman

BenchmarkImageCollection is a modified version of ImageCollection that is used along side the ros metrics node to record metrics

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

public class BenchmarkImageCollection : MonoBehaviour
{
    ROSConnection ros;
    Texture2D m_Texture;

    // Adjustable parameters
    public string imageTopicName = "image/compressed";
    public string ackedTopicName = "/ached";

    public int output_width;
    public int output_height;
    private ARCameraManager cameraManager;

    private bool allow_publish = true;

    private void Start()
    {        
        // Initialize ROS and register publisher
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<img>(imageTopicName);
        ros.Subscribe<EmptyMsg>(ackedTopicName, allow_message);

        // Initialize ARCameraManager
        cameraManager = FindObjectOfType<ARCameraManager>();
        cameraManager.frameReceived += OnCameraFrameReceived;

    }


    void allow_message(EmptyMsg msg)
    {
        // When the acked topic is recieved, allow the next image to be published
        this.allow_publish = true;
    }

    void OnCameraFrameReceived(ARCameraFrameEventArgs eventArgs)
    {   
        // check if image can be published
        if (this.allow_publish)
        {
            // When a camera frame is recieved, try to aquire and publish the image
            if (cameraManager.TryAcquireLatestCpuImage(out XRCpuImage image))
            {
                this.allow_publish = false;
                StartCoroutine(ProcessImage(image));

                image.Dispose();
            }
        }
    }

    IEnumerator ProcessImage(XRCpuImage image)
    {
        // Convert the image to an RGB texture
        var request = image.ConvertAsync(new XRCpuImage.ConversionParams
        {
            inputRect = new RectInt(0, 0, image.width, image.height),

            outputDimensions = new Vector2Int(output_width, output_height),

            outputFormat = TextureFormat.RGB24,

            transformation = XRCpuImage.Transformation.MirrorX
        });

        // Wait for the conversion to complete.
        while (!request.status.IsDone())
            yield return null;

        // Check status to see if the conversion completed successfully.
        if (request.status != XRCpuImage.AsyncConversionStatus.Ready)
        {
            Debug.LogErrorFormat("Request failed with status {0}", request.status);

            request.Dispose();
            yield break;
        }

        var rawData = request.GetData<byte>();

        if (m_Texture == null)
        {
            m_Texture = new Texture2D(
                request.conversionParams.outputDimensions.x,
                request.conversionParams.outputDimensions.y,
                request.conversionParams.outputFormat,
                false);
        }

        m_Texture.LoadRawTextureData(rawData);
        m_Texture.Apply();

        // Encode the texture to a jpg
        byte[] bytes = ImageConversion.EncodeToJPG(m_Texture);

        Destroy(m_Texture);

        // Publish the image
        img msg = new img(
            header: new HeaderMsg(),
            format: "jpeg",
            data: bytes
        );

        ros.Publish(imageTopicName, msg);

        request.Dispose();
    }
}