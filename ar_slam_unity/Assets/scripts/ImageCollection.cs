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
public class ImageCollection : MonoBehaviour
{

    ROSConnection ros;
    Texture2D m_Texture;
    private float timeElapsed;

    public string topicName = "image/compressed";
    public int output_width = 640;
    public int output_height =480;
    private ARCameraManager cameraManager;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<img>(topicName);

        cameraManager = FindObjectOfType<ARCameraManager>();
        cameraManager.frameReceived += OnCameraFrameReceived;

    }


    void OnCameraFrameReceived(ARCameraFrameEventArgs eventArgs)
    {   
        // Get information about the device camera image.
        if (cameraManager.TryAcquireLatestCpuImage(out XRCpuImage image))
        {
            // If successful, launch a coroutine that waits for the image
            // to be ready, then apply it to a texture.
            StartCoroutine(ProcessImage(image));

            // It's safe to dispose the image before the async operation completes.
            image.Dispose();
        }
    }

    IEnumerator ProcessImage(XRCpuImage image)
    {
        // Create the async conversion request.
        var request = image.ConvertAsync(new XRCpuImage.ConversionParams
        {
            // Use the full image.
            inputRect = new RectInt(0, 0, image.width, image.height),

            // Downsample by 2.
            outputDimensions = new Vector2Int(output_width, output_height),

            // Color image format.
            outputFormat = TextureFormat.RGB24,

            // Flip across the Y axis.
            transformation = XRCpuImage.Transformation.MirrorX
        });

        // Wait for the conversion to complete.
        while (!request.status.IsDone())
            yield return null;

        // Check status to see if the conversion completed successfully.
        if (request.status != XRCpuImage.AsyncConversionStatus.Ready)
        {
            // Something went wrong.
            Debug.LogErrorFormat("Request failed with status {0}", request.status);

            // Dispose even if there is an error.
            request.Dispose();
            yield break;
        }

        // Image data is ready. Let's apply it to a Texture2D.
        var rawData = request.GetData<byte>();

        // Create a texture if necessary.
        if (m_Texture == null)
        {
            m_Texture = new Texture2D(
                request.conversionParams.outputDimensions.x,
                request.conversionParams.outputDimensions.y,
                request.conversionParams.outputFormat,
                false);
        }


        // Copy the image data into the texture.
        m_Texture.LoadRawTextureData(rawData);
        m_Texture.Apply();

        byte[] bytes = ImageConversion.EncodeToJPG(m_Texture);

        Destroy(m_Texture);

        img msg = new img(
            header: new HeaderMsg(),
            format: "jpeg",
            data: bytes
        );

        ros.Publish(topicName, msg);
        // Need to dispose the request to delete resources associated
        // with the request, including the raw data.
        request.Dispose();
    }
}