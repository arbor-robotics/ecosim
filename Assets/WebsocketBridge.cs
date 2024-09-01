using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using NativeWebSocket;
using System;
using SimpleJSON;

[Serializable]
public struct SubscriptionRequest
{
    public string op;
    public string topic;
}

[Serializable]
public struct RosbridgeMessage
{
    public string op;
    public string topic;
    // public string service;
    // public string id;
    public string msg;
    public string type;
}

public struct TwistMsg
{
    public Vector3 linear;
    public Vector3 angular;
}

public class WebsocketBridge : MonoBehaviour
{
    WebSocket websocket;
    ArcadeCarController carController;

    // Start is called before the first frame update

    private TwistMsg ToTwist(JSONNode str)
    {
        TwistMsg msg = new();

        msg.linear.x = str["linear"]["x"];
        msg.linear.y = str["linear"]["y"];
        msg.linear.z = str["linear"]["z"];

        msg.angular.x = str["angular"]["x"];
        msg.angular.y = str["angular"]["y"];
        msg.angular.z = str["angular"]["z"];

        return msg;
    }

    async void AdvertisePublishers()
    {
        // Front camera
        RosbridgeMessage req = new()
        {
            op = "advertise",
            topic = "/camera/front/image_color",
            type = "sensor_msgs/Image"
        };

        string reqString = JsonUtility.ToJson(req);

        Debug.Log($"Sending: {reqString} {req}");

        await websocket.SendText(reqString);
    }
    async void Start()
    {
        websocket = new WebSocket("ws://localhost:9090");

        carController = GetComponent<ArcadeCarController>();

        websocket.OnOpen += async () =>
        {
            Debug.Log("Connection open!");

            SubscriptionRequest req = new()
            {
                op = "subscribe",
                topic = "/cmd_vel"
            };

            string reqString = JsonUtility.ToJson(req);

            Debug.Log($"Sending: {reqString} {req}");

            await websocket.SendText(reqString);

            AdvertisePublishers();

        };

        websocket.OnError += (e) =>
            {
                Debug.Log("Error! " + e);
            };

        websocket.OnClose += (e) =>
        {
            Debug.Log("Connection closed!");
        };

        websocket.OnMessage += (bytes) =>
        {
            // Debug.Log("OnMessage!");
            // Debug.Log(bytes);

            // getting the message as a string
            var messageString = System.Text.Encoding.UTF8.GetString(bytes);

            JSONNode root = JSONNode.Parse(messageString);
            Debug.Log("OnMessage! " + messageString);

            if (root["topic"] == "/cmd_vel")
            {
                TwistMsg msg = ToTwist(root["msg"]);
                carController.throttle = msg.linear.x;
                carController.turn = msg.angular.z;
            }
            Debug.Log(root["msg"]);

            // RosbridgeMessage msg = JsonUtility.FromJson<RosbridgeMessage>(messageString);
            // Debug.Log(msg.msg);

        };

        // Keep sending messages at every 0.3s
        // InvokeRepeating("SendWebSocketMessage", 0.0f, 0.3f);

        // waiting for messages
        await websocket.Connect();
    }

    void Update()
    {
#if !UNITY_WEBGL || UNITY_EDITOR
        websocket.DispatchMessageQueue();
#endif
    }

    async void PublishCameraImage(byte[] imageData)
    {
        if (websocket.State == WebSocketState.Open)
        {
            // Sending bytes
            // await websocket.Send(new byte[] { 10, 20, 30 });

            // Sending plain text
            await websocket.SendText("YEET");
        }
    }

    async void SendWebSocketMessage(string message)
    {
        if (websocket.State == WebSocketState.Open)
        {
            // Sending bytes
            // await websocket.Send(new byte[] { 10, 20, 30 });

            // Sending plain text
            await websocket.SendText(message);
        }
    }

    private async void OnApplicationQuit()
    {
        await websocket.Close();
    }
}
