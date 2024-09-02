using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using NativeWebSocket;
using System;
using SimpleJSON;
using UnityEditor.VersionControl;
using ROS2;
using System.Text.Json;
using System.Security.Cryptography;

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
    public RosMessage msg;
    public string type;
}
public class Subscription
{
    private Action cb;
    public string id;
    public string topic;
    public Subscription(string newId, string newTopic, Action newCb)
    {
        cb = newCb;
        id = newId;
        topic = newTopic;
    }
}

public class Publisher
{
    public string id;
    public string topic;
    private Queue<string> sendQueue_;
    public Publisher(string newId, string newTopic, Queue<string> sendQueue)
    {
        id = newId;
        topic = newTopic;
        sendQueue_ = sendQueue;
    }

    public void publish(RosMessage msg)
    {
        string msgString = JsonUtility.ToJson(msg);

        // string publishOp = $"{{ op: 'publish','topic': {topic},'msg': {msgString}}}";
        string publishOp = $"{{ \"op\": \"publish\",\"topic\": \"{topic}\", \"msg\":{msgString}}}";
        // Debug.Log($"Publishing {json}");
        sendQueue_.Enqueue(publishOp);
    }
}

public class WebsocketBridge : MonoBehaviour
{
    WebSocket websocket;
    ArcadeCarController carController;

    private List<Subscription> subscriptions;
    private List<Publisher> publishers;

    private Queue<string> sendQueue;

    public bool isReady;


    // public Subscription CreateSubscription<T>(string topic, Action<T> callback)
    // {
    //     string guid = Guid.NewGuid().ToString();
    //     Subscription<T> sub = new Subscription<T>(guid, topic, callback);
    //     subscriptions.Add((Subscription<RosMessage>)sub);

    //     Debug.Log($"Added subscription with ID {sub.id}");

    //     return sub;
    // }

    public Publisher CreatePublisher(string topic, string type)
    {
        string guid = Guid.NewGuid().ToString();
        Publisher pub = new Publisher(guid, topic, sendQueue);
        publishers.Add(pub);

        RosbridgeMessage req = new()
        {
            op = "advertise",
            topic = topic,
            type = type
        };

        string reqString = JsonUtility.ToJson(req);

        Debug.Log($"Queued advertisement for {topic}");

        // websocket.SendText(reqString);
        sendQueue.Enqueue(reqString);

        Debug.Log($"Added publisher with ID {pub.id}");

        return pub;
    }

    // private TwistMsg ToTwist(JSONNode str)
    // {
    //     TwistMsg msg = new();

    //     msg.linear.x = str["linear"]["x"];
    //     msg.linear.y = str["linear"]["y"];
    //     msg.linear.z = str["linear"]["z"];

    //     msg.angular.x = str["angular"]["x"];
    //     msg.angular.y = str["angular"]["y"];
    //     msg.angular.z = str["angular"]["z"];

    //     return msg;
    // }

    async void Start()
    {
        isReady = false;
        websocket = new WebSocket("ws://localhost:9090");

        publishers = new() { };
        subscriptions = new() { };
        sendQueue = new();

        carController = GetComponent<ArcadeCarController>();

        websocket.OnOpen += async () =>
        {
            Debug.Log("Connection open!");
            isReady = true;

            SubscriptionRequest req = new()
            {
                op = "subscribe",
                topic = "/cmd_vel"
            };

            string reqString = JsonUtility.ToJson(req);

            Debug.Log($"Sending: {reqString} {req}");

            await websocket.SendText(reqString);


        };

        websocket.OnError += (e) =>
            {
                Debug.Log("Error! " + e);
            };

        websocket.OnClose += (e) =>
        {
            Debug.Log("Connection closed!");
            isReady = false;
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
                // TwistMsg msg = ToTwist(root["msg"]);
                // carController.throttle = msg.linear.x;
                // carController.turn = msg.angular.z;
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

        if (isReady)
        {
            while (sendQueue.Count > 0)
            {
                string request = sendQueue.Dequeue();
                // Debug.Log($"Sending {request}");
                websocket.SendText(request);
            }
        }
    }

    public void scheduleMessage(string message)
    {
        sendQueue.Enqueue(message);
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
