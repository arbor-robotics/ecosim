using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using NativeWebSocket;
using System;

[Serializable]
public struct SubscriptionRequest
{
    public string op;
    public string topic;
}

public class WebsocketBridge : MonoBehaviour
{
    WebSocket websocket;

    // Start is called before the first frame update
    async void Start()
    {
        websocket = new WebSocket("ws://localhost:9090");

        websocket.OnOpen += async () =>
        {
            Debug.Log("Connection open!");

            SubscriptionRequest req = new()
            {
                op = "subscribe",
                topic = "/test"
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
        };

        websocket.OnMessage += (bytes) =>
        {
            // Debug.Log("OnMessage!");
            // Debug.Log(bytes);

            // getting the message as a string
            var message = System.Text.Encoding.UTF8.GetString(bytes);
            Debug.Log("OnMessage! " + message);
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
