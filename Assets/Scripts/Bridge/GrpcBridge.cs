using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Grpc.Net.Client;
// using Helloworld;

// using GrpcChannel channel = GrpcChannel.ForAddress("https://localhost:50051");
public class GrpcBridge : MonoBehaviour
{
    // Start is called before the first frame update

    GrpcChannelOptions options;
    // options
    Greeter.GreeterClient client;
    GrpcChannel channel;
    void Start()
    {
        // options.
        channel = GrpcChannel.ForAddress("http://127.0.0.1:50051");
        // Greeter.
        client = new Greeter.GreeterClient(channel);
        var reply = client.SayHello(new HelloRequest { Name = "GreeterClient" });
        Debug.Log(reply.Message);
    }

    // Update is called once per frame
    void Update()
    {

    }
}
