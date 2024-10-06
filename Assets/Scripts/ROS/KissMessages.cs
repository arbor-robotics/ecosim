using System;
using UnityEngine;

namespace KISS
{
    public enum MessageType : byte
    {
        IMAGE = 0x00,
        SUBSCRIBE = 0x01,
        TELEOP = 0x02
    }
}