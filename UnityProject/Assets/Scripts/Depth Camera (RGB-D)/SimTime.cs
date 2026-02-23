// SimTime.cs
using System;
using Unity.Robotics.Core;
using RosMessageTypes.BuiltinInterfaces;

public static class SimTime
{
    public static TimeMsg Now()
    {
        var t = Clock.time;                 // frame-start time (matches example)
        var sec = (int)Math.Floor(t);
        var nsec = (uint)((t - sec) * 1e9);
        return new TimeMsg(sec, nsec);
    }
}
