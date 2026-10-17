using OrbicalcLib.Util;
using PeterO.Numbers;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;

namespace OrbicalcLib.Data
{
    public class SimObject
    {
        public BigFloatVector2 Position { get; internal set; } = BigFloatVector2.Zero;
        public BigFloatVector2 Velocity { get; internal set; } = BigFloatVector2.Zero;
        public BigFloatVector2 Acceleration { get; internal set; } = BigFloatVector2.Zero;
        public EDecimal Mass { get; internal set; } = EDecimal.Zero;
        public EDecimal Size { get; internal set; } = EDecimal.Zero;
        public EDecimal DoomSize { get; internal set; } = EDecimal.Zero;

        public SimObject(BigFloatVector2 position, EDecimal mass, EDecimal size)
        {
            Position = position;
            Mass = mass;
            Size = size;
        }

        public SimObject(BigFloatVector2 position, BigFloatVector2 veclocity, EDecimal mass, EDecimal size)
        {
            Position = position;
            Velocity = veclocity;
            Mass = mass;
            Size = size;
        }

        public SimObject(BigFloatVector2 position, EDecimal mass, EDecimal size, EDecimal doomSize)
        {
            Position = position;
            Mass = mass;
            Size = size;
            DoomSize = doomSize;
        }

        public SimObject(BigFloatVector2 position, BigFloatVector2 velocity, EDecimal mass, EDecimal size, EDecimal doomSize)
        {
            Position = position;
            Velocity = velocity;
            Mass = mass;
            Size = size;
            DoomSize = doomSize;
        }
    }
}
