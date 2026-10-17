using OrbicalcLib.Data;
using OrbicalcLib.Interfaces;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices.Marshalling;
using System.Text;

namespace OrbicalcLib.Handlers
{
    public class SpaceshipHandler : ISingleObjectHandler
    {
        private volatile SimObject? _object;
        private readonly Lock _lock = new();
        public SimObject? Object
        {
            get
            {
                return _object;
            }
            set
            {
                ArgumentNullException.ThrowIfNull(value);
                lock (_lock)
                {
                    _object = value;
                }
            }
        }
    }
}
