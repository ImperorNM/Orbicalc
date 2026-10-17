using OrbicalcLib.Data;
using OrbicalcLib.Interfaces;
using System.Collections.Immutable;
using System.Linq;

namespace OrbicalcLib.Handlers
{
    public class SimObjectHandler : IObjectHandler
    {
        private volatile SimObject[] _objects = [];
        private readonly Lock _lock = new();

        public SimObject[] Objects
        {
            get
            {
                return _objects;
            }
        }

        public void Add(SimObject simObject)
        {
            ArgumentNullException.ThrowIfNull(simObject);
            lock (_lock)
            {
                var current = _objects;
                var next = new SimObject[current.Length + 1];

                Array.Copy(current, next, current.Length);
                next[^1] = simObject;

                _objects = next;
            }
        }

        public bool Remove(SimObject simObject)
        {
            ArgumentNullException.ThrowIfNull(simObject);
            lock (_lock)
            {
                var current = _objects;
                int index = Array.IndexOf(current, simObject);

                if (index < 0) return false;

                var next = new SimObject[current.Length - 1];
                if (index > 0)
                {
                    Array.Copy(current, 0, next, 0, index);
                }
                if (index < current.Length - 1)
                {
                    Array.Copy(current, index + 1, next, index, current.Length - index - 1);
                }

                _objects = next;
                return true;
            }
        }
    }
}
