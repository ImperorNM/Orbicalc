using OrbicalcLib.Data;
using System;
using System.Collections.Generic;
using System.Text;

namespace OrbicalcLib.Interfaces
{
    public interface IObjectHandler
    {
        SimObject[] Objects { get; }
        void Add(SimObject simObject);
        bool Remove(SimObject simObject);
    }
}
