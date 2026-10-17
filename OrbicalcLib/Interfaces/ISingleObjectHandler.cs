using OrbicalcLib.Data;
using System;
using System.Collections.Generic;
using System.Text;

namespace OrbicalcLib.Interfaces
{
    public interface ISingleObjectHandler
    {
        SimObject? Object { get; set; }
    }
}
