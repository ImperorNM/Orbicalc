using System;
using System.Collections.Generic;
using System.Data;
using System.Text;

namespace OrbicalcLib.Interfaces
{
    public interface IEngineModule
    {
        void Update(TimeSpan deltaTime, TimeSpan fixedDeltaTime);
    }
}
