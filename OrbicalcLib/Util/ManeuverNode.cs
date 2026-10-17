using OrbicalcLib.Util;
using PeterO.Numbers;

namespace OrbicalcLib.Util
{
    public struct ManeuverNode
    {
        public long TargetTick { get; internal set; }
        public BigFloatVector2 DeltaV { get; internal set; }
        public bool Executed { get; internal set; }

        public ManeuverNode(long targetTick, BigFloatVector2 deltaV)
        {
            TargetTick = targetTick;
            DeltaV = deltaV;
            Executed = false;
        }
    }
}