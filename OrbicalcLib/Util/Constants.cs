using PeterO.Numbers;
using System;
using System.Collections.Generic;
using System.Text;

namespace OrbicalcLib.Util
{
    public static class Constants
    {
        public static readonly EContext PrecisionContext = EContext.ForPrecision(34);
        public static readonly EDecimal G = EDecimal.FromString("6.67430e-11");
        public static readonly EDecimal Half = EDecimal.FromString("0.5");
    }
}
