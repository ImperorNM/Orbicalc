using PeterO.Numbers;

namespace OrbicalcLib.Util
{
    public struct BigFloatVector2
    {
        public EDecimal X { get; internal set; }
        public EDecimal Y { get; internal set; }
        public readonly EDecimal Magnitude
        {
            get
            {
                return X.Multiply(X, Constants.PrecisionContext)
                    .Add(Y.Multiply(Y, Constants.PrecisionContext), Constants.PrecisionContext)
                    .Sqrt(Constants.PrecisionContext);
            }
        }
        public readonly BigFloatVector2 Normalized
        {
            get
            {
                EDecimal magnitude = Magnitude;
                if (magnitude.CompareTo(EDecimal.FromInt32(0)) > 0)
                {
                    return new BigFloatVector2(X.Divide(magnitude, Constants.PrecisionContext), Y.Divide(magnitude, Constants.PrecisionContext));
                }
                return new BigFloatVector2(0, 0);
            }
        }
        public static BigFloatVector2 Zero
        {
            get
            {
                return new BigFloatVector2(0, 0);
            }
        }

        public BigFloatVector2(EDecimal x, EDecimal y)
        {
            X = x;
            Y = y;
        }

        public static BigFloatVector2 operator+(BigFloatVector2 a, BigFloatVector2 b)
        {
            return new BigFloatVector2(a.X.Add(b.X, Constants.PrecisionContext), a.Y.Add(b.Y, Constants.PrecisionContext));
        }

        public static BigFloatVector2 operator-(BigFloatVector2 a, BigFloatVector2 b)
        {
            return new BigFloatVector2(a.X.Subtract(b.X, Constants.PrecisionContext), a.Y.Subtract(b.Y, Constants.PrecisionContext));
        }

        public static BigFloatVector2 operator*(BigFloatVector2 a, EDecimal b)
        {
            return new BigFloatVector2(a.X.Multiply(b, Constants.PrecisionContext), a.Y.Multiply(b, Constants.PrecisionContext));
        }

        public static BigFloatVector2 operator /(BigFloatVector2 a, EDecimal b)
        {
            return new BigFloatVector2(a.X.Divide(b, Constants.PrecisionContext), a.Y.Divide(b, Constants.PrecisionContext));
        }
    }
}
