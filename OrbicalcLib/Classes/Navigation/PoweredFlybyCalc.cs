using OrbicalcLib.Util;
using PeterO.Numbers;
using System;
using System.Collections.Generic;
using System.Text;

namespace OrbicalcLib.Classes.Navigation
{
    public static class PoweredFlybyCalculator
    {
        public record FlybyResult(
            EDecimal VInfInMag,
            EDecimal VInfOutMag,
            EDecimal DeflectionAngleRad,
            EDecimal PeriapsisVelocityBefore,
            EDecimal PeriapsisVelocityAfter
        );

        public static FlybyResult CalculatePoweredFlyby(
            BigFloatVector2 vInfIn,
            EDecimal planetMass,
            EDecimal periapsisRadius,
            EDecimal deltaVBurn)
        {
            EDecimal vInfInMag = vInfIn.Magnitude;
            EDecimal vInfInSq = vInfInMag.Multiply(vInfInMag, Constants.PrecisionContext);

            EDecimal mu = Constants.G.Multiply(planetMass, Constants.PrecisionContext);

            EDecimal escapeVelSq = EDecimal.FromInt32(2)
                .Multiply(mu, Constants.PrecisionContext)
                .Divide(periapsisRadius, Constants.PrecisionContext);

            EDecimal vpBefore = vInfInSq.Add(escapeVelSq, Constants.PrecisionContext).Sqrt(Constants.PrecisionContext);

            EDecimal vpAfter = vpBefore.Add(deltaVBurn, Constants.PrecisionContext);
            EDecimal vpAfterSq = vpAfter.Multiply(vpAfter, Constants.PrecisionContext);

            EDecimal vInfOutSq = vpAfterSq.Subtract(escapeVelSq, Constants.PrecisionContext);
            EDecimal vInfOutMag = vInfOutSq.CompareTo(EDecimal.Zero) > 0
                ? vInfOutSq.Sqrt(Constants.PrecisionContext)
                : EDecimal.Zero;

            EDecimal eIn = EDecimal.One.Add(
                periapsisRadius.Multiply(vInfInSq, Constants.PrecisionContext)
                .Divide(mu, Constants.PrecisionContext), Constants.PrecisionContext);

            EDecimal eOut = EDecimal.One.Add(
                periapsisRadius.Multiply(vInfOutSq, Constants.PrecisionContext)
                .Divide(mu, Constants.PrecisionContext), Constants.PrecisionContext);

            double eInDouble = eIn.ToDouble();
            double eOutDouble = eOut.ToDouble();

            double deflectionAngle = Math.Asin(1.0 / eInDouble) + Math.Asin(1.0 / eOutDouble);

            return new FlybyResult(
                vInfInMag,
                vInfOutMag,
                EDecimal.FromDouble(deflectionAngle),
                vpBefore,
                vpAfter
            );
        }
    }
}
