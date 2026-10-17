using OrbicalcLib.Data;
using OrbicalcLib.Util;
using PeterO.Numbers;
using System;
using System.Collections.Generic;

namespace OrbicalcLib.Classes.Navigation
{
    public record FlybyPlanResult(
        bool Success,
        long DepartureTick,
        long FlybyTick,
        long ArrivalTick,
        BigFloatVector2 DepartureDeltaV,
        BigFloatVector2 FlybyBurnDeltaV,
        EDecimal FlybyPeriapsisKm,
        EDecimal TotalDeltaV
    );

    public static class GravityAssistPlanner
    {
        public static BigFloatVector2 SolveLambert2(
            BigFloatVector2 pos1,
            BigFloatVector2 pos2,
            EDecimal flightTimeSeconds,
            EDecimal centralBodyMass)
        {
            if (flightTimeSeconds.CompareTo(EDecimal.Zero) <= 0)
                return BigFloatVector2.Zero;

            double dt = flightTimeSeconds.ToDouble();
            double r1 = pos1.Magnitude.ToDouble();
            double r2 = pos2.Magnitude.ToDouble();

            double x1 = pos1.X.ToDouble();
            double y1 = pos1.Y.ToDouble();
            double x2 = pos2.X.ToDouble();
            double y2 = pos2.Y.ToDouble();

            double chord = Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
            if (chord < 1e-5) return BigFloatVector2.Zero;

            double semiPerimeter = (r1 + r2 + chord) / 2.0;
            double mu = (Constants.G.Multiply(centralBodyMass, Constants.PrecisionContext)).ToDouble();

            double aMin = semiPerimeter / 2.0;
            double alpha0 = 2.0 * Math.Asin(Math.Min(1.0, Math.Sqrt(semiPerimeter / (2.0 * aMin))));
            double beta0 = 2.0 * Math.Asin(Math.Min(1.0, Math.Sqrt((semiPerimeter - chord) / (2.0 * aMin))));
            double tMin = Math.Sqrt((aMin * aMin * aMin) / mu) * ((alpha0 - beta0) - (Math.Sin(alpha0) - Math.Sin(beta0)));

            double a = aMin;
            if (dt > tMin)
            {
                a = aMin * Math.Pow(dt / tMin, 0.66);
            }

            double alpha = 2.0 * Math.Asin(Math.Min(1.0, Math.Sqrt(semiPerimeter / (2.0 * a))));
            double beta = 2.0 * Math.Asin(Math.Min(1.0, Math.Sqrt((semiPerimeter - chord) / (2.0 * a))));

            double A = Math.Sqrt(mu / (4.0 * a)) * (1.0 / Math.Tan(alpha / 2.0) + 1.0 / Math.Tan(beta / 2.0));
            double B = Math.Sqrt(mu / (4.0 * a)) * (1.0 / Math.Tan(alpha / 2.0) - 1.0 / Math.Tan(beta / 2.0));

            double u1X = (x2 - x1) / chord;
            double u1Y = (y2 - y1) / chord;

            double v1X = (B + A) * (x1 / r1) + (B - A) * u1X;
            double v1Y = (B + A) * (y1 / r1) + (B - A) * u1Y;

            return new BigFloatVector2(EDecimal.FromDouble(v1X), EDecimal.FromDouble(v1Y));
        }

        public static BigFloatVector2 SolveLambert(
            BigFloatVector2 pos1,
            BigFloatVector2 pos2,
            EDecimal flightTimeSeconds,
            EDecimal centralBodyMass)
        {
            var ctx = Constants.PrecisionContext;

            if (flightTimeSeconds.CompareTo(EDecimal.Zero) <= 0)
                return BigFloatVector2.Zero;

            EDecimal x1 = pos1.X, y1 = pos1.Y;
            EDecimal x2 = pos2.X, y2 = pos2.Y;

            EDecimal r1sq = x1.Multiply(x1, ctx).Add(y1.Multiply(y1, ctx), ctx);
            EDecimal r2sq = x2.Multiply(x2, ctx).Add(y2.Multiply(y2, ctx), ctx);
            EDecimal r1 = r1sq.Sqrt(ctx);
            EDecimal r2 = r2sq.Sqrt(ctx);

            if (r1.IsZero || r2.IsZero) return BigFloatVector2.Zero;

            EDecimal mu = Constants.G.Multiply(centralBodyMass, ctx);
            if (mu.CompareTo(EDecimal.Zero) <= 0) return BigFloatVector2.Zero;

            EDecimal dot = x1.Multiply(x2, ctx).Add(y1.Multiply(y2, ctx), ctx);
            EDecimal cross = x1.Multiply(y2, ctx).Subtract(y1.Multiply(x2, ctx), ctx);

            EDecimal r1r2 = r1.Multiply(r2, ctx);

            EDecimal cosDnu = dot.Divide(r1r2, ctx);
            EDecimal one = EDecimal.One;
            EDecimal negOne = one.Negate();
            if (cosDnu.CompareTo(one) > 0) cosDnu = one;
            if (cosDnu.CompareTo(negOne) < 0) cosDnu = negOne;

            EDecimal sinDnuMag = cross.Abs().Divide(r1r2, ctx);

            EDecimal denom = one.Subtract(cosDnu, ctx);
            EDecimal tinyDenom = EDecimal.FromString("1e-20");
            if (denom.CompareTo(tinyDenom) < 0) denom = tinyDenom;

            // A = sinDnu * sqrt(r1*r2 / (1-cos))
            EDecimal A = sinDnuMag.Multiply(r1r2.Divide(denom, ctx).Sqrt(ctx), ctx);
            if (A.Abs().CompareTo(EDecimal.FromString("1e-12")) < 0)
                return BigFloatVector2.Zero;

            EDecimal sqrtMu = mu.Sqrt(ctx);

            double zLow = -4.0 * Math.PI * Math.PI + 1e-3;
            double zHigh = 4.0 * Math.PI * Math.PI - 1e-3;
            double z = 0.0;

            EDecimal yFinal = EDecimal.Zero;
            bool found = false;

            for (int iter = 0; iter < 200; iter++)
            {
                (EDecimal C, EDecimal S) = StumpffED(z);

                EDecimal sqrtC = C.Sqrt(ctx);
                if (sqrtC.IsZero) { zHigh = z; z = 0.5 * (zLow + zHigh); continue; }

                // y = r1 + r2 + A * (z*S - 1) / sqrt(C)
                EDecimal zED = EDecimal.FromDouble(z);
                EDecimal num = zED.Multiply(S, ctx).Subtract(one, ctx);
                EDecimal y = r1.Add(r2, ctx)
                              .Add(A.Multiply(num, ctx).Divide(sqrtC, ctx), ctx);

                if (y.CompareTo(EDecimal.Zero) <= 0)
                {
                    zHigh = z;
                    z = 0.5 * (zLow + zHigh);
                    continue;
                }

                // F = (y/C)^1.5 * S + A*sqrt(y) - sqrt(mu)*dt
                EDecimal yOverC = y.Divide(C, ctx);
                EDecimal yOverC15 = yOverC.Sqrt(ctx).Multiply(yOverC, ctx);  // (y/C)^1.5
                EDecimal sqrtY = y.Sqrt(ctx);

                EDecimal F = yOverC15.Multiply(S, ctx)
                    .Add(A.Multiply(sqrtY, ctx), ctx)
                    .Subtract(sqrtMu.Multiply(flightTimeSeconds, ctx), ctx);

                double Fd = F.ToDouble();
                if (Math.Abs(Fd) < 1e-6) { yFinal = y; found = true; break; }
                if (Fd < 0) zLow = z; else zHigh = z;
                z = 0.5 * (zLow + zHigh);
                if (Math.Abs(zHigh - zLow) < 1e-12) { yFinal = y; found = true; break; }
            }

            if (!found) return BigFloatVector2.Zero;

            // f = 1 - y/r1 ;  g = A * sqrt(y/mu)
            EDecimal f = one.Subtract(yFinal.Divide(r1, ctx), ctx);
            EDecimal g = A.Multiply(yFinal.Divide(mu, ctx).Sqrt(ctx), ctx);

            if (g.Abs().CompareTo(EDecimal.FromString("1e-12")) < 0)
                return BigFloatVector2.Zero;

            EDecimal v1x = x2.Subtract(f.Multiply(x1, ctx), ctx).Divide(g, ctx);
            EDecimal v1y = y2.Subtract(f.Multiply(y1, ctx), ctx).Divide(g, ctx);

            return new BigFloatVector2(v1x, v1y);
        }

        public static void DiagnosticLambertTest(SimObject[] planets, EDecimal dtSeconds)
        {
            var ctx = Constants.PrecisionContext;
            var sun = planets[0];
            var earth = planets[3];
            var mars = planets[4];

            long ticks = 2160; // 250 суток при dt=10000
            EDecimal t0 = EDecimal.Zero;
            EDecimal t1 = dtSeconds.Multiply(EDecimal.FromInt64(ticks), ctx);

            var p1 = GravityAssistPlanner.PredictPlanetPosition(earth, sun.Position, t0, sun.Mass);
            var p2 = GravityAssistPlanner.PredictPlanetPosition(mars, sun.Position, t1, sun.Mass);

            var v1 = GravityAssistPlanner.SolveLambert(p1, p2, t1, sun.Mass);

            var ship = new SimObject(p1, v1, EDecimal.Zero, EDecimal.Zero);
            var onlySun = new SimObject[] { sun };
            var traj = TrajectoryPredictor.PredictTrajectory(
                ship, onlySun, new List<ManeuverNode>(), 0, dtSeconds, (int)ticks);

            var reached = traj[traj.Count - 1];
            var miss = (reached - p2).Magnitude;

            System.Diagnostics.Debug.WriteLine($"p1 = {p1.X.ToDouble():E3}, {p1.Y.ToDouble():E3}");
            System.Diagnostics.Debug.WriteLine($"p2 = {p2.X.ToDouble():E3}, {p2.Y.ToDouble():E3}");
            System.Diagnostics.Debug.WriteLine($"v1 = {v1.X.ToDouble():E3}, {v1.Y.ToDouble():E3}  |v1| = {v1.Magnitude.ToDouble():E3} м/с");
            System.Diagnostics.Debug.WriteLine($"|v_earth| = {earth.Velocity.Magnitude.ToDouble():E3} м/с");
            System.Diagnostics.Debug.WriteLine($"ПРОМАХ: {miss.ToDouble():E3} м");
        }

        private static (EDecimal C, EDecimal S) StumpffED(double z)
        {
            if (Math.Abs(z) < 1e-8)
                return (EDecimal.FromString("0.5"), EDecimal.FromString("0.1666666666666667"));

            if (z > 0.0)
            {
                double sz = Math.Sqrt(z);
                double C = (1.0 - Math.Cos(sz)) / z;
                double S = (sz - Math.Sin(sz)) / (sz * sz * sz);
                return (EDecimal.FromDouble(C), EDecimal.FromDouble(S));
            }
            else
            {
                double sz = Math.Sqrt(-z);
                double C = (Math.Cosh(sz) - 1.0) / (-z);
                double S = (Math.Sinh(sz) - sz) / (sz * sz * sz);
                return (EDecimal.FromDouble(C), EDecimal.FromDouble(S));
            }
        }


        public static FlybyPlanResult FindOptimalFlybyPlan(
            SimObject[] planetState,
            int originIndex,
            int flybyIndex,
            int targetIndex,
            long startTick,
            EDecimal dtSeconds)
        {
            var sunMass = planetState[0].Mass;
            BigFloatVector2 sunPos = planetState[0].Position;
            var origin = planetState[originIndex];
            var flybyBody = planetState[flybyIndex];
            var target = planetState[targetIndex];
            
            EDecimal bestTotalDeltaV = EDecimal.FromString("1e35");
            FlybyPlanResult? bestPlan = null;

            long ticksPerDay = (long)(86400.0 / dtSeconds.ToDouble());
            long departureWindowTicks = 500 * ticksPerDay;
            long departureStepTicks = 5 * ticksPerDay;
            long minLegTicks = 80 * ticksPerDay; // 80 суток
            long maxLegTicks = 500 * ticksPerDay; // 500 суток
            long legStepTicks = 10 * ticksPerDay; // 10 суток

            for (long depTick = startTick; depTick < startTick + departureWindowTicks; depTick += departureStepTicks)
            {
                EDecimal depTime = dtSeconds.Multiply(EDecimal.FromInt64(depTick - startTick), Constants.PrecisionContext);

                BigFloatVector2 posOrigin = PredictPlanetPosition(origin, planetState[0].Position, depTime, sunMass);
                BigFloatVector2 velOrigin = PredictPlanetVelocity(origin, sunPos, depTime, sunMass);

                for (long leg1 = minLegTicks; leg1 <= maxLegTicks; leg1 += legStepTicks)
                {
                    long flybyTick = depTick + leg1;
                    EDecimal flybyTime = dtSeconds.Multiply(EDecimal.FromInt64(flybyTick - startTick), Constants.PrecisionContext);
                    EDecimal dt1 = dtSeconds.Multiply(EDecimal.FromInt64(leg1), Constants.PrecisionContext);

                    BigFloatVector2 posFlyby = PredictPlanetPosition(flybyBody, planetState[0].Position, flybyTime, sunMass);

                    BigFloatVector2 velFlyby = PredictPlanetVelocity(flybyBody, sunPos, flybyTime, sunMass);

                    BigFloatVector2 vRequired1 = SolveLambert(posOrigin, posFlyby, dt1, sunMass);
                    BigFloatVector2 deltaV1 = vRequired1 - velOrigin;

                    for (long leg2 = minLegTicks; leg2 <= maxLegTicks; leg2 += legStepTicks)
                    {
                        long arrTick = flybyTick + leg2;
                        EDecimal arrTime = dtSeconds.Multiply(EDecimal.FromInt64(arrTick - startTick), Constants.PrecisionContext);
                        EDecimal dt2 = dtSeconds.Multiply(EDecimal.FromInt64(leg2), Constants.PrecisionContext);

                        BigFloatVector2 posTarget = PredictPlanetPosition(target, planetState[0].Position, arrTime, sunMass);

                        BigFloatVector2 vRequired2 = SolveLambert(posFlyby, posTarget, dt2, sunMass);

                        BigFloatVector2 vInfIn = vRequired1 - velFlyby;
                        BigFloatVector2 vInfOut = vRequired2 - velFlyby;

                        double vInfInMag = vInfIn.Magnitude.ToDouble();
                        double vInfOutMag = vInfOut.Magnitude.ToDouble();

                        if (vInfInMag <= 1.0 || vInfOutMag <= 1.0) continue;

                        EDecimal periapsisRadius = flybyBody.Size.Add(flybyBody.DoomSize, Constants.PrecisionContext);
                        double muFlyby = (Constants.G.Multiply(flybyBody.Mass, Constants.PrecisionContext)).ToDouble();
                        double rp = periapsisRadius.ToDouble();

                        double maxDeflectionAngle = 2.0 * Math.Asin(1.0 / (1.0 + (rp * vInfInMag * vInfInMag / muFlyby)));

                        double dotProduct = (vInfIn.X.ToDouble() * vInfOut.X.ToDouble() + vInfIn.Y.ToDouble() * vInfOut.Y.ToDouble());

                        double clampedCos = Math.Clamp(dotProduct / (vInfInMag * vInfOutMag), -1.0, 1.0);
                        if (double.IsNaN(clampedCos)) clampedCos = 1.0;
                        double reqAngle = Math.Acos(clampedCos);

                        EDecimal flybyBurnMag = EDecimal.Zero;
                        if (reqAngle > maxDeflectionAngle)
                        {
                            double excessAngle = reqAngle - maxDeflectionAngle;
                            flybyBurnMag = EDecimal.FromDouble(vInfInMag * Math.Sin(excessAngle));
                        }
                        else
                        {
                            flybyBurnMag = EDecimal.FromDouble(Math.Abs(vInfOutMag - vInfInMag));
                        }

                        EDecimal totalDeltaV = deltaV1.Magnitude.Add(flybyBurnMag, Constants.PrecisionContext);
                        BigFloatVector2 flybyBurnVector = BigFloatVector2.Zero;
                        double vOutMag = vInfOut.Magnitude.ToDouble();
                        if (vOutMag > 0.0001)
                        {
                            flybyBurnVector = (vInfOut / EDecimal.FromDouble(vOutMag)) * flybyBurnMag;
                        }

                        if (totalDeltaV.CompareTo(EDecimal.FromString("15000")) < 0)
                        {
                            System.Diagnostics.Debug.WriteLine(
                                $"dep={depTick} leg1={leg1} leg2={leg2} " +
                                $"|vReq1|={vRequired1.Magnitude.ToDouble():F0} " +
                                $"|vOrb|={velOrigin.Magnitude.ToDouble():F0} " +
                                $"|Δv1|={deltaV1.Magnitude.ToDouble():F0} " +
                                $"totalΔv={totalDeltaV.ToDouble():F0}");
                        }

                        if (totalDeltaV.CompareTo(bestTotalDeltaV) < 0)
                        {
                            bestTotalDeltaV = totalDeltaV;
                            bestPlan = new FlybyPlanResult(
                                Success: true,
                                DepartureTick: depTick,
                                FlybyTick: flybyTick,
                                ArrivalTick: arrTick,
                                DepartureDeltaV: deltaV1,
                                FlybyBurnDeltaV: flybyBurnVector,
                                FlybyPeriapsisKm: periapsisRadius.Divide(EDecimal.FromString("1000"), Constants.PrecisionContext),
                                TotalDeltaV: totalDeltaV
                            );
                        }
                    }
                }
            }

            if (bestPlan != null && bestPlan.Success)
            {
                var ship = planetState[originIndex];
                var optResult = FlybyOptimizer.OptimizeManeuver(
                    spaceship: ship,
                    planets: planetState,
                    targetPlanetIndex: flybyIndex,
                    departureTick: bestPlan.DepartureTick,
                    currentTick: startTick,
                    fixedDeltaTimeSeconds: dtSeconds,
                    totalSteps: (int)(bestPlan.FlybyTick - startTick),
                    initialGuessDeltaV: bestPlan.DepartureDeltaV,
                    maxIterations: 25,
                    toleranceMeters: 5000000.0
                );

                if (optResult.Converged)
                {
                    bestPlan = bestPlan with { DepartureDeltaV = optResult.BestDeltaV };
                }

                System.Diagnostics.Debug.WriteLine($"=== НАЙДЕН ПЛАН МАНЕВРА ===");
                System.Diagnostics.Debug.WriteLine($"DepartureTick: {bestPlan.DepartureTick}");
                System.Diagnostics.Debug.WriteLine($"FlybyTick:     {bestPlan.FlybyTick} (Разница: {bestPlan.FlybyTick - bestPlan.DepartureTick} тиков)");
                System.Diagnostics.Debug.WriteLine($"ArrivalTick:   {bestPlan.ArrivalTick}");
                System.Diagnostics.Debug.WriteLine($"Departure Δv:  {bestPlan.DepartureDeltaV.Magnitude.ToDouble():F2} m/s");
                System.Diagnostics.Debug.WriteLine($"FlybyBurn Δv:  {bestPlan.FlybyBurnDeltaV.Magnitude.ToDouble():F2} m/s");
                System.Diagnostics.Debug.WriteLine($"Total Δv:      {bestPlan.TotalDeltaV.ToDouble():F2} m/s");
                System.Diagnostics.Debug.WriteLine($"===========================");
            }

            return bestPlan ?? new FlybyPlanResult(false, 0, 0, 0, BigFloatVector2.Zero, BigFloatVector2.Zero, EDecimal.Zero, EDecimal.Zero);
        }

        internal static BigFloatVector2 PredictPlanetPosition(SimObject planet, BigFloatVector2 sunPos, EDecimal timeSeconds, EDecimal sunMass)
        {
            double dt = timeSeconds.ToDouble();
            BigFloatVector2 relPos = planet.Position - sunPos;
            double r = relPos.Magnitude.ToDouble();
            if (r < 1.0) return planet.Position;

            double mu = (Constants.G.Multiply(sunMass, Constants.PrecisionContext)).ToDouble();
            double omega = Math.Sqrt(mu / (r * r * r)); // Угловая скорость
            double dAngle = omega * dt;

            double cosA = Math.Cos(dAngle);
            double sinA = Math.Sin(dAngle);

            double xNew = relPos.X.ToDouble() * cosA - relPos.Y.ToDouble() * sinA;
            double yNew = relPos.X.ToDouble() * sinA + relPos.Y.ToDouble() * cosA;

            return sunPos + new BigFloatVector2(EDecimal.FromDouble(xNew), EDecimal.FromDouble(yNew));
        }

        internal static BigFloatVector2 PredictPlanetVelocity(
            SimObject planet,
            BigFloatVector2 sunPos,
            EDecimal timeSeconds,
            EDecimal sunMass)
        {
            double dt = timeSeconds.ToDouble();

            BigFloatVector2 relPos = planet.Position - sunPos;
            double r = relPos.Magnitude.ToDouble();
            if (r < 1.0) return planet.Velocity;

            double mu = (Constants.G.Multiply(sunMass, Constants.PrecisionContext)).ToDouble();
            double omega = Math.Sqrt(mu / (r * r * r));
            double dAngle = omega * dt;

            double cosA = Math.Cos(dAngle);
            double sinA = Math.Sin(dAngle);

            double vxNew = planet.Velocity.X.ToDouble() * cosA - planet.Velocity.Y.ToDouble() * sinA;
            double vyNew = planet.Velocity.X.ToDouble() * sinA + planet.Velocity.Y.ToDouble() * cosA;

            return new BigFloatVector2(EDecimal.FromDouble(vxNew), EDecimal.FromDouble(vyNew));
        }
    }
}