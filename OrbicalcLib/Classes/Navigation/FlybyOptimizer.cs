using OrbicalcLib.Data;
using OrbicalcLib.Util;
using PeterO.Numbers;

namespace OrbicalcLib.Classes.Navigation
{
    public static class FlybyOptimizer
    {
        public record OptimizationResult(
            BigFloatVector2 BestDeltaV,
            EDecimal MinimumDistanceMeters,
            bool Converged
        );

        public static OptimizationResult OptimizeManeuver(
    SimObject spaceship,
    SimObject[] planets,
    int targetPlanetIndex,
    long departureTick,
    long currentTick,
    EDecimal fixedDeltaTimeSeconds,
    int totalSteps,
    BigFloatVector2 initialGuessDeltaV,
    int maxIterations = 50,
    double toleranceMeters = 1000000.0)
        {
            var ctx = Constants.PrecisionContext;

            var currentDeltaV = initialGuessDeltaV;

            var bestDeltaV = initialGuessDeltaV;
            EDecimal bestDistance = EDecimal.FromString("1e35");

            EDecimal step = EDecimal.FromString("0.001");
            EDecimal learningRate = EDecimal.FromString("0.000001");
            EDecimal maxStep = EDecimal.FromString("50.0");
            EDecimal toleranceED = EDecimal.FromDouble(toleranceMeters);

            for (int iter = 0; iter < maxIterations; iter++)
            {
                EDecimal currentDist = EvaluateMissDistance(
                    spaceship, planets, targetPlanetIndex,
                    departureTick, currentTick, fixedDeltaTimeSeconds,
                    totalSteps, currentDeltaV);

                if (currentDist.CompareTo(bestDistance) < 0)
                {
                    bestDistance = currentDist;
                    bestDeltaV = currentDeltaV;
                }

                if (currentDist.CompareTo(toleranceED) <= 0)
                {
                    return new OptimizationResult(bestDeltaV, bestDistance, true);
                }

                var deltaVXPlus = new BigFloatVector2(
                    currentDeltaV.X.Add(step, ctx), currentDeltaV.Y);
                EDecimal distXPlus = EvaluateMissDistance(
                    spaceship, planets, targetPlanetIndex,
                    departureTick, currentTick, fixedDeltaTimeSeconds,
                    totalSteps, deltaVXPlus);

                var deltaVYPlus = new BigFloatVector2(
                    currentDeltaV.X, currentDeltaV.Y.Add(step, ctx));
                EDecimal distYPlus = EvaluateMissDistance(
                    spaceship, planets, targetPlanetIndex,
                    departureTick, currentTick, fixedDeltaTimeSeconds,
                    totalSteps, deltaVYPlus);

                EDecimal gradX = distXPlus.Subtract(currentDist, ctx).Divide(step, ctx);
                EDecimal gradY = distYPlus.Subtract(currentDist, ctx).Divide(step, ctx);

                EDecimal changeX = gradX.Multiply(learningRate, ctx);
                EDecimal changeY = gradY.Multiply(learningRate, ctx);

                if (changeX.Abs().CompareTo(maxStep) > 0)
                    changeX = changeX.CompareTo(EDecimal.Zero) > 0 ? maxStep : maxStep.Negate();
                if (changeY.Abs().CompareTo(maxStep) > 0)
                    changeY = changeY.CompareTo(EDecimal.Zero) > 0 ? maxStep : maxStep.Negate();

                EDecimal newVx = currentDeltaV.X.Subtract(changeX, ctx);
                EDecimal newVy = currentDeltaV.Y.Subtract(changeY, ctx);

                currentDeltaV = new BigFloatVector2(newVx, newVy);
            }

            return new OptimizationResult(bestDeltaV, bestDistance, false);
        }

        private static EDecimal EvaluateMissDistance(
            SimObject spaceship,
            SimObject[] planets,
            int targetPlanetIndex,
            long departureTick,
            long currentTick,
            EDecimal fixedDeltaTimeSeconds,
            int totalSteps,
            BigFloatVector2 testDeltaV)
        {
            var testNode = new ManeuverNode(departureTick, testDeltaV);
            var nodes = new List<ManeuverNode> { testNode };

            var trajectory = TrajectoryPredictor.PredictTrajectory(
                spaceship, planets, nodes, currentTick, fixedDeltaTimeSeconds, totalSteps);

            var targetPlanet = planets[targetPlanetIndex];
            var sun = planets[0]; //Солнце
            EDecimal minDistanceSq = EDecimal.FromString("1e70");

            for (int step = 0; step < trajectory.Count; step++)
            {
                var shipPos = trajectory[step];

                EDecimal stepTime = fixedDeltaTimeSeconds.Multiply(
                    EDecimal.FromInt32(step + 1), Constants.PrecisionContext);

                BigFloatVector2 planetPosAtStep = GravityAssistPlanner.PredictPlanetPosition(
                    targetPlanet, sun.Position, stepTime, sun.Mass);

                BigFloatVector2 diff = shipPos - planetPosAtStep;
                EDecimal distSq = diff.X.Multiply(diff.X, Constants.PrecisionContext)
                    .Add(diff.Y.Multiply(diff.Y, Constants.PrecisionContext), Constants.PrecisionContext);

                if (distSq.CompareTo(minDistanceSq) < 0)
                {
                    minDistanceSq = distSq;
                }
            }

            return minDistanceSq.Sqrt(Constants.PrecisionContext);
        }
    }
}
