using OrbicalcLib.Data;
using OrbicalcLib.Util;
using PeterO.Numbers;
using System;
using System.Collections.Generic;

namespace OrbicalcLib.Classes.Navigation
{
    public class TrajectoryPredictor
    {
        public static List<BigFloatVector2> PredictTrajectory(
            SimObject spaceship,
            SimObject[] planets,
            List<ManeuverNode> nodes,
            long currentTick,
            EDecimal fixedDeltaTimeSeconds,
            int totalSteps)
        {
            var trajectory = new List<BigFloatVector2>(totalSteps);

            var shipPos = spaceship.Position;
            var shipVel = spaceship.Velocity;

            int planetCount = planets.Length;
            var planetPositions = new BigFloatVector2[planetCount];
            var planetVelocities = new BigFloatVector2[planetCount];
            var planetMasses = new EDecimal[planetCount];

            for (int i = 0; i < planetCount; i++)
            {
                planetPositions[i] = planets[i].Position;
                planetVelocities[i] = planets[i].Velocity;
                planetMasses[i] = planets[i].Mass;
            }

            EDecimal dt = fixedDeltaTimeSeconds;
            EDecimal halfDt = dt.Multiply(Constants.Half, Constants.PrecisionContext);
            EDecimal dtSquare = dt.Multiply(dt, Constants.PrecisionContext);

            var activeNodes = new List<ManeuverNode>();
            foreach (var node in nodes)
            {
                if (node.TargetTick >= currentTick)
                {
                    activeNodes.Add(new ManeuverNode(node.TargetTick, node.DeltaV));
                }
            }

            BigFloatVector2 shipAcc = CalculateShipAcceleration(shipPos, planetPositions, planetMasses);
            long simTick = currentTick;

            for (int step = 0; step < totalSteps; step++)
            {
                for (int i = 0; i < activeNodes.Count; i++)
                {
                    var node = activeNodes[i];
                    if (!node.Executed && simTick >= node.TargetTick)
                    {
                        shipVel += node.DeltaV;
                        node.Executed = true;
                        activeNodes[i] = node;
                    }
                }

                BigFloatVector2 accTerm = shipAcc * Constants.Half * dtSquare;
                shipPos += (shipVel * dt) + accTerm;
                shipVel += shipAcc * halfDt;

                for (int p = 1; p < planetCount; p++)
                {
                    EDecimal stepTime = dt.Multiply(EDecimal.FromInt32(step + 1), Constants.PrecisionContext);
                    planetPositions[p] = GravityAssistPlanner.PredictPlanetPosition(planets[p], planetPositions[0], stepTime, planetMasses[0]);
                }

                BigFloatVector2 newAcc = CalculateShipAcceleration(shipPos, planetPositions, planetMasses);
                shipVel += newAcc * halfDt;
                shipAcc = newAcc;

                simTick++;
                trajectory.Add(shipPos);
            }

            return trajectory;
        }

        private static BigFloatVector2 CalculateShipAcceleration(
            BigFloatVector2 shipPos,
            BigFloatVector2[] planetPositions,
            EDecimal[] planetMasses)
        {
            BigFloatVector2 acc = BigFloatVector2.Zero;
            EDecimal softeningSq = EDecimal.FromString("1000000000000");

            for (int i = 0; i < planetPositions.Length; i++)
            {
                BigFloatVector2 dir = planetPositions[i] - shipPos;
                EDecimal distSq = dir.X.Multiply(dir.X, Constants.PrecisionContext)
                    .Add(dir.Y.Multiply(dir.Y, Constants.PrecisionContext), Constants.PrecisionContext);

                if (distSq.IsZero) continue;

                EDecimal softenedSq = distSq.Add(softeningSq, Constants.PrecisionContext);
                EDecimal dist = softenedSq.Sqrt(Constants.PrecisionContext);
                EDecimal distCubed = dist.Multiply(softenedSq, Constants.PrecisionContext);

                EDecimal gMass = Constants.G.Multiply(planetMasses[i], Constants.PrecisionContext);
                EDecimal factor = gMass.Divide(distCubed, Constants.PrecisionContext);

                acc += dir * factor;
            }

            return acc;
        }
    }
}