using OrbicalcLib.Data;
using OrbicalcLib.Interfaces;
using OrbicalcLib.Util;
using PeterO.Numbers;
using System;
using System.Collections.Generic;
using System.Text;

namespace OrbicalcLib.Modules
{
    public class SpaceshipGravityModule : IEngineModule
    {
        private readonly IObjectHandler _simObjectsHandler;
        private readonly ISingleObjectHandler _spaceshipHandler;

        public SpaceshipGravityModule(IObjectHandler objectHandler, ISingleObjectHandler singleObjectHandler)
        {
            _simObjectsHandler = objectHandler;
            _spaceshipHandler = singleObjectHandler;
        }

        public void Update(TimeSpan deltaTime, TimeSpan fixedDeltaTime)
        {
            var spaceship = _spaceshipHandler.Object;
            if (spaceship == null) return;

            EDecimal dt = EDecimal.FromDouble(fixedDeltaTime.TotalSeconds);
            EDecimal halfDt = dt.Multiply(Constants.Half, Constants.PrecisionContext);
            EDecimal dtSquare = dt.Multiply(dt, Constants.PrecisionContext);

            BigFloatVector2 accTerm = spaceship.Acceleration * Constants.Half * dtSquare;
            spaceship.Position += (spaceship.Velocity * dt) + accTerm;
            spaceship.Velocity += spaceship.Acceleration * halfDt;

            BigFloatVector2 newAcceleration = CalculateAcceleration(spaceship, _simObjectsHandler.Objects);
            spaceship.Velocity += newAcceleration * halfDt;
            spaceship.Acceleration = newAcceleration;
        }

        public BigFloatVector2 CalculateAcceleration(SimObject spaceship, SimObject[] simObjects)
        {
            BigFloatVector2 totalAcceleration = BigFloatVector2.Zero;
            EDecimal softening = EDecimal.FromInt64(1_000_000);
            EDecimal softeningSquare = softening.Multiply(softening, Constants.PrecisionContext);

            for (int i = 0; i < simObjects.Length; i++)
            {
                var body = simObjects[i];
                BigFloatVector2 direction = body.Position - spaceship.Position;

                EDecimal dx = direction.X;
                EDecimal dy = direction.Y;

                EDecimal distSquare = (dx.Multiply(dx, Constants.PrecisionContext))
                    .Add(dy.Multiply(dy, Constants.PrecisionContext), Constants.PrecisionContext);

                if (distSquare.IsZero) continue;

                EDecimal softenedDistSquare = distSquare.Add(softeningSquare, Constants.PrecisionContext);
                EDecimal distance = softenedDistSquare.Sqrt(Constants.PrecisionContext);
                EDecimal distCubed = distance.Multiply(softenedDistSquare, Constants.PrecisionContext);
                EDecimal gMass = Constants.G.Multiply(body.Mass, Constants.PrecisionContext);
                EDecimal factor = gMass.Divide(distCubed, Constants.PrecisionContext);

                totalAcceleration += direction * factor;
            }

            return totalAcceleration;
        }
    }
}
