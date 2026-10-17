using OrbicalcLib.Data;
using OrbicalcLib.Interfaces;
using OrbicalcLib.Util;
using PeterO.Numbers;
using System.Net.Security;


namespace OrbicalcLib.Modules
{
    public class GravityModule : IEngineModule
    {
        private readonly IObjectHandler _simObjectHandler;

        public GravityModule(IObjectHandler simObjectHandler)
        {
            _simObjectHandler = simObjectHandler;

            var objects = _simObjectHandler.Objects;
            if (objects.Length > 0)
            {
                var initialAccs = CalculateAccelerations(objects);
                for (int i = 0; i < objects.Length; i++)
                {
                    objects[i].Acceleration = initialAccs[i];
                }
            }
        }

        public void Update(TimeSpan deltaTime, TimeSpan fixedDeltaTime)
        {
            var objects = _simObjectHandler.Objects;
            EDecimal dt = EDecimal.FromDouble(fixedDeltaTime.TotalSeconds);
            EDecimal halfDt = dt.Multiply(Constants.Half, Constants.PrecisionContext);
            EDecimal dtSquare = dt.Multiply(dt, Constants.PrecisionContext);

            foreach (var obj in objects)
            {
                BigFloatVector2 accTerm = obj.Acceleration * Constants.Half * dtSquare;
                obj.Position += (obj.Velocity * dt) + accTerm;

                obj.Velocity += obj.Acceleration * halfDt;
            }

            BigFloatVector2[] newAccelerations = CalculateAccelerations(objects);

            for (int i = 0; i < objects.Length; i++)
            {
                var obj = objects[i];
                BigFloatVector2 newAcc = newAccelerations[i];
                obj.Velocity += newAcc * halfDt;
                obj.Acceleration = newAcc;
            }
        }

        private BigFloatVector2[] CalculateAccelerations(SimObject[] objects)
        {
            var newAccelerations = new BigFloatVector2[objects.Length];

            for (int i = 0; i < objects.Length; i++)
            {
                newAccelerations[i] = BigFloatVector2.Zero;
            }

            for (int i = 0; i < objects.Length; i++)
            {
                for (int j = i + 1; j < objects.Length; j++)
                {
                    var objA = objects[i];
                    var objB = objects[j];

                    BigFloatVector2 direction = objB.Position - objA.Position;

                    EDecimal distanceSqr = direction.X.Multiply(direction.X, Constants.PrecisionContext)
                        .Add(direction.Y.Multiply(direction.Y, Constants.PrecisionContext), Constants.PrecisionContext);

                    if (distanceSqr.CompareTo(EDecimal.FromString("1e-10")) < 0)
                        continue;

                    EDecimal distance = direction.Magnitude;
                    BigFloatVector2 normal = direction / distance;

                    EDecimal forceMagA = Constants.G.Multiply(objB.Mass, Constants.PrecisionContext)
                                                    .Divide(distanceSqr, Constants.PrecisionContext);

                    EDecimal forceMagB = Constants.G.Multiply(objA.Mass, Constants.PrecisionContext)
                                                    .Divide(distanceSqr, Constants.PrecisionContext);

                    newAccelerations[i] += normal * forceMagA;
                    newAccelerations[j] -= normal * forceMagB;
                }
            }

            return newAccelerations;
        }
    }
}
