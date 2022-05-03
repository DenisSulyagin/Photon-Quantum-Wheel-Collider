using Photon.Deterministic;

namespace Quantum.Game;

public unsafe class VehicleSystem : SystemMainThread
{
    FP throttle;
    FP brake;
    bool reverseGear;

    FPVector3 vehiclePositionPrev;
    FP vehicleSpeed;

    public override void Update(Frame f)
    {
        var input = f.GetPlayerInput(0);

        UpdateInput();

        var vehicles = f.Unsafe.GetComponentBlockIterator<PlayerVehicle>();

        foreach (var vehicle in vehicles)
        {
            var chassisBody = f.Unsafe.GetPointer<PhysicsBody3D>(vehicle.Component->chassis);
            var chassisTransform = f.Unsafe.GetPointer<Transform3D>(vehicle.Component->chassis);

            var tridimensionalWheels = vehicle.Component->tridimensionalWheels;
            var maxAngularVelocityWheels = vehicle.Component->maxAngularVelocityWheels;

            var axles = vehicle.Component->axles;

            if (input->reset)
            {
                var euler = chassisTransform->EulerAngles;
                chassisTransform->Position += FPVector3.Up * 2;
                chassisTransform->Rotation = FPQuaternion.Euler(new FPVector3(euler.X, euler.Y, 0));
            }

            vehicleSpeed = FPVector3.Dot(chassisTransform->Position - vehiclePositionPrev, chassisTransform->Forward) / f.DeltaTime;
            vehiclePositionPrev = chassisTransform->Position;

            if (FPMath.Abs(vehicleSpeed) <= vehicle.Component->reverseGearSpeedTreshold)
                if (input->brake)
                    reverseGear = true;
                else if (input->throttle)
                    reverseGear = false;

            //Wheels
            for (int i = 0; i < axles.Length; i++)
            {
                var axle = axles[i];

                for (int j = 0; j < axle.wheels.Length; j++)
                {
                    var wheel = axles.GetPointer(i)->wheels.GetPointer(j);
                    var right = wheel->right ? 1 : -1;

                    wheel->dummyLocalPosition = new FPVector3(axle.width * FP._0_50 * right, axle.verticalPosition, axle.horizontalPosition);

                    UpdateWheelSystem(axle, wheel);
                }
            }

            void UpdateWheelSystem(Axle axle, Wheel* wheel)
            {
                var wheelTransform = f.Unsafe.GetPointer<Transform3D>(wheel->wheel);

                var steerAngle = axle.steerAngle * input->steer;

                wheel->dummy.Position = chassisTransform->TransformPoint(wheel->dummyLocalPosition);
                wheel->dummy.Rotation = chassisTransform->Rotation * FPQuaternion.Euler(FPVector3.Up * steerAngle);

                UpdateSuspension(axle, wheel);

                UpdateWheel(axle, wheel, wheelTransform);

                if (wheel->isGrounded)
                {
                    CalculateSlips(axle, wheel);
                    CalculateForcesFromSlips(axle, wheel);

                    chassisBody->AddForceAtPosition(wheel->totalForce, wheelTransform->Position, chassisTransform);
                }
            }

            void UpdateSuspension(Axle axle, Wheel* wheel)
            {
                var hitRDown = f.Physics3D.Raycast(
                    wheel->dummy.Position,
                    -wheel->dummy.Up,
                    axle.wheelRadius + axle.suspensionDistance,
                    -1,
                    QueryOptions.ComputeDetailedInfo | QueryOptions.HitAll
                    );

                if (tridimensionalWheels)
                    UpdateTrideminsional(axle, wheel);// Quantum physics unstable results

                if (hitRDown.HasValue)
                {
                    if (!wheel->isGrounded)
                        wheel->prevPosition = chassisTransform->Position;

                    wheel->isGrounded = true;
                    wheel->point = hitRDown.Value.Point;
                    wheel->normal = hitRDown.Value.Normal;

                    wheel->suspensionCompressionPrev = wheel->suspensionCompression;

                    wheel->suspensionCompression = (1 - hitRDown.Value.CastDistanceNormalized) * (axle.suspensionDistance + axle.wheelRadius);

                    Draw.Ray(wheel->dummy.Position, -wheel->dummy.Up * (axle.wheelRadius + axle.suspensionDistance));
                }
                else
                {
                    wheel->suspensionCompression = 0;
                    wheel->isGrounded = false;
                    wheel->forwardSlip = 0;
                    wheel->sidewaysSlip = 0;

                    Draw.Ray(wheel->dummy.Position, -wheel->dummy.Up * (axle.wheelRadius + axle.suspensionDistance), ColorRGBA.Red);
                }
            }

            Physics3D.Hit3D? UpdateTrideminsional(Axle axle, Wheel* wheel)
            {
                var dummyPosition = wheel->dummy.Position;
                var dummyUp = wheel->dummy.Up;

                var startP = dummyPosition + dummyUp * axle.wheelRadius;
                var translationV = -dummyUp * (axle.wheelRadius + axle.suspensionDistance);

                Shape3D castShape = Shape3D.CreateSphere(axle.wheelRadius);

                var hitS = f.Physics3D.ShapeCast(
                    startP,
                    FPQuaternion.Identity,
                    &castShape,
                    translationV,
                    -1,
                    QueryOptions.ComputeDetailedInfo | QueryOptions.HitAll
                );

                Draw.Sphere(dummyPosition - dummyUp * axle.suspensionDistance, axle.wheelRadius, new ColorRGBA("00FF0040"));

                if (hitS.HasValue)
                {
                    var distance = hitS.Value.CastDistanceNormalized;

                    Draw.Sphere(dummyPosition - dummyUp * distance, axle.wheelRadius, new ColorRGBA("FF000040"));

                    Draw.Line(dummyPosition, hitS.Value.Point, ColorRGBA.Black);
                }

                return hitS;
            }

            void UpdateWheel(Axle axle, Wheel* wheel, Transform3D* wheelTransform)
            {
                var angularVelocity = wheel->angularVelocity;

                //Visual
                wheel->rotationAngle += angularVelocity * f.DeltaTime;

                //Transform
                wheelTransform->Rotation = wheel->dummy.Rotation * FPQuaternion.Euler(FPVector3.Right * wheel->rotationAngle);
                wheelTransform->Position = wheel->dummy.Position - wheel->dummy.Up * (axle.suspensionDistance - wheel->suspensionCompression);

                var torque = axle.motorTorque * throttle;
                var brakeTorque = axle.brakeTorque * brake;

                //Friction Torque
                angularVelocity -= ForwardFriction(wheel, wheel->forwardSlip) / (FP.Pi * 2 * axle.wheelRadius) / axle.wheelMass;

                //Input Torque
                angularVelocity += torque / axle.wheelRadius / axle.wheelMass * f.DeltaTime;

                //Friction
                var friction = (brakeTorque + axle.wheelFriction) * axle.wheelRadius / axle.wheelMass * f.DeltaTime;
                angularVelocity -= FPMath.Sign(angularVelocity) * FPMath.Min(FPMath.Abs(angularVelocity), friction);

                //Clamp Angular Velocity
                angularVelocity = FPMath.Clamp(angularVelocity, -maxAngularVelocityWheels, maxAngularVelocityWheels);

                wheel->angularVelocity = angularVelocity;// caching to next frame


                Draw.Ray(wheel->dummy.Position, wheel->dummy.Right, ColorRGBA.Red);
                Draw.Ray(wheel->dummy.Position, wheel->dummy.Forward, ColorRGBA.Blue);
            }

            void CalculateSlips(Axle axle, Wheel* wheel)
            {
                FPVector3 velocity = (wheel->dummy.Position - wheel->prevPosition) / f.DeltaTime;
                wheel->prevPosition = wheel->dummy.Position;

                FPVector3 forward = wheel->dummy.Forward;
                FPVector3 sideways = -wheel->dummy.Right;

                FPVector3 forwardVelocity = FPVector3.Dot(velocity, forward) * forward;
                FPVector3 sidewaysVelocity = FPVector3.Dot(velocity, sideways) * sideways;

                wheel->forwardSlip = -FPMath.Sign(FPVector3.Dot(forward, forwardVelocity)) * forwardVelocity.Magnitude + (wheel->angularVelocity * FP.Pi / 180 * axle.wheelRadius);
                wheel->sidewaysSlip = -FPMath.Sign(FPVector3.Dot(sideways, sidewaysVelocity)) * sidewaysVelocity.Magnitude;
            }

            void CalculateForcesFromSlips(Axle axle, Wheel* wheel)
            {
                var sin = FPVector3.Dot(wheel->dummy.Right, wheel->normal);
                var right = wheel->dummy.Right - wheel->normal * sin;
                var forward = FPVector3.Cross(right, wheel->normal);

                //Forward slip force
                var forceForward = forward * ForwardFriction(wheel, wheel->forwardSlip);
                wheel->totalForce = forceForward;

                //Lateral slip force
                var forceRight = right * SidewaysFriction(wheel, wheel->sidewaysSlip);
                wheel->totalForce -= forceRight;

                //Spring force
                wheel->forceUp = wheel->suspensionCompression * axle.suspensionSpring;

                var up = wheel->normal;
                var forceUp = up * wheel->forceUp;
                wheel->totalForce += forceUp;

                //Spring damping force
                wheel->totalForce += wheel->dummy.Up * (wheel->suspensionCompression - wheel->suspensionCompressionPrev) / f.DeltaTime * axle.suspensionDamper;


                var wheelTransform = f.Unsafe.GetPointer<Transform3D>(wheel->wheel);
                Draw.Ray(wheelTransform->Position, wheel->dummy.Right * wheel->sidewaysSlip, ColorRGBA.Red);
                Draw.Ray(wheelTransform->Position, wheel->dummy.Forward * wheel->forwardSlip, ColorRGBA.Blue);
                Draw.Ray(wheel->point, forceForward, ColorRGBA.Blue);
                Draw.Ray(wheel->point, forceUp, ColorRGBA.Green);
                Draw.Ray(wheel->point, forceRight, ColorRGBA.Red);
            }

            FP ForwardFriction(Wheel* wheel, FP slip)
            {
                return FPMath.Atan(slip * 5) * FP._0_50 * wheel->forceUp;
            }

            FP SidewaysFriction(Wheel* wheel, FP slip)
            {
                return FPMath.Atan(slip) * FP._0_50 * wheel->forceUp;
            }
        }

        void UpdateInput()
        {
            if (reverseGear)
            {
                if (input->throttle)
                    brake += f.DeltaTime;
                else
                    brake = 0;

                if (input->brake)
                    throttle -= f.DeltaTime;
                else
                    throttle = 0;

                throttle = FPMath.Max(throttle, -1);
            }
            else
            {
                if (input->throttle)
                    throttle += f.DeltaTime;
                else
                    throttle = 0;

                if (input->brake)
                    brake += f.DeltaTime;
                else
                    brake = 0;

                throttle = FPMath.Min(throttle, 1);
            }

            brake = FPMath.Clamp01(brake);
        }
    }
}