using Photon.Deterministic;

namespace Quantum.Game;

public unsafe class VehicleSystem : SystemMainThread
{
    PhysicsBody3D* chassisBody;
    Transform3D* chassisTransform;
    bool tridimensionalWheels;
    FP maxAngularVelocityWheels;

    public override void Update(Frame f)
    {
        var vehicles = f.Unsafe.GetComponentBlockIterator<PlayerVehicle>();
        var input = f.GetPlayerInput(0);

        foreach (var vehicle in vehicles)
        {
            chassisBody = f.Unsafe.GetPointer<PhysicsBody3D>(vehicle.Component->chassis);
            chassisTransform = f.Unsafe.GetPointer<Transform3D>(vehicle.Component->chassis);

            tridimensionalWheels = vehicle.Component->tridimensionalWheels;

            var wheels = vehicle.Component->wheels;
            maxAngularVelocityWheels = vehicle.Component->maxAngularVelocityWheels;

            for (int i = 0; i < wheels.Length; i++)
            {
                var wheel = wheels.GetPointer(i);

                wheel->motorInput = input->Movement.Y > 0 ? input->Movement.Y : 0;
                wheel->brakeInput = input->Movement.Y < 0 ? -input->Movement.Y : 0;

                WheelUpdate(wheel, i < 2 ? input->Movement.X * vehicle.Component->steerAngle : 0);//TODO
            }
        }

        void WheelUpdate(Wheel* wheel, FP steerAngle)
        {
            var wheelTransform = f.Unsafe.GetPointer<Transform3D>(wheel->wheel);

            wheel->dummy.Position = chassisTransform->TransformPoint(wheel->dummyLocalPosition);
            wheel->dummy.Rotation = FPQuaternion.Euler(new FPVector3(0, steerAngle, 0)) * chassisTransform->Rotation;

            UpdateSuspension(wheel);

            UpdateWheel(wheel, wheelTransform);

            if (wheel->isGrounded)
            {
                CalculateSlips(wheel);
                CalculateForcesFromSlips(wheel);

                chassisBody->AddForceAtPosition(wheel->totalForce, wheelTransform->Position, chassisTransform);
            }
        }

        void UpdateSuspension(Wheel* wheel)
        {
            var hitRDown = f.Physics3D.Raycast(wheel->dummy.Position, -wheel->dummy.Up, wheel->radius + wheel->suspensionDistance); //Down

            if (tridimensionalWheels)
                UpdateTrideminsional(wheel);// Quantum physics unstable results

            if (hitRDown.HasValue)
            {
                if (!wheel->isGrounded)
                    wheel->prevPosition = chassisTransform->Position;

                wheel->isGrounded = true;
                wheel->point = hitRDown.Value.Point;

                wheel->suspensionCompressionPrev = wheel->suspensionCompression;

                var distance = hitRDown.Value.CastDistanceNormalized;
                //var distance = FPMath.Clamp01(FPVector3.Distance(wheel->dummy.Position, hitS.Value.Point) / (wheel->radius + wheel->suspensionDistance));

                wheel->suspensionCompression = (1 - distance) * (wheel->suspensionDistance + wheel->radius);

                Draw.Ray(wheel->dummy.Position, -wheel->dummy.Up * (wheel->radius + wheel->suspensionDistance));
            }
            else
            {
                wheel->suspensionCompression = 0;
                wheel->isGrounded = false;

                Draw.Ray(wheel->dummy.Position, -wheel->dummy.Up * (wheel->radius + wheel->suspensionDistance), ColorRGBA.Red);
            }
        }

        Physics3D.Hit3D? UpdateTrideminsional(Wheel* wheel)
        {
            var radius = wheel->radius;
            var dummyPosition = wheel->dummy.Position;
            var dummyUp = wheel->dummy.Up;

            var startP = dummyPosition + dummyUp * radius;
            var translationV = -dummyUp * (wheel->radius + wheel->suspensionDistance);

            Shape3D castShape = Shape3D.CreateSphere(radius);

            var hitS = f.Physics3D.ShapeCast(
                startP,
                FPQuaternion.Identity,
                &castShape,
                translationV,
                - 1,
                QueryOptions.ComputeDetailedInfo | QueryOptions.HitAll
            );

            Draw.Sphere(dummyPosition - dummyUp * wheel->suspensionDistance, radius, new ColorRGBA("00FF0040"));

            if (hitS.HasValue)
            {
                //var distance = FPMath.Clamp01(FPVector3.Distance(wheel->dummy.Position, hitS.Value.Point) / (wheel->radius + wheel->suspensionDistance));
                var distance = hitS.Value.CastDistanceNormalized;

                Draw.Sphere(dummyPosition - dummyUp * distance, radius, new ColorRGBA("FF000040"));

                Draw.Line(dummyPosition, hitS.Value.Point, ColorRGBA.Black);
            }

            return hitS;
        }

        void UpdateWheel(Wheel* wheel, Transform3D* wheelTransform)
        {
            //Angular velocity
            wheel->rotationAngle += wheel->angularVelocity * f.DeltaTime;

            wheelTransform->Rotation = wheel->dummy.Rotation * FPQuaternion.Euler(FPVector3.Right * wheel->rotationAngle);
            wheelTransform->Position = wheel->dummy.Position - FPVector3.Up * (wheel->suspensionDistance - wheel->suspensionCompression);

            var torque = wheel->motorTorque * wheel->motorInput;
            var brakeTorque = wheel->brakeTorque * wheel->brakeInput;

            if (wheel->angularVelocity <= 0 && wheel->brakeInput > 0 )//Inverse (GEAR R) BAD
            {
                torque = -wheel->motorTorque * wheel->brakeInput;
                brakeTorque = wheel->brakeTorque * wheel->motorInput;
            }

            if (wheel->isGrounded && torque == 0)//TODO
                wheel->angularVelocity -= ForwardFriction(wheel, wheel->forwardSlip) / (FP.Pi * 2 * wheel->radius) / wheel->mass;

            //Torque
            wheel->angularVelocity += torque / wheel->radius / wheel->mass * f.DeltaTime;

            //Brake
            wheel->angularVelocity -= FPMath.Sign(wheel->angularVelocity) * FPMath.Min(
                FPMath.Abs(wheel->angularVelocity),
                brakeTorque * wheel->radius / wheel->mass * f.DeltaTime
                );

            wheel->angularVelocity = FPMath.Clamp(wheel->angularVelocity, -maxAngularVelocityWheels, maxAngularVelocityWheels);

            Draw.Ray(wheel->dummy.Position, wheel->dummy.Right, ColorRGBA.Red);
            Draw.Ray(wheel->dummy.Position, wheel->dummy.Forward, ColorRGBA.Blue);
        }

        void CalculateSlips(Wheel* wheel)
        {
            FPVector3 velocity = (wheel->dummy.Position - wheel->prevPosition) / f.DeltaTime;
            wheel->prevPosition = wheel->dummy.Position;

            FPVector3 forward = wheel->dummy.Forward;
            FPVector3 sideways = -wheel->dummy.Right;

            FPVector3 forwardVelocity = FPVector3.Dot(velocity, forward) * forward;
            FPVector3 sidewaysVelocity = FPVector3.Dot(velocity, sideways) * sideways;

            wheel->forwardSlip = -FPMath.Sign(FPVector3.Dot(forward, forwardVelocity)) * forwardVelocity.Magnitude + (wheel->angularVelocity * FP.Pi / 180 * wheel->radius);
            wheel->sidewaysSlip = -FPMath.Sign(FPVector3.Dot(sideways, sidewaysVelocity)) * sidewaysVelocity.Magnitude;

        }

        void CalculateForcesFromSlips(Wheel* wheel)
        {
            //Forward slip force
            var forceForward = wheel->dummy.Forward * ForwardFriction(wheel, wheel->forwardSlip);
            wheel->totalForce = forceForward;

            //Lateral slip force
            var forceRight = wheel->dummy.Right * SidewaysFriction(wheel, wheel->sidewaysSlip);
            wheel->totalForce -= forceRight;

            //Spring force
            wheel->forceUp = wheel->suspensionCompression * wheel->suspensionSpring;

            var forceUp = FPVector3.Up * wheel->forceUp;
            wheel->totalForce += forceUp;

            //Spring damping force
            wheel->totalForce += wheel->dummy.Up * (wheel->suspensionCompression - wheel->suspensionCompressionPrev) / f.DeltaTime * wheel->suspensionDamper;

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
}