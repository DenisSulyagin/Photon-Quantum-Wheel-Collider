// <auto-generated>
// This code was auto-generated by a tool, every time
// the tool executes this code will be reset.
// </auto-generated>

using System;
namespace Quantum.Prototypes.Unity {
  [System.SerializableAttribute()]
  [Quantum.Prototypes.PrototypeAttribute(typeof(Quantum.PlayerVehicle))]
  public class PlayerVehicle_Prototype : Quantum.PrototypeAdapter<Quantum.Prototypes.PlayerVehicle_Prototype> {
    [Quantum.LocalReference]
    public global::EntityPrototype chassis;
    public Photon.Deterministic.FP maxAngularVelocityWheels;
    public Photon.Deterministic.FP reverseGearSpeedTreshold;
    public Quantum.QBoolean tridimensionalWheels;
    [Quantum.Inspector.ArrayLengthAttribute((Int32)2)]
    public Axle_Prototype[] axles = new Axle_Prototype[2];

    public sealed override Quantum.Prototypes.PlayerVehicle_Prototype Convert(EntityPrototypeConverter converter) {
      var result = new Quantum.Prototypes.PlayerVehicle_Prototype();
      converter.Convert(this.chassis, out result.chassis);
      result.maxAngularVelocityWheels = this.maxAngularVelocityWheels;
      result.reverseGearSpeedTreshold = this.reverseGearSpeedTreshold;
      result.tridimensionalWheels = this.tridimensionalWheels;
      result.axles = System.Array.ConvertAll(this.axles, x => x.Convert(converter));
      return result;
    }
  }
  [System.SerializableAttribute()]
  [Quantum.Prototypes.PrototypeAttribute(typeof(Quantum.Axle))]
  public class Axle_Prototype : Quantum.PrototypeAdapter<Quantum.Prototypes.Axle_Prototype> {
    public Photon.Deterministic.FP width;
    public Photon.Deterministic.FP horizontalPosition;
    public Photon.Deterministic.FP verticalPosition;
    public Quantum.QBoolean rear;
    public Photon.Deterministic.FP steerAngle;
    public Photon.Deterministic.FP wheelRadius;
    public Photon.Deterministic.FP wheelMass;
    public Photon.Deterministic.FP wheelFriction;
    public Photon.Deterministic.FP wheelWidth;
    public Photon.Deterministic.FP motorTorque;
    public Photon.Deterministic.FP brakeTorque;
    public Photon.Deterministic.FP suspensionDistance;
    public Photon.Deterministic.FP suspensionSpring;
    public Photon.Deterministic.FP suspensionDamper;
    [Quantum.Inspector.ArrayLengthAttribute((Int32)2)]
    public Wheel_Prototype[] wheels = new Wheel_Prototype[2];

    public sealed override Quantum.Prototypes.Axle_Prototype Convert(EntityPrototypeConverter converter) {
      var result = new Quantum.Prototypes.Axle_Prototype();
      result.width = this.width;
      result.horizontalPosition = this.horizontalPosition;
      result.verticalPosition = this.verticalPosition;
      result.rear = this.rear;
      result.steerAngle = this.steerAngle;
      result.wheelRadius = this.wheelRadius;
      result.wheelMass = this.wheelMass;
      result.wheelFriction = this.wheelFriction;
      result.wheelWidth = this.wheelWidth;
      result.motorTorque = this.motorTorque;
      result.brakeTorque = this.brakeTorque;
      result.suspensionDistance = this.suspensionDistance;
      result.suspensionSpring = this.suspensionSpring;
      result.suspensionDamper = this.suspensionDamper;
      result.wheels = System.Array.ConvertAll(this.wheels, x => x.Convert(converter));
      return result;
    }
  }
  [System.SerializableAttribute()]
  [Quantum.Prototypes.PrototypeAttribute(typeof(Quantum.Wheel))]
  public class Wheel_Prototype : Quantum.PrototypeAdapter<Quantum.Prototypes.Wheel_Prototype> {
    [Quantum.LocalReference]
    public global::EntityPrototype wheel;
    public Quantum.QBoolean right;
    public Quantum.LayerMask layerMask;

    public sealed override Quantum.Prototypes.Wheel_Prototype Convert(EntityPrototypeConverter converter) {
      var result = new Quantum.Prototypes.Wheel_Prototype();
      converter.Convert(this.wheel, out result.wheel);
      result.right = this.right;
      result.layerMask = this.layerMask;
      return result;
    }
  }
  [System.SerializableAttribute()]
  [Quantum.Prototypes.PrototypeAttribute(typeof(Quantum.PhysicsJoints3D))]
  public class PhysicsJoints3D_Prototype : Quantum.PrototypeAdapter<Quantum.Prototypes.PhysicsJoints3D_Prototype> {
    [Quantum.Inspector.DynamicCollectionAttribute()]
    public Joint3D_Prototype[] JointConfigs = System.Array.Empty<Joint3D_Prototype>();

    public sealed override Quantum.Prototypes.PhysicsJoints3D_Prototype Convert(EntityPrototypeConverter converter) {
      var result = new Quantum.Prototypes.PhysicsJoints3D_Prototype();
      result.JointConfigs = System.Array.ConvertAll(this.JointConfigs, x => x.Convert(converter));
      return result;
    }
  }
  [System.SerializableAttribute()]
  [Quantum.Prototypes.PrototypeAttribute(typeof(Quantum.Physics3D.Joint3D))]
  public class Joint3D_Prototype : Quantum.PrototypeAdapter<Quantum.Prototypes.Joint3D_Prototype> {
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("If the joint should be materialized with Enabled set to false, not being considered by the Physics Engine.")]
    public System.Boolean StartDisabled;
    [Quantum.Inspector.DisplayNameAttribute("Type")]
    [Quantum.Inspector.TooltipAttribute("The type of the joint, implying which constraints are applied.")]
    public Quantum.Physics3D.JointType3D JointType;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("A numerical tag that can be used to identify a joint or a group of joints.")]
    public System.Int32 UserTag;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("A Map Entity that the joint might be connected to.\nThe entity must have at least a transform component.")]
    [Quantum.LocalReference]
    public global::EntityPrototype ConnectedEntity;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The anchor point to which the joint connects to.\nIf a Connected Entity is provided, this represents an offset in its local space. Otherwise, the connected anchor is a position in world space.")]
    public Photon.Deterministic.FPVector3 ConnectedAnchor;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The anchor offset, in the local space of this joint entity's transform.\nThis is the point considered for the joint constraints and where the forces will be applied in the joint entity's body.")]
    public Photon.Deterministic.FPVector3 Anchor;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("Axis around which the joint rotates, defined in the local space of the entity.\nThe vector is normalized before set. If zeroed, FPVector3.Right is used instead.")]
    public Photon.Deterministic.FPVector3 Axis;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)2, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The frequency in Hertz (Hz) at which the spring joint will attempt to oscillate.\nTypical values are below half the frequency of the simulation.")]
    public Photon.Deterministic.FP Frequency;
    [Quantum.Inspector.RangeAttribute((Single)0, (Single)2)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)2, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("A dimensionless value representing the damper capacity of suppressing the spring oscillation, typically between 0 and 1.")]
    public Photon.Deterministic.FP DampingRatio;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("Automatically configure the target Distance to be the current distance between the anchor points in the scene.")]
    public System.Boolean AutoConfigureDistance;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)2, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("AutoConfigureDistance", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)2)]
    [Quantum.Inspector.TooltipAttribute("The distance between the anchor points that the joint will attempt to maintain.")]
    public Photon.Deterministic.FP Distance;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("AutoConfigureDistance", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)2)]
    [Quantum.Inspector.TooltipAttribute("The minimum distance between the anchor points that the joint will attempt to maintain.")]
    public Photon.Deterministic.FP MinDistance;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("AutoConfigureDistance", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)2)]
    [Quantum.Inspector.TooltipAttribute("The maximum distance between the anchor points that the joint will attempt to maintain.")]
    public Photon.Deterministic.FP MaxDistance;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("If the relative angle between the joint transform and its connected anchor should be limited by the hinge joint.\nSet this checkbox to configure the lower and upper limiting angles.")]
    public System.Boolean UseAngleLimits;
    [Quantum.Inspector.DrawIfAttribute("UseAngleLimits", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The lower limiting angle of the allowed arc of rotation around the connected anchor, in degrees.")]
    public Photon.Deterministic.FP LowerAngle;
    [Quantum.Inspector.DrawIfAttribute("UseAngleLimits", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The upper limiting  angle of the allowed arc of rotation around the connected anchor, in degrees.")]
    public Photon.Deterministic.FP UpperAngle;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("If the hinge joint uses a motor.\nSet this checkbox to configure the motor speed and max torque.")]
    public System.Boolean UseMotor;
    [Quantum.Inspector.DrawIfAttribute("UseMotor", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The speed at which the hinge motor will attempt to rotate, in angles per second.")]
    public Photon.Deterministic.FP MotorSpeed;
    [Quantum.Inspector.DrawIfAttribute("UseMotor", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The maximum torque produced by the hinge motor in order to achieve the target motor speed.\nLeave this checkbox unchecked and the motor toque should not be limited.")]
    public Photon.Deterministic.NullableFP MaxMotorTorque;

    public sealed override Quantum.Prototypes.Joint3D_Prototype Convert(EntityPrototypeConverter converter) {
      var result = new Quantum.Prototypes.Joint3D_Prototype();
      result.StartDisabled = this.StartDisabled;
      result.JointType = this.JointType;
      result.UserTag = this.UserTag;
      converter.Convert(this.ConnectedEntity, out result.ConnectedEntity);
      result.ConnectedAnchor = this.ConnectedAnchor;
      result.Anchor = this.Anchor;
      result.Axis = this.Axis;
      result.Frequency = this.Frequency;
      result.DampingRatio = this.DampingRatio;
      result.AutoConfigureDistance = this.AutoConfigureDistance;
      result.Distance = this.Distance;
      result.MinDistance = this.MinDistance;
      result.MaxDistance = this.MaxDistance;
      result.UseAngleLimits = this.UseAngleLimits;
      result.LowerAngle = this.LowerAngle;
      result.UpperAngle = this.UpperAngle;
      result.UseMotor = this.UseMotor;
      result.MotorSpeed = this.MotorSpeed;
      result.MaxMotorTorque = this.MaxMotorTorque;
      return result;
    }
  }
  [System.SerializableAttribute()]
  [Quantum.Prototypes.PrototypeAttribute(typeof(Quantum.PhysicsJoints2D))]
  public class PhysicsJoints2D_Prototype : Quantum.PrototypeAdapter<Quantum.Prototypes.PhysicsJoints2D_Prototype> {
    [Quantum.Inspector.DynamicCollectionAttribute()]
    public Joint2D_Prototype[] JointConfigs = System.Array.Empty<Joint2D_Prototype>();

    public sealed override Quantum.Prototypes.PhysicsJoints2D_Prototype Convert(EntityPrototypeConverter converter) {
      var result = new Quantum.Prototypes.PhysicsJoints2D_Prototype();
      result.JointConfigs = System.Array.ConvertAll(this.JointConfigs, x => x.Convert(converter));
      return result;
    }
  }
  [System.SerializableAttribute()]
  [Quantum.Prototypes.PrototypeAttribute(typeof(Quantum.Physics2D.Joint))]
  public class Joint2D_Prototype : Quantum.PrototypeAdapter<Quantum.Prototypes.Joint2D_Prototype> {
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("If the joint should be materialized with Enabled set to false, not being considered by the Physics Engine.")]
    public System.Boolean StartDisabled;
    [Quantum.Inspector.DisplayNameAttribute("Type")]
    [Quantum.Inspector.TooltipAttribute("The type of the joint, implying which constraints are applied.")]
    public Quantum.Physics2D.JointType JointType;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("A numerical tag that can be used to identify a joint or a group of joints.")]
    public System.Int32 UserTag;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("A Map Entity that the joint might be connected to.\nThe entity must have at least a Transform2D component.")]
    [Quantum.LocalReference]
    public global::EntityPrototype ConnectedEntity;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The anchor point to which the joint connects to.\nIf a Connected Entity is provided, this represents an offset in its local space. Otherwise, the connected anchor is a position in world space.")]
    public Photon.Deterministic.FPVector2 ConnectedAnchor;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The anchor offset, in the local space of this joint entity's transform.\nThis is the point considered for the joint constraints and where the forces will be applied in the joint entity's body.")]
    public Photon.Deterministic.FPVector2 Anchor;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)2, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The frequency in Hertz (Hz) at which the spring joint will attempt to oscillate.\nTypical values are below half the frequency of the simulation.")]
    public Photon.Deterministic.FP Frequency;
    [Quantum.Inspector.RangeAttribute((Single)0, (Single)2)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)2, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("A dimensionless value representing the damper capacity of suppressing the spring oscillation, typically between 0 and 1.")]
    public Photon.Deterministic.FP DampingRatio;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)1, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("Automatically configure the target Distance to be the current distance between the anchor points in the scene.")]
    public System.Boolean AutoConfigureDistance;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)2, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("AutoConfigureDistance", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)2)]
    [Quantum.Inspector.TooltipAttribute("The distance between the anchor points that the joint will attempt to maintain.")]
    public Photon.Deterministic.FP Distance;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("AutoConfigureDistance", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)2)]
    [Quantum.Inspector.TooltipAttribute("The minimum distance between the anchor points that the joint will attempt to maintain.")]
    public Photon.Deterministic.FP MinDistance;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("AutoConfigureDistance", (Int64)0, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)2)]
    [Quantum.Inspector.TooltipAttribute("The maximum distance between the anchor points that the joint will attempt to maintain.")]
    public Photon.Deterministic.FP MaxDistance;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("If the relative angle between the joint transform and its connected anchor should be limited by the hinge joint.\nSet this checkbox to configure the lower and upper limiting angles.")]
    public System.Boolean UseAngleLimits;
    [Quantum.Inspector.DegreesAttribute()]
    [Quantum.Inspector.DrawIfAttribute("UseAngleLimits", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The lower limiting angle of the allowed arc of rotation around the connected anchor, in degrees.")]
    public Photon.Deterministic.FP LowerAngle;
    [Quantum.Inspector.DegreesAttribute()]
    [Quantum.Inspector.DrawIfAttribute("UseAngleLimits", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The upper limiting  angle of the allowed arc of rotation around the connected anchor, in degrees.")]
    public Photon.Deterministic.FP UpperAngle;
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("If the hinge joint uses a motor.\nSet this checkbox to configure the motor speed and max torque.")]
    public System.Boolean UseMotor;
    [Quantum.Inspector.DegreesAttribute()]
    [Quantum.Inspector.DrawIfAttribute("UseMotor", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The speed at which the hinge motor will attempt to rotate, in angles per second.")]
    public Photon.Deterministic.FP MotorSpeed;
    [Quantum.Inspector.DrawIfAttribute("UseMotor", (Int64)1, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.DrawIfAttribute("JointType", (Int64)3, (Quantum.Inspector.DrawIfCompareOperator)0, (Quantum.Inspector.DrawIfHideType)3)]
    [Quantum.Inspector.TooltipAttribute("The maximum torque produced by the hinge motor in order to achieve the target motor speed.\nLeave this checkbox unchecked and the motor toque should not be limited.")]
    public Photon.Deterministic.NullableFP MaxMotorTorque;

    public sealed override Quantum.Prototypes.Joint2D_Prototype Convert(EntityPrototypeConverter converter) {
      var result = new Quantum.Prototypes.Joint2D_Prototype();
      result.StartDisabled = this.StartDisabled;
      result.JointType = this.JointType;
      result.UserTag = this.UserTag;
      converter.Convert(this.ConnectedEntity, out result.ConnectedEntity);
      result.ConnectedAnchor = this.ConnectedAnchor;
      result.Anchor = this.Anchor;
      result.Frequency = this.Frequency;
      result.DampingRatio = this.DampingRatio;
      result.AutoConfigureDistance = this.AutoConfigureDistance;
      result.Distance = this.Distance;
      result.MinDistance = this.MinDistance;
      result.MaxDistance = this.MaxDistance;
      result.UseAngleLimits = this.UseAngleLimits;
      result.LowerAngle = this.LowerAngle;
      result.UpperAngle = this.UpperAngle;
      result.UseMotor = this.UseMotor;
      result.MotorSpeed = this.MotorSpeed;
      result.MaxMotorTorque = this.MaxMotorTorque;
      return result;
    }
  }

}
