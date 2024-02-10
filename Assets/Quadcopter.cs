using System;
using System.Linq;
using UnityEngine;

[Serializable]
public class Quadcopter
{
    [Serializable]
    public class SpecificationData
    {
        /// <summary>
        /// Number of rotors
        /// </summary>
        public int NumRotors => 4;

        /// <summary>
        /// Arm length in meters
        /// </summary>
        public float ArmLength = 0.29f;

        /// <summary>
        /// Mass of quadcopter (kg)
        /// </summary>
        public float Mass = 1f;

        /// <summary>
        /// Maximum rpm of the 4 motors
        /// </summary>
        public float MaxMotorRPM = 1000;

        /// <summary>
        /// Thrust coeffecient (kg-m)
        /// </summary>
        public float ThrustCoefficient = 2.39e-5f;

        /// <summary>
        /// Moment of inertia on each axis in body frame (kg-m^2)
        /// </summary>
        public Vector3 MomentOfInertia = new(0.04989f, 0.04989f, 0.24057f);

        /// <summary>
        /// Drag coefficients on each axis (kg/s)
        /// </summary>
        public Vector3 DragCoefficients = new(0.164f, 0.319f, 0f);

        /// <summary>
        /// Drag torque coefficients (kg-m^2/s^2)
        /// </summary>
        public float DragTorqueCoefficient = 1.39e-6f;
    }

    public SpecificationData Specification;

    #region Forces

    /// <summary>
    /// Thrust from motors in body frame (N)
    /// </summary>
    public Vector3 ForceThrust => new(0, 0, Specification.ThrustCoefficient * MotorAngularVelocity.sqrMagnitude);

    /// <summary>
    /// Force of gravity in inertial frame (N)
    /// </summary>
    public Vector3 ForceGravity => new(0, 0, -9.82f);

    /// <summary>
    /// Force of drag in body frame (N)
    /// </summary>
    public Vector3 ForceDrag => Vector3.Scale(-Specification.DragCoefficients, Velocity);

    #endregion

    #region State

    [Header("State")]
    /// <summary>
    /// Angular velocity for each motor (radian/seconds)
    /// </summary>
    public Vector4 MotorAngularVelocity = new();

    /// <summary>
    /// Position in intertial frame
    /// </summary>
    public Vector3 Position = new();

    /// <summary>
    /// Roll, pitch, yaw in body frame
    /// </summary>
    public Vector3 EulerAngles = new();

    /// <summary>
    /// Velocity of quadcopter in bodyframe (m/s)
    /// </summary>
    public Vector3 Velocity = new();

    /// <summary>
    /// Angular velocity of quadcopter in 
    /// </summary>
    public Vector3 AngularVelocity = new();

    #endregion

    public void SetInitialState(Vector3 position, Vector3 eulerAngles)
    {
        Position = position;
        EulerAngles = eulerAngles;
        Velocity = Vector3.zero;
        AngularVelocity = Vector3.zero;
        for (int i = 0; i < Specification.NumRotors; i++) 
        {
            MotorAngularVelocity[i] = 0;
        }
        AngularVelocity.y = 5;
        //DisturbAngularVelocity();
    }


    private Vector3 Acceleration()
    {
        var bodyToInertialRotation = Quaternion.Euler(Mathf.Rad2Deg * EulerAngles);
        var thrustInInertialFrame = bodyToInertialRotation * ForceThrust;
        return ForceGravity + ForceDrag + thrustInInertialFrame / Specification.Mass;
    }

    /// <summary>
    /// Computes the angular acceleration as the derivatitive of the angular velocity vector
    /// </summary>
    /// <returns></returns>
    private Vector3 AngularAcceleration()
    {
        var m = Vector4.Scale(MotorAngularVelocity, MotorAngularVelocity);
        Vector3 torque = new(
            Specification.ArmLength * Specification.ThrustCoefficient * (m[3] - m[1]),
            Specification.ArmLength * Specification.ThrustCoefficient * (m[0] - m[2]),
            Specification.DragTorqueCoefficient * (m[0] - m[1] + m[2] - m[3])
            );
        var w = Assets.AngularVelocity.ToAngularVelocityVector(EulerAngles, AngularVelocity);

        Vector3 inverseMomentOfInertia = new(1 / Specification.MomentOfInertia.x, 1 / Specification.MomentOfInertia.y, 1 / Specification.MomentOfInertia.z);
        // Derivative of angular velocity vector
        var wdot = Vector3.Scale(inverseMomentOfInertia, torque - Vector3.Cross(w, Vector3.Scale(Specification.MomentOfInertia, w)));
        return wdot;
    }

    public void Update(float dt)
    {
        var acceleration = Acceleration();
        var angularAcceleration = AngularAcceleration();

        var angularVelocityVector = Assets.AngularVelocity.ToAngularVelocityVector(EulerAngles, AngularVelocity);
        angularVelocityVector += dt * angularAcceleration;
        AngularVelocity = Assets.AngularVelocity.FromAngularVelocityVector(EulerAngles, angularVelocityVector);
        EulerAngles += dt * AngularVelocity;

        Velocity += dt * acceleration;
        Position += dt * Velocity;
    }


    private void DisturbAngularVelocity()
    {
        float distrubance = 1;

        Vector3 velocityDisturbance = UnityEngine.Random.insideUnitSphere;
        velocityDisturbance *= distrubance;
        velocityDisturbance -= Vector3.one * distrubance;

        AngularVelocity += velocityDisturbance;
    }
}
