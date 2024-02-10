﻿using Assets;
using System;
using UnityEngine;

public class PDController
{
	private readonly Quadcopter _quadcopter;
	private readonly Gyro _gyro;
    private Vector3 _eulerAngleIntegrated;
    
	public float DerivativeScale = 5f;
	public float ProportionalScale = 3.0f;


    public PDController(Quadcopter quadcopter, Gyro gyro)
	{
		_quadcopter= quadcopter;
		_gyro = gyro;


		// fix this later
		_eulerAngleIntegrated = quadcopter.EulerAngles;
    }

	private void IntegrateEulerAngle(Vector3 angularVelocities, float dt)
	{
		_eulerAngleIntegrated += angularVelocities * dt;
	}

	private Vector3 ComputerErrors(Vector3 angularVelocities)
	{
		var error  = DerivativeScale * angularVelocities;
		error += ProportionalScale * _eulerAngleIntegrated;

		return error;
    }

	private Vector4 AdjustInput(Vector3 angularVelocities)
	{
        var I = _quadcopter.Specification.MomentOfInertia;
		var m = _quadcopter.Specification.Mass;
		var b = _quadcopter.Specification.DragTorqueCoefficient;
		var k = _quadcopter.Specification.ThrustCoefficient;
		var g = -_quadcopter.ForceGravity.z;
		var L = _quadcopter.Specification.ArmLength;

		var errors = ComputerErrors(angularVelocities);

        var thrustConstraint = (m * g.z) / (4 * k * Mathf.Cos(_eulerAngleIntegrated.x) * Mathf.Cos(_eulerAngleIntegrated.y));


		float const1 = errors.z * I.z/(4f * b);
        float const2 = errors.y * I.y / (2 * k * L);
        float const3 = (2 * b * errors.x * I.x + errors.z * I.z * k * L) / (4f * b * k * L);
        float const4 = (-2 * b * errors.x * I.x + errors.z * I.z * k * L) / (4f * b * k * L);


		var gammas = new Vector4(
			thrustConstraint - const3,
			thrustConstraint + const1 - const2,
			thrustConstraint - const4,
			thrustConstraint + const1 + const2
			);

		gammas.x = Mathf.Clamp(gammas.x, 0, _quadcopter.Specification.MaxMotorRPM * _quadcopter.Specification.MaxMotorRPM);
        gammas.y = Mathf.Clamp(gammas.y, 0, _quadcopter.Specification.MaxMotorRPM * _quadcopter.Specification.MaxMotorRPM);
        gammas.z = Mathf.Clamp(gammas.z, 0, _quadcopter.Specification.MaxMotorRPM * _quadcopter.Specification.MaxMotorRPM);
        gammas.w = Mathf.Clamp(gammas.w, 0, _quadcopter.Specification.MaxMotorRPM * _quadcopter.Specification.MaxMotorRPM);
        return gammas;
    }

	public void Update(float dt)
	{
		var angularVelocities = _gyro.AngularVelocity;

        var gammas = AdjustInput(angularVelocities);
        IntegrateEulerAngle(angularVelocities, dt);

		_quadcopter.MotorAngularVelocity = new float[4] { 
			Mathf.Sqrt(gammas[0]), 
			Mathf.Sqrt(gammas[1]), 
			Mathf.Sqrt(gammas[2]), 
			Mathf.Sqrt(gammas[3]) };
    }
}
