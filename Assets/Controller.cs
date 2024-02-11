using Assets;
using System;
using UnityEngine;

[Serializable]
public class PDController
{
	private readonly Quadcopter _quadcopter;
	private readonly Gyro _gyro;
    private Vector3 _eulerAngleIntegrated;
    
	public float DerivativeScale = .8f;
	public float ProportionalScale = 1f;

	private struct ControlInput
	{
		public float Thrust;
		public Vector3 EulerAngles;
	}


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

	private Vector4 AdjustInput2(Vector3 angularVelocities)
	{
        var I = _quadcopter.Specification.MomentOfInertia;
		var m = _quadcopter.Specification.Mass;
		var b = _quadcopter.Specification.DragTorqueCoefficient;
		var k = _quadcopter.Specification.ThrustCoefficient;
		var g = _quadcopter.Specification.Gravity;
		var L = _quadcopter.Specification.ArmLength;

		var errors = ComputerErrors(angularVelocities);

		var u1 = (m * g) / (Mathf.Cos(_eulerAngleIntegrated.x) * Mathf.Cos(_eulerAngleIntegrated.y));


        var thrustConstraint = (m * g) / (4 * k * Mathf.Cos(_eulerAngleIntegrated.x) * Mathf.Cos(_eulerAngleIntegrated.y));


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

	private void ClampControlInputs(Vector4 inputs)
	{
        var m = _quadcopter.Specification.Mass;
        var b = _quadcopter.Specification.DragTorqueCoefficient;
        var k = _quadcopter.Specification.ThrustCoefficient;
        var g = _quadcopter.Specification.Gravity;
        var L = _quadcopter.Specification.ArmLength;

		var wmin = 0;
		var wmax = _quadcopter.Specification.MaxMotorRPM * _quadcopter.Specification.MaxMotorRPM;


        Mathf.Clamp(inputs.x, 4 * k * wmin, 4 * k * wmax);
		Mathf.Clamp(inputs.y, -L * k * wmax, L * k * wmax);
        Mathf.Clamp(inputs.z, -L * k * wmax, L * k * wmax);
        Mathf.Clamp(inputs.w, -2 * b * wmax, 2 * b * wmax);
    }

    private Vector4 AdjustInput(Vector3 angularVelocities)
    {
        var m = _quadcopter.Specification.Mass;
        var b = _quadcopter.Specification.DragTorqueCoefficient;
        var k = _quadcopter.Specification.ThrustCoefficient;
        var g = _quadcopter.Specification.Gravity;
        var L = _quadcopter.Specification.ArmLength;

        var errors = ComputerErrors(angularVelocities);

        var u1 = (m * g) / (Mathf.Cos(_eulerAngleIntegrated.x) * Mathf.Cos(_eulerAngleIntegrated.y));
		var inputs = new Vector4(u1, errors.x, errors.y, errors.z);
		ClampControlInputs(inputs);


        var gammas = new Vector4(
            inputs.x / (4 * k) - inputs.z / (2 * L * k) - inputs.w / (4 * b),
            inputs.x / (4 * k) + inputs.y / (2 * L * k) + inputs.w / (4 * b),
            inputs.x / (4 * k) + inputs.z / (2 * L * k) - inputs.w / (4 * b),
            inputs.x / (4 * k) - inputs.y / (2 * L * k) + inputs.w / (4 * b)
            );

		// The clamps are required, because of the thrust from the yaw control (u4)
		// might be greater than the thrust required to hover (u1), resulting in
		// negative motor speeds. This has to be possible to adjust for,
		// but not sure how...
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

		_quadcopter.MotorAngularVelocity = new Vector4 ( 
			Mathf.Sqrt(gammas[0]), 
			Mathf.Sqrt(gammas[1]), 
			Mathf.Sqrt(gammas[2]), 
			Mathf.Sqrt(gammas[3]) );
    }
}
