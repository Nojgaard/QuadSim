using Assets;
using System;
using UnityEngine;

[Serializable]
public class PDController
{
    private readonly Quadcopter _quadcopter;
    private readonly MPU _gyro;

    public float DerivativeScale = .8f;
    public float ProportionalScale = 1f;

    [Serializable]
    public class PIDError
    {
        public Vector3 Target = new();
        public float DerivativeScale = .8f;
        public float ProportionalScale = 1f;

        private Vector3 _errorDerivative = new();
        private Vector3 _errorProportional = new();
        private Vector3 _errorIntegral = new();

        public void Update(Vector3 measuredValue, float dt)
        {
            var lastErrorProportional = _errorProportional;
            _errorProportional = Target - measuredValue;
            _errorDerivative = (_errorProportional - lastErrorProportional) / dt;
            _errorIntegral += _errorProportional * dt;
        }

        public Vector3 GetErrors() => DerivativeScale * _errorDerivative + ProportionalScale * _errorProportional;

        public void Reset()
        {
            _errorDerivative = new();
            _errorProportional = new();
            _errorIntegral = new();
        }
    }

    public PIDError PIDEulerAngles = new();

    public PDController(Quadcopter quadcopter, MPU gyro)
    {
        _quadcopter = quadcopter;
        _gyro = gyro;;
    }

    private void ClampControlInputs(Vector4 inputs)
    {
        var b = _quadcopter.Specification.DragTorqueCoefficient;
        var k = _quadcopter.Specification.ThrustCoefficient;
        var L = _quadcopter.Specification.ArmLength;

        var wmin = 0;
        var wmax = _quadcopter.Specification.MaxMotorRPM * _quadcopter.Specification.MaxMotorRPM;


        Mathf.Clamp(inputs.x, 4 * k * wmin, 4 * k * wmax); // thrust
        Mathf.Clamp(inputs.y, -L * k * wmax, L * k * wmax); // roll torque
        Mathf.Clamp(inputs.z, -L * k * wmax, L * k * wmax); // pitch torque
        Mathf.Clamp(inputs.w, -2 * b * wmax, 2 * b * wmax); // yaw torque
    }

    private Vector4 ComputeDesiredMotorSpeeds(Vector3 measuredEulerAngles)
    {
        var m = _quadcopter.Specification.Mass;
        var b = _quadcopter.Specification.DragTorqueCoefficient;
        var k = _quadcopter.Specification.ThrustCoefficient;
        var g = _quadcopter.Specification.Gravity;
        var L = _quadcopter.Specification.ArmLength;

        var errors = PIDEulerAngles.GetErrors();

        var u1 = (m * g) / (Mathf.Cos(measuredEulerAngles.x) * Mathf.Cos(measuredEulerAngles.y));
        var inputs = new Vector4(u1, errors.x, errors.y, errors.z);
        ClampControlInputs(inputs);


        var gammas = new Vector4(
            inputs.x / (4 * k) + inputs.z / (2 * L * k) + inputs.w / (4 * b),
            inputs.x / (4 * k) - inputs.y / (2 * L * k) - inputs.w / (4 * b),
            inputs.x / (4 * k) - inputs.z / (2 * L * k) + inputs.w / (4 * b),
            inputs.x / (4 * k) + inputs.y / (2 * L * k) - inputs.w / (4 * b)
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
        var measuredEulerAngles = _gyro.ReadEulerAngles(dt);
        PIDEulerAngles.Update(measuredEulerAngles, dt);

        var gammas = ComputeDesiredMotorSpeeds(measuredEulerAngles);

        _quadcopter.MotorAngularVelocity = new Vector4(
            Mathf.Sqrt(gammas[0]),
            Mathf.Sqrt(gammas[1]),
            Mathf.Sqrt(gammas[2]),
            Mathf.Sqrt(gammas[3]));
    }

    public void Reset()
    {
        PIDEulerAngles.Reset();
        _gyro.Reset();
    }
}
