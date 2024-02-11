using System;
using UnityEngine;


namespace Assets
{
	public class Gyro
	{
		private readonly float _sigma;

		private readonly Quadcopter _quadcopter;

		private Vector3 _measuredEulerAngles;

		public Gyro(float sigma, Quadcopter quadcopter)
		{
			_sigma = sigma;
			_quadcopter = quadcopter;
			_measuredEulerAngles = new();
		}

		private Vector3 Polute(Vector3 eulerAngles)
		{
			return _sigma * UnityEngine.Random.insideUnitSphere + eulerAngles;
		}

		public Vector3 AngularVelocity => Polute(_quadcopter.AngularVelocity);

		public Vector3 ReadEulerAngles(float dt)
		{
			var measuredAngularVelocity = Polute(_quadcopter.AngularVelocity);
			_measuredEulerAngles += measuredAngularVelocity * dt;
			return _measuredEulerAngles;
        }

		public void Reset()
		{
			_measuredEulerAngles = new();
		}

	}

}