using System;
using UnityEngine;


namespace Assets
{
	public class Gyro
	{
		private readonly float _sigma;

		private readonly Quadcopter _quadcopter;

		public Gyro(float sigma, Quadcopter quadcopter)
		{
			_sigma = sigma;
			_quadcopter = quadcopter;
		}

		private Vector3 Polute(Vector3 eulerAngles)
		{
			return _sigma * UnityEngine.Random.insideUnitSphere + eulerAngles;
		}

		public Vector3 AngularVelocity => Polute(_quadcopter.AngularVelocity);

	}

}