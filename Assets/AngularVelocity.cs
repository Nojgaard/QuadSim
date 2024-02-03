using Unity.VisualScripting;
using UnityEngine;

namespace Assets
{
    internal static class AngularVelocity
    {
        private static Matrix4x4 VelocityToVector(Vector3 angles)
        {
            Vector4 col1 = new(1f, 0f, 0f, 0f);
            Vector4 col2 = new(0, Mathf.Cos(angles.x), -Mathf.Sin(angles.x));
            Vector4 col3 = new(-Mathf.Sin(angles.y), Mathf.Cos(angles.y) * Mathf.Sin(angles.x), Mathf.Cos(angles.y)*Mathf.Cos(angles.x));
            Vector4 col4 = new(0, 0, 0, 1);
            return new Matrix4x4(col1, col2, col3, col4);
        }

        /// <summary>
        /// Transforms angular velocities into a vector pointing along the axis of rotation
        /// </summary>
        public static Vector3 ToAngularVelocityVector(Vector3 angles, Vector3 angularVelocity)
        {
            return VelocityToVector(angles) * angularVelocity;
        }

        private static float Sec(float x) => 1 / (Mathf.Cos(x) + float.Epsilon);

        private static float Cos(float x) => Mathf.Cos(x);
        private static float Sin(float x) => Mathf.Sin(x);

        /// <summary>
        /// Gets the angular velocities from the vector that points along the axis of rotation
        /// </summary>
        public static Vector3 FromAngularVelocityVector(Vector3 angles, Vector3 angularVelocityVector)
        {
            /*Vector4 col1 = new(1f, 0f, 0f, 0f);
            Vector4 col2 = new(Mathf.Sin(angles.x) * Mathf.Tan(angles.y), Mathf.Cos(angles.x), Mathf.Sin(angles.x) * Sec(angles.y));
            Vector4 col3 = new(Cos(angles.x) * Mathf.Tan(angles.y), -Sin(angles.x), Cos(angles.x) * Sec(angles.y));
            Vector4 col4 = new(0, 0, 0, 1);
            return new Matrix4x4(col1, col2, col3, col4) * angularVelocityVector;*/
            return VelocityToVector(angles).inverse * angularVelocityVector;
        }
    }
}
