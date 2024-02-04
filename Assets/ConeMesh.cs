using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets
{
    internal static class ConeMesh
    {
        private static Vector3[] newVertices = new Vector3[]
    {
            new Vector3(-0.0f, 1.0f, 0.0f),
            new Vector3(0.2f, 0.0f, 0.4f),
            new Vector3(0.5f, 0.0f, 0.0f),
            new Vector3(0.0f, 0.0f, 0.0f),
            new Vector3(-0.3f, 0.0f, 0.4f),
            new Vector3(-0.5f, 0.0f, 0.0f),
            new Vector3(-0.2f, 0.0f, -0.4f),
            new Vector3(0.2f, 0.0f, -0.4f),
    };
        private static Vector3[] newNormals = new Vector3[]
        {
            new Vector3(0,1,0),
            new Vector3(1,0,1),
            new Vector3(1,0,0),
            new Vector3(0,-1,0),
            new Vector3(-1,0,1),
            new Vector3(-1,0,0),
            new Vector3(-1,0,-1),
            new Vector3(1,0,-1),
        };
        private static int[] newTriangles = new int[]
        {
            0, 1, 2, 2, 1, 3, 0, 4, 1, 1, 4, 3, 0, 5, 4, 4, 5, 3, 0, 6, 5, 5, 6, 3, 0, 7, 6, 6, 7, 3, 0, 2, 7, 7, 2, 3
        };

        private static Mesh mesh;

        static ConeMesh()
        {
            mesh = new Mesh();
            mesh.vertices = newVertices;
            mesh.triangles = newTriangles;
            mesh.normals = newNormals;
        }

        public static void DrawCone(Vector3 position, Vector3 rotation, float scale)
        {
            Gizmos.DrawMesh(mesh, position, Quaternion.FromToRotation(Vector3.up, rotation), new Vector3(scale, 2 * scale, scale));
        }
    }
}
