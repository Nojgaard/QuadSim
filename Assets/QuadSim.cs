using Assets;
using System.Collections;
using System.Collections.Generic;
using TMPro.EditorUtilities;
using UnityEngine;
using UnityEngine.XR;

[RequireComponent(typeof(MeshRenderer))]
public class QuadSim : MonoBehaviour
{
    public Quadcopter quadcopter = new();

    public float[] Speeds = new float[4];

    public void OnEnable()
    {
        var renderer = GetComponent<MeshRenderer>();

        var stackSize = new Vector3(0.1f, 0.1f, 0.1f);
        var motorSize = new Vector3(0.02f, 0.02f, 0.02f);

        var _quadcopterMesh = GameObject.CreatePrimitive(PrimitiveType.Cube);
        _quadcopterMesh.transform.SetParent(transform);
        _quadcopterMesh.transform.localScale = stackSize;
        _quadcopterMesh.transform.localPosition = Vector3.zero;

        var motors = new GameObject[4];
        for (int i = 0; i < motors.Length; i++)
        {
            motors[i] = GameObject.CreatePrimitive(PrimitiveType.Cube);
            motors[i].transform.SetParent(transform);
            motors[i].transform.localScale = motorSize;
        }

        motors[0].transform.localPosition = (Vector3.right * quadcopter.ArmLength);
        motors[1].transform.localPosition = (Vector3.left * quadcopter.ArmLength);
        motors[2].transform.localPosition = (Vector3.forward * quadcopter.ArmLength);
        motors[3].transform.localPosition = (Vector3.back * quadcopter.ArmLength);

    }

    public void OnDrawGizmosSelected()
    {
        // Draw body frame coordinate system 

        // draw z axis with arrow (this is the yaw axis)
        Vector3 up = transform.localToWorldMatrix * (_inertialToUnity * new Vector3(0, 0, 1)); 
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, (transform.position + (up)));
        
        // draw y axis with arrow (this is the pitch axis)
        Gizmos.color = Color.green;
        Vector3 right = transform.localToWorldMatrix * (_inertialToUnity * new Vector3(0, 1, 0));        
        Gizmos.DrawLine(transform.position, (transform.position + right));

        // draw x axis with arrow (this is the roll axis)
        Gizmos.color = Color.red;
        Vector3 forward = transform.localToWorldMatrix * (_inertialToUnity * new Vector3(1, 0, 0));
        Gizmos.DrawLine(transform.position, (transform.position + forward));


        // Draw motor torque vector
        var scaleRMP = 1 / quadcopter.MaxMotorRPM * 4; // note this has to be adjusted in the long run

        Gizmos.color = Color.black;
        Vector3 torque0 = up * Speeds[0] * scaleRMP;
        var originMotor0 = transform.position + right * quadcopter.ArmLength;
        Gizmos.DrawLine(originMotor0, originMotor0 + torque0);
        ConeMesh.DrawCone(originMotor0 + torque0, up, 0.5f * Speeds[0] * scaleRMP);

        Gizmos.color = Color.black;
        Vector3 torque1 = up * Speeds[1] * scaleRMP;
        var originMotor1 = transform.position + forward * quadcopter.ArmLength;
        Gizmos.DrawLine(originMotor1, originMotor1 + torque1);
        ConeMesh.DrawCone(originMotor1 + torque1, up, 0.5f * Speeds[1] * scaleRMP);

        Gizmos.color = Color.black;
        Vector3 torque2 = up * Speeds[2] * scaleRMP;
        var originMotor2 = transform.position - right * quadcopter.ArmLength;
        Gizmos.DrawLine(originMotor2, originMotor2 + torque2);
        ConeMesh.DrawCone(originMotor2 + torque2, up, 0.5f * Speeds[2] * scaleRMP);

        Gizmos.color = Color.black;
        Vector3 torque3 = up * Speeds[3] * scaleRMP;
        var originMotor3 = transform.position - forward * quadcopter.ArmLength;
        Gizmos.DrawLine(originMotor3, originMotor3 + torque3);
        ConeMesh.DrawCone(originMotor3 + torque3, up, 0.5f * Speeds[3] * scaleRMP);
    }

    private Matrix4x4 _inertialToUnity = new(new(1, 0, 0), new(0, 0, 1), new Vector4(0, 1, 0), new(0, 0, 0, 1));
    private Matrix4x4 _unityToInertial => _inertialToUnity.inverse;


    void OnGUI()
    {
        Event e = Event.current;
        if (e.isKey && e.keyCode == KeyCode.UpArrow)
        {
            for (int i = 0; i < Speeds.Length; i++)
            {
                Speeds[i] += 100;
            }
        }
        if (e.isKey && e.keyCode == KeyCode.DownArrow)
        {
            for (int i = 0; i < Speeds.Length; i++)
            {
                Speeds[i] -= 100;
            }
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        quadcopter.SetInitialState(_unityToInertial * transform.localPosition, _unityToInertial * transform.eulerAngles);
        quadcopter.MotorAngularVelocity[0] = 300;
        quadcopter.MotorAngularVelocity[3] = 0;
    }

    // Update is called once per frame
    void Update()
    {
        quadcopter.MotorAngularVelocity = Speeds;
        var dt = Time.deltaTime;
        quadcopter.Update(dt*.7f);

        transform.localPosition = _inertialToUnity * quadcopter.Position;
        transform.eulerAngles = _inertialToUnity * (Mathf.Rad2Deg * quadcopter.EulerAngles);
    }
}
