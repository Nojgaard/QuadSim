using Assets;
using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
public class QuadSim : MonoBehaviour
{
    public Quadcopter quadcopter = new();

    public PDController controller;

    [Range(0, 1)]
    public float timeScale = .7f;

    public bool EnableController = true;


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

        motors[0].transform.localPosition = (Vector3.right * quadcopter.Specification.ArmLength);
        motors[1].transform.localPosition = (Vector3.left * quadcopter.Specification.ArmLength);
        motors[2].transform.localPosition = (Vector3.forward * quadcopter.Specification.ArmLength);
        motors[3].transform.localPosition = (Vector3.back * quadcopter.Specification.ArmLength);

    }

    public void OnDrawGizmosSelected()
    {
        // Draw body frame coordinate system
        var colors = new Color[] {Color.red, Color.green, Color.blue};
        // Forward, right, and up directions of body frame
        var bodyDirs = new Vector3[3];
        for (int i = 0; i < 3; i++)
        {
            var axisDirInInertial = Vector3.zero;
            axisDirInInertial[i] = 1;

            bodyDirs[i] = transform.localToWorldMatrix * _inertialToUnity * axisDirInInertial;
            Gizmos.color = colors[i];
            Gizmos.DrawLine(transform.position, transform.position + bodyDirs[i]);
        }

        // Draw motor torque vector
        var scaleRMP = 1 / quadcopter.Specification.MaxMotorRPM * 4; // note this has to be adjusted in the long run
        var up = bodyDirs[2];
        Vector3[] rotorDirs = { bodyDirs[0], bodyDirs[1], -bodyDirs[0], -bodyDirs[1] };
        Gizmos.color = Color.black;

        for (int i = 0; i < quadcopter.Specification.NumRotors; i++)
        {
            Vector3 relativeTorque = scaleRMP * quadcopter.MotorAngularVelocity[i] * up;
            var originRotor = transform.position + rotorDirs[i] * quadcopter.Specification.ArmLength;
            Gizmos.DrawLine(originRotor, originRotor + relativeTorque);
            ConeMesh.DrawCone(originRotor + relativeTorque, up, 0.5f * quadcopter.MotorAngularVelocity[i] * scaleRMP);
        }
    }

    private Matrix4x4 _inertialToUnity = new(new(1, 0, 0), new(0, 0, 1), new Vector4(0, 1, 0), new(0, 0, 0, 1));
    private Matrix4x4 _unityToInertial => _inertialToUnity.inverse;


    void OnGUI()
    {
        Event e = Event.current;
        if (Input.GetKeyUp(KeyCode.R))
        {
            quadcopter.ResetState();
            controller.Reset();
        }

        /*if (e.isKey && e.keyCode == KeyCode.UpArrow)
        {
            for (int i = 0; i < 4; i++)
            {
                quadcopter.MotorAngularVelocity[i] += 100;
            }
        }
        if (e.isKey && e.keyCode == KeyCode.DownArrow)
        {
            for (int i = 0; i < 4; i++)
            {
                quadcopter.MotorAngularVelocity[i] -= 100;
            }
        }*/
    }

    // Start is called before the first frame update
    void Start()
    {
        quadcopter.Position = _unityToInertial * transform.localPosition;
        quadcopter.EulerAngles = _unityToInertial * transform.eulerAngles;
        quadcopter.ResetState();

        controller = new PDController(quadcopter, new Gyro(0f, quadcopter));
    }

    // Update is called once per frame
    void Update()
    {
        if (float.IsNaN(quadcopter.EulerAngles.x))
            return;

        var dt = Time.deltaTime * timeScale;
        quadcopter.Update(dt);

        if (EnableController)
        {
            controller.Update(dt);
        }

        if (float.IsNaN(quadcopter.EulerAngles.x))
            return;

        transform.localPosition = _inertialToUnity * quadcopter.Position;
        transform.eulerAngles = _inertialToUnity * (Mathf.Rad2Deg * quadcopter.EulerAngles);
    }
}
