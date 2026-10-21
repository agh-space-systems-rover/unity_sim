using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.InputSystem;

using ROSBridge;
using ROSBridge.KalmanInterfaces;

public class KalmanVehicle : MonoBehaviour
{
    [SerializeField]
    private WheelCollider flWheel, frWheel, blWheel, brWheel;

    [SerializeField]
    private Transform rootBone;

    [SerializeField]
    private Transform differentialBone;

    [SerializeField]
    private Transform flSuspBone, frSuspBone, blSuspBone, brSuspBone;

    [SerializeField]
    private Transform flTurnBone, frTurnBone, blTurnBone, brTurnBone;

    [SerializeField]
    private Transform flWheelBone, frWheelBone, blWheelBone, brWheelBone;
    [SerializeField]
    private string wheelStatesTopic = "wheel_states";
    [SerializeField]
    private string wheelStatesReturnTopic = "wheel_states/return";
    [SerializeField]
    private string wheelTempsTopic = "wheel_temps";
    [SerializeField]
    private float wheelStatesReturnRate = 5;
    [SerializeField]
    private float wheelTempUpdateRate = 1;

    private const float unitTorque = 80;
    private const float manualSpeed = 1.0F;
    private const float manualTurnRadius = 1.0F;
    private const float maxTurnSpeed = 90;

    private bool isRotatingInPlace = false;
    private Vector2 currentInput = Vector2.zero;

    private Dictionary<Transform, Quaternion> ogBoneRots = new Dictionary<Transform, Quaternion>();
    private Dictionary<Transform, Vector3> ogSupToWheelVecs = new Dictionary<Transform, Vector3>();
    private Quaternion ogDiffRot;

    private ROS ros;
    private WheelStates wheelStates;
    private PID flTurnPid, frTurnPid, blTurnPid, brTurnPid;
    private PID flTorquePid, frTorquePid, blTorquePid, brTorquePid;
    private Publisher<WheelStates> returnPublisher;
    private double lastReturnTime = 0.0;
    private Publisher<WheelTemperatures> wheelTempPublisher;
    private double lastTempTime = 0.0;
    private float[] motorTemps = new float[4] { 20, 20, 20, 20 };
    private float[] swivelTemps = new float[4] { 20, 20, 20, 20 };

    private void OnRotate(InputValue value)
    {
        isRotatingInPlace = value.isPressed;
        UpdateWheelStates(); 
    }

    private void OnMove(InputValue value)
    {
        currentInput = value.Get<Vector2>();
        UpdateWheelStates();
    }

    private void UpdateWheelStates()
    {
        if (isRotatingInPlace)
        {
            float rotationAngle = -Mathf.PI / 4;            
            wheelStates.FrontLeft = new WheelState
            {
                Velocity = manualSpeed * currentInput.y,
                Angle = rotationAngle
            };
            wheelStates.FrontRight = new WheelState
            {
                Velocity = -manualSpeed * currentInput.y,
                Angle = -rotationAngle
            };
            wheelStates.BackLeft = new WheelState
            {
                Velocity = manualSpeed * currentInput.y,
                Angle = -rotationAngle
            };
            wheelStates.BackRight = new WheelState
            {
                Velocity = -manualSpeed * currentInput.y,
                Angle = rotationAngle
            };
            return;
        }

        float turnRadius = -manualTurnRadius / (currentInput.x != 0 ? currentInput.x : 0.0001f);
        float linearVelocity = manualSpeed;
        float angularVelocity = manualSpeed / turnRadius;

        Vector2[] turnVectors = new Vector2[]
        {
            new Vector2(-0.33F, 0.4F),
            new Vector2(0.33F, 0.4F),
            new Vector2(-0.33F, -0.4F),
            new Vector2(0.33F, -0.4F)
        };

        for (int i = 0; i < 4; i++)
        {
            float robotRadius = 0.5F;
            Vector2 linearVector = new Vector2(linearVelocity, 0);
            Vector2 angularVector = (turnVectors[i] / turnVectors[i].magnitude) * angularVelocity * robotRadius;
            Vector2 resultantVector = angularVector + linearVector;
            float velocity = resultantVector.magnitude;
            float angle = Mathf.Atan2(resultantVector.y, resultantVector.x);

            switch (i)
            {
                case 0:
                    wheelStates.FrontLeft = new WheelState { Velocity = velocity * currentInput.y, Angle = angle };
                    break;
                case 1:
                    wheelStates.FrontRight = new WheelState { Velocity = velocity * currentInput.y, Angle = angle };
                    break;
                case 2:
                    wheelStates.BackLeft = new WheelState { Velocity = velocity * currentInput.y, Angle = angle };
                    break;
                case 3:
                    wheelStates.BackRight = new WheelState { Velocity = velocity * currentInput.y, Angle = angle };
                    break;
            }
        }
    }

    private void Start()
    {
        GetComponent<Rigidbody>().centerOfMass = new Vector3(0, 0.25F, -0.1F);

        if (differentialBone != null)
        {
            ogDiffRot = differentialBone.localRotation;
        }

        ros = new ROS();

        wheelStates = new WheelStates
        {
            FrontLeft = new WheelState { Velocity = 0, Angle = 0 },
            FrontRight = new WheelState { Velocity = 0, Angle = 0 },
            BackLeft = new WheelState { Velocity = 0, Angle = 0 },
            BackRight = new WheelState { Velocity = 0, Angle = 0 }
        };

        flTurnPid = frTurnPid = blTurnPid = brTurnPid = new PID(0.2F, 0, 0);
        flTorquePid = frTorquePid = blTorquePid = brTorquePid = new PID(1, 0, 0);

        ros.CreateSubscription<WheelStates>(wheelStatesTopic, (msg) =>
        {
            wheelStates = msg;
        });

        returnPublisher = ros.CreatePublisher<WheelStates>(wheelStatesReturnTopic);
        wheelTempPublisher = ros.CreatePublisher<WheelTemperatures>(wheelTempsTopic);
    }

    private void OnApplicationQuit()
    {
        if (ros != null)
        {
            ros.Close();
        }
    }

    private async void FixedUpdate()
    {
        float[] oldSteerAngles = new float[4] { flWheel.steerAngle, frWheel.steerAngle, blWheel.steerAngle, brWheel.steerAngle };

        flWheel.steerAngle += Mathf.Clamp(flTurnPid.Update(flWheel.steerAngle, -wheelStates.FrontLeft.Angle * Mathf.Rad2Deg, Time.fixedDeltaTime), -maxTurnSpeed * Time.fixedDeltaTime, maxTurnSpeed * Time.fixedDeltaTime);
        frWheel.steerAngle += Mathf.Clamp(frTurnPid.Update(frWheel.steerAngle, -wheelStates.FrontRight.Angle * Mathf.Rad2Deg, Time.fixedDeltaTime), -maxTurnSpeed * Time.fixedDeltaTime, maxTurnSpeed * Time.fixedDeltaTime);
        blWheel.steerAngle += Mathf.Clamp(blTurnPid.Update(blWheel.steerAngle, -wheelStates.BackLeft.Angle * Mathf.Rad2Deg, Time.fixedDeltaTime), -maxTurnSpeed * Time.fixedDeltaTime, maxTurnSpeed * Time.fixedDeltaTime);
        brWheel.steerAngle += Mathf.Clamp(brTurnPid.Update(brWheel.steerAngle, -wheelStates.BackRight.Angle * Mathf.Rad2Deg, Time.fixedDeltaTime), -maxTurnSpeed * Time.fixedDeltaTime, maxTurnSpeed * Time.fixedDeltaTime);

        flWheel.motorTorque += flTorquePid.Update(flWheel.motorTorque, unitTorque * wheelStates.FrontLeft.Velocity, Time.fixedDeltaTime);
        frWheel.motorTorque += frTorquePid.Update(frWheel.motorTorque, unitTorque * wheelStates.FrontRight.Velocity, Time.fixedDeltaTime);
        blWheel.motorTorque += blTorquePid.Update(blWheel.motorTorque, unitTorque * wheelStates.BackLeft.Velocity, Time.fixedDeltaTime);
        brWheel.motorTorque += brTorquePid.Update(brWheel.motorTorque, unitTorque * wheelStates.BackRight.Velocity, Time.fixedDeltaTime);

        BrakeWheelIfStationary(flWheel);
        BrakeWheelIfStationary(frWheel);
        BrakeWheelIfStationary(blWheel);
        BrakeWheelIfStationary(brWheel);

        for (int i = 0; i < 4; i++)
        {
            float swivelEnergy = Mathf.Abs((oldSteerAngles[i] - i switch
            {
                0 => flWheel.steerAngle,
                1 => frWheel.steerAngle,
                2 => blWheel.steerAngle,
                3 => brWheel.steerAngle,
                _ => 0
            }) / Time.fixedDeltaTime) * 0.001F;
            swivelTemps[i] += swivelEnergy;
            swivelTemps[i] = Mathf.Clamp(Mathf.Lerp(swivelTemps[i], 20, Time.fixedDeltaTime * 0.1F), 20, 100);

            float motorEnergy = Mathf.Abs((i switch
            {
                0 => flWheel.motorTorque,
                1 => frWheel.motorTorque,
                2 => blWheel.motorTorque,
                3 => brWheel.motorTorque,
                _ => 0
            }) / Time.fixedDeltaTime) * 0.0001F;
            motorTemps[i] += motorEnergy;
            motorTemps[i] = Mathf.Clamp(Mathf.Lerp(motorTemps[i], 20, Time.fixedDeltaTime * 0.1F), 20, 100);
        }

        double now = Time.unscaledTimeAsDouble;
        if (now - lastReturnTime > 1.0 / wheelStatesReturnRate && returnPublisher != null)
        {
            lastReturnTime = now;
            await returnPublisher.Publish(new WheelStates
            {
                FrontLeft = new WheelState { Velocity = wheelStates.FrontLeft.Velocity, Angle = -flWheel.steerAngle * Mathf.Deg2Rad },
                FrontRight = new WheelState { Velocity = wheelStates.FrontRight.Velocity, Angle = -frWheel.steerAngle * Mathf.Deg2Rad },
                BackLeft = new WheelState { Velocity = wheelStates.BackLeft.Velocity, Angle = -blWheel.steerAngle * Mathf.Deg2Rad },
                BackRight = new WheelState { Velocity = wheelStates.BackRight.Velocity, Angle = -brWheel.steerAngle * Mathf.Deg2Rad }
            });
        }

        if (now - lastTempTime > 1.0 / wheelTempUpdateRate && wheelTempPublisher != null)
        {
            lastTempTime = now;
            await wheelTempPublisher.Publish(new WheelTemperatures
            {
                FrontLeft = new WheelTemperature { Motor = motorTemps[0] + 273.15F, Swivel = swivelTemps[0] + 273.15F },
                FrontRight = new WheelTemperature { Motor = motorTemps[1] + 273.15F, Swivel = swivelTemps[1] + 273.15F },
                BackLeft = new WheelTemperature { Motor = motorTemps[2] + 273.15F, Swivel = swivelTemps[2] + 273.15F },
                BackRight = new WheelTemperature { Motor = motorTemps[3] + 273.15F, Swivel = swivelTemps[3] + 273.15F }
            });
        }
    }

    private void BrakeWheelIfStationary(WheelCollider wheel)
    {
        wheel.brakeTorque = Mathf.Abs(wheel.motorTorque) < 0.1F ? 1 : 0;
    }

    private void Update()
    {
        CopyWheelTransform(flWheel, flSuspBone, flTurnBone, flWheelBone, false, true);
        CopyWheelTransform(frWheel, frSuspBone, frTurnBone, frWheelBone, true, true);
        CopyWheelTransform(blWheel, blSuspBone, blTurnBone, blWheelBone, false, false);
        CopyWheelTransform(brWheel, brSuspBone, brTurnBone, brWheelBone, true, false);

        UpdateDifferentialRotation();
    }

    private void UpdateDifferentialRotation()
    {
        if (differentialBone == null || blWheel == null || brWheel == null) return;

        blWheel.GetWorldPose(out Vector3 blPos, out _);
        brWheel.GetWorldPose(out Vector3 brPos, out _);

        Transform parentTransform = differentialBone.parent != null ? differentialBone.parent : transform;
        Vector3 blLocal = parentTransform.InverseTransformPoint(blPos);
        Vector3 brLocal = parentTransform.InverseTransformPoint(brPos);
        Vector3 axleLocal = brLocal - blLocal;

        Vector2 axleXZ = new Vector2(axleLocal.x, axleLocal.z);

        float angleY = Vector2.SignedAngle(Vector2.right, axleXZ);

        differentialBone.localRotation = ogDiffRot * Quaternion.Euler(0, angleY, 0);
    }

    private void CopyWheelTransform(WheelCollider wheel, Transform suspBone, Transform turnBone, Transform wheelBone, bool rightSide, bool front)
    {
        if (!ogBoneRots.ContainsKey(turnBone))
        {
            ogBoneRots.Add(turnBone, turnBone.localRotation);
        }
        if (!ogBoneRots.ContainsKey(suspBone))
        {
            ogBoneRots.Add(suspBone, suspBone.localRotation);
        }

        if (!ogSupToWheelVecs.ContainsKey(wheelBone))
        {
            Matrix4x4 worldToLocal = rootBone.parent.worldToLocalMatrix;
            Vector3 from = worldToLocal.MultiplyPoint(suspBone.position);
            Vector3 to = worldToLocal.MultiplyPoint(wheelBone.position);
            ogSupToWheelVecs.Add(wheelBone, to - from);
        }

        wheel.GetWorldPose(out Vector3 position, out Quaternion rotation);

        float angularSpeed = (wheel.rpm / 60.0f) * 360.0f * (rightSide ? 1 : -1);
        wheelBone.Rotate(Vector3.up, angularSpeed * Time.deltaTime);

        Quaternion newRot = ogBoneRots[turnBone] * Quaternion.Euler(0, -wheel.steerAngle, 0);
        turnBone.localRotation = newRot;

        Vector3 ogToWheel = ogSupToWheelVecs[wheelBone];
        Matrix4x4 worldToLocal2 = rootBone.parent.worldToLocalMatrix;
        Vector3 from2 = worldToLocal2.MultiplyPoint(suspBone.position);
        Vector3 to2 = worldToLocal2.MultiplyPoint(position);
        Vector3 toWheel = to2 - from2;
        float angle = Vector3.SignedAngle(ogToWheel, toWheel, Vector3.right);

        newRot = ogBoneRots[suspBone] * Quaternion.Euler(0, angle * (rightSide ? -1 : 1) * (front ? 2 : 1), 0);
        suspBone.localRotation = Quaternion.Slerp(suspBone.localRotation, newRot, Time.deltaTime * 10.0F);
    }
}