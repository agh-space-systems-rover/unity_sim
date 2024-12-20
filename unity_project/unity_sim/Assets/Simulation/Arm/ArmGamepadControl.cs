using UnityEngine;
using UnityEngine.InputSystem;

public class ArmGamepadControl : MonoBehaviour
{
    [SerializeField]
    private float speed = 40;

    private ArmJoint[] joints = new ArmJoint[7];
    private float[] jointSpeeds = new float[7];

    private void Start()
    {
        GameObject curr = gameObject;
        for (int i = 0; i < 7; i++) {
            curr = curr.transform.GetChild(0).gameObject;
            joints[i] = curr.GetComponent<ArmJoint>();
        }
    }

    private void Update()
    {
        float dt = Time.deltaTime;

        for (int i = 0; i < 7; i++) {
            if (joints[i] != null) {
                joints[i].OffsetTargetAngle(jointSpeeds[i] * dt * speed);
            }
        }
    }

    private void OnJoint1(InputValue value) {
        jointSpeeds[0] = value.Get<float>();
    }

    private void OnJoint2(InputValue value) {
        jointSpeeds[1] = value.Get<float>();
    }

    private void OnJoint3(InputValue value) {
        jointSpeeds[2] = value.Get<float>();
    }

    private void OnJoint4(InputValue value) {
        jointSpeeds[3] = value.Get<float>();
    }

    private void OnJoint5(InputValue value) {
        jointSpeeds[4] = value.Get<float>();
    }

    private void OnJoint6(InputValue value) {
        jointSpeeds[5] = value.Get<float>();
    }

    private void OnJaw(InputValue value) {
        jointSpeeds[6] = value.Get<float>();
    }
}
