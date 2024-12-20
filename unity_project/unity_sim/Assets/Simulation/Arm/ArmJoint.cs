using UnityEngine;

enum Axis
{
    X,
    Y,
    Z
};

public class ArmJoint : MonoBehaviour
{
    [SerializeField]
    private float mass = 1.0f;
    [SerializeField]
    private Axis axis = Axis.X;
    [SerializeField]
    private bool reverseAngle = false;
    [SerializeField]
    private float targetAngle = 0.0f;
    [SerializeField]
    private float minAngle = -180;
    [SerializeField]
    private float maxAngle = 180;

    private bool initialized = false;
    private Rigidbody rb;
    private HingeJoint hj;

    public void SetTargetAngle(float angle) {
        targetAngle = angle;
        targetAngle = Mathf.Clamp(targetAngle, minAngle, maxAngle);
    }

    public void OffsetTargetAngle(float angle) {
        targetAngle += angle;
        targetAngle = Mathf.Clamp(targetAngle, minAngle, maxAngle);
    }

    #if UNITY_EDITOR
    private void OnValidate()
    {
        transform.localEulerAngles = GetAxisVector() * targetAngle;
    }
    #endif

    private void Awake()
    {
        transform.localEulerAngles = Vector3.zero;

    }

    private float targetAngleSmooth = 0.0f;
    private const float MAX_TARGET_RATE_OF_CHANGE = 60.0f;
    private const float COLLISION_THRESHOLD = 10.0f;
    private float springDisabledTimer = 0.0f;
    private const float SPRING_DISABLE_DURATION = 0.05f;

    private void Start()
    {
        if (initialized)
        {
            return;
        }
        initialized = true;

        // Add a rigidbody to the object
        rb = gameObject.AddComponent<Rigidbody>();
        rb.mass = mass;
        rb.solverIterations = 255;
        // override center of mass to 0,0,0
        rb.centerOfMass = Vector3.zero;

        // Add a hinge joint to the object
        hj = gameObject.AddComponent<HingeJoint>();
        hj.axis = GetAxisVector();
        hj.useSpring = true;
        JointSpring spring = hj.spring;
        spring.spring = 1000000.0f;
        spring.damper = 0;
        hj.useAcceleration = true;
        hj.spring = spring;
        hj.enableCollision = true;
        hj.anchor = Vector3.zero;

        // Attach hinge joint to parent
        if (transform.parent)
        {
            Rigidbody parentRb = transform.parent.GetComponent<Rigidbody>();
            ArmJoint parentArmJoint = transform.parent.GetComponent<ArmJoint>();

            if (!parentRb && parentArmJoint)
            {
                // Parent is another arm joint that has not been yet initialized
                parentArmJoint.Start();
                parentRb = transform.parent.GetComponent<Rigidbody>();
            }

            if (!parentArmJoint) {
                // Parent is not an arm joint
                parentRb = getYoungestAncestorRb();
            }

            parentRb.solverIterations = 255;

            if (parentRb)
            {
                hj.connectedBody = parentRb;
            }
        }
    }

    private void FixedUpdate()
    {
        // Smoothly move smooth target towards raw target angle
        float targetAngleDelta = targetAngle - targetAngleSmooth;
        targetAngleDelta = Mathf.Clamp(targetAngleDelta, -MAX_TARGET_RATE_OF_CHANGE * Time.fixedDeltaTime, MAX_TARGET_RATE_OF_CHANGE * Time.fixedDeltaTime);
        targetAngleSmooth += targetAngleDelta;

        // Update hinge target
        JointSpring spring = hj.spring;
        spring.targetPosition = NormalizeAngle(targetAngleSmooth);
        hj.spring = spring;

        // // Check if joint is colliding
        // float currAngle = hj.angle;
        // if (float.IsNaN(currAngle))
        // {
        //     currAngle = 0;
        // }
        // if (Mathf.Abs(targetAngleSmooth - currAngle) > COLLISION_THRESHOLD)
        // {
        //     // Reset target angle to current angle
        //     targetAngle = currAngle;
        //     springDisabledTimer = SPRING_DISABLE_DURATION;
        //     hj.useSpring = false;
        //     Debug.Log("Joint " + name + " is colliding.");
        // }

        // // Re-enable spring after a short delay
        // if (springDisabledTimer > 0.0f)
        // {
        //     springDisabledTimer -= Time.fixedDeltaTime;
        //     if (springDisabledTimer <= 0.0f)
        //     {
        //         hj.useSpring = true;
        //     }
        // }

        // Debug.Log("Joint " + name + " target: " + targetAngleSmooth + " current: " + currAngle);
    }

    private Rigidbody getYoungestAncestorRb() {
        Transform parent = transform.parent;
        while (parent)
        {
            Rigidbody parentRb = parent.GetComponent<Rigidbody>();
            if (parentRb)
            {
                return parentRb;
            }
            parent = parent.parent;
        }
        return null;
    }

    private float NormalizeAngle(float angle)
    {
        return Mathf.Atan2(Mathf.Sin(angle * Mathf.Deg2Rad), Mathf.Cos(angle * Mathf.Deg2Rad)) * Mathf.Rad2Deg;
    }

    private Vector3 GetAxisVector()
    {
        Vector3 axisVector = Vector3.zero;
        switch (axis)
        {
            case Axis.X:
                axisVector = Vector3.right;
                break;
            case Axis.Y:
                axisVector = Vector3.up;
                break;
            case Axis.Z:
                axisVector = Vector3.forward;
                break;
        }
        if (reverseAngle)
        {
            axisVector *= -1;
        }
        return axisVector;
    }
}
