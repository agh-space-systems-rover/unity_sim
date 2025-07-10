using UnityEngine;
using System.Collections.Generic;

enum Axis
{
    X,
    Y,
    Z
};

public class ArmJoint : MonoBehaviour
{
    private const float SPRING_FORCE = 1000;
    private const float GRIP_FORCE = 10;
    private const float MAX_TARGET_RATE_OF_CHANGE = 60; // deg/s
    private const float COLLISION_THRESHOLD = 10;
    private const float COLLISION_THRESHOLD_DURATION = 0.1f;

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
    private Vector3 ogParentPosOffset = Vector3.zero;

    private int envCollCount = 0;
    private int grabCount = 0;
    private float targetAngleSmooth = 0;
    private float collisionTimer = 0;

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
        rb.solverVelocityIterations = 255;
        // override center of mass to 0,0,0
        rb.centerOfMass = Vector3.zero;

        // Add a hinge joint to the object
        hj = gameObject.AddComponent<HingeJoint>();
        hj.axis = GetAxisVector();
        hj.useSpring = true;
        hj.enableCollision = true;
        hj.anchor = Vector3.zero;
        JointLimits limits = hj.limits;
        limits.min = minAngle;
        limits.bounciness = 0;
        limits.bounceMinVelocity = 0;
        limits.max = maxAngle;
        hj.limits = limits;
        hj.useLimits = true;
        hj.extendedLimits = true;

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
            rb.solverVelocityIterations = 255;

            if (parentRb)
            {
                hj.connectedBody = parentRb;
            }

            ogParentPosOffset = transform.localPosition;
        }

        Debug.Log("Joint " + name + " initialized. Is jaw: " + isThisJaw());
    }

    private void FixedUpdate()
    {
        // Move smooth target towards raw target angle
        float targetAngleDelta = targetAngle - targetAngleSmooth;
        targetAngleDelta = Mathf.Clamp(targetAngleDelta, -MAX_TARGET_RATE_OF_CHANGE * Time.fixedDeltaTime, MAX_TARGET_RATE_OF_CHANGE * Time.fixedDeltaTime);
        targetAngleSmooth += targetAngleDelta;

        // Update hinge target
        JointSpring spring = hj.spring;
        spring.targetPosition = NormalizeAngle(targetAngleSmooth);
        spring.spring = isThisJaw() && grabCount > 0 ? GRIP_FORCE : SPRING_FORCE;
        spring.damper = spring.spring * 0.01f;
        hj.spring = spring;

        // Lock position for extra stability
        if (!isJawGrabbing()) {
            Matrix4x4 parentToWorld = transform.parent.localToWorldMatrix;
            rb.MovePosition(parentToWorld.MultiplyPoint(ogParentPosOffset));
        }

        // Detect and mitigate collisions
        // Get current angle
        if (isChildCollidingWithEnvironment())
        {
            float currAngle = CurrentAngle();
            // If target cannot be reached...
            if (Mathf.Abs(Mathf.DeltaAngle(targetAngleSmooth, currAngle)) > COLLISION_THRESHOLD)
            {
                collisionTimer += Time.fixedDeltaTime;
            } else {
                collisionTimer = 0;
            }
            if (collisionTimer > COLLISION_THRESHOLD_DURATION)
            {
                Debug.Log("Joint " + name + " is colliding.");
                targetAngle = LerpAngle(targetAngle, currAngle, 0.5f);
                collisionTimer = 0;
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        // Count number of grabbed objects
        if (isThisJaw()) {
            if (isGameObjectGrabbable(collision.gameObject)) {
                grabCount++;
                Debug.Log("Grabbed " + collision.gameObject.name + " (" + grabCount + ")");
            }
        }

        // Count number of collisions with environment
        {
            ArmJoint otherJoint = collision.gameObject.GetComponent<ArmJoint>();
            if (otherJoint == null)
            {
                envCollCount++;
            }
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        // Count number of grabbed objects
        if (isThisJaw()) {
            if (isGameObjectGrabbable(collision.gameObject)) {
                Debug.Log("Released " + collision.gameObject.name + " (" + grabCount + ")");
                grabCount--;
            }
        }

        // Count number of collisions with environment
        {
            ArmJoint otherJoint = collision.gameObject.GetComponent<ArmJoint>();
            if (otherJoint == null)
            {
                envCollCount--;
            }
        }
    }

    // UTILITIES

    private float LerpAngle(float a, float b, float t)
    {
        float delta = Mathf.DeltaAngle(a, b);
        return a + delta * t;
    }

    // private float ClosestNumber(float x, float[] options)
    // {
    //     float closest = options[0];
    //     float distance = Mathf.Abs(x - closest);
    //     for (int i = 1; i < options.Length; i++)
    //     {
    //         float newDistance = Mathf.Abs(x - options[i]);
    //         if (newDistance < distance)
    //         {
    //             closest = options[i];
    //             distance = newDistance;
    //         }
    //     }
    //     return closest;
    // }

    // private Vector3 LockEulerAngles(Vector3 eulerAngles, Axis axis)
    // {
    //     switch (axis)
    //     {
    //         case Axis.X:
    //             eulerAngles.y = ClosestNumber(eulerAngles.y, new float[] { 0, 180, 360 });
    //             eulerAngles.z = ClosestNumber(eulerAngles.z, new float[] { 0, 180, 360 });
    //             break;
    //         case Axis.Y:
    //             eulerAngles.x = ClosestNumber(eulerAngles.x, new float[] { 0, 180, 360 });
    //             eulerAngles.z = ClosestNumber(eulerAngles.z, new float[] { 0, 180, 360 });
    //             break;
    //         case Axis.Z:
    //             eulerAngles.x = ClosestNumber(eulerAngles.x, new float[] { 0, 180, 360 });
    //             eulerAngles.y = ClosestNumber(eulerAngles.y, new float[] { 0, 180, 360 });
    //             break;
    //     }
    //     return eulerAngles;
    // }

    public float CurrentAngle()
    {
        float angle;
        Vector3 axis;
        transform.localRotation.ToAngleAxis(out angle, out axis);
        Vector3 rotationAxis = GetAxisVector();
        float signedAngle = angle * Vector3.Dot(rotationAxis, axis);
        return NormalizeAngle(signedAngle);
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

    private bool isThisJaw()
    {
        foreach (Transform child in transform)
        {
            if (child.GetComponent<ArmJoint>())
            {
                return false;
            }
        }
        return true;
    }

    private ArmJoint findJaw()
    {
        ArmJoint candidate = this;
        while (!candidate.isThisJaw())
        {
            foreach (Transform child in candidate.transform)
            {
                ArmJoint childJoint = child.GetComponent<ArmJoint>();
                if (childJoint)
                {
                    candidate = childJoint;
                    break;
                }
            }
        }
        return candidate;
    }

    private bool isJawGrabbing() {
        return findJaw().grabCount > 0;
    }

    private bool isChildCollidingWithEnvironment() {
        Transform tf = transform;
        while (tf)
        {
            ArmJoint childJoint = null;
            foreach (Transform child in tf)
            {
                childJoint = child.GetComponent<ArmJoint>();
                if (childJoint) {
                    break;
                }
            }
            if (childJoint) {
                if (childJoint.envCollCount > 0) {
                    return true;
                } else {
                    tf = childJoint.transform;
                }
            } else {
                tf = null;
            }
        }
        return false;
    }

    private bool isGameObjectGrabbable(GameObject go) {
        Rigidbody rb = go.GetComponent<Rigidbody>();
        if (rb) {
            // Check if gameobject is our parent
            Transform tf = transform;
            while (tf)
            {
                if (tf.gameObject == go) {
                    return false;
                }
                tf = tf.parent;
            }
            // check if gameobject is a child
            Stack<Transform> stack = new Stack<Transform>();
            stack.Push(transform);
            while (stack.Count > 0) {
                Transform current = stack.Pop();
                foreach (Transform child in current) {
                    if (child.gameObject == go) {
                        return false;
                    }
                    stack.Push(child);
                }
            }
            return true;
        }
        return false;
    }
}
