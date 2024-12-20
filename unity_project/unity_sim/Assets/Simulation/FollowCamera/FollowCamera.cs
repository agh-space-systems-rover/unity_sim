using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;


public class FollowCamera : MonoBehaviour
{
    private const float FOLLOW_RADIUS = 2.0F;
    private const float FOLLOW_HEIGHT = 0.5F;
    private const float MAX_FOLLOW_SPEED = 20.0F;
    private const float MOUSE_SENSITIVITY = 0.1F;

    [SerializeField]
    private List<GameObject> targets;

    // private Vector2 lastMouseInput = Vector2.zero;
    private bool hasFocus = false;
    private bool manual = false;
    private float manualRadius = 0;

    private Vector2 mouseInput = Vector2.zero;
    private Vector2 mouseInputSmooth = Vector2.zero;
    private float lastTimeMouseMoved = 0;
    private void OnLook(InputValue value)
    {
        Vector2 mouseDelta = value.Get<Vector2>();
        mouseInput += mouseDelta;
        lastTimeMouseMoved = Time.time;
        if (!manual)
        {
            manualRadius = (transform.position - meanTargetPos()).magnitude;
        }
        manual = true;
    }

    private void OnClick()
    {
        hasFocus = true;
    }

    private void OnCancel()
    {
        hasFocus = false;
    }

    void Start()
    {
        if (targets.Count == 0)
        {
            Debug.LogError("No targets for FollowCamera.");
        }

        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    void Update()
    {
        Vector3 meanPos = meanTargetPos();

        // Find target camera position.
        Vector3 targetCameraPos;
        if (manual)
        {
            // Use camera angle and current radius.

            targetCameraPos = meanPos + transform.rotation * Vector3.back * manualRadius;
        }
        else
        {
            // Use relative position of objects.

            // Compute 2D ground-plane vector from target to camera.
            Vector3 targetToCamera2d = transform.position - meanPos;
            targetToCamera2d.y = 0.0F;

            // Position camera on a FOLLOW_RADIUS ring around the target at FOLLOW_HEIGHT.
            targetCameraPos = meanPos;
            targetCameraPos.y += FOLLOW_HEIGHT;
            targetCameraPos += targetToCamera2d.normalized * FOLLOW_RADIUS;
        }

        // Move the camera towards the target.
        Vector3 toTarget = targetCameraPos - transform.position;
        if (!manual && toTarget.magnitude > 0.001F)
        {
            float speed = (1 - Mathf.Exp(-toTarget.magnitude / 5)) * MAX_FOLLOW_SPEED;
            transform.position += toTarget.normalized * speed * Time.deltaTime;
        }
        else
        {
            transform.position = targetCameraPos;
        }

        // Disable manual mode if there is no mouse input.
        if (Time.time - lastTimeMouseMoved > 1.0F)
        {
            manual = false;
        }

        if (hasFocus)
        {
            // Rotate the camera around the target based on mouse input.

            mouseInputSmooth = Vector2.Lerp(mouseInputSmooth, mouseInput, 0.2F);
            mouseInput = Vector2.zero;

            Vector3 eulerAngles = transform.eulerAngles;
            eulerAngles.x -= mouseInputSmooth.y * MOUSE_SENSITIVITY;
            eulerAngles.x = eulerAngles.x > 180 ? eulerAngles.x - 360 : eulerAngles.x;
            eulerAngles.x = Mathf.Clamp(eulerAngles.x, -89, 89);
            eulerAngles.y += mouseInputSmooth.x * MOUSE_SENSITIVITY;
            Vector3 eulerDelta = eulerAngles - transform.eulerAngles;

            // Clamp for safety
            // eulerDelta.x = Mathf.Clamp(eulerDelta.x, -10, 10);
            // eulerDelta.y = Mathf.Clamp(eulerDelta.y, -10, 10);

            transform.RotateAround(meanPos, Vector3.up, eulerDelta.y);
            transform.RotateAround(meanPos, transform.right, eulerDelta.x);
        }

        // Find terrain height in camera's XZ position.
        float height = -Mathf.Infinity;
        RaycastHit hit;
        if (Physics.Raycast(transform.position + Vector3.up * 1000, Vector3.down, out hit, 2000))
        {
            // Check if terrain was hit.
            if (hit.collider.gameObject.GetComponent<Terrain>() != null)
            {
                // Move camera to hit position.
                height = Mathf.Max(height, hit.point.y);
            }
        }

        // Move camera to height if below it.
        height += 0.5F;
        if (height > transform.position.y)
        {
            transform.position = new Vector3(transform.position.x, height, transform.position.z);
        }

        // Rotate the camera to look at the target.
        transform.LookAt(meanPos);
    }

    Vector3 meanTargetPos() {
        if (targets.Count == 0)
        {
            return Vector3.zero;
        }

        // Compute the mean position of the targets.
        Vector3 meanPosition = Vector3.zero;
        foreach (GameObject target in targets)
        {
            meanPosition += target.transform.position;
        }
        meanPosition /= targets.Count;
        return meanPosition;
    }
}
