// Smooth Follow from Standard Assets
// If you have C# code and you want to edit SmoothFollow's vars ingame, use this instead.

using UnityEngine;

public class SmoothFollow : MonoBehaviour
{
    public Transform target;

    public float distance = 10.0f;

    public float height = 5.0f;

    public bool clampToFloor;

    // How much we 
    public float heightDamping = 2.0f;
    public float rotationDamping = 3.0f;

    [AddComponentMenu("Camera-Control/Smooth Follow")]
    void Start()
    {
    }

    void LateUpdate()
    {
        if (!target) return;

        float wantedRotationAngle = target.eulerAngles.y;
        float wantedHeight = clampToFloor ? height : target.position.y + height;

        float currentRotationAngle = transform.eulerAngles.y;
        float currentHeight = transform.position.y;

        currentRotationAngle =
            Mathf.LerpAngle(currentRotationAngle, wantedRotationAngle, rotationDamping * Time.deltaTime);

        currentHeight = Mathf.Lerp(currentHeight, wantedHeight, heightDamping * Time.deltaTime);

        var currentRotation = Quaternion.Euler(0, currentRotationAngle, 0);

        transform.position = target.position;
        transform.position -= currentRotation * Vector3.forward * distance;

        transform.position = new Vector3(transform.position.x, currentHeight, transform.position.z);

        transform.LookAt(target);
    }
}