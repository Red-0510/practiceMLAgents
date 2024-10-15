using System.Collections.Generic;
using UnityEngine;

public class ImitationReward : MonoBehaviour
{
    // Teacher and Ragdoll body parts (set in the Unity Editor or dynamically)
    public List<Transform> teacherBodyParts;  // Reference to the teacher body parts
    public List<Transform> ragdollBodyParts;  // Reference to the ragdoll body parts

    // Two orientation cubes for reference
    public Transform teacherOrientationCube;
    public Transform ragdollOrientationCube;

    // Weight parameters for different reward components
    public float poseWeight = 0.6f;
    public float velocityWeight = 0.1f;
    public float endEffectorWeight = 0.2f;
    public float contactWeight = 0.1f;
    public float taskWeight = 0.1f;

    // Optional task-related goal (e.g., standing up)
    public float targetHeight = 0.5f;

    // To store previous angular velocities for velocity reward
    private List<Vector3> previousRagdollAngularVelocities;
    private List<Vector3> previousTeacherAngularVelocities;

    void Start()
    {
        // Initialize angular velocities
        previousRagdollAngularVelocities = new List<Vector3>(new Vector3[ragdollBodyParts.Count]);
        previousTeacherAngularVelocities = new List<Vector3>(new Vector3[teacherBodyParts.Count]);
    }

    public float CalculateReward()
    {
        float totalReward = 0.0f;
        // Loop through each body part
        for (int i = 0; i < ragdollBodyParts.Count; i++)
        {
            // Pose Reward: Match rotations relative to orientation cubes
            totalReward += poseWeight * PoseReward(teacherBodyParts[i], ragdollBodyParts[i]);

            // Velocity Reward: Match angular velocities
            totalReward += velocityWeight * VelocityReward(teacherBodyParts[i], ragdollBodyParts[i], i);

            // End Effector Reward: Match end-effector positions (hands, feet, etc.)
            totalReward += endEffectorWeight * EndEffectorReward(teacherBodyParts[i], ragdollBodyParts[i]);

            // Contact Reward: Penalize/Reward for ground contact matching
            // totalReward += contactWeight * GroundContactReward(teacherBodyParts[i], ragdollBodyParts[i]);
        }

        // Task-specific Reward: For example, standing up
        totalReward += taskWeight * TaskReward();

        return totalReward;
    }

    private float PoseReward(Transform teacherPart, Transform ragdollPart)
    {
        // Calculate relative rotations using the respective orientation cubes
        Quaternion teacherRelativeRotation = Quaternion.Inverse(teacherOrientationCube.rotation) * teacherPart.rotation;
        Quaternion ragdollRelativeRotation = Quaternion.Inverse(ragdollOrientationCube.rotation) * ragdollPart.rotation;

        float angleDifference = Quaternion.Angle(teacherRelativeRotation, ragdollRelativeRotation);
        return Mathf.Exp(-angleDifference * angleDifference);
    }

    private float VelocityReward(Transform teacherPart, Transform ragdollPart, int index)
    {
        Vector3 currentRagdollAngularVelocity = ragdollPart.GetComponent<Rigidbody>().angularVelocity;
        Vector3 currentTeacherAngularVelocity = teacherPart.GetComponent<Rigidbody>().angularVelocity;

        // Transform velocities to their respective orientation cube's local space
        currentRagdollAngularVelocity = ragdollOrientationCube.InverseTransformDirection(currentRagdollAngularVelocity);
        currentTeacherAngularVelocity = teacherOrientationCube.InverseTransformDirection(currentTeacherAngularVelocity);

        Vector3 velocityDifference = currentTeacherAngularVelocity - currentRagdollAngularVelocity;
        float velocityReward = Mathf.Exp(-velocityDifference.sqrMagnitude);

        // Update previous angular velocities
        previousRagdollAngularVelocities[index] = currentRagdollAngularVelocity;
        previousTeacherAngularVelocities[index] = currentTeacherAngularVelocity;

        return velocityReward;
    }

    private float EndEffectorReward(Transform teacherPart, Transform ragdollPart)
    {
        // Calculate position difference relative to their respective orientation cubes
        Vector3 teacherLocalPosition = teacherOrientationCube.InverseTransformPoint(teacherPart.position);
        Vector3 ragdollLocalPosition = ragdollOrientationCube.InverseTransformPoint(ragdollPart.position);

        Vector3 positionDifference = teacherLocalPosition - ragdollLocalPosition;
        return Mathf.Exp(-40 * positionDifference.sqrMagnitude); // Adjust the factor as needed
    }

    // Uncomment and implement if necessary
    // private float GroundContactReward(Transform teacherPart, Transform ragdollPart) { ... }

    private float TaskReward()
    {
        float ragdollHeight = ragdollBodyParts[0].position.y; // Assuming the first body part is the pelvis
        return Mathf.Exp(-Mathf.Pow(ragdollHeight - targetHeight, 2));
    }
}
