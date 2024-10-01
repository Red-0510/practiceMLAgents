using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.Assets.Scenes.walker.BodyPart;
using JointDriveController = Unity.Assets.Scenes.walker.MyJointDriveController;
using Random = UnityEngine.Random;

public class Walker : Agent
{
    // Start is called before the first frame update

    [Header("Walk Speed")]
    [Range(0.1f, 10)]
    [SerializeField]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = 10;

    public float MTargetWalkingSpeed // property
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    const float m_maxWalkingSpeed = 10;

    [Header("Body Parts")] public Transform hips;
    public Transform leftUpperLeg;
    public Transform rightUpperLeg;
    public Transform leftLowerLeg;
    public Transform rightLowerLeg;
    public Transform leftFoot;
    public Transform rightFoot;
    public Transform spine;
    public Transform head;
    public Transform leftUpperArm;
    public Transform rightUpperArm;
    public Transform leftLowerArm;
    public Transform rightLowerArm;

    JointDriveController m_JdController;
    OrientationCubeController m_OrientationCube;
    EnvironmentParameters m_ResetParams;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        // m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        //Setup each body part
        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(hips,true);
        m_JdController.SetupBodyPart(leftUpperLeg,true);
        m_JdController.SetupBodyPart(spine,true);
        m_JdController.SetupBodyPart(head,true);
        m_JdController.SetupBodyPart(leftLowerLeg,true);
        m_JdController.SetupBodyPart(leftFoot,false);
        m_JdController.SetupBodyPart(rightUpperLeg,true);
        m_JdController.SetupBodyPart(rightLowerLeg,true);
        m_JdController.SetupBodyPart(rightFoot,false);
        m_JdController.SetupBodyPart(leftUpperArm,true);
        m_JdController.SetupBodyPart(leftLowerArm,true);
        m_JdController.SetupBodyPart(rightUpperArm,true);
        m_JdController.SetupBodyPart(rightLowerArm,true);

        m_ResetParams = Academy.Instance.EnvironmentParameters;
    }

    public override void OnEpisodeBegin()
    {
        //Reset all of the body parts
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        // hips.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        // UpdateOrientationObjects();

        // //Set our goal walking speed
        // MTargetWalkingSpeed =
        //     randomizeWalkSpeedEachEpisode ? Random.Range(0.1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;
    }

    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        //Get velocities in the context of our orientation cube's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        // sensor.AddObservation(bp.rb.angularVelocity);
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        // sensor.AddObservation(bp.rb.angularVelocity);

        //Get position relative to hips in the context of our orientation cube's space
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - hips.position));
        // sensor.AddObservation(bp.rb.position - hips.position);

        // if (bp.rb.transform != hips && bp.rb.transform != leftUpperArm && bp.rb.transform != leftLowerArm)
        // {
            sensor.AddObservation(bp.rb.transform.localRotation);
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        // }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;
        // var cubeForward = transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * MTargetWalkingSpeed;
        //ragdoll's avg vel
        var avgVel = GetAvgVelocity();

        //current ragdoll velocity. normalized
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        // sensor.AddObservation(avgVel);
        //vel goal relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        // sensor.AddObservation(velGoal);

        //rotation deltas
        sensor.AddObservation(Quaternion.FromToRotation(hips.forward, cubeForward));
        sensor.AddObservation(Quaternion.FromToRotation(head.forward, cubeForward));

        //Position of target position relative to cube
        // sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(target.transform.position));
        // sensor.AddObservation(target.transform.position);

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)

    {
        var bpDict = m_JdController.bodyPartsDict;
        var i = -1;

        var continuousActions = actionBuffers.ContinuousActions;
        // bpDict[chest].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[spine].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        bpDict[leftUpperLeg].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[rightUpperLeg].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leftLowerLeg].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[rightLowerLeg].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leftFoot].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[leftFoot].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        bpDict[leftUpperArm].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[rightUpperArm].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leftLowerArm].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[rightLowerArm].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[head].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        //update joint strength settings
        // bpDict[chest].SetJointStrength(continuousActions[++i]);
        bpDict[spine].SetJointStrength(continuousActions[++i]);
        bpDict[head].SetJointStrength(continuousActions[++i]);
        bpDict[leftUpperLeg].SetJointStrength(continuousActions[++i]);
        bpDict[leftLowerLeg].SetJointStrength(continuousActions[++i]);
        bpDict[leftFoot].SetJointStrength(continuousActions[++i]);
        bpDict[rightUpperLeg].SetJointStrength(continuousActions[++i]);
        bpDict[rightLowerLeg].SetJointStrength(continuousActions[++i]);
        bpDict[rightFoot].SetJointStrength(continuousActions[++i]);
        bpDict[leftUpperArm].SetJointStrength(continuousActions[++i]);
        bpDict[leftLowerArm].SetJointStrength(continuousActions[++i]);
        bpDict[rightUpperArm].SetJointStrength(continuousActions[++i]);
        bpDict[rightLowerArm].SetJointStrength(continuousActions[++i]);
    }

    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.velocity;
        }

        var avgVel = velSum / numOfRb;
        return avgVel;
    }

    void UpdateOrientationObjects()
    {
        // m_WorldDirToWalk = target.position - hips.position;
        // m_OrientationCube.UpdateOrientation(hips, target);
        // if (m_DirectionIndicator)
        // {
        //     m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        // }
    }

//     void FixedUpdate()
//     {
//         UpdateOrientationObjects();
//         Transform _hips=hips;
//         Debug.Log(hips.position.y+" "+head.position.y);
//         var hipReward = hips.position.y+0.5f;
//         var headReward = head.position.y+0.4f;
//         AddReward(hipReward+headReward);
//         // if(_hips.position.y>.7f){
//         //     AddReward(0.01f);
//         // }
//         // if(head.position.y>1.2f){
//         //     AddReward(0.04f);
//         // }

//         var cubeForward = m_OrientationCube.transform.forward;

//         // Set reward for this step according to mixture of the following elements.
//         // a. Match target speed
//         //This reward will approach 1 if it matches perfectly and approach zero as it deviates
//         // var matchSpeedReward = GetMatchingVelocityReward(cubeForward * MTargetWalkingSpeed, GetAvgVelocity());

//         //Check for NaNs
//         // if (float.IsNaN(matchSpeedReward))
//         // {
//         //     throw new ArgumentException(
//         //         "NaN in moveTowardsTargetReward.\n" +
//         //         $" cubeForward: {cubeForward}\n" +
//         //         $" hips.velocity: {m_JdController.bodyPartsDict[hips].rb.velocity}\n" +
//         //         $" maximumWalkingSpeed: {m_maxWalkingSpeed}"
//         //     );
//         // }

//         // b. Rotation alignment with target direction.
//         //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
//         // var headForward = head.forward;
//         // headForward.y = 0;
//         // var lookAtTargetReward = (Vector3.Dot(cubeForward, head.forward) + 1) * .5F;
//         // var lookAtTargetReward = (Vector3.Dot(cubeForward, headForward) + 1) * .5F;

//         //Check for NaNs
//         // if (float.IsNaN(lookAtTargetReward))
//         // {
//         //     throw new ArgumentException(
//         //         "NaN in lookAtTargetReward.\n" +
//         //         $" cubeForward: {cubeForward}\n" +
//         //         $" head.forward: {head.forward}"
//         //     );
//         // }

//         // AddReward(matchSpeedReward * lookAtTargetReward);
        
//     }
// }

void FixedUpdate()
{
    UpdateOrientationObjects();
    
    // Get relevant body part references
    Transform _hips = hips;
    Transform _head = head;
    Transform _spine = spine;
    Transform _leftFoot = leftFoot;
    Transform _rightFoot = rightFoot;

    // Calculate rewards for standing up straight
    float hipHeight = _hips.position.y;
    float headHeight = _head.position.y;

    // Reward for keeping the hips and head above a certain height
    if (hipHeight > 0.5f)
    {
        AddReward(1.2f * (hipHeight - 0.5f)); // Increased reward for hip height
    }

    if (headHeight > 1.0f)
    {
        AddReward(2.2f * (headHeight - 1.0f)); // Increased reward for head height
    }

    // Reward for head being above a certain height
    if (headHeight > 0.7f)
    {
        AddReward(2.5f); // Good reward if the head height is above 0.7 units
    }

    // Penalty for head or spine touching the ground
    if (_head.position.y < 0.3f || _spine.position.y < 0.3f)
    {
        AddReward(-5.0f); // High penalty for head or spine touching the ground
    }

    // Reward for forward movement
    Vector3 cubeForward = m_OrientationCube.transform.forward;
    Vector3 avgVelocity = GetAvgVelocity();
    float forwardVelReward = Vector3.Dot(cubeForward, avgVelocity);

    if (forwardVelReward > 0.01f)
    {
        AddReward(5.2f * forwardVelReward); // Increased reward for moving in the desired direction
    }

    // Reward for walking more distance
    float distanceTraveled = Vector3.Distance(_hips.position, m_OrientationCube.transform.position);
    AddReward(0.2f * distanceTraveled); // Increased reward for covering more distance effectively

    // Penalize staying in the same position (lack of movement)
    if (avgVelocity.magnitude < 0.01f)
    {
        AddReward(-4.0f); // Huge penalty if the agent stays in the same place
    }

    // Reward for maintaining body alignment (balance)
    float alignmentReward = Quaternion.Dot(Quaternion.LookRotation(cubeForward), Quaternion.LookRotation(_head.forward));
    AddReward(0.02f * alignmentReward); // Reward for aligning the head with the target direction

    // Penalize large deviations in body posture
    foreach (var bodyPart in m_JdController.bodyPartsList)
    {
        float angleToUpright = Quaternion.Angle(bodyPart.rb.rotation, Quaternion.identity);
        AddReward(-0.001f * angleToUpright); // Small penalty for deviating from upright position
    }

    // Reward for placing feet forward
    Vector3 forwardDirection = cubeForward.normalized;

    // Check left foot movement
    float leftFootForwardMovement = Vector3.Dot(forwardDirection, (_leftFoot.position - _hips.position).normalized);
    if (leftFootForwardMovement > 0.1f) // If left foot moves forward significantly
    {
        AddReward(5.0f); // Huge reward for moving the left foot forward
    }

    // Check right foot movement
    float rightFootForwardMovement = Vector3.Dot(forwardDirection, (_rightFoot.position - _hips.position).normalized);
    if (rightFootForwardMovement > 0.1f) // If right foot moves forward significantly
    {
        AddReward(5.0f); // Huge reward for moving the right foot forward
    }
}







}


