using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.Assets.Scenes.walker.BodyPart;
using JointDriveController = Unity.Assets.Scenes.walker.MyJointDriveController;
using System.Collections.Generic;

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
    List<Vector3> initialPos = new List<Vector3>();

    public static event Action onEpisodeBegin;

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
        
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            initialPos.Add(bodyPart.rb.transform.position - m_OrientationCube.transform.position);
        }
        
        m_ResetParams = Academy.Instance.EnvironmentParameters;
    }

    void Start(){
        onEpisodeBegin?.Invoke();
    }

    public override void OnEpisodeBegin()
    {
        //Reset all of the body parts
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        onEpisodeBegin?.Invoke();

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
        // sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        // sensor.AddObservation(avgVel);
        //vel goal relative to cube
        // sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
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

        bpDict[leftUpperLeg].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[rightUpperLeg].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[leftLowerLeg].SetJointTargetRotation(continuousActions[++i], continuousActions[++i],0);
        bpDict[rightLowerLeg].SetJointTargetRotation(continuousActions[++i], continuousActions[++i],0);
        // bpDict[leftFoot].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        // bpDict[rightFoot].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        bpDict[leftUpperArm].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[rightUpperArm].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[leftLowerArm].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[rightLowerArm].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        // bpDict[head].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        //update joint strength settings
        // bpDict[chest].SetJointStrength(continuousActions[++i]);
        bpDict[spine].SetJointStrength(continuousActions[++i]);
        // bpDict[head].SetJointStrength(continuousActions[++i]);
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

    void getReward(){
        float postureReward = 0f;
        float maxDistanceReward = -1f;  // Maximum distance penalty
        float maxAngleReward = -1f;     // Maximum orientation penalty
        // List<BodyPart> teacherBodyPartsList = m_TeacherJdController.bodyPartsList;
        // Debug.Log(teacherBodyPartsList.Count,m_JdController.rb.transform);
        // for (int i = 0; i < m_JdController.bodyPartsList.Count; i++)
        // {   
        //     Debug.Log(m_JdController.bodyPartsList[i].rb.transform.position+" and "+teacherBodyPartsList[i].rb.transform.position);
        // }
        // foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        // {
        //     Transform agentPart = bodyPart.rb.transform;
        //     Transform teacherPart = GetCorrespondingTeacherBodyPart(agentPart);

        //     // Position alignment: Minimize distance between agent and teacher body parts
        //     float distance = Vector3.Distance(agentPart.position, teacherPart.position);
        //     float distanceReward = Mathf.Exp(-distance);  // Higher reward for smaller distances

        //     // Orientation alignment: Minimize angular difference between agent and teacher orientations
        //     Quaternion agentRotation = agentPart.rotation;
        //     Quaternion teacherRotation = teacherPart.rotation;
        //     float angle = Quaternion.Angle(agentRotation, teacherRotation);
        //     float angleReward = Mathf.Exp(-angle * Mathf.Deg2Rad);  // Higher reward for smaller angles

        //     // Combine both rewards
        //     postureReward += (distanceReward + angleReward) * 0.5f;

        //     // Penalty for large deviations
        //     maxDistanceReward = Mathf.Max(maxDistanceReward, -distance);
        //     maxAngleReward = Mathf.Max(maxAngleReward, -angle);
        // }

        // // Normalize the reward
        // postureReward /= m_JdController.bodyPartsDict.Values.Count;

        // // Add posture reward, and penalize large deviations
        // AddReward(postureReward + maxDistanceReward + maxAngleReward);
    }

    // private void Start(){
    //     m_TeacherJdController = teacher.GetComponent<JointDriveController>();
    // }

    void FixedUpdate()
    {
        UpdateOrientationObjects();
        var rewardComponent = GetComponent<ImitationReward>();
        if(rewardComponent!=null){
            float reward = rewardComponent.CalculateReward();
            // Debug.Log(reward);
            AddReward(reward);
        }
        // getReward();
        // float error=0;
        // foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        // {
        //     Vector3 temp=bodyPart.rb.transform.position - m_OrientationCube.transform.position;
        //     error+=temp.sqrMagnitude;
        // }
        // error*=(-0.1f);
        // AddReward(error);
        // Debug.Log(hips.position.y+" "+head.position.y);
        // var hipReward = hips.position.y+0.5f;
        // var headReward = head.position.y+0.4f;
        // AddReward(hipReward+headReward);
        // if(_hips.position.y>.7f){
        //     AddReward(0.01f);
        // }
        // if(head.position.y>1.2f){
        //     AddReward(0.04f);
        // }

        var cubeForward = m_OrientationCube.transform.forward;

        // Set reward for this step according to mixture of the following elements.
        // a. Match target speed
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        // var matchSpeedReward = GetMatchingVelocityReward(cubeForward * MTargetWalkingSpeed, GetAvgVelocity());

        //Check for NaNs
        // if (float.IsNaN(matchSpeedReward))
        // {
        //     throw new ArgumentException(
        //         "NaN in moveTowardsTargetReward.\n" +
        //         $" cubeForward: {cubeForward}\n" +
        //         $" hips.velocity: {m_JdController.bodyPartsDict[hips].rb.velocity}\n" +
        //         $" maximumWalkingSpeed: {m_maxWalkingSpeed}"
        //     );
        // }

        // b. Rotation alignment with target direction.
        //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        // var headForward = head.forward;
        // headForward.y = 0;
        // var lookAtTargetReward = (Vector3.Dot(cubeForward, head.forward) + 1) * .5F;
        // var lookAtTargetReward = (Vector3.Dot(cubeForward, headForward) + 1) * .5F;

        //Check for NaNs
        // if (float.IsNaN(lookAtTargetReward))
        // {
        //     throw new ArgumentException(
        //         "NaN in lookAtTargetReward.\n" +
        //         $" cubeForward: {cubeForward}\n" +
        //         $" head.forward: {head.forward}"
        //     );
        // }

        // AddReward(matchSpeedReward * lookAtTargetReward);
        
    }
}

