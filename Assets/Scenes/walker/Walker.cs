using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
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
        m_JdController.SetupBodyPart(hips);
        m_JdController.SetupBodyPart(leftUpperLeg);
        m_JdController.SetupBodyPart(spine);
        m_JdController.SetupBodyPart(head);
        m_JdController.SetupBodyPart(leftLowerLeg);
        m_JdController.SetupBodyPart(leftFoot);
        m_JdController.SetupBodyPart(rightUpperLeg);
        m_JdController.SetupBodyPart(rightLowerLeg);
        m_JdController.SetupBodyPart(rightFoot);
        m_JdController.SetupBodyPart(leftUpperArm);
        m_JdController.SetupBodyPart(leftLowerArm);
        m_JdController.SetupBodyPart(rightUpperArm);
        m_JdController.SetupBodyPart(rightLowerArm);

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

    public 
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}

