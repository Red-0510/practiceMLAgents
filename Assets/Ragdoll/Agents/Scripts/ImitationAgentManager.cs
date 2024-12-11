// The Master script is attached to the Agent object. Calculates the distance metrics
// used by the Agent to calculate rewards: distances between the rotations of agent's
// body parts and Animator's body parts, the distance between the angular momentum, and
// the center of masses. 

using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using MLAgents;
using System;
using Unity.Assets.Ragdoll.Agents.Scripts;
using Unity.MLAgents;


public class ImitationAgentManager : MonoBehaviour {

	public float FixedDeltaTime = 0.005f;
	public bool visualizeAnimator = true;

	// general observations
	public List<Muscle> Muscles;
	public List<BodyPart> BodyParts;
	public float ObsPhase;
	public Vector3 ObsCenterOfMass;
	public Vector3 ObsAngularMoment;
	public Vector3 ObsVelocity;

	// model observations
	// i.e. model = difference between mocap and actual)
	// ideally we dont want to generate model at inference
	public float EndEffectorDistance; // feet, hands, head
	public float EndEffectorVelocityDistance; // feet, hands, head
	public float JointAngularVelocityDistance;
	public float JointAngularVelocityDistanceWorld;
	public float RotationDistance;
	public float CenterOfMassVelocityDistance;
	public float CenterOfMassDistance;
	public float AngularMomentDistance;
	public float SensorDistance;

	public float MaxEndEffectorDistance; // feet, hands, head
	public float MaxEndEffectorVelocityDistance; // feet, hands, head
	public float MaxJointAngularVelocityDistance;
	public float MaxJointAngularVelocityDistanceWorld;
	public float MaxRotationDistance;
	public float MaxCenterOfMassVelocityDistance;
	public float MaxCenterOfMassDistance;
	public float MaxAngularMomentDistance;
	public float MaxSensorDistance;

	// debug variables
	public bool IgnorRewardUntilObservation;
	public float ErrorCutoff;
	public bool DebugShowWithOffset;
	public bool DebugMode;
	public bool DebugDisableMotor;
    [Range(-100,100)]
	public int DebugAnimOffset;

	public float TimeStep;
	public int AnimationIndex;
	public int EpisodeAnimationIndex;
	public int StartAnimationIndex;
	public bool UseRandomIndexForTraining;
	public bool UseRandomIndexForInference;
	public bool CameraFollowMe;
	public Transform CameraTarget;

	private bool _isDone;
	bool _resetCenterOfMassOnLastUpdate;
	bool _fakeVelocity;
	bool _waitingForAnimation;

	private ExpertAnimator _muscleAnimator;
	private ImitationRunningAgent _agent;
	ExpertAnimator _styleAnimator;
	ExpertAnimator _localStyleAnimator;
	DecisionRequester _decisionRequester;

	public bool IsInferenceMode;
	bool _phaseIsRunning;
    // UnityEngine.Random _random = new UnityEngine.Random();
	Vector3 _lastCenterOfMass;

	public BodyConfig BodyConfig;

	// Use this for initialization
	void Awake () {
		foreach (var rb in GetComponentsInChildren<Rigidbody>())
		{
			if (rb.useGravity == false)
				rb.solverVelocityIterations = 255;
		}
		var masters = FindObjectsOfType<ImitationAgentManager>().ToList();
		// if (masters.Count(x=>x.CameraFollowMe) < 1)
		// 	CameraFollowMe = true;
	}

    // Initialize the Agent. Sets up Body Parts, Muscles. 
	public void OnInitializeAgent()
    {
		Time.fixedDeltaTime = FixedDeltaTime;
		_waitingForAnimation = true;
		_decisionRequester = GetComponent<DecisionRequester>();

		BodyParts = new List<BodyPart> ();
		BodyPart root = null;
		foreach (var t in GetComponentsInChildren<Transform>())
		{
			if (BodyConfig.GetBodyPartGroup(t.name) == BodyHelper.BodyPartGroup.None)
				continue;
			
			var bodyPart = new BodyPart{
				Rigidbody = t.GetComponent<Rigidbody>(),
				Transform = t,
				Name = t.name,
				Group = BodyConfig.GetBodyPartGroup(t.name), 
			};
			if (bodyPart.Group == BodyConfig.GetRootBodyPart())
				root = bodyPart;
			bodyPart.Root = root;
			bodyPart.Init();
			BodyParts.Add(bodyPart);
		}
		var partCount = BodyParts.Count;

		Muscles = new List<Muscle> ();
		var muscles = GetComponentsInChildren<ConfigurableJoint>();
		ConfigurableJoint rootConfigurableJoint = null;
		var ragDoll = GetComponent<RagdollWalker>();
		foreach (var m in muscles)
		{
			var maximumForce = ragDoll.MusclePowers.First(x=>x.Muscle == m.name).PowerVector;
			var muscle = new Muscle{
				Rigidbody = m.GetComponent<Rigidbody>(),
				Transform = m.GetComponent<Transform>(),
				ConfigurableJoint = m,
				Name = m.name,
				Group = BodyConfig.GetMuscleGroup(m.name),
				MaximumForce = maximumForce
			};
			if (muscle.Group == BodyConfig.GetRootMuscle())
				rootConfigurableJoint = muscle.ConfigurableJoint;
			muscle.RootConfigurableJoint = rootConfigurableJoint;
			muscle.Init();

			Muscles.Add(muscle);			
		}
		var spawnableEnv = GetComponentInParent<SpawnableEnv>();
		_localStyleAnimator = spawnableEnv.gameObject.GetComponentInChildren<ExpertAnimator>();
		_styleAnimator = _localStyleAnimator.GetFirstOfThisAnim();
		_muscleAnimator = _styleAnimator;
		_agent = GetComponent<ImitationRunningAgent>();

		IsInferenceMode = !Academy.Instance.IsCommunicatorOn;
	}
	
	// Update is called once per frame
	void Update () {
	}

	public void OnAgentAction()
	{
		if (_waitingForAnimation && _styleAnimator.AnimationStepsReady){
			_waitingForAnimation = false;
			ResetPhase();
		}
		var animStep = UpdateObservations();
		Step(animStep);
	}

	// Calculate values like ObsCenterOfMass, ObsAngularMoment that will be used
    // as observations fed into the neural network for training and inference. Also
    // calculates the distance metrics between animation's and agents' body parts.
    // The distances are used by the Agent to calculate rewards. The max distances
    // can be used to manually tune the denominators in exp(-distance/denominator)
    // when forming reward function. 
	ExpertAnimator.AnimationStep UpdateObservations()
	{
		if (DebugMode)
			AnimationIndex = 0;
		var debugStepIdx = AnimationIndex;
		ExpertAnimator.AnimationStep animStep = null;
		ExpertAnimator.AnimationStep debugAnimStep = null;
		if (_phaseIsRunning) {
				debugStepIdx += DebugAnimOffset;
			if (DebugShowWithOffset){
				debugStepIdx = Mathf.Clamp(debugStepIdx, 0, _muscleAnimator.AnimationSteps.Count);
				debugAnimStep = _muscleAnimator.AnimationSteps[debugStepIdx];
			}
			animStep = _muscleAnimator.AnimationSteps[AnimationIndex];
		}
		EndEffectorDistance = 0f;
		EndEffectorVelocityDistance = 0;
		JointAngularVelocityDistance = 0;
		JointAngularVelocityDistanceWorld = 0;
		RotationDistance = 0f;
		CenterOfMassVelocityDistance = 0f;
		CenterOfMassDistance = 0f;
		AngularMomentDistance = 0f;
		SensorDistance = 0f;
		if (_phaseIsRunning && DebugShowWithOffset)
			MimicAnimationFrame(debugAnimStep);
		else if (_phaseIsRunning)
			CompareAnimationFrame(animStep);
		foreach (var muscle in Muscles)
		{
			var i = Muscles.IndexOf(muscle);
			muscle.UpdateObservations();
			if (!DebugShowWithOffset && !DebugDisableMotor)
				muscle.UpdateMotor();
			if (!muscle.Rigidbody.useGravity)
				continue; // skip sub joints
		}
		foreach (var bodyPart in BodyParts)
		{
			if (_phaseIsRunning){
				bodyPart.UpdateObservations();

				var rotDistance = bodyPart.ObsAngleDeltaFromAnimationRotation;
				var squareRotDistance = Mathf.Pow(rotDistance,2);
				RotationDistance += squareRotDistance;

				JointAngularVelocityDistance += bodyPart.ObsDeltaFromAnimationAngularVelocity.sqrMagnitude;
				JointAngularVelocityDistanceWorld += bodyPart.ObsDeltaFromAnimationAngularVelocityWorld.sqrMagnitude;

				if (bodyPart.Group == BodyHelper.BodyPartGroup.Hand
					|| bodyPart.Group == BodyHelper.BodyPartGroup.Torso
					|| bodyPart.Group == BodyHelper.BodyPartGroup.Foot)
				{
					EndEffectorDistance += bodyPart.ObsDeltaFromAnimationPosition.sqrMagnitude;
					EndEffectorVelocityDistance += bodyPart.ObsDeltaFromAnimationVelocity.sqrMagnitude;
				}
			}
		}

		ObsCenterOfMass = JointHelper.GetCenterOfMassRelativeToRoot(BodyParts);		
		ObsAngularMoment = JointHelper.GetAngularMoment(BodyParts);

		if (_phaseIsRunning) {
			CenterOfMassDistance = (animStep.CenterOfMass - ObsCenterOfMass).sqrMagnitude;
			AngularMomentDistance = (animStep.AngularMoment - ObsAngularMoment).sqrMagnitude;
		}

		ObsVelocity = ObsCenterOfMass - _lastCenterOfMass;
		if (_fakeVelocity)
			ObsVelocity = animStep.CenterOfMassVelocity;
		_lastCenterOfMass = ObsCenterOfMass;

		if (!_resetCenterOfMassOnLastUpdate)
			_fakeVelocity = false;

		if (_phaseIsRunning){
			var animVelocity = animStep.CenterOfMassVelocity / (Time.fixedDeltaTime * _decisionRequester.DecisionPeriod);
			ObsVelocity /= (Time.fixedDeltaTime * _decisionRequester.DecisionPeriod);

			CenterOfMassVelocityDistance = (ObsVelocity - animVelocity).sqrMagnitude;

            SensorDistance = 0.0f;
			var sensorDistanceStep = 1.0f / _agent.SensorIsInTouch.Count;
			for (int i = 0; i < _agent.SensorIsInTouch.Count; i++)
			{
				if (animStep.SensorIsInTouch[i] != _agent.SensorIsInTouch[i]) {
					SensorDistance += sensorDistanceStep;
				}
			}
		}

		if (!IgnorRewardUntilObservation){
			MaxEndEffectorDistance = Mathf.Max(MaxEndEffectorDistance, EndEffectorDistance);
			MaxEndEffectorVelocityDistance = Mathf.Max(MaxEndEffectorVelocityDistance, EndEffectorVelocityDistance);
			MaxRotationDistance = Mathf.Max(MaxRotationDistance, RotationDistance);
			MaxCenterOfMassVelocityDistance = Mathf.Max(MaxCenterOfMassVelocityDistance, CenterOfMassVelocityDistance);
			MaxEndEffectorVelocityDistance = Mathf.Max(MaxEndEffectorVelocityDistance, EndEffectorVelocityDistance);
			MaxJointAngularVelocityDistance = Mathf.Max(MaxJointAngularVelocityDistance, JointAngularVelocityDistance);
			MaxJointAngularVelocityDistanceWorld = Mathf.Max(MaxJointAngularVelocityDistanceWorld, JointAngularVelocityDistanceWorld);
			MaxCenterOfMassDistance = Mathf.Max(MaxCenterOfMassDistance, CenterOfMassDistance);
			MaxAngularMomentDistance = Mathf.Max(MaxAngularMomentDistance, AngularMomentDistance);
			MaxSensorDistance = Mathf.Max(MaxSensorDistance, SensorDistance);
		}

		if (IgnorRewardUntilObservation)
			IgnorRewardUntilObservation = false;
		ObsPhase = _muscleAnimator.AnimationSteps[AnimationIndex].NormalizedTime % 1f;
		return animStep;
	}

    // Increment animation index. The index is used to get the current AnimStep structure.
	void Step(ExpertAnimator.AnimationStep animStep)
	{
		if (_phaseIsRunning){
			if (!DebugShowWithOffset)
				AnimationIndex++;
			if (AnimationIndex>=_muscleAnimator.AnimationSteps.Count) {
				Done();
				AnimationIndex--;
			}
		}
		if (_phaseIsRunning && IsInferenceMode)
		{
			// Debug.Log("enabling");
			_muscleAnimator.anim.enabled = true;
			_muscleAnimator.anim.Play("Record",0, animStep.NormalizedTime);
			_muscleAnimator.anim.transform.position = animStep.TransformPosition;
			_muscleAnimator.anim.transform.rotation = animStep.TransformRotation;
		}
	}

	void CompareAnimationFrame(ExpertAnimator.AnimationStep animStep)
	{
		MimicAnimationFrame(animStep, true);
	}

    // Moves Agent's body part to animation's position and rotation if onlySetAnimation = false.
    // Then sets the target animation for each Agent's body part. The animation's
    // positions and rotations are later on used by the Body Part object to calculate
    // distance metrics to the target animation. 
	void MimicAnimationFrame(ExpertAnimator.AnimationStep animStep, bool onlySetAnimation = false)
	{
		if (!onlySetAnimation)
		{
			foreach (var rb in GetComponentsInChildren<Rigidbody>())
			{
				rb.angularVelocity = Vector3.zero;
				rb.velocity = Vector3.zero;
			}
		}

		foreach (var bodyPart in BodyParts)
		{
			var i = animStep.Names.IndexOf(bodyPart.Name);
			Vector3 animPosition = bodyPart.InitialRootPosition + animStep.Positions[0];
            Quaternion animRotation = bodyPart.InitialRootRotation * animStep.Rotations[0];
			if (i != 0) {
				animPosition += animStep.Positions[i];
				animRotation *= animStep.Rotations[i];
			}
			Vector3 angularVelocity = animStep.AngularVelocities[i];
			Vector3 velocity = animStep.Velocities[i];

            Vector3 angularVelocityLocal = animStep.AngularVelocitiesLocal[i];
			Vector3 velocityLocal = animStep.VelocitiesLocal[i];

			bool setAnim = !onlySetAnimation;
			if (bodyPart.Name.Contains("head") || bodyPart.Name.Contains("upper_waist"))
				setAnim = false;
			if (setAnim) {
				bodyPart.MoveToAnim(animPosition, animRotation, angularVelocity, velocity);
			}
                
			bodyPart.SetAnimationPosition(animPosition, animStep.Rotations[i], velocity, angularVelocityLocal, angularVelocity);
		}

	}

	protected virtual void LateUpdate() {
		if (_resetCenterOfMassOnLastUpdate){
			ObsCenterOfMass = JointHelper.GetCenterOfMassRelativeToRoot(BodyParts);
			//ObsCenterOfMass = GetCenterOfMass();
			_lastCenterOfMass = ObsCenterOfMass;
			_resetCenterOfMassOnLastUpdate = false;
		}
		#if UNITY_EDITOR
			VisualizeTargetPose();
		#endif
	}

	public bool IsDone()
	{
		return _isDone;
	}

	void Done()
	{
		_isDone = true;
	}

	public void ResetPhase()
	{
		if (_waitingForAnimation)
			return;
		_decisionRequester.enabled = true;
		_agent.SetTotalAnimFrames(_muscleAnimator.AnimationSteps.Count);
		SetStartIndex(0); // HACK for gym
		UpdateObservations();
	}

	public void SetStartIndex(int startIdx)
	{
		_decisionRequester.enabled = false;

		if (!_phaseIsRunning){
			StartAnimationIndex = _muscleAnimator.AnimationSteps.Count-1;
			EpisodeAnimationIndex = _muscleAnimator.AnimationSteps.Count-1;
			AnimationIndex = EpisodeAnimationIndex;
			// if (CameraFollowMe){
			// 	var camera = FindObjectOfType<Camera>();
			// 	var follow = camera.GetComponent<SmoothFollow>();
			// 	follow.target = CameraTarget;
			// }
		}

		AnimationIndex = startIdx;
		if (_decisionRequester?.DecisionPeriod > 1)
			AnimationIndex *= this._decisionRequester.DecisionPeriod;
		StartAnimationIndex = AnimationIndex;
		EpisodeAnimationIndex = AnimationIndex;
		_phaseIsRunning = true;
		_isDone = false;
		var animStep = _muscleAnimator.AnimationSteps[AnimationIndex];
		TimeStep = animStep.TimeStep;
		EndEffectorDistance = 0f;
		EndEffectorVelocityDistance = 0f;
		JointAngularVelocityDistance = 0;
		JointAngularVelocityDistanceWorld = 0;
		RotationDistance = 0f;
		CenterOfMassVelocityDistance = 0f;
		IgnorRewardUntilObservation = true;
		_resetCenterOfMassOnLastUpdate = true;
		_fakeVelocity = true;
		foreach (var muscle in Muscles)
			muscle.Init();
		foreach (var bodyPart in BodyParts)
			bodyPart.Init();
		MimicAnimationFrame(animStep);
		EpisodeAnimationIndex = AnimationIndex;
	}

	private void VisualizeTargetPose() {
		if (!visualizeAnimator) return;
		if (!Application.isEditor) return;
	}
	
	// Recursively visualizes a bone hierarchy
	private void VisualizeHierarchy(Transform t, Color color) {
		for (int i = 0; i < t.childCount; i++) {
			Debug.DrawLine(t.position, t.GetChild(i).position, color);
			VisualizeHierarchy(t.GetChild(i), color);
		}
	}


}
