using System.Collections;
using System.Collections.Generic;
using MLAgents;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;
using static BodyHelper;

public class RagdollTerrainWalkerAgent : Agent, IOnTerrainCollision
{
    BodyManager _bodyManager;
    TerrainGenerator _terrainGenerator;
    SpawnableEnv _spawnableEnv;
    int _stepCountAtLastMeter;
    public int lastXPosInMeters;
    public int maxXPosInMeters;
	float _pain;

	List<float> distances;
	float fraction;
	bool _isDone;
	bool _isTraining=false;
	bool _hasLazyInitialized;

    public static BodyConfig BodyConfig = new BodyConfig
	{
		GetBodyPartGroup = (name) =>
		{
			name = name.ToLower();
			if (name.Contains("mixamorig"))
				return BodyPartGroup.None;

			if (name.Contains("butt"))
				return BodyPartGroup.Hips;
			if (name.Contains("torso"))
				return BodyPartGroup.Torso;
			if (name.Contains("head"))
				return BodyPartGroup.Head;
			if (name.Contains("waist"))
				return BodyPartGroup.Spine;

			if (name.Contains("thigh"))
				return BodyPartGroup.LegUpper;
			if (name.Contains("shin"))
				return BodyPartGroup.LegLower;
			if (name.Contains("right_right_foot") || name.Contains("left_left_foot"))
				return BodyPartGroup.Foot;
			if (name.Contains("upper_arm"))
				return BodyPartGroup.ArmUpper;
			if (name.Contains("larm"))
				return BodyPartGroup.ArmLower;
			if (name.Contains("hand"))
				return BodyPartGroup.Hand;

			return BodyPartGroup.None;
		},
		GetMuscleGroup = (name) =>
		{
			name = name.ToLower();
			if (name.Contains("mixamorig"))
				return MuscleGroup.None;
			if (name.Contains("butt"))
				return MuscleGroup.Hips;
			if (name.Contains("lower_waist")
				|| name.Contains("abdomen_y"))
				return MuscleGroup.Spine;
			if (name.Contains("thigh")
				|| name.Contains("hip"))
				return MuscleGroup.LegUpper;
			if (name.Contains("shin"))
				return MuscleGroup.LegLower;
			if (name.Contains("right_right_foot")
				|| name.Contains("left_left_foot")
				|| name.Contains("ankle_x"))
				return MuscleGroup.Foot;
			if (name.Contains("upper_arm"))
				return MuscleGroup.ArmUpper;
			if (name.Contains("larm"))
				return MuscleGroup.ArmLower;
			if (name.Contains("hand"))
				return MuscleGroup.Hand;

			return MuscleGroup.None;
		},
        GetRootBodyPart = () => BodyPartGroup.Hips,
        GetRootMuscle = () => MuscleGroup.Hips
    };

    override public void CollectObservations(VectorSensor sensor)
	{
		// var sensor = this;
		if (!_hasLazyInitialized)
		{
			AgentReset();
		}

		Vector3 normalizedVelocity = _bodyManager.GetNormalizedVelocity();
        var pelvis = _bodyManager.GetFirstBodyPart(BodyPartGroup.Hips);
        var shoulders = _bodyManager.GetFirstBodyPart(BodyPartGroup.Torso);

        sensor.AddObservation(normalizedVelocity); 
        sensor.AddObservation(pelvis.Rigidbody.transform.forward); // gyroscope 
        sensor.AddObservation(pelvis.Rigidbody.transform.up);

        sensor.AddObservation(shoulders.Rigidbody.transform.forward); // gyroscope 
        sensor.AddObservation(shoulders.Rigidbody.transform.up);

		sensor.AddObservation(_bodyManager.GetSensorIsInTouch());
		foreach (var bodyPart in _bodyManager.BodyParts)
		{
			bodyPart.UpdateObservations();
			sensor.AddObservation(bodyPart.ObsLocalPosition);
			sensor.AddObservation(bodyPart.ObsRotation);
			sensor.AddObservation(bodyPart.ObsRotationVelocity);
			sensor.AddObservation(bodyPart.ObsVelocity);
		}
		sensor.AddObservation(_bodyManager.GetSensorObservations());

        (distances, fraction) = 
            _terrainGenerator.GetDistances2d(
                pelvis.Rigidbody.transform.position, true);
        sensor.AddObservation(distances);
        sensor.AddObservation(fraction);
		Debug.Log(fraction);
		// _bodyManager.OnCollectObservationsHandleDebug(GetInfo());
	}

    public override void OnActionReceived(ActionBuffers actionBuffers)
	{
		if (!_hasLazyInitialized)
		{
			return;
		}
		_isDone = false;
		// apply actions to body
        var vectorAction = actionBuffers.ContinuousActions.Array;
        // Debug.Log(vectorAction.Length);
		_bodyManager.OnAgentAction(vectorAction);

		// manage reward
        // float velocity = Mathf.Clamp(_bodyManager.GetNormalizedVelocity().x, 0f, 1f);
		// var actionDifference = _bodyManager.GetActionDifference();
		// var actionsAbsolute = vectorAction.Select(x=>Mathf.Abs(x)).ToList();
		// var actionsAtLimit = actionsAbsolute.Select(x=> x>=1f ? 1f : 0f).ToList();
		// float actionaAtLimitCount = actionsAtLimit.Sum();
        // float notAtLimitBonus = 1f - (actionaAtLimitCount / (float) actionsAbsolute.Count);
        // float reducedPowerBonus = 1f - actionsAbsolute.Average();

		//J-added

		// velocity *= 0.85f;
		// reducedPowerBonus *=0f;
		// notAtLimitBonus *=.1f;
		// actionDifference *=.05f;
        // var reward = velocity
		// 				+ notAtLimitBonus
		// 				+ reducedPowerBonus
		// 				+ actionDifference;		
        // var pelvis = _bodyManager.GetFirstBodyPart(BodyPartGroup.Hips);
		// if (pelvis.Transform.position.y<0){
        //     //J-modify
		// 	EndEpisode();
        //     //Done();
		// }

        // var reward = velocity;

		// AddReward(reward);
		// _bodyManager.SetDebugFrameReward(reward);
	}

    // public override void AgentReset()
	// {
	// 	if (!_hasLazyInitialized)
	// 	{
	// 		_bodyManager = GetComponent<BodyManager>();
	// 		_bodyManager.BodyConfig = MarathonManAgent.BodyConfig;
	// 		_bodyManager.OnInitializeAgent();
	// 		_hasLazyInitialized = true;
	// 	}
	// 	_isDone = true;
	// 	_bodyManager.OnAgentReset();
	// }

    public void AgentReset(){
        if (!_hasLazyInitialized)
		{
			_bodyManager = GetComponent<BodyManager>();
			_bodyManager.BodyConfig = RagdollStraightWalkerAgent.BodyConfig;
			_bodyManager.OnInitializeAgent();
			_hasLazyInitialized = true;
		}
		_isDone = true;
		_isTraining = true;
		_bodyManager.OnAgentReset();
        if (_terrainGenerator == null)
            _terrainGenerator = GetComponent<TerrainGenerator>();
		if (_spawnableEnv == null)
			_spawnableEnv = GetComponentInParent<SpawnableEnv>();
        _terrainGenerator.Reset();
		lastXPosInMeters = (int)
            _bodyManager.GetBodyParts(BodyPartGroup.Foot)
            .Average(x=>x.Transform.position.x);
        _pain = 0f;
    }

    public override void OnEpisodeBegin(){
		_isTraining=false;
        AgentReset();
    }


    void Start()
    {
        
    }

    void Update()
    {
		if(Input.GetKeyDown(KeyCode.Escape)){
            SceneLoader.Load(SceneLoader.Scene.MainMenuScene);
        }
        
    }

	private void FixedUpdate() {
		if(!_isTraining) return;
		var pelvis = _bodyManager.GetFirstBodyPart(BodyPartGroup.Hips);
		// if (pelvis.Transform.position.y<0){
        //     //J-modify
		// 	AddReward(-1f);
		// 	EndEpisode();
		// 	return;
        //     //Done();
		// }
		float velocity = Mathf.Clamp(_bodyManager.GetNormalizedVelocity().x, 0f, 1f);
		float reward = velocity;
		AddReward(reward);

        float xpos = 
            _bodyManager.GetBodyParts(BodyPartGroup.Foot)
            .Average(x=>x.Transform.position.x);
		int newXPosInMeters = (int) xpos;
        if (newXPosInMeters > lastXPosInMeters) {
            lastXPosInMeters = newXPosInMeters;
            _stepCountAtLastMeter = this.StepCount;
        }
		if (newXPosInMeters > maxXPosInMeters)
			maxXPosInMeters = newXPosInMeters;
		var terminate = false;
		// bool isInBounds = _spawnableEnv.IsPointWithinBoundsInWorldSpace(pelvis.Transform.position);
		// if (!isInBounds)
        // if (pelvis.Rigidbody.transform.position.y < 0f)
		if (_terrainGenerator.IsPointOffEdge(pelvis.Transform.position)){
            terminate = true;
            AddReward(-1f);
		}
        if (this.StepCount -_stepCountAtLastMeter >= (200*5))
            terminate = true;
		else if (xpos < 4f && _pain > 1f)
            terminate = true;
        else if (xpos < 2f && _pain > 0f)
            terminate = true;
		else if (_pain > 2f)
            terminate = true;
        if (terminate){
			EndEpisode();
		}
        _pain = 0f;
	}

    public virtual void OnTerrainCollision(GameObject other, GameObject terrain)
	{
		// if (string.Compare(terrain.name, "Terrain", true) != 0)
		if (terrain.GetComponent<Terrain>() == null)
			return;
		// if (!_styleAnimator.AnimationStepsReady)
		// 	return;
        // HACK - for when agent has not been initialized
		if (_bodyManager == null)
			return;
		var bodyPart = _bodyManager.BodyParts.FirstOrDefault(x=>x.Transform.gameObject == other);
		if (bodyPart == null)
			return;
		switch (bodyPart.Group)
		{
			case BodyHelper.BodyPartGroup.None:
			case BodyHelper.BodyPartGroup.Foot:
			case BodyHelper.BodyPartGroup.LegLower:
                break;
            case BodyHelper.BodyPartGroup.LegUpper:
			case BodyHelper.BodyPartGroup.Hand:
			case BodyHelper.BodyPartGroup.ArmLower:
			case BodyHelper.BodyPartGroup.ArmUpper:
				_pain+=.1f;
                break;
			default:
				// Debug.Log("Dead by touch:"+bodyPart.Group);
                _pain+=5f;
				// AddReward(-1f);
				// if (!_isDone){
				// 	_isDone=true;
				// 	EndEpisode();
				// }
				break;
		}
	}
}
