using System.Collections;
using System.Collections.Generic;
using MLAgents;
using UnityEngine;
using System.Linq;
using System;
using static BodyHelper;
using Unity.Assets.Ragdoll.Agents.Scripts;
using Unity.MLAgents;

public class BodyManager : MonoBehaviour, IOnSensorCollision {

    public Transform CameraTarget;
    public float FixedDeltaTime = 0.005f;
	public bool ShowMonitor = false;
	public bool DebugDisableMotor;
	public bool DebugShowWithOffset;

    public List<Muscle> Muscles;
    public List<BodyPart> BodyParts;
    public List<float> SensorIsInTouch;
	public List<float> Observations;
	public int ObservationNormalizedErrors;
	public int MaxObservationNormalizedErrors;
	public List<GameObject> Sensors;
	public float FrameReward;
	public float AverageReward;
    Vector3 startPosition;

	Dictionary<GameObject, Vector3> transformsPosition;
	Dictionary<GameObject, Quaternion> transformsRotation;

	Agent _agent;
	SpawnableEnv _spawnableEnv;
	TerrainGenerator _terrainGenerator;
	DecisionRequester _decisionRequester;

	static int _startCount;

    float[] lastVectorAction;
	float[] vectorDifference;
	List <Vector3> mphBuffer;

    [Tooltip("Max distance travelled across all episodes")]
	/**< \brief Max distance travelled across all episodes*/
	public float MaxDistanceTraveled;

	[Tooltip("Distance travelled this episode")]
	/**< \brief Distance travelled this episode*/
	public float DistanceTraveled;

	List<SphereCollider> sensorColliders;
	static int _spawnCount;
    public BodyConfig BodyConfig;



    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void OnInitializeAgent()
    {
		_spawnableEnv = GetComponentInParent<SpawnableEnv>();
		_terrainGenerator = GetComponentInParent<TerrainGenerator>();
		SetupBody();
		DistanceTraveled = float.MinValue;
		_spawnableEnv.UpdateBounds();
	}

    public void OnAgentReset()
	{
        //J-commented
		// if (DistanceTraveled != float.MinValue)
		// {
		// 	var scorer = FindObjectOfType<Scorer>();
		// 	scorer?.ReportScore(DistanceTraveled, "Distance Traveled");
		// }
		HandleModelReset();
		Sensors = _agent.GetComponentsInChildren<SensorBehavior>()
			.Select(x=>x.gameObject)
			.ToList();
		sensorColliders = Sensors
			.Select(x=>x.GetComponent<SphereCollider>())
			.ToList();
		SensorIsInTouch = Enumerable.Range(0,Sensors.Count).Select(x=>0f).ToList();
		// HACK first spawned agent should grab the camera
		var smoothFollow = GameObject.FindObjectOfType<SmoothFollow>();
		if (smoothFollow != null && smoothFollow.target == null) {
			if (_spawnCount == 0) // HACK follow nth agent
			{
				smoothFollow.target = CameraTarget;
				ShowMonitor = true;   
			}
			else
				_spawnCount++;             
		}
		lastVectorAction = null;
		vectorDifference = null;		
		mphBuffer = new List<Vector3>();
	}

public void OnAgentAction(float[] vectorAction)
	{
		if (lastVectorAction == null){
			lastVectorAction = vectorAction.Select(x=>0f).ToArray();
			vectorDifference = vectorAction.Select(x=>0f).ToArray();
		}
		int i = 0;
		foreach (var muscle in Muscles)
		{
			// if(muscle.Parent == null)
			// 	continue;
			if (muscle.ConfigurableJoint.angularXMotion != ConfigurableJointMotion.Locked){
				vectorDifference[i] = Mathf.Abs(vectorAction[i]-lastVectorAction[i]);
				muscle.TargetNormalizedRotationX = vectorAction[i++];
			}
			if (muscle.ConfigurableJoint.angularYMotion != ConfigurableJointMotion.Locked){
				vectorDifference[i] = Mathf.Abs(vectorAction[i]-lastVectorAction[i]);
				muscle.TargetNormalizedRotationY = vectorAction[i++];
			}
			if (muscle.ConfigurableJoint.angularZMotion != ConfigurableJointMotion.Locked){
				vectorDifference[i] = Mathf.Abs(vectorAction[i]-lastVectorAction[i]);
				muscle.TargetNormalizedRotationZ = vectorAction[i++];
			}
			if (!DebugDisableMotor)
				muscle.UpdateMotor();
		}

        if (ShowMonitor)
        {
            // var hist = new[] {velocity, uprightBonus, heightPenality, effort}.ToList();
            // Monitor.Log("rewardHist", hist.ToArray(), displayType: Monitor.DisplayType.Independent);
        }
	}

    public BodyPart GetFirstBodyPart(BodyPartGroup bodyPartGroup)
    {
        var bodyPart = BodyParts.FirstOrDefault(x=>x.Group == bodyPartGroup);
        return bodyPart;
    }

    public List<BodyPart> GetBodyParts()
    {
        return BodyParts;
    }

    public List<BodyPart> GetBodyParts(BodyPartGroup bodyPartGroup)
    {
        return BodyParts.Where(x=>x.Group == bodyPartGroup).ToList();
    }

    public float GetActionDifference()
    {
		float actionDifference = 1f - vectorDifference.Average();
		actionDifference = Mathf.Clamp(actionDifference, 0, 1);
		actionDifference = Mathf.Pow(actionDifference,2);
        return actionDifference;
    }

    void SetupBody()
    {
        _agent = GetComponent<Agent>();
		_decisionRequester = GetComponent<DecisionRequester>();
		Time.fixedDeltaTime = FixedDeltaTime;

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
			maximumForce *= ragDoll.MotorScale;
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
		_startCount++;        
    }

    void HandleModelReset()
	{
		Transform[] allChildren = _agent.GetComponentsInChildren<Transform>();
		if (transformsPosition != null)
		{
			foreach (var child in allChildren)
			{
				child.position = transformsPosition[child.gameObject];
				child.rotation = transformsRotation[child.gameObject];
				var childRb = child.GetComponent<Rigidbody>();
				if (childRb != null)
				{
					childRb.angularVelocity = Vector3.zero;
					childRb.velocity = Vector3.zero;
				}
			}

		}
		else
		{
			startPosition = _agent.transform.position;
			transformsPosition = new Dictionary<GameObject, Vector3>();
			transformsRotation = new Dictionary<GameObject, Quaternion>();
			foreach (Transform child in allChildren)
			{
				transformsPosition[child.gameObject] = child.position;
				transformsRotation[child.gameObject] = child.rotation;
			}
		}
	}

    public float GetHeightNormalizedReward(float maxHeight)
	{
		var height = GetHeight();
		var heightPenalty = maxHeight - height;
		heightPenalty = Mathf.Clamp(heightPenalty, 0f, maxHeight);
		var reward = 1f - heightPenalty;
		reward = Mathf.Clamp(reward, 0f, 1f);
		return reward;
	}
	internal float GetHeight()
	{
		var feetYpos = BodyParts
			.Where(x => x.Group == BodyPartGroup.Foot)
			.Select(x => x.Transform.position.y)
			.OrderBy(x => x)
			.ToList();
		float lowestFoot = 0f;
		if (feetYpos != null && feetYpos.Count != 0)
			lowestFoot = feetYpos[0];
		var height = GetFirstBodyPart(BodyPartGroup.Head).Transform.position.y - lowestFoot;
		return height;
	}

    public float GetDirectionNormalizedReward(BodyPartGroup bodyPartGroup, Vector3 direction)
	{
		BodyPart bodyPart = GetFirstBodyPart(bodyPartGroup);
		float maxBonus = 1f;
		var toFocalAngle = bodyPart.ToFocalRoation * bodyPart.Transform.right;
		var angle = Vector3.Angle(toFocalAngle, direction);
		var qpos2 = (angle % 180) / 180;
		var bonus = maxBonus * (2 - (Mathf.Abs(qpos2) * 2) - 1);
		return bonus;
	}

    public float GetUprightNormalizedReward(BodyPartGroup bodyPartGroup)
	{
		BodyPart bodyPart = GetFirstBodyPart(bodyPartGroup);
		float maxBonus = 1f;
		var toFocalAngle = bodyPart.ToFocalRoation * -bodyPart.Transform.forward;
		var angleFromUp = Vector3.Angle(toFocalAngle, Vector3.up);
		var qpos2 = (angleFromUp % 180) / 180;
		var uprightBonus = maxBonus * (2 - (Mathf.Abs(qpos2) * 2) - 1);
		return uprightBonus;
	}

    public float GetEffortNormalized(string[] ignorJoints = null)
	{
		double effort = 0;
		double jointEffort = 0;
		double joints = 0;
		foreach (var muscle in Muscles)
		{
			if(muscle.Parent == null)
				continue;
			var name = muscle.Name;
			if (ignorJoints != null && ignorJoints.Contains(name))
				continue;
			if (muscle.ConfigurableJoint.angularXMotion != ConfigurableJointMotion.Locked) {
				jointEffort = Mathf.Pow(Mathf.Abs(muscle.TargetNormalizedRotationX),2);
				effort += jointEffort;
				joints++;
			}
			if (muscle.ConfigurableJoint.angularYMotion != ConfigurableJointMotion.Locked) {
				jointEffort = Mathf.Pow(Mathf.Abs(muscle.TargetNormalizedRotationY),2);
				effort += jointEffort;
				joints++;
			}
			if (muscle.ConfigurableJoint.angularZMotion != ConfigurableJointMotion.Locked) {
				jointEffort = Mathf.Pow(Mathf.Abs(muscle.TargetNormalizedRotationZ),2);
				effort += jointEffort;
				joints++;
			}
		}

		return (float) (effort / joints);
	}

    public void OnSensorCollisionEnter(Collider sensorCollider, GameObject other) {
		// if (string.Compare(other.name, "Terrain", true) !=0)
		if (other.GetComponent<Terrain>() == null)
			return;
		var sensor = Sensors
			.FirstOrDefault(x=>x == sensorCollider.gameObject);
		if (sensor != null) {
			var idx = Sensors.IndexOf(sensor);
			SensorIsInTouch[idx] = 1f;
		}
	}
	public void OnSensorCollisionExit(Collider sensorCollider, GameObject other)
	{
		// if (string.Compare(other.gameObject.name, "Terrain", true) !=0)
		if (other.GetComponent<Terrain>() == null)
			return;
		var sensor = Sensors
			.FirstOrDefault(x=>x == sensorCollider.gameObject);
		if (sensor != null) {
			var idx = Sensors.IndexOf(sensor);
			SensorIsInTouch[idx] = 0f;
		}
	}

    public Vector3 GetLocalCenterOfMass()
    {
        var centerOfMass = GetCenterOfMass();
		centerOfMass -= transform.position;
        return centerOfMass;
    }
	public Vector3 GetCenterOfMass()
	{
		var centerOfMass = Vector3.zero;
		float totalMass = 0f;
		var bodies = BodyParts
			.Select(x=>x.Rigidbody)
			.Where(x=>x!=null)
			.ToList();
		foreach (Rigidbody rb in bodies)
		{
			centerOfMass += rb.worldCenterOfMass * rb.mass;
			totalMass += rb.mass;
		}
		centerOfMass /= totalMass;
		return centerOfMass;
	}

    public Vector3 GetNormalizedVelocity()
    {
        var pelvis = GetFirstBodyPart(BodyConfig.GetRootBodyPart());
        Vector3 metersPerSecond = pelvis.Rigidbody.velocity;
        var n = GetNormalizedVelocity(metersPerSecond);
        return n;
    }
    public Vector3 GetNormalizedVelocity(Vector3 metersPerSecond)
	{
		var maxMetersPerSecond = _spawnableEnv.bounds.size
			/ _agent.MaxStep
			/ Time.fixedDeltaTime;

		var maxXZ = Mathf.Max(maxMetersPerSecond.x, maxMetersPerSecond.z);
		maxMetersPerSecond.x = maxXZ;
		maxMetersPerSecond.z = maxXZ;
		maxMetersPerSecond.y = 53; // override with
		float x = metersPerSecond.x / maxMetersPerSecond.x;
		float y = metersPerSecond.y / maxMetersPerSecond.y;
		float z = metersPerSecond.z / maxMetersPerSecond.z;
		// clamp result
		x = Mathf.Clamp(x, -1f, 1f);
		y = Mathf.Clamp(y, -1f, 1f);
		z = Mathf.Clamp(z, -1f, 1f);
		Vector3 normalizedVelocity = new Vector3(x,y,z);
		return normalizedVelocity;
	}
	public Vector3 GetNormalizedPosition(Vector3 pos)
	{
		var maxPos = new Vector3(500f,10f,50f);
		float x = pos.x / maxPos.x;
		float y = pos.y / maxPos.y;
		float z = pos.z / maxPos.z;
		// clamp result
		x = Mathf.Clamp(x, -1f, 1f);
		y = Mathf.Clamp(y, -1f, 1f);
		z = Mathf.Clamp(z, -1f, 1f);
		Vector3 normalizedPos = new Vector3(x,y,z);
		return normalizedPos;
	}

    public Vector3 GetNormalizedPosition()
    {
		// var position = GetCenterOfMass();
        var pelvis = GetFirstBodyPart(BodyConfig.GetRootBodyPart()); 
		var position = pelvis.Transform.position;
		var normalizedPosition = GetNormalizedPosition(position - startPosition);
        return normalizedPosition;
    }

    public void SetDebugFrameReward(float reward)
	{
		FrameReward = reward;
		var stepCount = _agent.StepCount > 0 ? _agent.StepCount : 1;
		if (_decisionRequester?.DecisionPeriod > 1)
			stepCount /= _decisionRequester.DecisionPeriod;
		AverageReward = _agent.GetCumulativeReward() / (float) stepCount;		
	}

    public List<float> GetSensorIsInTouch()
    {
        return SensorIsInTouch;
    }

    public List<float> GetMusclesObservations()
    {
        List<float> vectorObservation = new List<float>();
		foreach (var muscle in Muscles)
		{
			muscle.UpdateObservations();
			if (muscle.ConfigurableJoint.angularXMotion != ConfigurableJointMotion.Locked)
				vectorObservation.Add(muscle.TargetNormalizedRotationX);
			if (muscle.ConfigurableJoint.angularYMotion != ConfigurableJointMotion.Locked)
				vectorObservation.Add(muscle.TargetNormalizedRotationY);
			if (muscle.ConfigurableJoint.angularZMotion != ConfigurableJointMotion.Locked)
				vectorObservation.Add(muscle.TargetNormalizedRotationZ);
        }        
        return vectorObservation;
    }
    [Obsolete("use GetSensorObservations()")]
    public List<float> GetSensorYPositions()
    {
		var sensorYpositions = Sensors
			.Select(x=> this.GetNormalizedPosition(x.transform.position - startPosition))
			.Select(x=>x.y)
			.ToList();
        return sensorYpositions;
    }
    [Obsolete("use GetSensorObservations()")]
	public List<float> GetSensorZPositions()
    {
		var sensorYpositions = Sensors
			.Select(x=> this.GetNormalizedPosition(x.transform.position - startPosition))
			.Select(x=>x.z)
			.ToList();
        return sensorYpositions;
    }

    public List<float> GetSensorObservations()
	{
		var localSensorsPos = new Vector3[Sensors.Count];
		var globalSensorsPos = new Vector3[Sensors.Count];
		for (int i = 0; i < Sensors.Count; i++) {
			globalSensorsPos[i] = sensorColliders[i].transform.TransformPoint(sensorColliders[i].center);
			localSensorsPos[i] = globalSensorsPos[i] - startPosition;
		}

		// get heights based on global senor position
		var sensorsPos = Sensors
			.Select(x=>x.transform.position).ToList();
        // var sensorHeights = Enumerable.Range(0, globalSensorsPos.Length).Select(x=>0f).ToList();
		var sensorHeights = _terrainGenerator != null
			? _terrainGenerator.GetDistances2d(globalSensorsPos)
			: Enumerable.Range(0, globalSensorsPos.Length).Select(x=>0f).ToList();
		for (int i = 0; i < Sensors.Count; i++) {
			sensorHeights[i] -= sensorColliders[i].radius;
			if (sensorHeights[i] >= 1f)
				sensorHeights[i] = 1f;
		}
			
		// get z positions based on local positions
		var bounds = _spawnableEnv.bounds;
		var normalizedZ = localSensorsPos
			.Select(x=>x.z / (bounds.extents.z))
			.ToList();
		var observations = sensorHeights
			.Concat(normalizedZ)
			.ToList();
		return observations;
	}

    float NextGaussian(float mu = 0, float sigma = 1)
	{
		var u1 = UnityEngine.Random.value;
		var u2 = UnityEngine.Random.value;

		var rand_std_normal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
							Mathf.Sin(2.0f * Mathf.PI * u2);

		var rand_normal = mu + sigma * rand_std_normal;

		return rand_normal;
	}
}
