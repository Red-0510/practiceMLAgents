using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[System.Serializable]

public class Muscle{
    public string Name;
    public BodyHelper.MuscleGroup Group;

    [Range(-1,1)]
    public float TargetNormalizedRotationX;
    [Range(-1,1)]
    public float TargetNormalizedRotationY;
    [Range(-1,1)]
    public float TargetNormalizedRotationZ;
    public Vector3 MaximumForce;

    public Vector3 ObsLocalPosition;
    public Quaternion ObsRotation;
    public Vector3 ObsNormalizedRotation;
    public Vector3 ObsNormalizedDeltaFromTargetRotation;
    public Vector3 ObsRotationVelocity;
    public Vector3 ObsVelocity;

    public Vector3 DebugMaxRotationVelocity;
    public Vector3 DebugMaxVelocity;


    public Quaternion DefaultLocalRotation;
    public Quaternion ToJointSpaceInverse;
    public Quaternion ToJointSpaceDefault;

    public Rigidbody Rigidbody;
    public Transform Transform;
    public ConfigurableJoint ConfigurableJoint;
    public Rigidbody Parent;
    public ConfigurableJoint RootConfigurableJoint;
    public Quaternion InitialRootRotation;
    public Vector3 InitialRootPosition;

    Quaternion _lastObsRotation;
    Vector3 _lastLocalPosition;
    float _lastUpdateObsTime;
    bool _firstRunComplete;
    bool _hasRanVeryFirstInit;

    public void UpdateMotor(){
        float powerMultiplier = 2.5f;
        var t=ConfigurableJoint.targetAngularVelocity;

        t.x = TargetNormalizedRotationX * MaximumForce.x;
		t.y = TargetNormalizedRotationY * MaximumForce.y;
		t.z = TargetNormalizedRotationZ * MaximumForce.z;
		ConfigurableJoint.targetAngularVelocity = t;

        var angX = ConfigurableJoint.angularXDrive;
		angX.positionSpring = 1f;
		var scale = MaximumForce.x * Mathf.Pow(Mathf.Abs(TargetNormalizedRotationX), 3);
		angX.positionDamper = Mathf.Max(1f, scale);
		angX.maximumForce = Mathf.Max(1f,MaximumForce.x * powerMultiplier);
		ConfigurableJoint.angularXDrive = angX;

        var maxForce = Mathf.Max(MaximumForce.y, MaximumForce.z);
		var angYZ = ConfigurableJoint.angularYZDrive;
		angYZ.positionSpring = 1f;
        var maxAbsRotXY = Mathf.Max(Mathf.Abs(TargetNormalizedRotationY) + Mathf.Abs(TargetNormalizedRotationZ));
		scale = maxForce * Mathf.Pow(maxAbsRotXY, 3);
		angYZ.positionDamper = Mathf.Max(1f, scale);
		angYZ.maximumForce = Mathf.Max(1f, maxForce * powerMultiplier);
		ConfigurableJoint.angularYZDrive = angYZ;
    }

    static Vector3 NormalizedEulerAngles(Vector3 eulerAngles){
        var x = eulerAngles.x < 180f ?
            eulerAngles.x :
            - 360 + eulerAngles.x;
        var y = eulerAngles.y < 180f ?
            eulerAngles.y :
            - 360 + eulerAngles.y;
        var z = eulerAngles.z < 180f ?
            eulerAngles.z :
            - 360 + eulerAngles.z;
        x = x / 180f;
        y = y / 180f;
        z = z / 180f;
        return new Vector3(x,y,z);
    }

    static Vector3 ScaleNormalizedByJoint(Vector3 normalizedRotation, ConfigurableJoint configurableJoint)
    {
        var x = normalizedRotation.x > 0f ?
            (normalizedRotation.x * 180f) / configurableJoint.highAngularXLimit.limit :
            (-normalizedRotation.x * 180f) / configurableJoint.lowAngularXLimit.limit;
        var y = (normalizedRotation.y * 180f) / configurableJoint.angularYLimit.limit;
        var z = (normalizedRotation.z * 180f) / configurableJoint.angularZLimit.limit;
        var scaledNormalizedRotation = new Vector3(x,y,z);
        return scaledNormalizedRotation;
    }

    static Vector3 Vector3Max (Vector3 a, Vector3 b)
    {
        var answer = new Vector3(
            Mathf.Max(Mathf.Abs(a.x), Mathf.Abs(b.x)),
            Mathf.Max(Mathf.Abs(a.y), Mathf.Abs(b.y)),
            Mathf.Max(Mathf.Abs(a.z), Mathf.Abs(b.z)));
        return answer;
    }

    public void Init(){
        _firstRunComplete = false;
        Rigidbody.angularVelocity = Vector3.zero;
        Rigidbody.velocity = Vector3.zero;

        if (!_hasRanVeryFirstInit) {
			Parent = ConfigurableJoint.connectedBody;
			
            InitialRootRotation = RootConfigurableJoint.transform.rotation;
            InitialRootPosition = RootConfigurableJoint.transform.position;

			DefaultLocalRotation = LocalRotation;
			// Vector3 forward = Vector3.Cross (ConfigurableJoint.axis, ConfigurableJoint.secondaryAxis).normalized;
			//Vector3 up = Vector3.Cross (forward, ConfigurableJoint.axis).normalized;
            Vector3 forward = this.Transform.forward;
            //J-here change from forward->up
            Vector3 up = this.Transform.up;
			Quaternion toJointSpace = Quaternion.LookRotation(forward, up);
			
			ToJointSpaceInverse = Quaternion.Inverse(toJointSpace);
			ToJointSpaceDefault = DefaultLocalRotation * toJointSpace;
            _hasRanVeryFirstInit = true;

        }
    }

    public void UpdateObservations()
    {
        ObsRotation = this.LocalRotation;
        ObsRotation = (ToJointSpaceInverse * UnityEngine.Quaternion.Inverse(this.LocalRotation) * this.ToJointSpaceDefault);
        var r2 = (ToJointSpaceInverse * UnityEngine.Quaternion.Inverse(this.Transform.rotation) * this.ToJointSpaceDefault);
        var s1 = ScaleNormalizedByJoint(NormalizedEulerAngles((this.LocalRotation * ToJointSpaceDefault).eulerAngles), ConfigurableJoint);
        var s2 = ScaleNormalizedByJoint(NormalizedEulerAngles((Transform.localRotation * ToJointSpaceDefault).eulerAngles), ConfigurableJoint);
        var s3 = ScaleNormalizedByJoint(NormalizedEulerAngles((this.LocalRotation * ToJointSpaceInverse).eulerAngles), ConfigurableJoint);
        var s4 = ScaleNormalizedByJoint(NormalizedEulerAngles((Transform.localRotation * ToJointSpaceInverse).eulerAngles), ConfigurableJoint);
        var s5 = ScaleNormalizedByJoint(NormalizedEulerAngles((UnityEngine.Quaternion.Inverse(this.LocalRotation) * ToJointSpaceDefault).eulerAngles), ConfigurableJoint);
        
        var normalizedRotation = NormalizedEulerAngles(ObsRotation.eulerAngles);
        // var normalizedRotation = NormalizedEulerAngles(this.LocalRotation.eulerAngles);
        ObsNormalizedRotation = ScaleNormalizedByJoint(normalizedRotation, ConfigurableJoint);
        ObsNormalizedDeltaFromTargetRotation = 
            new Vector3(TargetNormalizedRotationX, TargetNormalizedRotationY, TargetNormalizedRotationZ) - ObsNormalizedRotation;

        if (_firstRunComplete == false){
            _lastUpdateObsTime = Time.time;
            _lastObsRotation = ObsRotation;
            _lastLocalPosition = Transform.localPosition;
        }
        var dt = Time.time - _lastUpdateObsTime;
        _lastUpdateObsTime = Time.time;
        var rotationVelocity = ObsRotation.eulerAngles - _lastObsRotation.eulerAngles;
        rotationVelocity = NormalizedEulerAngles(rotationVelocity);
        rotationVelocity /= 128f;
        if (dt > 0f)
            rotationVelocity /= dt;
        ObsRotationVelocity = rotationVelocity;
        _lastObsRotation = ObsRotation;
        var rootBone = RootConfigurableJoint.transform;
        var toRootSpace = Quaternion.Inverse(RootConfigurableJoint.transform.rotation) * rootBone.rotation;
        Quaternion rootRotation = Quaternion.Inverse(rootBone.rotation * toRootSpace) * Transform.rotation;
        ObsLocalPosition = Transform.position - RootConfigurableJoint.transform.position;
        var velocity = ObsLocalPosition - _lastLocalPosition;
        ObsVelocity = velocity;
        if (dt > 0f)
            velocity /= dt;
        _lastLocalPosition = ObsLocalPosition;

        DebugMaxRotationVelocity = Vector3Max(DebugMaxRotationVelocity, rotationVelocity);
        DebugMaxVelocity = Vector3Max(DebugMaxVelocity, velocity);

        _firstRunComplete = true;
    }


    public Quaternion LocalRotation {
        get {
            // around root Rotation 
            return Quaternion.Inverse(RootRotation) * Transform.rotation;

            // around parent space
            // return Quaternion.Inverse(ParentRotation) * transform.rotation;
        }
    }

    public Quaternion RootRotation {
        get {
            return InitialRootRotation;
        }
    }
}