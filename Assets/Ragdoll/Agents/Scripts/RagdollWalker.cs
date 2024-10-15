using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class RagdollWalker : MonoBehaviour
{

    [System.Serializable]
    public class MusclePower{
        public string Muscle;
        public Vector3 PowerVector;
    }

    public List<MusclePower> MusclePowers;
    public float MotorScale=1f;
    // Start is called before the first frame update
    void Start()
    {
        Setup();
    }

    void Setup(){
        IgnoreCollision("torso", new []{"left_upper_arm", "right_upper_arm"});
        IgnoreCollision("butt", new []{"left_thigh", "right_thigh"});

        IgnoreCollision("left_larm", new []{"left_upper_arm"});
        IgnoreCollision("right_larm", new []{"right_upper_arm"});
        IgnoreCollision("left_shin", new []{"left_thigh"});
        IgnoreCollision("right_shin", new []{"right_thigh"});

        IgnoreCollision("right_shin", new []{"right_right_foot"});
        IgnoreCollision("left_shin", new []{"left_left_foot"});

        var joints = GetComponentsInChildren<Joint>().ToList();
        foreach (var joint in joints){
            joint.enablePreprocessing = false;
        }
    }

    void IgnoreCollision(string first, string[] seconds)
    {
        foreach (var second in seconds)
        {
            IgnoreCollision(first, second);
        }
    }

    void IgnoreCollision(string first,string second){
        var rigidBodies = GetComponentsInChildren<Rigidbody>().ToList();
        var colliderOnes = rigidBodies.FirstOrDefault(x=>x.name.Contains(first))?.GetComponents<Collider>();
        var colliderTwos = rigidBodies.FirstOrDefault(x=>x.name.Contains(second))?.GetComponents<Collider>();

        if(colliderOnes==null || colliderTwos==null) return;
        foreach(var c1 in colliderOnes){
            foreach(var c2 in colliderTwos){
                Physics.IgnoreCollision(c1, c2);
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
