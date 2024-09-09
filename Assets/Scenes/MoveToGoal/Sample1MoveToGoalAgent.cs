using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class Sample1MoveToGoalAgent : Agent
{

    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial,loseMaterial,gridMatFloor;
    [SerializeField] private MeshRenderer floorMeshRenderer;
    public float moveSpeed=1f;
    Vector3 startPos;
    private bool isPrevSuccess=false;

    public void Update(){
        // Debug.Log("updating");
        AddReward(-0.001f);
    }

    public override void OnEpisodeBegin(){
        transform.localPosition = new Vector3(Random.Range(-8.5f,8.5f),0,Random.Range(-6f,6f));
        targetTransform.localPosition = new Vector3(Random.Range(-8.5f,8.5f),0,Random.Range(-6f,6f));
        if(!isPrevSuccess)floorMeshRenderer.material = loseMaterial;
        else floorMeshRenderer.material = winMaterial;
        isPrevSuccess=false;
    }

    public override void CollectObservations(VectorSensor sensor){
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(targetTransform.localPosition);
    }
    public override void OnActionReceived(ActionBuffers actions){
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];

        transform.localPosition+=new Vector3(moveX,0,moveZ) * Time.deltaTime * moveSpeed;
    }

    public override void Heuristic(in ActionBuffers actionsOut){
        ActionSegment<float> continousActions = actionsOut.ContinuousActions;
        continousActions[0] = Input.GetAxisRaw("Horizontal");
        continousActions[1] = Input.GetAxisRaw("Vertical");
    }

    private void OnTriggerEnter(Collider other) {
        Debug.Log(other);
        if(other.gameObject.tag=="goal"){
            AddReward(5f);
            isPrevSuccess=true;
            // floorMeshRenderer.material = winMaterial;
            EndEpisode();
        }
        else if(other.gameObject.tag =="border"){
            AddReward(-2f);
            // floorMeshRenderer.material = loseMaterial;
            EndEpisode();
        }
    }
}
