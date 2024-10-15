using UnityEngine;

namespace MLAgents
{
    public class SensorBehavior : MonoBehaviour
    {
        RagdollStraightWalkerAgent _ragdollStraightWalkerAgent;
        IOnSensorCollision _onSensorCollision;

        Collider _collider;

        void Start()
        {
            _ragdollStraightWalkerAgent = GetComponentInParent<RagdollStraightWalkerAgent>();
            _onSensorCollision = GetComponentInParent<IOnSensorCollision>();
            _collider = GetComponent<Collider>();
        }

        void OnCollisionEnter(Collision other)
        {
            //J-commented
            // if (_ragdollStraightWalkerAgent != null)
            //     _ragdollStraightWalkerAgent.SensorCollisionEnter(_collider, other);
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionEnter(_collider, other.gameObject);
        }

        void OnCollisionExit(Collision other)
        {
            //J-commented
            // if (_ragdollStraightWalkerAgent != null)
            //     _ragdollStraightWalkerAgent.SensorCollisionExit(_collider, other);
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionExit(_collider, other.gameObject);
        }

        void OnTriggerEnter(Collider other)
        {
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionEnter(_collider, other.gameObject);
        }
        void OnTriggerExit(Collider other)
        {
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionExit(_collider, other.gameObject);
        }
        void OnTriggerStay(Collider other)
        {
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionEnter(_collider, other.gameObject);
        }        
    }
}