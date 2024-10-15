using UnityEngine;

namespace MLAgents
{
    public class SendOnCollisionTrigger : MonoBehaviour
    {
        void OnCollisionEnter(Collision other)
        {
            // Messenger.
            var otherGameobject = other.gameObject;
            var ragdollAgent = otherGameobject.GetComponentInParent<RagdollStraightWalkerAgent>();
            if (ragdollAgent != null)
                ragdollAgent.OnTerrainCollision(otherGameobject, this.gameObject);
            var iOnTerrainCollision = otherGameobject.GetComponentInParent<IOnTerrainCollision>();
            if (iOnTerrainCollision != null)
                iOnTerrainCollision.OnTerrainCollision(otherGameobject, this.gameObject);
        }
    }
}