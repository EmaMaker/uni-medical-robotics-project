using UnityEngine;
//Questo script viene richiamato automaticamente al momento di Play della scena e assegna un collider ad ogni joint
public class FingerContactController : MonoBehaviour
{
    public PhysicsGraspController controller; // riferimento al manager della mano



    private void OnTriggerEnter(Collider other)
    {
        var myCollider = GetComponent<Collider>();
        controller?.OnFingerTouchEnter(myCollider, other);
    }

    private void OnTriggerExit(Collider other)
    {
        var myCollider = GetComponent<Collider>();
        controller?.OnFingerTouchExit(myCollider, other);
    }
}

