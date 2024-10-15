using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Teacher : MonoBehaviour
{
    // Start is called before the first frame update
    Animator anim;
    Vector3 initialPos;
    void Start()
    {
        initialPos = transform.localPosition;
        anim = GetComponent<Animator>();
        Walker.onEpisodeBegin += TriggerAnimation;
    }


    // private void onDestroy(){
    //     Walker.onEpisodeBegin -= TriggerAnimation;
    // }

    // Update is called once per frame

    // private void Update(){
    //     Debug.Log(anim.GetCurrentAnimatorStateInfo(0).ToString());
    // }
    private void TriggerAnimation(){

        if(anim!=null){
            // anim.SetTrigger("Start");
            transform.localPosition = initialPos;
            anim.SetTrigger("reset");
        }
    }
}
