using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Camera_Script : MonoBehaviour
{
    public Camera Main_Camera;
    public Camera Camera_1;
    public Camera Camera_2;
    public Camera Camera_3;
    public Camera Camera_4;
    public Camera Camera_5;
    public Camera Camera_6;
    public Camera Camera_7;

    void Start()
    {
        Main_Camera.gameObject.SetActive(true);
        Camera_1.gameObject.SetActive(false);
        Camera_2.gameObject.SetActive(false);
        Camera_3.gameObject.SetActive(false);
        Camera_4.gameObject.SetActive(false);
        Camera_5.gameObject.SetActive(false);
        Camera_6.gameObject.SetActive(false);
        Camera_7.gameObject.SetActive(false);
        Debug.Log("Switched to Main Camera");

    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Keypad0))
        {
            Main_Camera.gameObject.SetActive(true); //
            Camera_1.gameObject.SetActive(false);
            Camera_2.gameObject.SetActive(false);
            Camera_3.gameObject.SetActive(false);
            Camera_4.gameObject.SetActive(false);
            Camera_5.gameObject.SetActive(false);
            Camera_6.gameObject.SetActive(false);
            Camera_7.gameObject.SetActive(false);
            Debug.Log("Switched to Main Camera");
        }

        if (Input.GetKeyDown(KeyCode.Keypad1))
        {
            Main_Camera.gameObject.SetActive(false);
            Camera_1.gameObject.SetActive(true);    //
            Camera_2.gameObject.SetActive(false);
            Camera_3.gameObject.SetActive(false);
            Camera_4.gameObject.SetActive(false);
            Camera_5.gameObject.SetActive(false);
            Camera_6.gameObject.SetActive(false);
            Camera_7.gameObject.SetActive(false);
            Debug.Log("Switched to Camera 1");
        }

        if (Input.GetKeyDown(KeyCode.Keypad2))
        {
            Main_Camera.gameObject.SetActive(false);
            Camera_1.gameObject.SetActive(false);
            Camera_2.gameObject.SetActive(true);    //
            Camera_3.gameObject.SetActive(false);
            Camera_4.gameObject.SetActive(false);
            Camera_5.gameObject.SetActive(false);
            Camera_6.gameObject.SetActive(false);
            Camera_7.gameObject.SetActive(false);
            Debug.Log("Switched to Camera 2");
        }

        if (Input.GetKeyDown(KeyCode.Keypad3))
        {
            Main_Camera.gameObject.SetActive(false);
            Camera_1.gameObject.SetActive(false);
            Camera_2.gameObject.SetActive(false);
            Camera_3.gameObject.SetActive(true);    //
            Camera_4.gameObject.SetActive(false);
            Camera_5.gameObject.SetActive(false);
            Camera_6.gameObject.SetActive(false);
            Camera_7.gameObject.SetActive(false);
            Debug.Log("Switched to Camera 3");
        }

        if (Input.GetKeyDown(KeyCode.Keypad4))
        {
            Main_Camera.gameObject.SetActive(false);
            Camera_1.gameObject.SetActive(false);
            Camera_2.gameObject.SetActive(false);
            Camera_3.gameObject.SetActive(false);
            Camera_4.gameObject.SetActive(true);    //
            Camera_5.gameObject.SetActive(false);
            Camera_6.gameObject.SetActive(false);
            Camera_7.gameObject.SetActive(false);
            Debug.Log("Switched to Camera 4");
        }

        if (Input.GetKeyDown(KeyCode.Keypad5))
        {
            Main_Camera.gameObject.SetActive(false);
            Camera_1.gameObject.SetActive(false);
            Camera_2.gameObject.SetActive(false);
            Camera_3.gameObject.SetActive(false);
            Camera_4.gameObject.SetActive(false);
            Camera_5.gameObject.SetActive(true);   //
            Camera_6.gameObject.SetActive(false);
            Camera_7.gameObject.SetActive(false);
            Debug.Log("Switched to Camera 5");
        }

        if (Input.GetKeyDown(KeyCode.Keypad6))
        {
            Main_Camera.gameObject.SetActive(false);
            Camera_1.gameObject.SetActive(false);
            Camera_2.gameObject.SetActive(false);
            Camera_3.gameObject.SetActive(false);
            Camera_4.gameObject.SetActive(false);
            Camera_5.gameObject.SetActive(false);
            Camera_6.gameObject.SetActive(true);    //
            Camera_7.gameObject.SetActive(false);
            Debug.Log("Switched to Camera 6");

        }

        if (Input.GetKeyDown(KeyCode.Keypad7))
        {
            Main_Camera.gameObject.SetActive(false);
            Camera_1.gameObject.SetActive(false);
            Camera_2.gameObject.SetActive(false);
            Camera_3.gameObject.SetActive(false);
            Camera_4.gameObject.SetActive(false);
            Camera_5.gameObject.SetActive(false);
            Camera_6.gameObject.SetActive(false);
            Camera_7.gameObject.SetActive(true);    //
            Debug.Log("Switched to Camera 7");

        }
    }
}
