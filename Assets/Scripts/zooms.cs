using UnityEngine;

public class zooms : MonoBehaviour
{
    // Update is called once per frame
    void Update()
    {
        // based on the mouse scroll movement zoom in/out will be applied
        if (Input.mouseScrollDelta.y != 0)
        {
            // Adjust the camera's orthographic size
            if (Camera.main != null)
            {
                Camera.main.orthographicSize -= Input.mouseScrollDelta.y;

                // Clamp the orthographic size to a reasonable range (e.g., between 2 and 20)
                //Camera.main.orthographicSize = Mathf.Clamp(Camera.main.orthographicSize, 2f, 40f);
            }
            else
            {
                Debug.LogWarning("No camera found");
            }
        }

        //Move camera left
        if (Input.GetKey(KeyCode.A))
        {
            if (Camera.main != null)
            {
                Camera.main.transform.Translate(-15 * Time.deltaTime, 0, 0, Space.World);
            }
            else
            {
                Debug.LogWarning("No camera found");
            }
        }

        //Move camera right
        if (Input.GetKey(KeyCode.D))
        {
            if (Camera.main != null)
            {
                Camera.main.transform.Translate(15 * Time.deltaTime, 0, 0, Space.World);
            }
            else
            {
                Debug.LogWarning("No camera found");
            }
        }

        //Move camera up
        if (Input.GetKey(KeyCode.W))
        {
            if (Camera.main != null)
            {
                Camera.main.transform.Translate(0, 15 * Time.deltaTime, 0, Space.World);
            }
            else
            {
                Debug.LogWarning("No camera found");
            }
        }

        //Move camera down
        if (Input.GetKey(KeyCode.S))
        {
            if (Camera.main != null)
            {
                Camera.main.transform.Translate(0, -15 * Time.deltaTime, 0, Space.World);
            }
            else
            {
                Debug.LogWarning("No camera found");
            }
        }
    }
}