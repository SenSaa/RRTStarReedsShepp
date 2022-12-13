using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VisualisationHelper : MonoBehaviour
{

    public static void DrawPoint()
    {

    }

    public static void DrawPoint(Vector3 pointPos, float scale)
    {
        GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.transform.position = pointPos;
        go.transform.localScale = new Vector3(scale, scale, scale);
    }

    public static void DrawShape()
    {

    }

    public static void DrawLine(List<Vector3> pointPosSet, Color color)
    {
        for (int i = 1; i < pointPosSet.Count; i++)
        {
            Debug.DrawLine(pointPosSet[i], pointPosSet[i - 1], color, float.MaxValue);
        }
    }
    public static void DrawLine(Vector3 from, Vector3 to, Color color)
    {
            Debug.DrawLine(from, to, color, float.MaxValue);
    }

}
