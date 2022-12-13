using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UtilityFunctions;
using ReedsShepp;

public class ReedsSheppPathPlanning : MonoBehaviour
{

    private LineRenderer lineRenderer;
    private GameObject pointsParentObj;
    [SerializeField] private Material pointMaterial;
    [SerializeField] private GameObject arrowPrefab;

    void Start()
    {
        Debug.Log("--- Reeds Shepp path planner");

        // Start Pose
        double start_x = -1.0; // [m]
        double start_y = -4.0; // [m]
        double start_yaw = Mathf.Deg2Rad * (-20.0); // [rad]

        // End Pose
        double end_x = 5.0; // [m]
        double end_y = 5.0; // [m]
        double end_yaw = Mathf.Deg2Rad * (25.0); // [rad]

        // Other params
        double curvature = 0.1;
        double step_size = 0.05;

        // Plan Reeds Shepp Path
        ReedsSheppPath reedsSheepPath = new ReedsSheppPath();
        Tuple<List<double>, List<double>, List<double>, List<string>, List<double>> xs_ys_yaws_modes_lengths = reedsSheepPath.reeds_shepp_path_planning(
            start_x, start_y,
            start_yaw, end_x,
            end_y, end_yaw,
            curvature,
            step_size);

        List<double> xs = xs_ys_yaws_modes_lengths.Item1;
        List<double> ys = xs_ys_yaws_modes_lengths.Item2;
        List<double> yaws = xs_ys_yaws_modes_lengths.Item3;
        List<string> modes = xs_ys_yaws_modes_lengths.Item4;
        List<double> lengths = xs_ys_yaws_modes_lengths.Item5;
        
        // Plot
        initLineRenderer();
        createPointsParentObj();
        renderPath(xs, ys);
        plotPoseArrow(start_x, start_y, start_yaw);
        plotPoseArrow(end_x, end_y, end_yaw);
        StartCoroutine(agentUpdate(xs, ys, yaws));
    }

    private void initLineRenderer()
    {
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
        }
        else if (lineRenderer != null)
        {
            lineRenderer = GetComponent<LineRenderer>();
        }
    }

    private void renderPath(List<double> xs, List<double> ys)
    {
        lineRenderer.positionCount = xs.Count;
        lineRenderer.widthMultiplier = 0.4f;
        for (int i=0; i < xs.Count; i++) 
        {
            Vector3 position = new Vector3((float) xs[i], 0, (float) ys[i]);
            lineRenderer.SetPosition(i, position);

            // Render points
            renderPoints(i, position);
        }
    }

    private void createPointsParentObj()
    {
        pointsParentObj = new GameObject("PointsParentObj");
    }

    private void renderPoints(int index, Vector3 pos)
    {
        GameObject gameObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        gameObj.transform.parent = pointsParentObj.transform;
        gameObj.name = index.ToString();
        gameObj.transform.position = pos;
        gameObj.GetComponent<MeshRenderer>().material = pointMaterial;
    }

    private void plotPoseArrow(double x, double y, double yaw)
    {
        Vector3 position = new Vector3((float)x, 1, (float)y);
        float transformedYaw = Utils.transformYaw(yaw);
        Quaternion rotation = Quaternion.Euler(0, transformedYaw * Mathf.Rad2Deg, 0);
        Instantiate(arrowPrefab, position, rotation, pointsParentObj.transform);
    }

    IEnumerator agentUpdate(List<double> xs, List<double> ys, List<double> yaws)
    {
        for(int i=0; i < xs.Count; i++)
        {
            if (i%10 == 0)
            {
                plotPoseArrow(xs[i], ys[i], yaws[i]);
                yield return new WaitForSeconds(0.1f);
            }
        }
    }

}
