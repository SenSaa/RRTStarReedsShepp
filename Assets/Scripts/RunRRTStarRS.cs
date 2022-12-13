using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using rrt;
using ReedsShepp;
using System.Threading;
using Visualisation;
using UtilityFunctions;
using System.Linq;

public class RunRRTStarRS : MonoBehaviour
{

    [SerializeField] private Transform agent;
    private List<List<double>> path;
    RRTStarReedsShepp rrt_star_reeds_shepp;
    [SerializeField] private int pathIndex;
    private Vector3 nextHeading;
    private Vector3 lastHeading;
    [SerializeField] private float speed;
    private Thread planningThread;
    private Vector3 nextPos;
    private bool plotPathFlag;

    void Start()
    {
        setupRRTStarRS();
        planningThread = new Thread(runRRTStarRS);
        planningThread.Start();
        speed = 5f;
    }

    void Update()
    {
        if (rrt_star_reeds_shepp.DrawPath)
        {
            rrt_star_reeds_shepp.drawPath();
            rrt_star_reeds_shepp.DrawPath = false;
        }

        if (path == null) { return; }

        nextPos = new Vector3((float)path[pathIndex][0], 0, (float)path[pathIndex][1]);

        if (path[pathIndex].Count > 2)
        {
            float yawRadToDegree = (float)path[pathIndex][2] * Mathf.Rad2Deg;
            float transformYaw = Utils.transformYaw(yawRadToDegree); // 
            nextHeading = new Vector3(0, (float)transformYaw, 0);
        }

        if ((agent.position - nextPos).magnitude < 0.1f && pathIndex < path.Count - 1)
        {
            pathIndex++;
        }
        else
        {
            agent.position = Vector3.Lerp(agent.position, nextPos, Time.deltaTime * speed);
            agent.rotation = Quaternion.Euler(0, nextHeading.y, 0);
        }

        if (!plotPathFlag)
        {
            plotPathFlag = true;
            plotPath();
        }
    }

    public void main()
    {
        setupRRTStarRS();
    }

    public void setupRRTStarRS(int max_iter = 100, double connect_circle_dist = 15)
    {
        Debug.Log("Start ");

        List<Obstacle> obstacleList = new List<Obstacle>();
        obstacleList.Add(new Obstacle(5, 5, 1));
        obstacleList.Add(new Obstacle(4, 6, 1));
        obstacleList.Add(new Obstacle(4, 8, 1));
        obstacleList.Add(new Obstacle(4, 10, 1));
        obstacleList.Add(new Obstacle(6, 5, 1));
        obstacleList.Add(new Obstacle(7, 5, 1));
        obstacleList.Add(new Obstacle(8, 6, 1));
        obstacleList.Add(new Obstacle(8, 8, 1));
        obstacleList.Add(new Obstacle(8, 10, 1));

        // Set Initial parameters
        var startX = 0.0;
        var startZ = 0.0;
        var startHeading = 0.0;
        var endX = 6.0;
        var endZ = 7.0;
        var endHeading = 90.0;

        var start = new List<double> {
                startX,
                startZ,
                Mathf.Deg2Rad * (startHeading)
            };
        var goal = new List<double> {
                endX,
                endZ,
                Mathf.Deg2Rad * (endHeading)
            };
        rrt_star_reeds_shepp = 
            new RRTStarReedsShepp(start.ToArray(), goal.ToArray(), obstacleList, new List<int> {
                -2,
                15
            }.ToArray(), max_iter, connect_circle_dist);
         
        plotStart((float)startX, (float)startZ, (float)startHeading);
        plotGoal((float)endX, (float)endZ, (float)endHeading);
        plotObstacles(obstacleList);
    }

    public void runRRTStarRS()
    {
        path = rrt_star_reeds_shepp.planning(true, true);
    }

    private void plotStart(float x, float z, float heading)
    {
        plot(x, z, heading);
    }
    private void plotGoal(float x, float z, float heading)
    {
        plot(x, z, heading);
    }
    private void plotObstacles(List<Obstacle> obstacleList)
    {
        for(int i=0; i < obstacleList.Count; i++)
        {
            Obstacle obstacle = obstacleList[i];
            plot((float)obstacle.x, (float)obstacle.y);
        }
    }
    private void plot(float x, float z)
    {
        GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        obj.transform.position = new Vector3(x, 0, z);
    }
    private void plot(float x, float z, float heading)
    {
        GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        obj.transform.position = new Vector3(x, 0, z);
        obj.transform.rotation = Quaternion.Euler(0, heading, 0);
    }
    private void plotPath()
    {
        List<Tuple<double, double>> path_xy = new List<Tuple<double, double>>();
        List<double> yaws = new List<double>();
        foreach (var p in path)
        {
            path_xy.Add(Tuple.Create(p[0], p[1]));

            GameObject pathPoint = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            pathPoint.transform.position = new Vector3((float) p[0], 0, (float) p[1]);
            pathPoint.transform.localScale = new Vector3(.25f, .25f, .25f);
        }
        RenderPath renderPath = new RenderPath();
        renderPath.Draw(path_xy, transform, null, null);
    }

    private void OnApplicationQuit()
    {
        if (planningThread != null)
        {
            planningThread.Abort();
        }
    }

}
