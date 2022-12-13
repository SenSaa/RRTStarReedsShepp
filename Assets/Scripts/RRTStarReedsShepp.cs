// Path planning Sample Code with RRT with Reeds-Shepp path
// author: AtsushiSakai(@Atsushi_twi)
// https://github.com/AtsushiSakai/PythonRobotics

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using rrt;
using ReedsShepp;
using UtilityFunctions;

public class RRTStarReedsShepp : RRTStar
{
    Node start;
    Node end;
    int min_rand;
    int max_rand;
    int max_iter;
    List<Obstacle> obstacle_list;
    double connect_circle_dist;
    double curvature;
    double goal_yaw_th;
    double goal_xy_th;

    ReedsSheppPath reedsSheppPathPlanning;
    System.Random random;

    public bool DrawPath;

    public RRTStarReedsShepp(
        double[] start,
        double[] goal,
        List<Obstacle> obstacle_list,
        int[] rand_area,
        int max_iter = 100,
        double connect_circle_dist = 50.0)
        : base(start, goal, obstacle_list, rand_area)
    {
        this.start = new Node(start[0], start[1], start[2]);
        this.end = new Node(goal[0], goal[1], goal[2]);
        this.min_rand = rand_area[0];
        this.max_rand = rand_area[1];
        this.max_iter = max_iter;
        this.obstacle_list = obstacle_list;
        this.connect_circle_dist = connect_circle_dist;
        this.curvature = 1.0;
        this.goal_yaw_th = Mathf.Deg2Rad * (1.0);
        this.goal_xy_th = 0.5;

        reedsSheppPathPlanning = new ReedsSheppPath();
        random = new System.Random();
    }

    // RRT* Reeds Sheep Planning.
    int iter = 0;
    public List<List<double>> planning(bool animation = true, bool search_until_max_iter = true)
    {
        int last_index;
        this.node_list = new List<Node> { this.start };
        foreach (var i in Enumerable.Range(0, this.max_iter))
        {
            iter = i;
            if (i % 10 == 0) { Debug.Log("RRT*RS i = " + i); }
            try
            {
                Node rnd = this.get_random_node();
                var nearest_ind = getNearestNodeIndex(this.node_list, rnd);
                Node new_node = steer(this.node_list[nearest_ind], rnd);
                if (checkCollision(new_node, this.obstacle_list))
                {
                    var near_indexes = this.find_near_nodes(new_node);
                    new_node = this.choose_parent(new_node, near_indexes);
                    if (new_node != null)
                    {
                        this.node_list.Add(new_node);
                        this.rewire(new_node, near_indexes);
                        this.try_goal_path(new_node);
                    }
                }
                if (animation && new_node != null)
                {
                    DrawPath = true;
                }
                if (!search_until_max_iter && new_node != null)
                {
                    // check reaching the goal                    
                    last_index = this.search_best_goal_node();
                    if (last_index != int.MaxValue)
                    {
                        Debug.Log("Path Found.");
                        return this.generate_final_course(last_index);
                    }
                }
            }
            catch (Exception e) 
            { 
                if (e is NullReferenceException)
                {
                    Debug.LogWarning(e);
                    continue;
                }
                else
                {
                    Debug.LogError(e);
                    break;
                }

            }
        }
        Debug.Log("iter = " + iter);
        Debug.Log("reached max iteration");
        last_index = this.search_best_goal_node();
        if (last_index != int.MaxValue)
        {
            Debug.Log("Path Found.");
            return this.generate_final_course(last_index);
        }
        else
        {
            Debug.Log("Cannot find path");
        }
        return null;
    }

    public void try_goal_path(Node node)
    {
        Node goal = new Node(this.end.x, this.end.y, this.end.yaw);
        Node new_node = steer(node, goal);
        if (new_node == null)
        {
            return;
        }
        if (checkCollision(new_node, this.obstacle_list))
        {
            this.node_list.Add(new_node);
        }
    }
    
    // *** Important: Must override the "steer()" method so as to use the path yaw from RS path.
    override
    public Node steer(Node from_node, Node to_node, double expand_dis = 2.0)
    {
        var x_y_yaw_mode_course = reedsSheppPathPlanning.reeds_shepp_path_planning(from_node.x, from_node.y, from_node.yaw, to_node.x, to_node.y, to_node.yaw, this.curvature);
        var px = x_y_yaw_mode_course.Item1;
        var py = x_y_yaw_mode_course.Item2;
        var pyaw = x_y_yaw_mode_course.Item3;
        var mode = x_y_yaw_mode_course.Item4;
        var course_lengths = x_y_yaw_mode_course.Item5;
        if (px == null)
        {
            return null;
        }
        var new_node = from_node.DeepClone();
        new_node.x = px[px.Count - 1];
        new_node.y = py[py.Count - 1];
        new_node.yaw = pyaw[pyaw.Count - 1];
        new_node.path_x = px;
        new_node.path_y = py;
        new_node.path_yaw = pyaw;
        new_node.cost += (from l in course_lengths
                          select Math.Abs(l)).ToList().Sum();
        new_node.parent = from_node;

        return new_node;
    }

    public Node get_random_node()
    {
        var rnd = new Node(random.Next(this.min_rand, this.max_rand),
            random.Next(this.min_rand, this.max_rand),
            MathHelpers.getRandomNumber(-Math.PI, Math.PI));
        return rnd;
    }

    public int search_best_goal_node()
    {
        // dist check
        var goal_indexes = new List<int>();
        foreach (var _tup_1 in this.node_list.Select((_p_1, _p_2) => Tuple.Create(_p_2, _p_1)))
        {
            var i = _tup_1.Item1;
            var node = _tup_1.Item2;
            if (this.calculateDistToGoal(node.x, node.y) <= this.goal_xy_th)
            {
                goal_indexes.Add(i);
            }
        }

        // angle check
        var final_goal_indexes = new List<int>();
        foreach (var i in goal_indexes)
        {
            if (Math.Abs(this.node_list[i].yaw - this.end.yaw) <= this.goal_yaw_th)
            {
                final_goal_indexes.Add(i);
            }
        }

        if (final_goal_indexes.Count == 0 || final_goal_indexes == null)
        {
            return int.MaxValue;
        }

        var min_cost = double.MaxValue;
        var previousCost = double.MaxValue;
        foreach (var i in final_goal_indexes)
        {
            var tempcost = node_list[i].cost;
            if (tempcost < previousCost)
            {
                min_cost = tempcost;
            }
            else
            {
                min_cost = previousCost;
            }
            previousCost = tempcost;
        }

        foreach (var i in final_goal_indexes)
        {
            if (node_list[i].cost == min_cost)
            {
                return i;
            }
        }

        return int.MaxValue;
    }

    public List<List<double>> generate_final_course(int goal_index)
    {
        List<List<double>> path = new List<List<double>>();

        try
        {

            path.Add(new List<double>
        {
            end.x,
            end.y,
            end.yaw
        });
            var node = this.node_list[goal_index];

            while (node.parent != null)
            {
                // Reverse node path components
                node.path_x.Reverse();
                node.path_y.Reverse();
                node.path_yaw.Reverse();

                for (int i = 0; i < node.path_yaw.Count; i++)
                {
                    path.Add(new List<double> {
                            node.path_x[i],
                            node.path_y[i],
                            node.path_yaw[i]
                        });
                }

                node = node.parent;
            }

            path.Add(new List<double> { this.start.x, this.start.y, this.start.yaw });
        }
        catch (Exception e) { Debug.LogError(e); }

        return path;
    }


    private void plotPath(List<double> xPath, List<double> zPath)
    {
        GameObject pathParent = new GameObject("Path Parent");
        for(int i=0; i < xPath.Count; i++)
        {
            GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obj.transform.position = new Vector3((float)xPath[i], 0, (float)zPath[i]);
            obj.transform.parent = pathParent.transform;
            obj.transform.localScale = new Vector3(.5f, .5f, .5f);
        }
    }
    private void plotPath(List<List<double>> path)
    {
        GameObject pathParent = new GameObject("Path Parent");
        for (int i = 0; i < path.Count; i++)
        {
            GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obj.transform.position = new Vector3((float)path[i][0], 0, (float)path[i][1]);
            obj.transform.parent = pathParent.transform;
            obj.transform.localScale = new Vector3(.5f, .5f, .5f);
        }
    }
    public void drawPath()
    {
        GameObject go = new GameObject();
        LineRenderer lineRenderer = go.AddComponent<LineRenderer>();
        List<Vector3> nodePath = new List<Vector3>();
        Vector3 pos = Vector3.zero;
        foreach (var node in this.node_list.ToList())
        {
            if (node.parent != null)
            {
                for (int i=0; i < node.path_x.Count; i++)
                {
                    pos.Set((float)node.path_x[i], 0, (float)node.path_y[i]);
                    nodePath.Add(pos);
                }
            }
        }
        lineRenderer.positionCount = nodePath.Count;
        lineRenderer.SetPositions(nodePath.ToArray());
        lineRenderer.widthMultiplier = 0.1f;
    }

}
