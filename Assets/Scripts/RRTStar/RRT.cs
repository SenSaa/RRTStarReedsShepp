using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using ReedsShepp;
using UtilityFunctions;

// Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)
// author: AtsushiSakai(@Atsushi_twi)
// https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/RRT/rrt.py

namespace rrt
{
   
    public class RRT
    {

        // Setting Parameter
        // start:Start Position[x, y]
        // goal:Goal Position[x, y]
        // obstacleList:obstacle Positions[[x, y, size],...]
        // randArea:Random Sampling Area[min, max]

        public Node start;
        Node end;
        int min_rand;
        int max_rand;
        public double expand_dis;
        double path_resolution;
        int goal_sample_rate;
        public int max_iter;
        public List<Obstacle> obstacle_list;
        public List<Node> node_list;

        public RRT(double[] start, double[] goal, List<Obstacle> obstacle_list, int[] rand_area, double expand_dis = 2.0, double path_resolution = 1, int goal_sample_rate = 5, int max_iter = 2)
        {
            this.start = new Node(start[0], start[1]);
            this.end = new Node(goal[0], goal[1]);
            this.min_rand = rand_area[0];
            this.max_rand = rand_area[1];
            this.expand_dis = expand_dis;
            this.path_resolution = path_resolution;
            this.goal_sample_rate = goal_sample_rate;
            this.max_iter = max_iter;
            this.obstacle_list = obstacle_list;
            this.node_list = new List<Node>();
        }


        public List<Node> planning(bool animation = true)
        {
            Debug.Log("RRT");

            node_list.Add(start);

            for (int i = 0; i < max_iter; i++)
            {
                Node rnd_node = getRandomNode();
                int nearest_ind = getNearestNodeIndex(node_list, rnd_node);
                Node nearest_node = node_list[nearest_ind];

                Node new_node = steer(nearest_node, rnd_node, expand_dis);

                if (checkCollision(new_node, obstacle_list))
                {
                    node_list.Add(new_node);
                }

                if (calculateDistToGoal(node_list[node_list .Count - 1].x, node_list[node_list.Count - 1].y) <= expand_dis)
                {
                    Node final_node = steer(node_list[node_list.Count - 1], end, expand_dis);
                    if (checkCollision(final_node, obstacle_list))
                    {
                        return generateFinalCourse((node_list).Count - 1);
                    }
                }
            }
            return null;  // cannot find path
        }


        // *** The "steer()" method is virtual so that it could be overridden by RRT*RS's steer() function.
        public virtual Node steer(Node from_node, Node to_node, double extend_length = double.PositiveInfinity)
        {
            Node new_node = new Node(from_node.x, from_node.y);
            Tuple<double, double> d_theta = calculateDistAndAngle(new_node, to_node);

            new_node.path_x.Add(new_node.x);
            new_node.path_y.Add(new_node.y);

            if (extend_length > d_theta.Item1)
            {
                extend_length = d_theta.Item1;
            }

            double n_expand = Math.Floor(extend_length / path_resolution);

            for (int i = 0; i < n_expand; i++)
            {
                new_node.x += path_resolution * Math.Cos(d_theta.Item2);
                new_node.y += path_resolution * Math.Sin(d_theta.Item2);
                new_node.path_x.Add(new_node.x);
                new_node.path_y.Add(new_node.y);
            }

            Tuple<double, double> dist_angle = calculateDistAndAngle(new_node, to_node);
            if (dist_angle.Item1 <= path_resolution)
            {
                new_node.path_x.Add(to_node.x);
                new_node.path_y.Add(to_node.y);
                
                new_node.x = to_node.x;
                new_node.y = to_node.y;
            }

            new_node.parent = from_node;

            return new_node;
        }

        public List<Node> generateFinalCourse(int goal_ind)
        {
            List<Node> path = new List<Node>();
            path.Add(new Node(end.x, end.y)); ////////////////////////////////////////////////////////////////////
            Node node = node_list[goal_ind];
            while (node.parent != null)
            {
                path.Add(new Node(node.x, node.y));
                node = node.parent;
            }
            path.Add(new Node(node.x, node.y));

            //drawPath(path);

            return path;
        }


        public double calculateDistToGoal(double x, double y)
        {
            double dx = x - end.x;
            double dy = y - end.y;
            return MathHelpers.Hypotenuse(dx, dy);
        }


        public Node getRandomNode()
        {
            Node rnd = null;
            if (UnityEngine.Random.Range(0, 100) > goal_sample_rate)
            {
                double randX = UnityEngine.Random.Range(min_rand, max_rand);
                double randY = UnityEngine.Random.Range(min_rand, max_rand);
                rnd = new Node(randX, randY);
            }
            else // goal point sampling
            {
                rnd = new Node(end.x, end.y);
            }
            return rnd;
        }


        public static int getNearestNodeIndex(List<Node> node_list, Node rnd_node)
        {
            List<double> dlist = new List<double>();
            foreach (Node node in node_list)
            {
                dlist.Add(Math.Pow((node.x - rnd_node.x), 2) + Math.Pow((node.y - rnd_node.y), 2));
            }
            int minind = dlist.IndexOf(dlist.Min());
            return minind;
        }


        public static bool checkCollision(Node node, List<Obstacle> obstacleList)
        {
            if (node == null)
            {
                return false;
            }

            List<double> dx_list = new List<double>();
            List<double> dy_list = new List<double>();
            List<double> d_list = new List<double>();

            foreach (Obstacle obstacle in obstacleList)
            {
                foreach (double x in node.path_x)
                {
                    double ox = obstacle.x;
                    dx_list.Add(ox - x);
                }

                foreach (double y in node.path_y)
                {
                    double oy = obstacle.y;
                    dy_list.Add(oy - y);
                }

                var dx_list_dy_list = dx_list.Zip(dy_list, (x, y) => new { dx = x, dy = y });
                foreach (var dx_dy in dx_list_dy_list)
                {
                    d_list.Add(dx_dy.dx * dx_dy.dx + dx_dy.dy * dx_dy.dy);
                }

                if (d_list.Min() <= Math.Pow(obstacle.size, 2))
                {
                    return false;  // collision
                }
            }
            return true;  // safe
        }


        public static Tuple<double, double> calculateDistAndAngle(Node from_node, Node to_node)
        {
            double dx = to_node.x - from_node.x;
            double dy = to_node.y - from_node.y;
            double d = MathHelpers.Hypotenuse(dx, dy);
            double theta = Math.Atan2(dy, dx);
            Tuple<double, double> Dist_Angle = Tuple.Create(d, theta);
            return Dist_Angle;
        }


        public void drawNode(Node node)
        {
            Vector3 nodePos = new Vector3((float)node.x, 0, (float)node.y);
            VisualisationHelper.DrawPoint(nodePos, 0.5f);
        }

        private void drawPath(List<Node> nodes)
        {
            List<Vector3> pathNodePositions = new List<Vector3>();
            for (int i=0; i < nodes.Count; i++)
            {
                pathNodePositions.Add(new Vector3((float)nodes[i].x, 0, (float)nodes[i].y));
            }
            VisualisationHelper.DrawLine(pathNodePositions, Color.blue);
        }

        public void drawTreeLines(List<Node> node_list)
        {
            foreach (Node node in node_list)
            //for (int i=1; i < node_list.Count; i++)
            {
                if (node.parent != null)
                //if (node_list[i].parent != null)
                {
                    for (int i = 1; i < node.path_x.Count; i++)
                    {
                        Vector3 from = new Vector3((float)node.path_x[i-1], 0, (float)node.path_y[i-1]);
                        Vector3 to = new Vector3((float)node.path_x[i], 0, (float)node.path_y[i]);
                        VisualisationHelper.DrawLine(from, to, Color.yellow);
                    }
                }
            }
        }

    }


    /// <summary>
    /// New class type to use for obstacle x, y, size.
    /// </summary>
    public class Obstacle
    {
        public double x;
        public double y;
        public double size;

        public Obstacle(double x, double y, double size)
        {
            this.x = x;
            this.y = y;
            this.size = size;
        }
    }


}
