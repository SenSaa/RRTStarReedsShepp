using System;
using System.Collections;
using System.Collections.Generic;

namespace rrt
{
    // RRT Node

    [Serializable]
    public class Node
    {
        public double x;
        public double y;
        public List<double> path_x;
        public List<double> path_y;
        public Node parent;
        public double cost;

        public double yaw;
        public List<double> path_yaw;

        public Node(double x, double y, double yaw = 0)
        {
            this.x = x;
            this.y = y;
            path_x = new List<double>();
            path_y = new List<double>();
            parent = null;
            this.cost = 0.0;

            this.yaw = yaw;
            path_yaw = new List<double>();
        }
        

    }
}