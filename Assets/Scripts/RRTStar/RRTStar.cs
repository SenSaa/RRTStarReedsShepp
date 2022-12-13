// Path planning Sample Code with RRT*
// author: Atsushi Sakai(@Atsushi_twi)
// https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/RRTStar/rrt_star.py

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

namespace rrt
{
    public class RRTStar : RRT
    {

        private double connect_circle_dist;
        private Node goal_node;
        private bool search_until_max_iter;

        public RRTStar(
            double[] start,
            double[] goal,
            List<Obstacle> obstacle_list,
            int[] rand_area,
            double expand_dis = 30.0,
            double path_resolution = 1.0,
            int goal_sample_rate = 20,
            int max_iter = 300,
            double connect_circle_dist = 50.0,
            bool search_until_max_iter = false)
            : base(start, goal, obstacle_list, rand_area, expand_dis, path_resolution, goal_sample_rate, max_iter)
        {
            this.connect_circle_dist = connect_circle_dist;
            this.goal_node = new Node(goal[0], goal[1]);
            this.search_until_max_iter = search_until_max_iter;
        }

        // RRT8 path Planning
        public List<Node> planning(bool animation = true)
        {
            Debug.Log("RRT*");

            int last_index;
            this.node_list = new List<Node> {
                    this.start
                };
            foreach (var i in Enumerable.Range(0, max_iter))
            {
                Debug.Log("Iter:" + i + " , number of nodes: " + this.node_list.Count);
                var rnd = this.getRandomNode();
                var nearest_ind = getNearestNodeIndex(this.node_list, rnd);
                var new_node = this.steer(this.node_list[nearest_ind], rnd, this.expand_dis);
                //var new_node = this.steer(this.node_list[nearest_ind], rnd);
                var near_node = this.node_list[nearest_ind];
                new_node.cost = near_node.cost + MathHelpers.Hypotenuse(new_node.x - near_node.x, new_node.y - near_node.y);
                if (checkCollision(new_node, this.obstacle_list))
                {
                    var near_inds = this.find_near_nodes(new_node);
                    var node_with_updated_parent = this.choose_parent(new_node, near_inds);
                    if (node_with_updated_parent != null)
                    {
                        this.rewire(node_with_updated_parent, near_inds);
                        this.node_list.Add(node_with_updated_parent);
                    }
                    else
                    {
                        this.node_list.Add(new_node);
                    }
                }
                if (animation)
                {
                    drawNode(new_node);
                    drawTreeLines(node_list);
                }
                if (!search_until_max_iter && new_node != null)
                {
                    // if reaches goal
                    last_index = search_best_goal_node();
                    if (last_index != int.MaxValue)
                    {
                        return generateFinalCourse(last_index);
                    }
                }
            }
            Debug.Log("reached max iteration");
            last_index = search_best_goal_node();
            if (last_index != int.MaxValue)
            {
                return generateFinalCourse(last_index);
            }
            return null;
        }

        //         Computes the cheapest point to new_node contained in the list
        //         near_inds and set such a node as the parent of new_node.
        //             Arguments:
        //             --------
        //                 new_node, Node
        //                     randomly generated node with a path from its neared point
        //                     There are not coalitions between this node and th tree.
        //                 near_inds: list
        //                     Indices of indices of the nodes what are near to new_node
        // 
        //             Returns.
        //             ------
        //                 Node, a copy of new_node
        public Node choose_parent(Node new_node, List<int> near_inds)
        {
            if (near_inds.Count == 0 || near_inds == null)
            {
                return null;
            }
            // search nearest cost in near_inds
            var costs = new List<double>();
            foreach (var i in near_inds)
            {
                var near_node = this.node_list[i];
                var t_node = this.steer(near_node, new_node);
                if (t_node != null && checkCollision(t_node, obstacle_list))
                {
                    costs.Add(this.calc_new_cost(near_node, new_node));
                }
                else
                {
                    costs.Add(double.PositiveInfinity);
                }
            }
            var min_cost = costs.Min();
            if (min_cost == double.PositiveInfinity)
            {
                Debug.Log("There is no good path.(min_cost is inf)");
                return null;
            }
            var min_ind = near_inds[costs.IndexOf(min_cost)];
            new_node = this.steer(this.node_list[min_ind], new_node);
            new_node.cost = min_cost;
            return new_node;
        }

        private int search_best_goal_node()
        {
            var dist_to_goal_list = (from n in this.node_list
                                     select this.calculateDistToGoal(n.x, n.y)).ToList();
            var goal_inds = (from i in dist_to_goal_list
                             where i <= this.expand_dis
                             select dist_to_goal_list.IndexOf(i)).ToList();
            var safe_goal_inds = new List<int>();
            foreach (var goal_ind in goal_inds)
            {
                var t_node = this.steer(this.node_list[goal_ind], this.goal_node);
                if (checkCollision(t_node, this.obstacle_list))
                {
                    safe_goal_inds.Add(goal_ind);
                }
            }
            if (safe_goal_inds.Count == 0 || safe_goal_inds == null)
            {
                return int.MaxValue;
            }

            var min_cost = double.MaxValue;
            var previousCost = double.MaxValue;
            foreach (var i in safe_goal_inds)
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

            foreach (var i in safe_goal_inds)
            {
                if (node_list[i].cost == min_cost)
                {
                    return i;
                }
            }

            return int.MaxValue;
        }

        //         1) defines a ball centered on new_node
        //         2) Returns all nodes of the three that are inside this ball
        //             Arguments:
        //             ---------
        //                 new_node: Node
        //                     new randomly generated node, without collisions between
        //                     its nearest node
        //             Returns:
        //             -------
        //                 list
        //                     List with the indices of the nodes inside the ball of
        //                     radius r
        public List<int> find_near_nodes(Node new_node)
        {
            var nnode = this.node_list.Count + 1;
            var r = this.connect_circle_dist * Math.Sqrt(Math.Log(nnode) / nnode);
            // if expand_dist exists, search vertices in a range no more than expand_dist
            r = Math.Min(r, expand_dis);

            var dist_list = (from node in node_list
                             select (Math.Pow(node.x - new_node.x, 2) + Math.Pow(node.y - new_node.y, 2))).ToList();

            var near_inds = (from i in dist_list
                             where i <= Math.Pow(r, 2)
                             select dist_list.IndexOf(i)).ToList();
            return near_inds;
        }

        //             For each node in near_inds, this will check if it is cheaper to
        //             arrive to them from new_node.
        //             In such a case, this will re-assign the parent of the nodes in
        //             near_inds to new_node.
        //             Parameters:
        //             ----------
        //                 new_node, Node
        //                     Node randomly added which can be joined to the tree
        // 
        //                 near_inds, list of uints
        //                     A list of indices of the self.new_node which contains
        //                     nodes within a circle of a given radius.
        //             Remark: parent is designated in choose_parent.
        public void rewire(Node new_node, List<int> near_inds)
        {
            foreach (var i in near_inds)
            {
                var near_node = this.node_list[i];
                var edge_node = steer(new_node, near_node);
                if (edge_node == null)
                {
                    continue;
                }
                edge_node.cost = this.calc_new_cost(new_node, near_node);
                var no_collision = checkCollision(edge_node, obstacle_list);
                var improved_cost = near_node.cost > edge_node.cost;
                if (no_collision && improved_cost)
                {
                    near_node.x = edge_node.x;
                    near_node.y = edge_node.y;
                    near_node.cost = edge_node.cost;
                    near_node.path_x = edge_node.path_x;
                    near_node.path_y = edge_node.path_y;
                    near_node.parent = edge_node.parent;

                    this.propagate_cost_to_leaves(new_node);
                }
            }
        }

        private double calc_new_cost(Node from_node, Node to_node)
        {
            var _tup_1 = calculateDistAndAngle(from_node, to_node);
            var d = _tup_1.Item1;
            return from_node.cost + d;
        }

        private void propagate_cost_to_leaves(Node parent_node)
        {
            foreach (var node in this.node_list)
            {
                if (node.parent == parent_node)
                {
                    node.cost = this.calc_new_cost(parent_node, node);
                    this.propagate_cost_to_leaves(node);
                }
            }
        }
    }
}