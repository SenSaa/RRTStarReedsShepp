using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace Visualisation
{
    public class RenderObstacles
    {
        public void Draw(HashSet<Tuple<int, int>> obstacles, Transform transform, Material material)
        {
            List<Vector3> obstaclesPositions = processPoints(obstacles);
            renderObstacles(obstaclesPositions, transform, material);
        }

        private List<Vector3> processPoints(HashSet<Tuple<int, int>> obstacles)
        {
            List<Vector3> obstaclesPositions = new List<Vector3>();
            foreach (var obstacle in obstacles)
            {
                obstaclesPositions.Add(new Vector3(obstacle.Item1, 0, obstacle.Item2));
            }
            return obstaclesPositions;
        }

        private void renderObstacles(List<Vector3> obstaclePositions, Transform transform, Material material)
        {
            GameObject obstGO = new GameObject("Obstacles");
            obstGO.transform.parent = transform;
            foreach (var obstPos in obstaclePositions)
            {
                GameObject pointRenderGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
                pointRenderGo.transform.position = obstPos;
                pointRenderGo.transform.parent = obstGO.transform;
                pointRenderGo.transform.localScale = new Vector3(1f, 0.1f, 1f);
                pointRenderGo.name = "Obstacle";
                pointRenderGo.GetComponent<MeshRenderer>().material = material;
            }
        }
    }
}