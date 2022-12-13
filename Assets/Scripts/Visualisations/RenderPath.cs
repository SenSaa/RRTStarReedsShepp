using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace Visualisation
{
    public class RenderPath
    {
        public void Draw(List<Tuple<double, double>> points, Transform transform, Material pathMaterial, Material pathPointsMaterial)
        {
            List<Vector3> path = processPoints(points);
            renderPathLine(path, transform, pathMaterial);
        }


        private List<Vector3> processPoints(List<Tuple<double, double>> points)
        {
            List<Vector3> pathPoints = new List<Vector3>();
            foreach (var point in points)
            {
                pathPoints.Add(new Vector3((float)point.Item1, 0, (float)point.Item2));
            }
            return pathPoints;
        }

        private void renderPathLine(List<Vector3> points, Transform transform, Material material)
        {
            GameObject pathLineGO = new GameObject("PathLine");
            pathLineGO.transform.parent = transform;
            LineRenderer pathLineRender = pathLineGO.AddComponent<LineRenderer>();
            pathLineRender.positionCount = points.Count;
            pathLineRender.SetPositions(points.ToArray());
            pathLineRender.widthMultiplier = 0.25f;
            pathLineRender.material = material;
        }
        private void renderPathPoints(List<Vector3> points, Transform transform, Material material)
        {
            GameObject pathPtsGO = new GameObject("PathPoints");
            pathPtsGO.transform.parent = transform;
            foreach (var pt in points)
            {
                GameObject pointRenderGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                pointRenderGo.transform.position = pt;
                pointRenderGo.transform.parent = pathPtsGO.transform;
                pointRenderGo.transform.localScale = new Vector3(0.75f, 0.75f, 0.75f);
                pointRenderGo.name = "State";
                pointRenderGo.GetComponent<MeshRenderer>().material = material;
                pointRenderGo.GetComponent<Collider>().enabled = false;
            }
        }

    }
}
