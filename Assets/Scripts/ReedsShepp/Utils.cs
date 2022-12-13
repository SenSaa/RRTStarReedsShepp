using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using Newtonsoft.Json;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;

namespace UtilityFunctions
{

    public static class Utils
    {

        // Return a float with the magnitude (absolute value) of x but the sign of y.
        public static double CopySign(double x, double y)
        {
            if (y == 0) { return Math.Abs(x); }
            return Math.Abs(x) * Math.Sign(y);
        }

        // Zip three collections/lists.
        public static IEnumerable<TResult> ZipThree<T1, T2, T3, TResult>(
            this IEnumerable<T1> source,
            IEnumerable<T2> second,
            IEnumerable<T3> third,
            Func<T1, T2, T3, TResult> func)
        {
            using (var e1 = source.GetEnumerator())
            using (var e2 = second.GetEnumerator())
            using (var e3 = third.GetEnumerator())
            {
                while (e1.MoveNext() && e2.MoveNext() && e3.MoveNext())
                    yield return func(e1.Current, e2.Current, e3.Current);
            }
        }

        // Tranform RHS coordinate system (commonly used) to LHS (Unity ver).
        // By negating the yaw!
        // Then add 90. <- Probably due to the z-up in Unity in contrast to y-up used commonly.
        public static float transformYaw(double yaw)
        {
            float transformedYaw;
            transformedYaw = (float)-yaw + 90;
            return transformedYaw;
        }

        public static T DeepClone<T>(this T obj)
        {
            using (var ms = new MemoryStream())
            {
                var formatter = new BinaryFormatter();
                formatter.Serialize(ms, obj);
                ms.Position = 0;

                return (T)formatter.Deserialize(ms);
            }
        }

        public static string ListToString<T>(List<T> list)
        {
            string result = "";
            for (int i = 0; i < list.Count; i++)
            {
                result += list[i] + "   ";
            }
            return result;
        }

    }

}
