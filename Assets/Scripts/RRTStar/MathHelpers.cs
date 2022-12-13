using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class MathHelpers
{

    // Find hypotenuse.
    // List<double> as Input and Output.
    public static List<double> Hypotenuse(List<double> side1, List<double> side2)
    {
        List<double> hypot = new List<double>();
        for (int i = 0; i < side1.Count; i++)
        {
            hypot.Add(Math.Sqrt(Math.Pow(side1[i], 2) + Math.Pow(side2[i], 2)));
        }
        return hypot;
    }

    // double as Input & Output. 
    public static double Hypotenuse(double side1, double side2)
    {
        double hypot = 0;
        {
            hypot = Math.Sqrt(Math.Pow(side1, 2) + Math.Pow(side2, 2));
        }
        return hypot;
    }


    // Find Cumulative Sum of a set.
    public static List<double> cumulativeSum(List<double> sequence)
    {
        double sum = 0;
        List<double> cumSumList = new List<double>();
        foreach (var item in sequence)
        {
            sum += item;
            cumSumList.Add(sum);
        }
        return cumSumList;
    }

    // Transform 1D -> 2D Array.
    public static double[,] Make2DArray(List<double> input, int rows, int columns)
    {
        double[,] output = new double[rows, columns];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < columns; j++)
            {
                output[i, j] = input[i * columns + j];
            }
        }
        return output;
    }

    // https://www.techiedelight.com/find-first-or-last-occurrence-of-a-given-number-sorted-array/
    // Function to find the first occurrence of a given number
    // in a sorted integer array
    public static int findFirstOccurrence(List<double> A, double x)
    {
        // search space is `A[left…right]`
        int left = 0;
        int right = A.Count - 1;

        // initialize the result by -1
        int result = -1;

        // loop till the search space is exhausted
        while (left <= right)
        {
            // find the mid-value in the search space and compares it with the target
            int mid = (left + right) / 2;

            // if the key is located, update the result and
            // search towards the left (lower indices)
            if (x == A[mid])
            {
                result = mid;
                right = mid - 1;
            }

            // if the key is less than the middle element, discard the right half
            else if (x < A[mid])
            {
                right = mid - 1;
            }

            // if the key is more than the middle element, discard the left half
            else
            {
                left = mid + 1;
            }
        }

        // return the leftmost index, or -1 if the element is not found
        return result;
    }


    // https://www.techiedelight.com/find-first-or-last-occurrence-of-a-given-number-sorted-array/
    // Function to find the last occurrence of a given number
    // in a sorted integer array
    public static int findLastOccurrence(List<double> A, double x)
    {
        // search space is `A[left…right]`
        int left = 0;
        int right = A.Count - 1;

        // initialize the result by -1
        int result = -1;

        // loop till the search space is exhausted
        while (left <= right)
        {
            // find the mid-value in the search space and compares it with the target
            int mid = (left + right) / 2;

            // if the key is located, update the result and
            // search towards the right (higher indices)
            if (x == A[mid])
            {
                result = mid;
                left = mid + 1;
            }

            // if the key is less than the middle element, discard the right half
            else if (x < A[mid])
            {
                right = mid - 1;
            }

            // if the key is more than the middle element, discard the left half
            else
            {
                left = mid + 1;
            }
        }

        // return the leftmost index, or -1 if the element is not found
        return result;
    }

    // https://stackoverflow.com/questions/2260272/how-to-find-the-last-element-of-array-in-binary-search
    // Binary search to find the rightmost index for a given value in a set.
    public static int binsearch(List<double> a, double val, int left, int right)
    {
        if (left == right) return left;
        var mid = (left + right) / 2;
        if (a[mid] < val)
            return binsearch(a, val, mid + 1, right);
        else
            return binsearch(a, val, left, mid);
    }

    public static double getRandomNumber(double minimum, double maximum)
    {
        System.Random random = new System.Random();
        return random.NextDouble() * (maximum - minimum) + minimum;
    }

}
