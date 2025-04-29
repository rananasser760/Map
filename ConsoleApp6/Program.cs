using System;
using System.Collections.Generic;
using System.IO;
using System.Globalization;
using System.Diagnostics;

namespace MapRouting
{
    // 1) Basic types
    public class Node
    {
        public int Id;
        public double X, Y;
        public List<Edge> Neighbors = new List<Edge>();
    }

    public class Edge
    {
        public Node Target;
        public double LengthKm;    // in km
        public double SpeedKph;    // in km/h
        public double TimeHours => LengthKm / SpeedKph;
    }

    // 2) Graph loader
    public class Graph
    {
        public Dictionary<int, Node> Nodes = new Dictionary<int, Node>();

        public static Graph LoadFromFile(string mapPath)
        {
            var graph = new Graph();
            var lines = File.ReadAllLines(mapPath);
            int idx = 0;
            int n = int.Parse(lines[idx++]);
            // read nodes
            for (int i = 0; i < n; i++)
            {
                var parts = lines[idx++].Split();
                int id = int.Parse(parts[0]);
                double x = double.Parse(parts[1], CultureInfo.InvariantCulture);
                double y = double.Parse(parts[2], CultureInfo.InvariantCulture);
                graph.Nodes[id] = new Node { Id = id, X = x, Y = y };
            }
            int m = int.Parse(lines[idx++]);
            // read edges
            for (int i = 0; i < m; i++)
            {
                var p = lines[idx++].Split();
                int u = int.Parse(p[0]), v = int.Parse(p[1]);
                double len = double.Parse(p[2], CultureInfo.InvariantCulture);
                double spd = double.Parse(p[3], CultureInfo.InvariantCulture);
                var nu = graph.Nodes[u];
                var nv = graph.Nodes[v];
                nu.Neighbors.Add(new Edge { Target = nv, LengthKm = len, SpeedKph = spd });
                nv.Neighbors.Add(new Edge { Target = nu, LengthKm = len, SpeedKph = spd });
            }
            return graph;
        }
    }

    // 3) Query processing & main
    class Program
    {
        // walking speed constant
        const double WalkKph = 5.0;

        static void Main(string[] args)
        {
            if (args.Length < 2)
            {
                Console.WriteLine("Usage: MapRouting.exe <mapFile> <queryFile>");
                return;
            }
            var graph = Graph.LoadFromFile(args[0]);

            // TODO: load queries, for each:
            // 1. Find startCandidates = all nodes with euclidDist(src, node) <= R/1000
            // 2. Find endCandidates similarly
            // 3. Build a priority-queue Dijkstra over a virtual graph:
            //    – from src, can “walk” to each start node at time = dist/5
            //    – all road-edges as defined
            //    – from each end node, “walk” to dest.
            // 4. Reconstruct path & compute metrics
        }

        static double EuclidDistKm(double x1, double y1, double x2, double y2)
        {
            double dx = x1 - x2, dy = y1 - y2;
            return Math.Sqrt(dx * dx + dy * dy);
        }
    }
}
