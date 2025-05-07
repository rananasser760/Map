using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading.Tasks;

namespace NavigationSystem
{
    class Program
    {
        static Dictionary<int, (double x, double y)> intersections = new Dictionary<int, (double x, double y)>();
        static Dictionary<int, List<(int neighbor, double time)>> graph = new Dictionary<int, List<(int, double)>>();
        static List<(int id1, int id2, double length, double speed, double time)> edges = new List<(int, int, double, double, double)>();

        const double WALKING_SPEED = 5.0; // km/h
        const double INFINITY = double.MaxValue;

        // Virtual nodes
        const int SUPER_SOURCE = -1;
        const int SUPER_TARGET = -2;

        // Spatial indexing grid
        static Dictionary<(int, int), List<int>> spatialGrid = new Dictionary<(int, int), List<int>>();
        static double gridCellSize = 0.1; // km - adjustable based on map density
        static double mapMinX, mapMaxX, mapMinY, mapMaxY;

        static void Main(string[] args)
        {
            Console.WriteLine("Navigation System - Test Case Selection");
            Console.WriteLine("--------------------------------------");
            Console.WriteLine("1: Sample Cases (Map*)");
            Console.WriteLine("2: Medium Cases (OL Map)");
            Console.WriteLine("3: Large Cases (SF Map)");
            Console.Write("\nEnter test case number (1-3): ");

            int caseNumber;
            if (!int.TryParse(Console.ReadLine(), out caseNumber) || caseNumber < 1 || caseNumber > 3)
            {
                Console.WriteLine("Invalid input. Please enter a number between 1 and 3.");
                Console.WriteLine("Press any key to exit...");
                Console.ReadKey();
                return;
            }

            string projectRoot = Directory.GetCurrentDirectory();
            string caseFolder = "";
            string inputFile = "";
            string queryFile = "";
            string outputFile = "";

            switch (caseNumber)
            {
                case 1:
                    ProcessSampleCases(projectRoot);
                    break;
                case 2:
                    caseFolder = Path.Combine(projectRoot, "[2] Medium Cases");
                    inputFile = Path.Combine(caseFolder, "Input", "OLMap.txt");
                    queryFile = Path.Combine(caseFolder, "Input", "OLQueries.txt");
                    outputFile = Path.Combine(caseFolder, "Output", "OLOutput.txt");
                    ProcessSingleCase(inputFile, queryFile, outputFile);
                    break;
                case 3:
                    caseFolder = Path.Combine(projectRoot, "[3] Large Cases");
                    inputFile = Path.Combine(caseFolder, "Input", "SFMap.txt");
                    queryFile = Path.Combine(caseFolder, "Input", "SFQueries.txt");
                    outputFile = Path.Combine(caseFolder, "Output", "SFOutput.txt");
                    ProcessSingleCase(inputFile, queryFile, outputFile);
                    break;
            }

            Console.WriteLine("\nProcessing complete!");
            Console.WriteLine("Press any key to exit...");
            Console.ReadKey();
        }

        static void ProcessSampleCases(string projectRoot)
        {
            string inputFolder = Path.Combine(projectRoot, "[1] Sample Cases", "Input");
            string outputFolder = Path.Combine(projectRoot, "[1] Sample Cases", "Output");

            Console.WriteLine($"\nLooking for input files in: {inputFolder}");

            if (!Directory.Exists(inputFolder))
            {
                Console.WriteLine($"Input directory not found. Creating: {inputFolder}");
                Directory.CreateDirectory(inputFolder);
                Console.WriteLine("Please place your map and query files in the input directory and run the program again.");
                return;
            }

            if (!Directory.Exists(outputFolder))
            {
                Console.WriteLine($"Output directory not found. Creating: {outputFolder}");
                Directory.CreateDirectory(outputFolder);
            }

            string[] mapFiles = Directory.GetFiles(inputFolder, "map*.txt");

            if (mapFiles.Length == 0)
            {
                Console.WriteLine("No map files found in the input directory.");
                return;
            }

            Console.WriteLine($"Found {mapFiles.Length} map files.");
            foreach (string mapFile in mapFiles)
            {
                try
                {
                    string mapFileName = Path.GetFileNameWithoutExtension(mapFile);

                    intersections.Clear();
                    graph.Clear();
                    edges.Clear();

                    Console.WriteLine($"Processing map: {mapFileName}");
                    ConstructGraph(mapFile);

                    string mapNumber = mapFileName.Substring(3); // Extract the number part (e.g., "1" from "map1")
                    string queryFileName = "queries" + mapNumber;
                    string queryFile = Path.Combine(inputFolder, queryFileName + ".txt");

                    if (File.Exists(queryFile))
                    {
                        string outputFileName = "output" + mapNumber;
                        string outputFile = Path.Combine(outputFolder, outputFileName + ".txt");

                        Directory.CreateDirectory(outputFolder);

                        Console.WriteLine($"Processing queries: {queryFileName}");
                        ProcessQueries(queryFile, outputFile);
                        Console.WriteLine($"Output written to: {outputFile}");
                    }
                    else
                    {
                        Console.WriteLine($"No query file found for map: {mapFileName}");
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error processing map file: {mapFile}, Error: {ex.Message}");
                }
            }
        }

        static void ProcessSingleCase(string mapFile, string queryFile, string outputFile)
        {
            try
            {
                // Ensure directories exist
                string inputDir = Path.GetDirectoryName(mapFile);
                string outputDir = Path.GetDirectoryName(outputFile);

                if (!Directory.Exists(inputDir))
                {
                    Console.WriteLine($"Input directory not found. Creating: {inputDir}");
                    Directory.CreateDirectory(inputDir);
                    Console.WriteLine("Please place your map and query files in the input directory and run the program again.");
                    return;
                }

                if (!Directory.Exists(outputDir))
                {
                    Console.WriteLine($"Output directory not found. Creating: {outputDir}");
                    Directory.CreateDirectory(outputDir);
                }

                // Check if the files exist
                if (!File.Exists(mapFile))
                {
                    Console.WriteLine($"Map file not found: {mapFile}");
                    return;
                }

                if (!File.Exists(queryFile))
                {
                    Console.WriteLine($"Query file not found: {queryFile}");
                    return;
                }

                // Clear previous data
                intersections.Clear();
                graph.Clear();
                edges.Clear();
                spatialGrid.Clear();

                // Process the case
                Console.WriteLine($"\nProcessing map: {Path.GetFileName(mapFile)}");
                ConstructGraph(mapFile);

                Console.WriteLine($"Processing queries: {Path.GetFileName(queryFile)}");
                ProcessQueries(queryFile, outputFile);
                Console.WriteLine($"Output written to: {outputFile}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error processing case: {ex.Message}");
            }
        }

        static void ConstructGraph(string mapFile)
        {
            Console.WriteLine($"Reading map file: {mapFile}");
            try
            {
                using (StreamReader reader = new StreamReader(mapFile))
                {
                    int N = int.Parse(reader.ReadLine());

                    // Pre-allocate capacity
                    intersections = new Dictionary<int, (double x, double y)>(N);
                    graph = new Dictionary<int, List<(int, double)>>(N);

                    // First pass: read all intersections
                    mapMinX = double.MaxValue;
                    mapMaxX = double.MinValue;
                    mapMinY = double.MaxValue;
                    mapMaxY = double.MinValue;

                    for (int i = 0; i < N; i++)
                    {
                        string[] parts = reader.ReadLine().Split();
                        int id = int.Parse(parts[0]);
                        double x = double.Parse(parts[1]);
                        double y = double.Parse(parts[2]);

                        intersections[id] = (x, y);
                        graph[id] = new List<(int, double)>();

                        // Update map boundaries for spatial indexing
                        mapMinX = Math.Min(mapMinX, x);
                        mapMaxX = Math.Max(mapMaxX, x);
                        mapMinY = Math.Min(mapMinY, y);
                        mapMaxY = Math.Max(mapMaxY, y);
                    }

                    // Create spatial grid indexing after knowing map boundaries
                    CreateSpatialIndex();

                    int M = int.Parse(reader.ReadLine());
                    edges = new List<(int id1, int id2, double length, double speed, double time)>(M);

                    for (int i = 0; i < M; i++)
                    {
                        string[] parts = reader.ReadLine().Split();
                        int id1 = int.Parse(parts[0]);
                        int id2 = int.Parse(parts[1]);
                        double length = double.Parse(parts[2]);
                        double speed = double.Parse(parts[3]);

                        double time = length / speed;

                        graph[id1].Add((id2, time));
                        graph[id2].Add((id1, time));

                        edges.Add((id1, id2, length, speed, time));
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error reading map file: {ex.Message}");
                throw; // Re-throw to be caught in the main method
            }
        }

        static void CreateSpatialIndex()
        {
            // Adjust grid cell size based on map size
            double mapWidth = mapMaxX - mapMinX;
            double mapHeight = mapMaxY - mapMinY;
            int intersectionCount = intersections.Count;

            // Heuristic for grid size based on map size and intersection density
            if (intersectionCount > 100000)
            {
                gridCellSize = Math.Min(mapWidth, mapHeight) / 200;
            }
            else if (intersectionCount > 10000)
            {
                gridCellSize = Math.Min(mapWidth, mapHeight) / 100;
            }
            else
            {
                gridCellSize = Math.Min(mapWidth, mapHeight) / 50;
            }

            // Ensure grid cell size is at least 0.01 km
            gridCellSize = Math.Max(gridCellSize, 0.01);

            // Create grid
            foreach (var kvp in intersections)
            {
                int id = kvp.Key;
                var (x, y) = kvp.Value;

                int gridX = (int)((x - mapMinX) / gridCellSize);
                int gridY = (int)((y - mapMinY) / gridCellSize);

                var gridCell = (gridX, gridY);

                if (!spatialGrid.ContainsKey(gridCell))
                {
                    spatialGrid[gridCell] = new List<int>();
                }

                spatialGrid[gridCell].Add(id);
            }
        }

        // Calculate Euclidean distance between two points (squared)
        static double CalculateDistanceSquared(double x1, double y1, double x2, double y2)
        {
            return Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2);
        }

        // Calculate Euclidean distance between two points
        static double CalculateDistance(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt(CalculateDistanceSquared(x1, y1, x2, y2));
        }

        static void ProcessQueries(string queryFile, string outputFile)
        {
            Stopwatch totalTimer = new Stopwatch();
            totalTimer.Start();

            long totalQueryTime = 0;

            // Pre-compute edge lengths for faster lookup
            Dictionary<(int, int), double> edgeLengths = new Dictionary<(int, int), double>();
            foreach (var edge in edges)
            {
                edgeLengths[(edge.id1, edge.id2)] = edge.length;
                edgeLengths[(edge.id2, edge.id1)] = edge.length; // Both directions
            }

            try
            {
                List<string[]> queries = new List<string[]>();
                int Q = 0;

                // Read all queries upfront
                using (StreamReader reader = new StreamReader(queryFile))
                {
                    Q = int.Parse(reader.ReadLine());
                    for (int i = 0; i < Q; i++)
                    {
                        queries.Add(reader.ReadLine().Split());
                    }
                }

                // Process queries and collect results
                List<string> results = new List<string>();

                // Process queries in parallel for large cases
                if (Q > 100 && intersections.Count > 10000)
                {
                    List<string> queryResults = new List<string>(new string[Q]);
                    Stopwatch parallelTimer = new Stopwatch(); // Timer for parallel block
                    parallelTimer.Start();
                    Parallel.For(0, Q, q =>
                    {
                        string[] parts = queries[q];
                        var result = ProcessSingleQuery(parts, edgeLengths);
                        lock (queryResults)
                        {
                            queryResults[q] = result.output;
                        }
                    });
                    parallelTimer.Stop();
                    totalQueryTime = parallelTimer.ElapsedMilliseconds; // Use parallel block time
                    results.AddRange(queryResults);
                }
                else
                {
                    // Process sequentially for smaller cases
                    for (int q = 0; q < Q; q++)
                    {
                        Stopwatch queryTimer = new Stopwatch();
                        queryTimer.Start();
                        string[] parts = queries[q];
                        var result = ProcessSingleQuery(parts, edgeLengths);
                        queryTimer.Stop();
                        results.Add(result.output);
                        totalQueryTime += queryTimer.ElapsedMilliseconds;
                    }
                }

                // Write results to output file
                using (StreamWriter writer = new StreamWriter(outputFile))
                {
                    foreach (var result in results)
                    {
                        writer.Write(result);
                    }
                    writer.WriteLine($"{totalQueryTime} ms");
                    writer.WriteLine();
                    totalTimer.Stop();
                    writer.WriteLine($"{totalTimer.ElapsedMilliseconds} ms");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error processing queries: {ex.Message}");
                try
                {
                    using (StreamWriter writer = new StreamWriter(outputFile))
                    {
                        writer.WriteLine($"Error: {ex.Message}");
                        writer.WriteLine();
                        totalTimer.Stop();
                        writer.WriteLine($"{totalTimer.ElapsedMilliseconds} ms");
                    }
                }
                catch
                {
                    Console.WriteLine("Failed to write error to output file.");
                }
            }
        }

        static (string output, double totalTime) ProcessSingleQuery(string[] parts, Dictionary<(int, int), double> edgeLengths)
        {
            double srcX = double.Parse(parts[0]);
            double srcY = double.Parse(parts[1]);
            double dstX = double.Parse(parts[2]);
            double dstY = double.Parse(parts[3]);
            double R = double.Parse(parts[4]);
            double RKm = R / 1000.0; // Convert to km

            // Find nearby intersections efficiently using spatial indexing
            List<(int id, double dist, double time)> S = FindNearbyIntersectionsGrid(srcX, srcY, RKm);
            List<(int id, double dist, double time)> F = FindNearbyIntersectionsGrid(dstX, dstY, RKm);

            double bestTime = INFINITY;
            List<int> bestPath = new List<int>();
            int bestS = -1, bestF = -1;
            double bestVehicleDist = 0;
            double bestSDist = 0, bestFDist = 0;

            // Early termination if no path possible
            if (S.Count > 0 && F.Count > 0)
            {
                // Using Dijkstra with super source
                var result = DijkstraWithSuperSource(S, F, edgeLengths);
                bestTime = result.bestTime;
                bestPath = result.bestPath;
                bestS = result.bestS;
                bestF = result.bestF;
                bestSDist = result.bestSDist;
                bestFDist = result.bestFDist;
                bestVehicleDist = result.bestVehicleDist;
            }

            // Format output
            string output;
            if (bestPath.Count > 0)
            {
                output = string.Join(" ", bestPath) + "\n" +
                        $"{(bestTime * 60):F2} mins\n" +
                        $"{(bestSDist + bestVehicleDist + bestFDist):F2} km\n" +
                        $"{(bestSDist + bestFDist):F2} km\n" +
                        $"{bestVehicleDist:F2} km\n\n";
            }
            else
            {
                output = "No valid path found.\n\n";
            }

            return (output, bestTime);
        }

        // Enhanced priority queue for Dijkstra
        class MinHeap
        {
            private struct HeapNode
            {
                public int NodeId { get; set; }
                public double Priority { get; set; }

                public HeapNode(int nodeId, double priority)
                {
                    NodeId = nodeId;
                    Priority = priority;
                }
            }

            private List<HeapNode> heap;
            private Dictionary<int, int> nodePosition;

            public MinHeap(int capacity)
            {
                heap = new List<HeapNode>(capacity);
                nodePosition = new Dictionary<int, int>(capacity);
            }

            public void Enqueue(int nodeId, double priority)
            {
                HeapNode node = new HeapNode(nodeId, priority);

                if (Contains(nodeId))
                {
                    // Update existing node if it has a higher priority
                    int pos = nodePosition[nodeId];
                    if (priority < heap[pos].Priority)
                    {
                        heap[pos] = node;
                        SiftUp(pos);
                    }
                }
                else
                {
                    // Add new node
                    heap.Add(node);
                    int pos = heap.Count - 1;
                    nodePosition[nodeId] = pos;
                    SiftUp(pos);
                }
            }

            public int Dequeue()
            {
                if (heap.Count == 0)
                    throw new InvalidOperationException("Heap is empty");

                HeapNode min = heap[0];
                nodePosition.Remove(min.NodeId);

                // Replace root with last element and sift down
                if (heap.Count > 1)
                {
                    heap[0] = heap[heap.Count - 1];
                    nodePosition[heap[0].NodeId] = 0;
                    heap.RemoveAt(heap.Count - 1);
                    SiftDown(0);
                }
                else
                {
                    heap.RemoveAt(0);
                }

                return min.NodeId;
            }

            public bool IsEmpty()
            {
                return heap.Count == 0;
            }

            public bool Contains(int nodeId)
            {
                return nodePosition.ContainsKey(nodeId);
            }

            private void SiftUp(int index)
            {
                int parent = (index - 1) / 2;

                while (index > 0 && heap[index].Priority < heap[parent].Priority)
                {
                    Swap(index, parent);
                    index = parent;
                    parent = (index - 1) / 2;
                }
            }

            private void SiftDown(int index)
            {
                int smallest = index;
                int left = 2 * index + 1;
                int right = 2 * index + 2;

                if (left < heap.Count && heap[left].Priority < heap[smallest].Priority)
                    smallest = left;

                if (right < heap.Count && heap[right].Priority < heap[smallest].Priority)
                    smallest = right;

                if (smallest != index)
                {
                    Swap(index, smallest);
                    SiftDown(smallest);
                }
            }

            private void Swap(int i, int j)
            {
                HeapNode temp = heap[i];
                heap[i] = heap[j];
                heap[j] = temp;

                nodePosition[heap[i].NodeId] = i;
                nodePosition[heap[j].NodeId] = j;
            }
        }

        // Find nearby intersections using the spatial grid
        static List<(int id, double dist, double time)> FindNearbyIntersectionsGrid(double x, double y, double radius)
        {
            var result = new List<(int id, double dist, double time)>();

            // Calculate grid coordinates for the query point
            int gridX = (int)((x - mapMinX) / gridCellSize);
            int gridY = (int)((y - mapMinY) / gridCellSize);

            // Calculate how many cells to check in each direction
            int cellRadius = (int)(radius / gridCellSize) + 1;

            double radiusSquared = radius * radius;

            // Check cells in the radius
            for (int i = -cellRadius; i <= cellRadius; i++)
            {
                for (int j = -cellRadius; j <= cellRadius; j++)
                {
                    var gridCell = (gridX + i, gridY + j);

                    if (spatialGrid.TryGetValue(gridCell, out var nodesInCell))
                    {
                        foreach (int id in nodesInCell)
                        {
                            var (nodeX, nodeY) = intersections[id];
                            double distSquared = CalculateDistanceSquared(x, y, nodeX, nodeY);

                            if (distSquared <= radiusSquared)
                            {
                                double dist = Math.Sqrt(distSquared);
                                double time = dist / WALKING_SPEED;
                                result.Add((id, dist, time));
                            }
                        }
                    }
                }
            }

            return result;
        }

        static (double bestTime, List<int> bestPath, int bestS, int bestF, double bestSDist, double bestFDist, double bestVehicleDist)
        DijkstraWithSuperSource(
            List<(int id, double dist, double time)> S,
            List<(int id, double dist, double time)> F,
            Dictionary<(int, int), double> edgeLengths)
        {
            double bestTime = INFINITY;
            List<int> bestPath = new List<int>();
            int bestS = -1, bestF = -1;
            double bestSDist = 0, bestFDist = 0, bestVehicleDist = 0;

            // Create a temporary graph with super source
            var tempGraph = new Dictionary<int, List<(int neighbor, double time)>>(graph);

            // Add the super source node
            tempGraph[SUPER_SOURCE] = new List<(int neighbor, double time)>();

            // Connect super source to all starting points
            foreach (var start in S)
            {
                tempGraph[SUPER_SOURCE].Add((start.id, start.time));
            }

            // Create a set of all target nodes for easy lookup
            HashSet<int> targetNodes = new HashSet<int>(F.Select(f => f.id));

            // Create dictionaries for finish point data lookup
            Dictionary<int, double> finishDistances = new Dictionary<int, double>();
            Dictionary<int, double> finishTimes = new Dictionary<int, double>();

            foreach (var finish in F)
            {
                finishDistances[finish.id] = finish.dist;
                finishTimes[finish.id] = finish.time;
            }

            // Estimated size for collections based on graph size
            int estimatedSize = Math.Min(intersections.Count + 2, 10000); // +2 for super source and possibly super target

            // Use efficient priority queue for Dijkstra
            var pq = new MinHeap(estimatedSize);
            var distances = new Dictionary<int, double>(estimatedSize);
            var parents = new Dictionary<int, int>(estimatedSize);
            var visited = new HashSet<int>(estimatedSize);
            var vehicleDist = new Dictionary<int, double>(estimatedSize);

            // Initialize
            distances[SUPER_SOURCE] = 0;
            vehicleDist[SUPER_SOURCE] = 0;
            pq.Enqueue(SUPER_SOURCE, 0);

            // Dijkstra's algorithm
            while (!pq.IsEmpty())
            {
                int currentId = pq.Dequeue();

                // Skip if already processed
                if (visited.Contains(currentId))
                    continue;

                visited.Add(currentId);

                // Check if we've reached a target
                if (targetNodes.Contains(currentId))
                {
                    double totalTime = distances[currentId] + finishTimes[currentId];

                    if (totalTime < bestTime)
                    {
                        bestTime = totalTime;
                        bestF = currentId;
                        bestFDist = finishDistances[currentId];

                        // Reconstruct path to find the start node
                        List<int> fullPath = new List<int>();
                        int current = currentId;

                        while (parents.ContainsKey(current))
                        {
                            int parent = parents[current];
                            if (parent == SUPER_SOURCE)
                            {
                                bestS = current; // This is the starting intersection
                                break;
                            }
                            fullPath.Add(current);
                            current = parent;
                        }

                        // Add starting node
                        fullPath.Add(bestS);
                        fullPath.Reverse();

                        // Store best path
                        bestPath = fullPath;

                        // Find the starting distance
                        bestSDist = S.First(s => s.id == bestS).dist;

                        // Calculate total vehicle distance
                        bestVehicleDist = vehicleDist[currentId];
                    }
                }

                // Don't explore further from the super source or any node with distance worse than best
                if (currentId != SUPER_SOURCE && distances[currentId] >= bestTime)
                    continue;

                // Explore neighbors
                var neighbors = tempGraph.GetValueOrDefault(currentId, new List<(int, double)>());
                foreach (var edge in neighbors)
                {
                    int neighborId = edge.Item1;
                    double edgeTime = edge.Item2;

                    if (!visited.Contains(neighborId))
                    {
                        double newDist = distances[currentId] + edgeTime;

                        // Early skip if this path can't be better than best found
                        if (newDist >= bestTime)
                            continue;

                        // Calculate vehicle distance (only for non-super-source transitions)
                        double newVehicleDist = vehicleDist.GetValueOrDefault(currentId, 0);
                        if (currentId != SUPER_SOURCE && neighborId != SUPER_SOURCE)
                        {
                            var edgeKey = (currentId, neighborId);
                            double edgeDist = edgeLengths.ContainsKey(edgeKey) ?
                                edgeLengths[edgeKey] :
                                edgeLengths[(neighborId, currentId)];

                            newVehicleDist += edgeDist;
                        }

                        if (!distances.ContainsKey(neighborId) || newDist < distances[neighborId])
                        {
                            distances[neighborId] = newDist;
                            vehicleDist[neighborId] = newVehicleDist;
                            parents[neighborId] = currentId;

                            // Priority is just the distance for Dijkstra
                            pq.Enqueue(neighborId, newDist);
                        }
                    }
                }
            }

            return (bestTime, bestPath, bestS, bestF, bestSDist, bestFDist, bestVehicleDist);
        }
    }
}