# 🚗 Map Routing Project

This project aims to compute the **fastest route** between two points (S to D) on a map, combining **walking** and **driving** through intersections, using shortest path algorithms.

---

## 📌 Problem Overview

Given a start point `S` and destination `D`:
1. **Walk** from `S` to the nearest intersection.
2. **Drive** from the first intersection to the last using the road network.
3. **Walk** from the last intersection to `D`.

The goal is to minimize the **total travel time**.

---

## 📂 Project Steps

### 1️ Constructing the Graph from `map.txt`
- **Input Format**:
  - `N`: number of intersections (vertices)
  - `N` lines: intersection details `(id, x, y)`
  - `M`: number of roads (edges)
  - `M` lines: road details `(id1, id2, length, speed)`

- **Processing**:
  - Store intersections in a hashmap.
  - Convert edge weights to **travel time** = `length / speed` (in hours).
  - Represent the graph using an **adjacency list**:  
    `key = id1`, `value = list of (id2, time)`.

---

### 2️⃣ Finding the Starting Intersection `B`
- Given `(x, y, R)` of point `S`:
  - Compute **Euclidean distance** to all intersections.
  - Choose the **closest** one within radius `R`.
  - Walking speed = **5 km/s**, so time = `distance / 5`.
- **Data Structure**: List of tuples `(ID, distance, time)`
- **Complexity**: O(N)

---

### 3️⃣ Finding the Ending Intersection `F`
- Same as Step 2, but for the destination point `D`.

---

### 4️⃣ Finding the Shortest Time Path
- Total time = Walk(S→B) + Drive(B→F) + Walk(F→D)
- Use **Dijkstra’s algorithm** with a **min-heap (priority queue)** for best performance.

---

### 5️⃣ Constructing the Path
- After Dijkstra, reconstruct the **best-time path**.
- **Output Format**:
--- Line 1: Path of intersection IDs (space-separated)
--- Line 2: Total time (in hours)
--- Line 3: Total distance (km)
--- Line 4: Walking distance (km)
--- Line 5: Car distance (km)

### 7️⃣ Measuring Execution Time
- **Query Time**: Measure time spent on logic (exclude file I/O).
- **Total Time**: Includes both logic and file I/O.
- Use tools like `Stopwatch` (C#) or equivalent.
- 
## ✅ Features Summary
-  Efficient graph representation  
-  Fast pathfinding with Dijkstra  
-  Query support with execution time measurement  
-  Optional visualization & enhancements  
-  Clean and structured output format

---
## 🙋‍♀️ Team Members : 
--- Rana Nasser
--- Ahmed Khaled
--- Mohamed Khaled
--- Mohamed Tamer
--- Mohamed Ameer
--- Mohamed Saad
