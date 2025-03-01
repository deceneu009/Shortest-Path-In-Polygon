using System;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class Polygon : MonoBehaviour
{
    #region Variables

    private Vector3 _mousePosition;

    // Checkers
    private bool _polygonClosed;
    private bool _polygonTriangulated;
    private bool _polygonScanned; //this is for checking if the polygon is simple??? I need to check again
    private bool _polygonSimple;
    private bool _startPointSet;
    private bool _startPointSetting;
    private bool _endPointSet;
    private bool _endPointSetting;
    private bool _shortestPathSet;

    // Squares
    [SerializeField] private Texture2D tex;
    private Sprite _mySprite;
    private GameObject _square;

    // Square List
    private List<GameObject> _squareList = new List<GameObject>();
    private List<Vector3> _squaresPos = new List<Vector3>(); // the points locations

    // Edges
    private List<(Vector3, Vector3)> _edges = new List<(Vector3, Vector3)>();

    // Line renderer
    private List<LineRenderer> _polygonLineRenderers = new List<LineRenderer>();
    private List<LineRenderer> _triangulationLineRenderers = new List<LineRenderer>();
    private List<LineRenderer> _shortestPathLineRenderers = new List<LineRenderer>();

    // Start and Finish points
    private GameObject _start;
    private GameObject _end;

    // Delaunay triangulation storage
    private List<(Vector3, Vector3, Vector3)> _triangles = new List<(Vector3, Vector3, Vector3)>();
    private List<Vector3> _centroids = new List<Vector3>();


    //Shortest path storage
    private Dictionary<Vector3, List<(Vector3, float)>> _graph = new Dictionary<Vector3, List<(Vector3, float)>>();
    private List<Vector3> _shortestPath = new List<Vector3>();

    #endregion Variables

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // Create the sprite from the texture
        _mySprite = Sprite.Create(tex, new Rect(0, 0, tex.width - 2, tex.height - 2), new Vector2(0.5f, 0.5f),
            100.0f);
    }

    #region PolygonCreation

    bool onSamePoint(Vector3 point)
    {
        // Define a small tolerance for floating-point comparisons
        const float epsilon = 0.01f;

        if (_squareList.Count < 1)
        {
            return false;
        }

        foreach (GameObject square in _squareList)
        {
            // Get the position of the current square
            Vector3 squarePosition = square.transform.position;

            // Check if the point is close to the square's position
            if (Mathf.Abs(point.x - squarePosition.x) < epsilon && Mathf.Abs(point.y - squarePosition.y) < epsilon)
            {
                Debug.Log("There already is a point on that position");
                return true; // Exit the loop since we found a match
            }
        }

        return false;
    }

    void CreateSquare()
    {
        // Create a new GameObject for the square
        _square = new GameObject("Square");

        // Add a SpriteRenderer and assign the sprite in order to display the square(which is the node)
        SpriteRenderer spriteRenderer = _square.AddComponent<SpriteRenderer>();
        spriteRenderer.sprite = _mySprite;
        spriteRenderer.sortingOrder = 100;
        spriteRenderer.sortingLayerName = "AboveEverything";

        if (_startPointSetting == true)
        {
            _startPointSetting = false;
            spriteRenderer.color = Color.black;
        }
        else if (_endPointSetting == true)
        {
            _endPointSetting = false;
            spriteRenderer.color = Color.black;
        }
        else
        {
            spriteRenderer.color = Color.white;
        }

        // Position the square
        _square.transform.position = _mousePosition;
    }

    // Create a new LineRenderer between two squares
    void CreateLineBetweenSquares(Vector3 startPoint, Vector3 endPoint, bool isTriangulation = false,
        bool isShortestPath = false)
    {
        LineRenderer newLine = new GameObject("Line").AddComponent<LineRenderer>();
        newLine.positionCount = 2; // Two points for the edge
        newLine.SetPosition(0, startPoint);
        newLine.SetPosition(1, endPoint);
        newLine.startWidth = 0.06f;
        newLine.endWidth = 0.06f;

        // Assign a material
        Material lineMaterial = new Material(Shader.Find("Sprites/Default"));

        lineMaterial.color = Color.white; // default color

        if (isTriangulation)
        {
            lineMaterial.color = Color.red;
        }

        if (isShortestPath)
        {
            lineMaterial.color = Color.green;
        }

        newLine.material = lineMaterial;

        // Set sorting layer and order
        newLine.sortingLayerName = "Default"; // Or set a custom layer if needed

        newLine.sortingOrder = 2; //default sorting order

        if (isTriangulation)
        {
            newLine.sortingOrder = 1;
        }

        if (isShortestPath)
        {
            newLine.sortingOrder = 3;
        }

        // Add to appropriate lists
        if (isTriangulation)
        {
            _triangulationLineRenderers.Add(newLine); // Add to triangulation list
        }
        else if (isShortestPath)
        {
            _shortestPathLineRenderers.Add(newLine);
        }
        else
        {
            _polygonLineRenderers.Add(newLine); // Add to polygon edge list
        }

        Debug.LogWarning($"Line from {startPoint} to {startPoint}");
    }

    void ClosePolygon()
    {
        if (_polygonClosed)
        {
            Debug.Log("Polygon is already closed!");
            return;
        }

        if (_squareList.Count <= 2)
        {
            Debug.Log("There must be at least 2 points in order to close the polygon");
            return;
        }

        CreateLineBetweenSquares(_squaresPos[_squareList.Count - 1], _squaresPos[0]);
        _polygonClosed = true;
    }

    void UndoClosedPolygon()
    {
        if (_polygonClosed)
        {
            // Remove the last line (the closing line)
            if (_polygonLineRenderers.Count > 0)
            {
                LineRenderer closingLine = _polygonLineRenderers[_polygonLineRenderers.Count - 1];
                _polygonLineRenderers.RemoveAt(_polygonLineRenderers.Count - 1);

                // Destroy the GameObject, not just the LineRenderer component
                Destroy(closingLine.gameObject);
            }

            // Mark the polygon as not closed
            _polygonClosed = false;
        }
        else
        {
            Debug.Log("The polygon is not closed");
        }
    }

    #endregion PolygonCreation

    #region Simplicity

    // Orientation test for three points A, B, C
    // Returns:
    // 0 -> Collinear
    // 1 -> Clockwise
    // -1 -> Counterclockwise
    private int Orientation(Vector3 A, Vector3 B, Vector3 C)
    {
        float crossProduct = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);

        if (Mathf.Abs(crossProduct) < Mathf.Epsilon)
            return 0; // Collinear

        return crossProduct > 0 ? -1 : 1; // Counterclockwise (-1) or Clockwise (1)
    }

    // Check if point Q lies on segment PR
    private static bool OnSegment(Vector3 P, Vector3 R, Vector3 Q)
    {
        return Q.x <= Mathf.Max(P.x, R.x) && Q.x >= Mathf.Min(P.x, R.x) &&
               Q.y <= Mathf.Max(P.y, R.y) && Q.y >= Mathf.Min(P.y, R.y);
    }

    // Check if two line segments (P1, P2) and (Q1, Q2) intersect
    private bool DoSegmentsIntersect(Vector3 P1, Vector3 P2, Vector3 Q1, Vector3 Q2)
    {
        int o1 = Orientation(P1, P2, Q1);
        int o2 = Orientation(P1, P2, Q2);
        int o3 = Orientation(Q1, Q2, P1);
        int o4 = Orientation(Q1, Q2, P2);

        // General case: segments intersect if orientations are different
        if (o1 != o2 && o3 != o4)
            return true;

        // Special case: Check if points are collinear and overlap
        if (o1 == 0 && OnSegment(P1, P2, Q1)) return true;
        if (o2 == 0 && OnSegment(P1, P2, Q2)) return true;
        if (o3 == 0 && OnSegment(Q1, Q2, P1)) return true;
        if (o4 == 0 && OnSegment(Q1, Q2, P2)) return true;

        return false; // No intersection
    }

    // Check if a polygon is simple (no self-intersections)
    private bool IsPolygonSimple(List<Vector3> polygon)
    {
        int n = polygon.Count;

        // Loop through all pairs of edges in the polygon
        for (int i = 0; i < n; i++)
        {
            Vector3 P1 = polygon[i];
            Vector3 P2 = polygon[(i + 1) % n]; // Next vertex, wrap around

            for (int j = i + 1; j < n; j++)
            {
                Vector3 Q1 = polygon[j];
                Vector3 Q2 = polygon[(j + 1) % n]; // Next vertex, wrap around

                // Skip adjacent edges and the same edge
                if (i == j || (i + 1) % n == j || (j + 1) % n == i)
                    continue;

                // Check if the edges intersect
                if (DoSegmentsIntersect(P1, P2, Q1, Q2))
                {
                    _polygonSimple = false;
                    return false; // Found an intersection
                }
            }
        }

        _polygonSimple = true;
        return true; // No intersections found
    }

    #endregion Simplicity

    #region Utility

    void RemoveEdgeOnClick(Vector3 clickPos)
    {
        // Loop through all the lines in the triangulation list
        foreach (LineRenderer line in _triangulationLineRenderers)
        {
            Vector3 start = line.GetPosition(0);
            Vector3 end = line.GetPosition(1);

            // Check if the click is close to any edge
            if (IsClickOnEdge(clickPos, start, end))
            {
                // Remove this edge from the triangulation list
                _triangulationLineRenderers.Remove(line);
                Destroy(line.gameObject); // Destroy the LineRenderer GameObject

                Debug.Log($"Triangulated edge from {start} to {end} removed.");
                break; // Stop after removing the first matching edge
            }
        }

        // Also check in the polygon edges if the user clicked an edge there
        foreach (LineRenderer line in _polygonLineRenderers)
        {
            Vector3 start = line.GetPosition(0);
            Vector3 end = line.GetPosition(1);

            if (IsClickOnEdge(clickPos, start, end))
            {
                // Remove this edge from the polygon edges list
                _polygonLineRenderers.Remove(line);
                Destroy(line.gameObject); // Destroy the LineRenderer GameObject

                Debug.Log($"Polygon edge from {start} to {end} removed.");
                break;
            }
        }
    }

    bool IsClickOnEdge(Vector3 clickPos, Vector3 edgeStart, Vector3 edgeEnd)
    {
        // Calculate perpendicular distance from point to the line (edge)
        float threshold = 0.1f; // A small threshold to consider "close enough" click

        float distance = DistanceFromPointToLine(clickPos, edgeStart, edgeEnd);
        return distance <= threshold;
    }

    float DistanceFromPointToLine(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        // Calculate perpendicular distance from point to the line (edge)
        Vector3 lineDirection = lineEnd - lineStart;
        Vector3 pointToLineStart = point - lineStart;

        // Project the point onto the line and calculate the perpendicular distance
        float lineLength = lineDirection.magnitude;
        float projection = Vector3.Dot(pointToLineStart, lineDirection.normalized);
        Vector3 projectedPoint = lineStart + lineDirection.normalized * projection;
        return Vector3.Distance(point, projectedPoint);
    }

    #endregion

    #region Triangulation

    // // Function to calculate Euclidean distance between two points
    // private float EuclideanDistance(Vector3 a, Vector3 b)
    // {
    //     return Vector3.Distance(a, b);
    // }

    private void UndoTriangulation()
    {
        if (!_polygonTriangulated)
        {
            Debug.LogWarning("The polygon is not triangulated!");
            return;
        }

        Debug.Log($"Triangulation line count before deletion: {_triangulationLineRenderers.Count}");

        foreach (LineRenderer triangulatedLine in _triangulationLineRenderers)
        {
            if (triangulatedLine != null)
            {
                Destroy(triangulatedLine.gameObject); // Destroy the whole GameObject
            }
        }

        // Ensure the list is cleared after destroying
        _triangulationLineRenderers.Clear();

        Debug.Log("Triangulation undone successfully!");
        _polygonTriangulated = false;
        _triangles.Clear();
        // Optional: Check if there are remaining line renderers in the scene
    }
    private void PerformDelaunayTriangulation()
    {
        if (!_polygonClosed || _polygonTriangulated)
        {
            Debug.LogWarning("Polygon must be closed and not already triangulated.");
            return;
        }

        Debug.Log("Starting triangulation process...");

        // Clear previous triangulation data.
        UndoTriangulation();

        // Bowyer-Watson algorithm
        List<Vector3> points = new List<Vector3>(_squaresPos);

        // Create a super-triangle that encompasses all points. No need to create it dynamically because the zoom out is not made for huge distances.
        float maxCoordinate = 0;
        foreach (Vector3 point in points)
        {
            maxCoordinate = Mathf.Max(maxCoordinate, Mathf.Abs(point.x), Mathf.Abs(point.y));
        }

        maxCoordinate *= 10f; // Enlarge to ensure it covers all points
        Vector3 p1 = new Vector3(-maxCoordinate, -maxCoordinate, 0);
        Vector3 p2 = new Vector3(maxCoordinate, -maxCoordinate, 0);
        Vector3 p3 = new Vector3(0, maxCoordinate, 0);
        _triangles.Add((p1, p2, p3));

        // Incrementally add points
        foreach (Vector3 point in points)
        {
            List<(Vector3, Vector3, Vector3)> badTriangles = new List<(Vector3, Vector3, Vector3)>();

            // Identify bad triangles (those whose circumcircle contains the point)
            foreach (var triangle in _triangles)
            {
                if (IsPointInCircumcircle(point, triangle))
                {
                    badTriangles.Add(triangle);
                    Debug.Log($"Bad triangle found: {triangle}");
                }
            }

            // Find boundary edges of the polygonal hole
            List<(Vector3, Vector3)> polygon = new List<(Vector3, Vector3)>();
            foreach (var triangle in badTriangles)
            {
                AddEdgeIfUnique(polygon, triangle.Item1, triangle.Item2);
                AddEdgeIfUnique(polygon, triangle.Item2, triangle.Item3);
                AddEdgeIfUnique(polygon, triangle.Item3, triangle.Item1);
            }

            // Remove bad triangles
            foreach (var triangle in badTriangles)
                _triangles.Remove(triangle);

            // Create new triangles
            foreach (var edge in polygon)
                _triangles.Add((edge.Item1, edge.Item2, point));
        }

        // Remove super-triangle vertices
        _triangles.RemoveAll(t => t.Item1 == p1 || t.Item1 == p2 || t.Item1 == p3 ||
                                  t.Item2 == p1 || t.Item2 == p2 || t.Item2 == p3 ||
                                  t.Item3 == p1 || t.Item3 == p2 || t.Item3 == p3);

        Debug.Log($"Number of triangles after triangulation: {_triangles.Count}");

        if (_triangles.Count == 0)
        {
            Debug.LogError("The triangulation failed!");
            return;
        }

        // Draw triangulation
        foreach (var triangle in _triangles)
        {
            Debug.Log($"Drawing the lines for triangle: {triangle}");
            CreateLineBetweenSquares(triangle.Item1, triangle.Item2, true);
            CreateLineBetweenSquares(triangle.Item2, triangle.Item3, true);
            CreateLineBetweenSquares(triangle.Item3, triangle.Item1, true);
        }

        Debug.Log("Triangulation Completed!");
        _polygonTriangulated = true;
    }

    private bool IsPointInCircumcircle(Vector3 point, (Vector3, Vector3, Vector3) triangle)
    {
        // Compute circumcircle test
        Vector3 a = triangle.Item1;
        Vector3 b = triangle.Item2;
        Vector3 c = triangle.Item3;

        float ax = a.x - point.x;
        float ay = a.y - point.y;
        float bx = b.x - point.x;
        float by = b.y - point.y;
        float cx = c.x - point.x;
        float cy = c.y - point.y;

        float det = (ax * ax + ay * ay) * (bx * cy - by * cx) -
                    (bx * bx + by * by) * (ax * cy - ay * cx) +
                    (cx * cx + cy * cy) * (ax * by - ay * bx);
        return det > 0;
    }

    private void AddEdgeIfUnique(List<(Vector3, Vector3)> polygon, Vector3 v1, Vector3 v2)
    {
        if (!polygon.Contains((v1, v2)) && !polygon.Contains((v2, v1)))
        {
            polygon.Add((v1, v2));
        }
        else
        {
            // If it exists, remove it (it's a shared edge, no longer part of the boundary)
            polygon.Remove((v1, v2));
            polygon.Remove((v2, v1));
        }
    }

    // Checking if the Points are inside the Polygon
    private bool IsPointInPolygon(Vector3 p, List<Vector3> polygon)
    {
        double minX = polygon[0].x;
        double maxX = polygon[0].x;
        double minY = polygon[0].y;
        double maxY = polygon[0].y;
        for (int i = 1; i < polygon.Count; i++)
        {
            Vector3 q = polygon[i];
            minX = Math.Min(q.x, minX);
            maxX = Math.Max(q.x, maxX);
            minY = Math.Min(q.y, minY);
            maxY = Math.Max(q.y, maxY);
        }

        if (p.x < minX || p.x > maxX || p.y < minY || p.y > maxY)
        {
            return false;
        }

        bool inside = false;
        for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
        {
            if ((polygon[i].y > p.y) != (polygon[j].y > p.y) &&
                p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) / (polygon[j].y - polygon[i].y) +
                polygon[i].x)
            {
                inside = !inside;
            }
        }

        return inside;
    }

    // Function to update the triangulation lines based on midpoint check(semi-valid)
    // void UpdateTriangulationLines()
    // {
    //     // Loop through each triangulation line renderer
    //     for (int i = _triangulationLineRenderers.Count - 1; i >= 0; i--) // Iterate backward
    //     {
    //         var triangulationLineRenderer = _triangulationLineRenderers[i];

    //         if (triangulationLineRenderer.positionCount < 2)
    //             continue;

    //         Vector3 point1 = triangulationLineRenderer.GetPosition(0);
    //         Vector3 point2 = triangulationLineRenderer.GetPosition(1);

    //         // Calculate midpoint
    //         Vector3 midpoint = (point1 + point2) / 2f;

    //         // Check if the midpoint is inside the polygon
    //         if (IsPointInPolygon(midpoint, _squaresPos)) // Assuming _squaresPos contains polygon vertices
    //         {
    //             // Only add lines that are valid (midpoint inside the polygon)
    //             CreateLineBetweenSquares(point1, point2, isTriangulation: true);
    //         }
    //         else
    //         {
    //             // Remove the invalid line directly from the triangulation list
    //             _triangulationLineRenderers.RemoveAt(i);
    //             Destroy(triangulationLineRenderer.gameObject); // Optionally destroy the invalid lines
    //         }
    //     }
    // }


    // This computes the centroid of the triangles and checks if it is inside the polygon or not

    #endregion Triangulation

    #region ShortestPath

    private void UndoDijkstra()
    {
        if (!_shortestPathSet)
        {
            Debug.LogWarning("The shortest path was not computed!");
            return;
        }

        //_shortestPath.Clear();

        foreach (LineRenderer vertex in _shortestPathLineRenderers)
        {
            Destroy(vertex.gameObject);
        }

        _shortestPathLineRenderers.Clear();
        Debug.Log("Shortest path undone!");
        _shortestPathSet = false;
    }

    void CalculateCentroids()
    {
        _centroids.Clear();
        foreach (var triangle in _triangles)
        {
            Vector3 centroid = (triangle.Item1 + triangle.Item2 + triangle.Item3) / 3;
            _centroids.Add(centroid);
        }
    }

    // Add a new edge to the graph
    // Build graph from triangulation
    void BuildGraph()
    {
        _graph.Clear();

        // Add original points and initialize their adjacency list
        foreach (var point in _squaresPos)
        {
            if (!_graph.ContainsKey(point))
            {
                _graph[point] = new List<(Vector3, float)>();
            }
        }

        // Add centroids and initialize their adjacency list
        foreach (var centroid in _centroids)
        {
            if (!_graph.ContainsKey(centroid))
            {
                _graph[centroid] = new List<(Vector3, float)>();
            }
        }

        // Add edges for each triangle (vertices to centroid)
        foreach (var triangle in _triangles)
        {
            Vector3 centroid = (triangle.Item1 + triangle.Item2 + triangle.Item3) / 3;

            // Add edges from each vertex to centroid
            AddEdgeToGraph(triangle.Item1, centroid);
            AddEdgeToGraph(triangle.Item2, centroid);
            AddEdgeToGraph(triangle.Item3, centroid);

            // Add edges between vertices of the triangle
            AddEdgeToGraph(triangle.Item1, triangle.Item2);
            AddEdgeToGraph(triangle.Item2, triangle.Item3);
            AddEdgeToGraph(triangle.Item3, triangle.Item1);
        }
    }

    void AddEdgeToGraph(Vector3 pointA, Vector3 pointB)
    {
        float distance = Vector3.Distance(pointA, pointB);

        // Add edge from A to B
        if (!_graph.ContainsKey(pointA))
        {
            _graph[pointA] = new List<(Vector3, float)>();
        }

        _graph[pointA].Add((pointB, distance));

        // Add edge from B to A
        if (!_graph.ContainsKey(pointB))
        {
            _graph[pointB] = new List<(Vector3, float)>();
        }

        _graph[pointB].Add((pointA, distance));
    }

    List<Vector3> FindShortestPath(Vector3 start, Vector3 end)
    {
        // Priority queue for the Dijkstra algorithm
        var priorityQueue = new SortedSet<(float, Vector3)>(Comparer<(float, Vector3)>.Create((a, b) =>
        {
            int cmp = a.Item1.CompareTo(b.Item1);
            return cmp == 0 ? a.Item2.GetHashCode().CompareTo(b.Item2.GetHashCode()) : cmp;
        }));

        // Distance dictionary
        Dictionary<Vector3, float> distances = new Dictionary<Vector3, float>();
        Dictionary<Vector3, Vector3> previous = new Dictionary<Vector3, Vector3>();

        // Initialize distances
        foreach (var node in _graph.Keys)
        {
            distances[node] = float.MaxValue;
        }

        distances[start] = 0;

        priorityQueue.Add((0, start));

        while (priorityQueue.Count > 0)
        {
            var (currentDistance, currentNode) = priorityQueue.Min;
            priorityQueue.Remove(priorityQueue.Min);

            if (currentNode == end)
            {
                break; // Shortest path found
            }

            foreach (var (neighbor, weight) in _graph[currentNode])
            {
                float newDistance = currentDistance + weight;

                if (newDistance < distances[neighbor])
                {
                    priorityQueue.Remove((distances[neighbor], neighbor));
                    distances[neighbor] = newDistance;
                    previous[neighbor] = currentNode;
                    priorityQueue.Add((newDistance, neighbor));
                }
            }
        }

        // Reconstruct the shortest path
        var path = new List<Vector3>();
        if (previous.ContainsKey(end) || start == end)
        {
            var current = end;
            while (current != start)
            {
                path.Add(current);
                current = previous[current];
            }

            path.Add(start);
            path.Reverse();
        }

        return path;
    }

    // // Dijkstra's Algorithm to find the shortest path
    // private List<Vector3> Dijkstra(Vector3 start, Vector3 end)
    // {
    //     var distances = new Dictionary<Vector3, float>();
    //     var previous = new Dictionary<Vector3, Vector3>();
    //     var priorityQueue = new List<Vector3>();

    //     // Initialize distances and queue
    //     foreach (var node in _graph.Keys)
    //     {
    //         distances[node] = Mathf.Infinity;
    //         previous[node] = Vector3.zero;
    //         priorityQueue.Add(node);
    //     }

    //     distances[start] = 0;

    //     while (priorityQueue.Count > 0)
    //     {
    //         // Get the node with the smallest distance
    //         priorityQueue.Sort((a, b) => distances[a].CompareTo(distances[b]));
    //         Vector3 current = priorityQueue[0];
    //         priorityQueue.RemoveAt(0);

    //         Debug.Log($"Visiting node: {current}, Distance: {distances[current]}");

    //         if (current == end) break;

    //         foreach (var neighbor in _graph[current])
    //         {
    //             Vector3 neighborNode = neighbor.Item1;
    //             float edgeWeight = neighbor.Item2;
    //             float newDist = distances[current] + edgeWeight;

    //             if (newDist < distances[neighborNode])
    //             {
    //                 distances[neighborNode] = newDist;
    //                 previous[neighborNode] = current;
    //                 Debug.Log($"Updating path: {neighborNode} via {current}, New Distance: {newDist}");
    //             }
    //         }
    //     }

    //     // Reconstruct the shortest path
    //     // Reconstruct the shortest path
    //     List<Vector3> path = new List<Vector3>();
    //     Vector3 currentNode = end;

    //     // Check if there is a valid path
    //     if (!previous.ContainsKey(currentNode))
    //     {
    //         Debug.LogError("No path exists between the start and end nodes.");
    //         return path; // Return an empty path
    //     }

    //     while (!currentNode.Equals(start))
    //     {
    //         if (!previous.ContainsKey(currentNode))
    //         {
    //             Debug.LogError($"Path reconstruction failed: '{currentNode}' is not in the previous dictionary.");
    //             return new List<Vector3>(); // Return an empty path
    //         }

    //         path.Insert(0, currentNode);
    //         currentNode = previous[currentNode];
    //     }

    //     path.Insert(0, start);

    //     return path;
    // }

    void DrawShortestPath()
    {
        // Clear existing shortest path renderers
        foreach (var line in _shortestPathLineRenderers)
        {
            Destroy(line.gameObject);
        }

        _shortestPathLineRenderers.Clear();

        // Draw the new shortest path
        for (int i = 0; i < _shortestPath.Count - 1; i++)
        {
            CreateLineBetweenSquares(_shortestPath[i], _shortestPath[i + 1], isShortestPath: true);
        }
    }


    private void SetStartAndEnd()
    {
        CalculateCentroids();
        BuildGraph();
        _shortestPath = FindShortestPath(_start.transform.position, _end.transform.position);
        DrawShortestPath();
    }

    #endregion

    #region inputs

    // Update is called once per frame
    void Update()
    {
        if (Camera.main == null)
        {
            Debug.LogError("Main camera not found.");
            return;
        }

        // Left-click: Create a square
        if (Input.GetMouseButtonDown(0))
        {
            if (_polygonClosed)
            {
                Debug.Log(
                    "The polygon is closed! Can't add any more squares.(Press right click to undo the Closing)");
                return;
            }

            Debug.Log("Pressed left-click!");

            // Get mouse position in world space
            if (Camera.main != null)
            {
                _mousePosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
                _mousePosition.z = 0; // Set Z position to 0 for 2D
            }

            if (onSamePoint(_mousePosition))
            {
                Debug.LogWarning("There already is a point on that position");
                return;
            }

            // it's creating the gameObject
            CreateSquare();
            _squareList.Add(_square);
            _squaresPos.Add(_square.transform.position);

            if (_squareList.Count > 1)
            {
                // It's adding the lineRenderer to the squares
                CreateLineBetweenSquares(_squaresPos[_squareList.Count - 2], _squaresPos[_squareList.Count - 1],
                    false);
            }
        }

        // Right-click: Remove the last created square
        if (Input.GetMouseButtonDown(1))
        {
            if (_squareList.Count == 0)
            {
                Debug.LogError("No squares found!");
                return;
            }

            if (_shortestPathSet)
            {
                UndoDijkstra();
                Debug.Log("Path lines after deletion: {_shortestPathLineRenderers.Count}");
                return;
            }

            if (_polygonTriangulated)
            {
                UndoTriangulation();
                _polygonTriangulated = false;
                Debug.Log("Triangulation line count before deletion: {_triangulationLineRenderers.Count}");
                return;
            }

            if (_endPointSet)
            {
                _endPointSet = false;
                _squaresPos.RemoveAt(_squareList.Count - 1);
                Destroy(_end);
                return;
            }

            if (_startPointSet)
            {
                _startPointSet = false;
                _squaresPos.RemoveAt(_squareList.Count - 1);
                Destroy(_start);
                return;
            }

            if (_polygonClosed)
            {
                UndoClosedPolygon();
                Debug.Log("Polygon opened!");
                Debug.Log($"_squaresPos count after clear: {_squaresPos.Count}");
                Debug.Log($"_triangles count after clear: {_triangles.Count}");
                _polygonScanned = false;
                return;
            }

            Debug.Log("Pressed right-click!");

            // Get the last square in the list
            GameObject lastSquare = _squareList[_squareList.Count - 1];

            // Remove it from the list and destroy it
            _squareList.RemoveAt(_squareList.Count - 1);
            Destroy(lastSquare);

            // Also remove its position from the list
            _squaresPos.RemoveAt(_squaresPos.Count - 1);

            // Remove and destroy the last line (the GameObject containing the LineRenderer)
            if (_polygonLineRenderers.Count > 0)
            {
                LineRenderer lineRenderer = _polygonLineRenderers[_polygonLineRenderers.Count - 1];
                _polygonLineRenderers.RemoveAt(_polygonLineRenderers.Count - 1);

                // Destroy the GameObject, not just the LineRenderer component
                Destroy(lineRenderer.gameObject);
            }

            Debug.Log($"_squaresPos count after clear: {_squaresPos.Count}");
            Debug.Log($"_triangles count after clear: {_triangles.Count}");
        }

        // Mid-click
        // if (Input.GetMouseButtonDown(2))
        // {
        //     // Get mouse position in world space
        //     if (Camera.main != null)
        //     {
        //         _mousePosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        //         _mousePosition.z = 0; // Set Z position to 0 for 2D
        //     }

        //     RemoveEdgeOnClick(_mousePosition);
        // }

        // Closing the polygon
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ClosePolygon();

            // If the ClosePolygon function did not close the polygon then scanning if the polygon is simple will not happen
            if (!_polygonClosed)
            {
                return;
            }

            //If it was already checked to see if it is simple or not it will return without checking again
            if (_polygonScanned)
            {
                Debug.Log("Polygon was already scanned!");
                return;
            }

            Debug.Log(IsPolygonSimple(_squaresPos) ? "Polygon is simple" : "Polygon is not simple");

            _polygonScanned = true;
        }

        // Positioning the start point
        if (Input.GetKeyDown(KeyCode.Q))
        {
            if (!_polygonClosed)
            {
                Debug.LogWarning("Polygon is not closed!");
                return;
            }

            if (_startPointSet)
            {
                Debug.LogWarning("Start point is already set!");
                return;
            }

            Debug.Log("Setting start point!");

            // Get mouse position in world space
            if (Camera.main != null)
            {
                _mousePosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
                _mousePosition.z = 0; // Set Z position to 0 for 2D
            }

            if (!IsPointInPolygon(_mousePosition, _squaresPos))
            {
                Debug.LogError("The point is not inside the polygon!");
                return;
            }

            Debug.Log("Start point was set!");
            _startPointSetting = true;
            CreateSquare();
            _start = _square;

            _squaresPos.Add(_start.transform.position);

            _startPointSet = true;
        }

        // Positioning the end point
        if (Input.GetKeyDown(KeyCode.E))
        {
            if (!_polygonClosed)
            {
                Debug.LogWarning("Polygon is not closed!");
                return;
            }

            if (_endPointSet)
            {
                Debug.LogWarning("End point is already set!");
                return;
            }

            Debug.Log("Setting end point!");

            // Get mouse position in world space
            if (Camera.main != null)
            {
                _mousePosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
                _mousePosition.z = 0; // Set Z position to 0 for 2D
            }

            if (!IsPointInPolygon(_mousePosition, _squaresPos))
            {
                Debug.LogError("The point is not inside the polygon!");
                return;
            }

            Debug.Log("Start point was set!");
            _endPointSetting = true;
            CreateSquare();
            _end = _square;

            _squaresPos.Add(_end.transform.position);

            _endPointSet = true;
        }

        // Deleting the Points
        //if (Input.GetKeyDown(KeyCode.R))
        //{
        //    if (_endPointSet)
        //    {
        //        Destroy(_end);

        //        Debug.Log("End point removed");
        //        _squaresPos.RemoveAt(_squaresPos.Count - 1);

        //        _endPointSet = false;
        //    }

        //    if (_startPointSet)
        //    {
        //        Destroy(_start);

        //        Debug.Log("Start point removed");
        //        _squaresPos.RemoveAt(_squaresPos.Count - 1);

        //        _startPointSet = false;
        //    }

        //    UndoTriangulation();
        //    //UndoDijkstra();

        //    Debug.LogError("There are no start or end points!");
        //}

        // Starting the triangulation
        if (Input.GetKeyDown(KeyCode.T))
        {
            if (!_startPointSet || !_endPointSet)
            {
                Debug.LogWarning("Start point or/and end point are not set!");
                return;
            }

            if (!_polygonSimple)
            {
                Debug.Log("Polygon is not simple!");
                return;
            }

            if (_polygonTriangulated)
            {
                Debug.LogWarning("Polygon is already triangulated!");
                return;
            }

            Debug.Log("Starting triangulation!");

            PerformDelaunayTriangulation();
            //UpdateTriangulationLines();
            foreach (LineRenderer lineRenderer in _polygonLineRenderers)
            {
                // Get the positions of the current LineRenderer
                Vector3 startPos = lineRenderer.GetPosition(0);
                Vector3 endPos = lineRenderer.GetPosition(1);

                // Add these positions to the edges list
                _edges.Add((startPos, endPos));
            }

            Debug.Log($"There are {_edges.Count} edges");

            _polygonTriangulated = true;
        }

        // // Undoing the triangulation
        // if (Input.GetKeyDown(KeyCode.U))
        // {
        //     if (!_polygonTriangulated)
        //     {
        //         Debug.LogWarning("Polygon is not triangulated so there is no undoing necessary!");
        //         return;
        //     }
        //
        //     Debug.Log("Removing triangulation!");
        //     UndoTriangulation();
        //
        //     _edges.Clear();
        //
        //     Debug.Log($"There are {_edges.Count} edges");
        // }

        // Applying the shortest path
        if (Input.GetKeyDown(KeyCode.I))
        {
            if (!_polygonTriangulated)
            {
                Debug.LogWarning("Polygon needs to be triangulated first in order to compute the shortest path!");
                return;
            }

            if (_shortestPathSet)
            {
                Debug.LogWarning("Shortest path is already set!");
                return;
            }

            Debug.Log("Starting shortest path!");

            SetStartAndEnd();

            _shortestPathSet = true;
        }
    }

    #endregion inputs
}
