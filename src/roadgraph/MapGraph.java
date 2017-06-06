/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private int numEdges;
	
	
	private ArrayList<GeographicPoint> vertices; //use index of arrayList to mapping GeographicPoint to integer,
												 //use the integer as vertices presentation 
	
	private HashMap<Integer, ArrayList<MapEdge>> adjListMap; //mapping vertices to the edges, MapEdge contains endNode information
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() //initialize a empty map graph
	{
		// TODO: Implement in this constructor in WEEK 3
		numEdges = 0;
		vertices = new ArrayList<GeographicPoint>();
		adjListMap = new HashMap<Integer, ArrayList<MapEdge>>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.size(); // all vertices stores in ArrayList "vertices"
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		
		return new HashSet<GeographicPoint>(vertices); // all vertices stores in ArrayList "vertices"
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(getNodeIndex(location) != -1) //if the vertices is already exist
			return false;
		
		vertices.add(location);
		adjListMap.put(vertices.size() - 1, new ArrayList<MapEdge>()); //add new vertices to adjacency list, use integer present vertices
																	   //and mapping it to MapEdge
		
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		int startNode = -1;
		int endNode = -1;
		MapEdge newEdge;
		
		//parameter validation check
		if(length < 0)
		{
			throw new IllegalArgumentException();
		}
		
		if(from == null || to == null || roadName == null || roadType == null)
		{
			throw new IllegalArgumentException();
		}
		
		//
		
		//convert GeographicPoint to integer presentation
		startNode = getNodeIndex(from);
		endNode = getNodeIndex(to);
		
		//check if the node exist
		if(startNode == -1)
			throw new IllegalArgumentException();
		
		if(endNode == -1)
			throw new IllegalArgumentException();
		
		//create new map edge use given information
		newEdge = new MapEdge(roadName,roadType,length,endNode);

		//add new edge to the adjacency list with corresponding start node
		adjListMap.get(startNode).add(newEdge);
		numEdges ++;
	}
	
	public void printGraph()
	{
		//for debug, print the graph
		for(GeographicPoint v : vertices)
		{
			int index = vertices.indexOf(v);
			System.out.print("vertice" + index + ": (" + v.x  + "," + v.y + ")-->[");
			for(MapEdge me : adjListMap.get(index))
			{
				System.out.print("(" + vertices.get(me.getEndNode()) + ")" + me + ",");
			}
			
			System.out.print("]\n");
		}
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		LinkedList<Integer> queue = new LinkedList<Integer>(); //use for bfs
		HashSet<Integer> visited = new HashSet<Integer> (); // record visited vertices
		HashMap<Integer,Integer> searchPath = new HashMap<Integer,Integer>(); //record search path
		
		int indexStart  = getNodeIndex(start); 
		int indexEnd  = getNodeIndex(goal);
		
		//check if start and goal point exist
		if(indexStart == -1 || indexEnd == -1)
		{
			throw new IllegalArgumentException();
		}
		else
		{
			queue.addLast(indexStart);
			visited.add(indexStart);
			int currentNode;
			while(!queue.isEmpty())
			{
				currentNode = queue.removeFirst();
				if(currentNode == indexEnd)
				{
					return getResultPath(searchPath, indexStart,currentNode);
				}
				
				//get all linked edge form adjListMap
				for(MapEdge me: adjListMap.get(currentNode))
				{
					queue.addLast(me.getEndNode());
					visited.add(me.getEndNode());
					searchPath.put(me.endNode, currentNode); //insert current node's child into search path
															//and set theirs parent to current node
				}
			}
			return null;
		}

	}
	
	/**
	 * convert integer node presentation into GeographicPoint list
	 * @param searchPath:record of search path, format: <node,parent>
	 * @param start: start point
	 * @param goal: goal point
	 * @return Path from start GeographicPoint to goal GeographicPoint
	 */
	private List<GeographicPoint> getResultPath(HashMap<Integer,Integer> searchPath, int start,int goal)
	{
		LinkedList<GeographicPoint> retList = new LinkedList<GeographicPoint>();
		int currentNode = goal;
		
		while(true)
		{
			retList.addFirst(vertices.get(currentNode));
			if(currentNode == start)
			{
				return retList;
			}
			currentNode = searchPath.get(currentNode);
		}
	}
	
	/**
	 * convert GeographicPoint to integer presentation
	 * @param node
	 * @return index of node in ArrayList vertices
	 */
	private int getNodeIndex(GeographicPoint node)
	{
		
		for(GeographicPoint n:vertices)
		{
			if(n.equals(node))
				return vertices.indexOf(n);
		}
		
		//if the node dose not exist
		return -1;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		firstMap.printGraph();
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
//		MapGraph simpleTestMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
//		
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//		
//		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
//		
//		
//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
//		
//		// A very simple test using real data
//		testStart = new GeographicPoint(32.869423, -117.220917);
//		testEnd = new GeographicPoint(32.869255, -117.216927);
//		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		
//		
//		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
