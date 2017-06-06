package roadgraph;

public class MapEdge {
	//present edge in the map
	private String name;
	String type;
	double len;
	int endNode;
	
	
	public MapEdge(String roadName, String roadType, double length,int toNode)
	{
		name = roadName;
		type = roadType;
		len = length;
		endNode = toNode;
	}
	
	public double getLength()
	{
		return len;
	}
	
	public int getEndNode()
	{
		return endNode;
	}
	
	public String toString() //for debug use
	{
		return "<" + name + "," + type + "," + len + ">";
	}

}
