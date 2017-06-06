package roadgraph;

public class MapEdge {
	
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
	
	public String getName()
	{
		return name;
	}
	
	public String getType()
	{
		return type;
	}
	
	public double getLength()
	{
		return len;
	}
	
	public int getEndNode()
	{
		return endNode;
	}
	
	public String toString()
	{
		return "<" + name + "," + type + "," + len + ">";
	}

}
