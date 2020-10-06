package ctu.fel;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

public class Graph {
    private int numNodes;
    private int numEdges;

//    private int numEdgeParameters;

    private List<Node> nodes;
    private List<Edge> edges;
    private List<Integer> goals;
    private List<Boolean> isGoal;

    private List<List<Integer>> outgoingEdgeIndices;
    private List<List<Integer>> incomingEdgeIndices;

    int getNumNodes() {
        return numNodes;
    }

    int getNumEdges() {
        return numEdges;
    }

    boolean isGoal(int ind){
        return isGoal.get(ind);
    }

    /**
     * Returns the list of edges starting at a node
     * @param node  node to be examined
     * @return  ArrayList of outgoing edges
     */
    List<Edge> getOutgoingEdges(int node) {
        List<Edge> outEdges = new ArrayList<>();
        for (int e : outgoingEdgeIndices.get(node)) {
            outEdges.add(edges.get(e));
        }
        return outEdges;
    }

    /**
     * Returns the list of edges ending at a node
     * @param node  node to be examined
     * @return  ArrayList of incoming edges
     */
    List<Edge> getIncomingEdges(int node) {
        List<Edge> inEdges = new ArrayList<>();
        for (int e : incomingEdgeIndices.get(node)) {
            inEdges.add(edges.get(e));
        }
        return inEdges;
    }

    List<Integer> getGoals() {
        return goals;
    }

    /**
     * Graph class constructor
     * @param nodePath  path to the node file of form "elevation, id, latitude, projectedLatitude, longitude, projected Longitude"
     * @param edgePath  path to the edge file of form "consumption, start, time, end"
     * @param goalPath  path to the goal file of list of goal indices separated by newlines
     * @throws FileNotFoundException    if any of the files is not found
     */
    Graph(String nodePath, String edgePath, String goalPath) throws FileNotFoundException {
//    Graph(String nodePath, String edgePath) throws FileNotFoundException {
        nodes = new ArrayList<>();
        edges = new ArrayList<>();
        goals = new ArrayList<>();

        outgoingEdgeIndices = new ArrayList<>();
        incomingEdgeIndices = new ArrayList<>();

        //Read input node file
        File nodeFile = new File(nodePath);
        Scanner nodeScan = new Scanner(nodeFile);
        nodeScan.nextLine();
        while (nodeScan.hasNextLine()) {
            String[] split = nodeScan.nextLine().split(",");
            int id = Integer.parseInt(split[1]);
            float latProj = Float.parseFloat(split[3]);
            float lat = Float.parseFloat(split[2]);
            float lonProj = Float.parseFloat(split[5]);
            float lon = Float.parseFloat(split[4]);
            int elevation = Integer.parseInt(split[0]);

            nodes.add(new Node(id, elevation, lat, latProj, lon, lonProj));
            outgoingEdgeIndices.add(new ArrayList<>());
            incomingEdgeIndices.add(new ArrayList<>());
        }
        numNodes = nodes.size();

        // Read input edge file
        File edgeFile = new File(edgePath);
        Scanner edgeScan = new Scanner(edgeFile);
        edgeScan.nextLine();
        int index = 0;
        while (edgeScan.hasNextLine()) {
            String[] split = edgeScan.nextLine().split(",");
            int start = Integer.parseInt(split[1]);
            int end = Integer.parseInt(split[3]);
            int time = Integer.parseInt(split[2]);
            int cons = Integer.parseInt(split[0]);

            outgoingEdgeIndices.get(start).add(index);
            incomingEdgeIndices.get(end).add(index);

            edges.add(new Edge(start, end, new int[]{time, cons}));
            index += 1;
        }
        numEdges = edges.size();

        isGoal = new ArrayList<>(Collections.nCopies(numNodes, false));

//        for (int i = 0; i < 1000; ++i) {
//            goals.add(i);
//            isGoal.set(i, true);
//        }

//         Read goal file
        isGoal = new ArrayList<>(Collections.nCopies(numNodes, false));
        File goalFile = new File(goalPath);
        Scanner goalScan = new Scanner(goalFile);
        while (goalScan.hasNextLine()) {
            int newGoal = Integer.parseInt(goalScan.nextLine());
            goals.add(newGoal);
            isGoal.set(newGoal, true);
        }
    }

    public Node getNode(int id) {
        return nodes.get(id);
    }
}
