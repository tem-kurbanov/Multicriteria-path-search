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
     * @param distancePath  path to the edge distance file
     * @param timePath  path to the edge time file
     * @throws FileNotFoundException    if any of the files is not found
     */
//    Graph(String nodePath, String edgePath, String goalPath) throws FileNotFoundException {
    Graph(String distancePath, String timePath) throws FileNotFoundException {
        nodes = new ArrayList<>();
        edges = new ArrayList<>();
        goals = new ArrayList<>();

        outgoingEdgeIndices = new ArrayList<>();
        incomingEdgeIndices = new ArrayList<>();

        //Open input distance file
        File distanceFile = new File(distancePath);
        Scanner distanceScan = new Scanner(distanceFile);

        distanceScan.nextLine();
        distanceScan.nextLine();
        distanceScan.nextLine();
        distanceScan.nextLine();
        String[] splitSizes = distanceScan.nextLine().split(" ");

        numNodes = Integer.parseInt(splitSizes[2]);
        numEdges = Integer.parseInt(splitSizes[3]);

        distanceScan.nextLine();
        distanceScan.nextLine();

        // Open input time file
        File timeFile = new File(timePath);
        Scanner timeScan = new Scanner(timeFile);

        timeScan.nextLine();
        timeScan.nextLine();
        timeScan.nextLine();
        timeScan.nextLine();
        timeScan.nextLine();
        timeScan.nextLine();
        timeScan.nextLine();

        for (int i = 0; i < numNodes; ++i) {
            outgoingEdgeIndices.add(new ArrayList<>());
            incomingEdgeIndices.add(new ArrayList<>());
        }

        int index = 0;
        while (distanceScan.hasNextLine()) {
            String[] split = distanceScan.nextLine().split(" ");
            int start = Integer.parseInt(split[1]) - 1;
            int end = Integer.parseInt(split[2]) - 1;
            int distance = Integer.parseInt(split[3]);
            int time = Integer.parseInt(timeScan.nextLine().split(" ")[3]);

            outgoingEdgeIndices.get(start).add(index);
            incomingEdgeIndices.get(end).add(index);

            edges.add(new Edge(start, end, new int[]{distance, time}));
            index += 1;
        }

        isGoal = new ArrayList<>(Collections.nCopies(numNodes, false));

        for (int i = 0; i < numNodes; i += 320) {
            goals.add(i);
            isGoal.set(i, true);
        }

//         Read goal file
//        isGoal = new ArrayList<>(Collections.nCopies(numNodes, false));
//        File goalFile = new File(goalPath);
//        Scanner goalScan = new Scanner(goalFile);
//        while (goalScan.hasNextLine()) {
//            int newGoal = Integer.parseInt(goalScan.nextLine());
//            goals.add(newGoal);
//            isGoal.set(newGoal, true);
//        }
    }

    public Node getNode(int id) {
        return nodes.get(id);
    }
}
