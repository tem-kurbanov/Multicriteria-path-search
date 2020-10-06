package ctu.fel;

import com.sun.source.tree.Tree;

import java.util.*;

public class Planner {

    private boolean useEpsilonDominance = false;
    private float[] epsilons;

    private boolean useEllipsePruning = false;
    private float ellipseCoefficient = 0.5f;
    private Node ellipseGoal;
    private float ellipseDistance;

    static final boolean constrained = true;
    private static final int batteryCapacity = 40000000;

    private static int numPathParameters = 4;
    private static final boolean[] maximize = new boolean[] {false, false, false, true};

    /**
     * Set the goal for the ellipse heuristic
     * @param g graph instance to check if the node is available
     * @param node goal node id
     * @throws Exception if the node id is above the number of nodes in the graph
     */
    void setEllipseGoal(Graph g, int node) throws Exception {
        if (node > g.getNumNodes()) {
            throw new Exception("Node " + node + " does not exist");
        }
        ellipseGoal = g.getNode(node);
    }

    /**
     * Turn on or off epsilon heuristic
     * @param use usage boolean
     * @return true if coefficients are set and the value is adopted, false otherwise
     */
    boolean setEpsilonUsage(boolean use) {
        if (use) {
            if (epsilons != null) {
                useEpsilonDominance = true;
                return true;
            }
            else {
                return false;
            }
//            epsilons = new float[]{0.999f, 0.999f, 0.999f, 0.999f};
        }
        else {
            useEpsilonDominance = false;
            return true;
        }
    }

    /**
     * Set coefficients of epsilon dominance
     * @param coefficients coefficients to be saved
     * @return true if coefficients are valid, false if not
     */
    boolean setEpsilonCoefficients(float[] coefficients) {
        if (coefficients.length != numPathParameters) {
            return false;
        }
        for (float c : coefficients) {
            if (c < 0 || c > 1) {
                return false;
            }
        }
        epsilons = coefficients;
        return true;
    }

    /**
     * Turn on or off ellipse heuristic
     * @param use usage setting
     * @return true if the goal is set and the value is adopted, false otherwise
     */
    boolean setEllipseUsage(boolean use) {
        if (ellipseGoal == null) {
            return false;
        }
        useEllipsePruning = use;
        return true;
    }

    /**
     * Set the distance coefficient for ellipse pruning
     * @param coefficient coefficient value
     * @return  true if the coefficient is valid, false otherwise
     */
    boolean setEllipseCoefficient(float coefficient) {
        if (ellipseCoefficient < 0) {
            return false;
        }
        ellipseCoefficient = coefficient;
        return true;
    }

    private float getDistance(Node n1, Node n2) {
        return (float)Math.sqrt(Math.pow(n2.getLatitudeProjected() - n1.getLatitudeProjected(), 2) +
                Math.pow(n2.getLongitudeProjected() - n1.getLongitudeProjected(), 2));
    }

    private boolean isInEllipse(Node source, Node currentNode) {
        return (getDistance(source, currentNode) + getDistance(currentNode, ellipseGoal) <=
                getDistance(source, ellipseGoal) + 2 * ellipseDistance);
    }

    static int getBatteryCapacity(){
        return batteryCapacity;
    }

    static boolean getMaximize(int index) {
        return maximize[index];
    }

    static int[] getInitialPathParameters() {
        return new int[]{0, 0, 0, batteryCapacity};
    }

    /**
     * If the search task is constrained, the constrain satisfaction of a path is tested here
     * @param path  path to be tested
     * @return  true if the path is infeasible, false otherwise
     */
    static boolean infeasiblePath(Path path) {
        return path.getMinSoCBefore() > batteryCapacity || path.getMaxSoCAfter() < 0;
    }

    /**
     * Main function for dominance check
     * Employs standard dominance or epsilon dominance depending on the variable useEpsilonDominance
     * @param l1    path to dominate
     * @param l2    path to be dominated
     * @return      true if l1 dominates l2, false otherwise
     */
    private boolean dominates(Label l1, Label l2) {
        if (useEpsilonDominance) {
            return epsilonDominanceCheck(l1, l2);
        }
        else {
            return dominanceCheck(l1, l2);
        }
    }

    private boolean epsilonDominanceCheck(Label l1, Label l2) {
        int[] l1Parameters = l1.getParameters();
        int[] l2Parameters = l2.getParameters();

        for (int p = 0; p < numPathParameters; ++p) {
            if (maximize[p]) {
                if (l1Parameters[p] < l2Parameters[p] * epsilons[p]) {
                    return false;
                }
            }
            else {
                if (l1Parameters[p] * epsilons[p] > l2Parameters[p]) {
                    return false;
                }
            }
        }
        return true;
    }

    public static boolean dominanceCheck(Path p1, Path p2) {
        int[] p1Parameters = p1.getParameters();
        int[] p2Parameters = p2.getParameters();

        for (int p = 0; p < numPathParameters; ++p) {
            if (maximize[p]) {
                if (p1Parameters[p] < p2Parameters[p]) {
                    return false;
                }
            }
            else {
                if (p1Parameters[p] > p2Parameters[p]) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Main function for t-discarding
     * Employs standard dominance or epsilon dominance depending on the variable useEpsilonDominance
     * @param tSet  tSet of the node in consideration
     * @param newLabel  the label to be dominated
     * @return  true if tSet dominates newLabel, false otherwise
     */
    private boolean tDiscards(TreeSet<Integer[]> tSet, Label newLabel) {
        if (useEpsilonDominance) {
            for (Integer[] t : tSet) {
                if (epsilonTDominanceCheck(t, newLabel)) {
                    return true;
                }
            }
        }
        else {
            for (Integer[] t : tSet) {
                if (tDominanceCheck(t, newLabel)) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean epsilonTDominanceCheck(Integer[] t, Label newLabel) {
        int[] parameters = newLabel.getParameters();
        for (int i = 0; i < t.length; ++i) {
            if (maximize[i + 1]) {
                if (t[i] < parameters[i + 1] * epsilons[i + 1]) {
                    return false;
                }
            }
            else {
                if (t[i] * epsilons[i + 1] > parameters[i + 1]) {
                    return false;
                }
            }
        }
        return true;
    }

    private boolean tDominanceCheck(Integer[] t, Label newLabel) {
        int[] parameters = newLabel.getParameters();
        for (int i = 0; i < t.length; ++i) {
            if (maximize[i + 1]) {
                if (t[i] < parameters[i + 1]) {
                    return false;
                }
            }
            else {
                if (t[i] > parameters[i + 1]) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * When a new closed label is produced, this function updates the tSet of the corresponding node
     * @param tSet  tSet to be updated
     * @param newLabel  the label to be added
     */
    private void updateTSet(TreeSet<Integer[]> tSet, Label newLabel) {
        // Extract new tVector
        int[] newTVector = Arrays.copyOfRange(newLabel.getParameters(), 1, numPathParameters);

        // Remove newly dominated entries
        Integer[] tVectorCopy = Arrays.stream(newTVector).boxed().toArray( Integer[]::new );
        TreeSet<Integer[]> toExamine = (TreeSet<Integer[]>) tSet.tailSet(tVectorCopy);
        List<Integer[]> toRemove = new LinkedList<>();
        for (Integer[] current : toExamine) {
            boolean dominates = true;
            for (int i = 0; i < tVectorCopy.length; ++i) {
                if (maximize[i + 1]) {
                    if (tVectorCopy[i] < current[i]) {
                        dominates = false;
                        break;
                    }
                }
                else {
                    if (tVectorCopy[i] > current[i]) {
                        dominates = false;
                        break;
                    }
                }
            }
            if (dominates) {
                toRemove.add(current);
            }
        }

        for (Integer[] current : toRemove) {
            tSet.remove(current);
        }
        tSet.add(tVectorCopy);
    }

    /**
     * Link a suffix segment to a path and calculate the combined parameters
     * @param currentLabel  path to be augmented
     * @param suffixObject  segment to be connected to the path (can be either Edge or CoverEdge)
     * @return  the parameter array of the connected path
     */
    private int[] linkPaths(Label currentLabel, Object suffixObject) {
        if (suffixObject instanceof Edge) {
            Edge suffixEdge = (Edge) suffixObject;
            int[] parameters = new int[numPathParameters];
            int suffixConsumption = suffixEdge.getConsumption();

            // Path Parameter Update function
            parameters[0] = currentLabel.getTime() +  suffixEdge.getTime();       // time
            parameters[1] = Math.max(currentLabel.getMinSoCBefore(), currentLabel.getConsumption() + Math.max(0, suffixConsumption));    // minSoCBefore
            parameters[2] = Math.max(currentLabel.getConsumption() + suffixConsumption, currentLabel.getMinSoCBefore() -
                    Math.min(batteryCapacity, batteryCapacity - suffixConsumption));            // consumption
            parameters[3] = Math.min(currentLabel.getMaxSoCAfter() - suffixConsumption, Math.min(batteryCapacity,
                    batteryCapacity - suffixConsumption));             // maxSoCAfter

            return parameters;
        }
        else {
            CoverEdge suffixEdge = (CoverEdge) suffixObject;
            int[] parameters = new int[numPathParameters];

            // Path Parameter Update function
            parameters[0] = currentLabel.getTime() + suffixEdge.getTime();       // time
            parameters[1] = Math.max(currentLabel.getMinSoCBefore(), currentLabel.getConsumption() + suffixEdge.getMinSoCBefore());    // minSoCBefore
            parameters[2] = Math.max(currentLabel.getConsumption() + suffixEdge.getConsumption(), currentLabel.getMinSoCBefore() -
                    suffixEdge.getMaxSoCAfter());            // consumption
            parameters[3] = Math.min(currentLabel.getMaxSoCAfter() - suffixEdge.getConsumption(), suffixEdge.getMaxSoCAfter());             // maxSoCAfter

            return parameters;
        }
    }

    /**
     * Standard MLS algorithm
     * @param graph graph to run the search on
     * @param source    the node id of the search source
     * @return  the map of Pareto sets of the goal nodes defined in graph
     */
    Map<Integer, Set<Label>> mls(Graph graph, int source) {
        long startTime = System.nanoTime();

        int numNodes = graph.getNumNodes();

        Node sourceNode = graph.getNode(source);
        if (useEllipsePruning) {
            ellipseDistance = getDistance(sourceNode, ellipseGoal) * ellipseCoefficient;
        }

        List<Set<Label>> closedLabels = new ArrayList<>(numNodes);
        List<Set<Label>> openLabels = new ArrayList<>(numNodes);

        for (int i = 0; i < numNodes; ++i) {
            closedLabels.add(new HashSet<>());
            openLabels.add(new HashSet<>());
        }

        TreeSet<Label> toExpand = new TreeSet<>();
        toExpand.add(new Label(source, getInitialPathParameters()));

        int numIterations = 0;

        while (!toExpand.isEmpty()) {
            Label currentLabel = toExpand.pollFirst();
            int currentNode = currentLabel.getNode();

            if (numIterations % 100000 == 0) {
                System.out.println(toExpand.size());
            }
            numIterations += 1;

            openLabels.get(currentNode).remove(currentLabel);
            closedLabels.get(currentNode).add(currentLabel);

            for (Edge nextEdge : graph.getOutgoingEdges(currentNode)) {
                int nextNode = nextEdge.getEndPoint();
                // Ellipse pruning
                if (useEllipsePruning && !isInEllipse(sourceNode, graph.getNode(nextNode))) {
                    continue;
                }

                Label newLabel = new Label(nextNode, linkPaths(currentLabel, nextEdge));

                // Infeasible path
                if (constrained && infeasiblePath(newLabel)) {
                    continue;
                }

                boolean isDominated = false;

                // Dominance by closed labels
                Set<Label> currentClosedLabels = closedLabels.get(nextNode);
                for (Label closed : currentClosedLabels) {
                    if (dominates(closed, newLabel)) {
                        isDominated = true;
                        break;
                    }
                }
                if (isDominated) {
                    continue;
                }

                // Dominance by and of open labels
                List<Label> toRemove = new ArrayList<>();
                Set<Label> currentOpenLabels = openLabels.get(nextNode);
                for (Label open : currentOpenLabels) {
                    if (dominates(open, newLabel)) {
                        isDominated = true;
                        break;
                    }
                    else if (dominates(newLabel, open)) {
                        toRemove.add(open);
                    }
                }
                if (isDominated) {
                    continue;
                }

                // Remove dominated labels
                for (Label label : toRemove) {
                    currentOpenLabels.remove(label);
                    toExpand.remove(label);
                }

                toExpand.add(newLabel);
                currentOpenLabels.add(newLabel);
            }
        }
        Map<Integer, Set<Label>> paretoSets = new HashMap<>();
        for (int nodeIndex : graph.getGoals()) {
            paretoSets.put(nodeIndex, closedLabels.get(nodeIndex));
        }

        long endTime = System.nanoTime();
        long  queryDuration = ((endTime - startTime) / 1000000000);
        System.out.println("Query duration -- " + queryDuration);

        return paretoSets;
    }

    /**
     * MLS with t-discarding procedure
     * @param graph graph to run the search on
     * @param source    the node id of the search source
     * @return  the map of Pareto sets of the goal nodes defined in graph
     */
    Map<Integer, Set<Label>> tMls(Graph graph, int source) {
        long startTime = System.nanoTime();

        int numNodes = graph.getNumNodes();

        Node sourceNode = graph.getNode(source);
        if (useEllipsePruning) {
            ellipseDistance = getDistance(sourceNode, ellipseGoal) * ellipseCoefficient;
        }

        List<Set<Label>> closedLabels = new ArrayList<>(numNodes);
        List<Set<Label>> openLabels = new ArrayList<>(numNodes);
        // Arrays for tDiscarding
        List<TreeSet<Integer[]>> tSets = new ArrayList<>(numNodes);
//        int[] maxFirstParameter = new int[numNodes];

        for (int i = 0; i < numNodes; ++i) {
            closedLabels.add(new HashSet<>());
            openLabels.add(new HashSet<>());
            tSets.add(new TreeSet<>((integers, t1) -> {
                for (int t = 0; t < integers.length; ++t) {
                    if (integers[t] != t1[t]) {
                        if (maximize[t + 1]) {
                            return Integer.compare(t1[t], integers[t]);
                        }
                        else {
                            return Integer.compare(integers[t], t1[t]);
                        }
                    }
                }
                return 0;
            }));
//            maxFirstParameter[i] = 0;
        }

        TreeSet<Label> toExpand = new TreeSet<>();
        toExpand.add(new Label(source, getInitialPathParameters()));

        int numIterations = 0;

        while (!toExpand.isEmpty()) {
            Label currentLabel = toExpand.pollFirst();
            int currentNode = currentLabel.getNode();

            if (numIterations % 100000 == 0) {
                System.out.println(toExpand.size());
            }
            numIterations += 1;

            openLabels.get(currentNode).remove(currentLabel);
            closedLabels.get(currentNode).add(currentLabel);
            updateTSet(tSets.get(currentNode), currentLabel);

            // Update maximum value of the first parameter
//            if (maxFirstParameter[currentNode] < currentLabel.getTime())
//                maxFirstParameter[currentNode] = currentLabel.getTime();

            for (Edge nextEdge : graph.getOutgoingEdges(currentNode)) {
                int nextNode = nextEdge.getEndPoint();
                // Ellipse pruning
                if (useEllipsePruning && !isInEllipse(sourceNode, graph.getNode(nextNode))) {
                    continue;
                }

                Label newLabel = new Label(nextNode, linkPaths(currentLabel, nextEdge));

                // Infeasible path
                if (constrained && infeasiblePath(newLabel)) {
                    continue;
                }

                boolean isDominated = false;

                // tDiscarding by closed labels
                if (tDiscards(tSets.get(nextNode), newLabel)) {
                    continue;
                }

                // Dominance by and of open labels
                List<Label> toRemove = new ArrayList<>();
                Set<Label> currentOpenLabels = openLabels.get(nextNode);
                for (Label open : currentOpenLabels) {
                    if (dominates(open, newLabel)) {
                        isDominated = true;
                        break;
                    }
                    else if (dominates(newLabel, open)) {
                        toRemove.add(open);
                    }
                }
                if (isDominated) {
                    continue;
                }

                // Remove dominated labels
                for (Label label : toRemove) {
                    currentOpenLabels.remove(label);
                    toExpand.remove(label);
                }

                toExpand.add(newLabel);
                currentOpenLabels.add(newLabel);
            }
        }
        Map<Integer, Set<Label>> paretoSets = new HashMap<>();
        for (int nodeIndex : graph.getGoals()) {
            paretoSets.put(nodeIndex, closedLabels.get(nodeIndex));
        }

        long endTime = System.nanoTime();
        long  queryDuration = ((endTime - startTime) / 1000000000);
        System.out.println("Query duration -- " + queryDuration);

        return paretoSets;
    }

    /**
     * MLS on a kPC cover of the graph
     * @param graph the graph to run the search on
     * @param kpc   the KPC cover of the graph
     * @param source    the node id of the search source
     * @return  the map of Pareto sets of the goal nodes defined in graph
     */
    Map<Integer, Set<Label>> kpcMls(Graph graph, KPC kpc, int source) {
        long startTime = System.nanoTime();

        int numNodes = graph.getNumNodes();

        Node sourceNode = graph.getNode(source);
        if (useEllipsePruning) {
            ellipseDistance = getDistance(sourceNode, ellipseGoal) * ellipseCoefficient;
        }

        Map<Integer, Set<Label>> closedLabels = new HashMap<>(kpc.getNumCoverNodes());
        Map<Integer, Set<Label>> openLabels = new HashMap<>(kpc.getNumCoverNodes());

        for (int i = 0; i < numNodes; ++i) {
            if (kpc.inCover(i)) {
                closedLabels.put(i, new HashSet<>());
                openLabels.put(i, new HashSet<>());
            }
        }

        TreeSet<Label> toExpand = connectSourceToKpc(graph, kpc, source, openLabels, closedLabels);

        int numIterations = 0;

        while (!toExpand.isEmpty()) {
            Label currentLabel = toExpand.pollFirst();
            int currentNode = currentLabel.getNode();

            if (numIterations % 100000 == 0) {
                System.out.println(toExpand.size());
            }
            numIterations += 1;

            openLabels.get(currentNode).remove(currentLabel);
            closedLabels.get(currentNode).add(currentLabel);

            // Expand to every neighbor in kPC graph
            for (Map.Entry<Integer, List<CoverEdge>> entry : kpc.getOutgoingCoverEdges(currentNode).entrySet()) {
                int nextNode = entry.getKey();
                List<CoverEdge> availableEdges = entry.getValue();

                // Ellipse pruning
                if (useEllipsePruning && !isInEllipse(sourceNode, graph.getNode(nextNode))) {
                    continue;
                }

                // Expand across every edge
                for (CoverEdge nextEdge : availableEdges) {
                    Label newLabel = new Label(nextNode, linkPaths(currentLabel, nextEdge));

                    // Infeasible path
                    if (constrained && infeasiblePath(newLabel)) {
                        continue;
                    }

                    boolean isDominated = false;

                    // Dominance by closed labels
                    Set<Label> currentClosedLabels = closedLabels.get(nextNode);
                    for (Label closed : currentClosedLabels) {
                        if (dominates(closed, newLabel)) {
                            isDominated = true;
                            break;
                        }
                    }
                    if (isDominated) {
                        continue;
                    }

                    // Dominance by and of open labels
                    List<Label> toRemove = new ArrayList<>();
                    Set<Label> currentOpenLabels = openLabels.get(nextNode);
                    for (Label open : currentOpenLabels) {
                        if (dominates(open, newLabel)) {
                            isDominated = true;
                            break;
                        } else if (dominates(newLabel, open)) {
                            toRemove.add(open);
                        }
                    }
                    if (isDominated) {
                        continue;
                    }

                    // Remove dominated labels
                    for (Label label : toRemove) {
                        currentOpenLabels.remove(label);
                        toExpand.remove(label);
                    }

                    toExpand.add(newLabel);
                    currentOpenLabels.add(newLabel);
                }
            }

        }
        Map<Integer, Set<Label>> paretoSets = new HashMap<>();
        for (int nodeIndex : graph.getGoals()) {
            paretoSets.put(nodeIndex, closedLabels.get(nodeIndex));
        }

        long endTime = System.nanoTime();
        long  queryDuration = ((endTime - startTime) / 1000000000);
        System.out.println("Query duration -- " + queryDuration);

        return paretoSets;
    }

    /**
     * MLS on a kPC cover with t-discarding
     * @param graph the graph to run the search on
     * @param kpc   the KPC cover of the graph
     * @param source    the node id of the search source
     * @return  the map of Pareto sets of the goal nodes defined in graph
     */
    Map<Integer, Set<Label>> tKpcMls(Graph graph, KPC kpc, int source) {
        long startTime = System.nanoTime();

        int numNodes = graph.getNumNodes();

        Node sourceNode = graph.getNode(source);
        if (useEllipsePruning) {
            ellipseDistance = getDistance(sourceNode, ellipseGoal) * ellipseCoefficient;
        }

        Map<Integer, Set<Label>> closedLabels = new HashMap<>(kpc.getNumCoverNodes());
        Map<Integer, Set<Label>> openLabels = new HashMap<>(kpc.getNumCoverNodes());
        // Arrays for tDiscarding
        Map<Integer, TreeSet<Integer[]>> tSets = new HashMap<>(kpc.getNumCoverNodes());
//        Map<Integer, Integer> maxFirstParameter = new HashMap<>(kpc.getNumCoverNodes());

        for (int i = 0; i < numNodes; ++i) {
            if (kpc.inCover(i)) {
                closedLabels.put(i, new HashSet<>());
                openLabels.put(i, new HashSet<>());
                tSets.put(i, new TreeSet<>((integers, t1) -> {
                    for (int t = 0; t < integers.length; ++t) {
                        if (integers[t] != t1[t]) {
                            if (maximize[t + 1]) {
                                return Integer.compare(t1[t], integers[t]);
                            }
                            else {
                                return Integer.compare(integers[t], t1[t]);
                            }
                        }
                    }
                    return 0;
                }));
//            maxFirstParameter[i] = 0;
            }
        }

        TreeSet<Label> toExpand = connectSourceToKpc(graph, kpc, source, openLabels, closedLabels);
        for (Map.Entry<Integer, Set<Label>> entry : closedLabels.entrySet()) {
            for (Label lab : entry.getValue()) {
                updateTSet(tSets.get(entry.getKey()), lab);
            }
        }

        int numIterations = 0;

        while (!toExpand.isEmpty()) {
            Label currentLabel = toExpand.pollFirst();
            int currentNode = currentLabel.getNode();

            if (numIterations % 100000 == 0) {
                System.out.println(toExpand.size());
            }
            numIterations += 1;

            openLabels.get(currentNode).remove(currentLabel);
            closedLabels.get(currentNode).add(currentLabel);
            updateTSet(tSets.get(currentNode), currentLabel);

            // Expand to every neighbor in kPC graph
            for (Map.Entry<Integer, List<CoverEdge>> entry : kpc.getOutgoingCoverEdges(currentNode).entrySet()) {
                int nextNode = entry.getKey();
                List<CoverEdge> availableEdges = entry.getValue();

                // Ellipse pruning
                if (useEllipsePruning && !isInEllipse(sourceNode, graph.getNode(nextNode))) {
                    continue;
                }

                // Expand across every edge
                for (CoverEdge nextEdge : availableEdges) {
                    Label newLabel = new Label(nextNode, linkPaths(currentLabel, nextEdge));

                    // Infeasible path
                    if (constrained && infeasiblePath(newLabel)) {
                        continue;
                    }

                    boolean isDominated = false;

                    // tDiscarding by closed labels
                    if (tDiscards(tSets.get(nextNode), newLabel)) {
                        continue;
                    }

                    // Dominance by and of open labels
                    List<Label> toRemove = new ArrayList<>();
                    Set<Label> currentOpenLabels = openLabels.get(nextNode);
                    for (Label open : currentOpenLabels) {
                        if (dominates(open, newLabel)) {
                            isDominated = true;
                            break;
                        } else if (dominates(newLabel, open)) {
                            toRemove.add(open);
                        }
                    }
                    if (isDominated) {
                        continue;
                    }

                    // Remove dominated labels
                    for (Label label : toRemove) {
                        currentOpenLabels.remove(label);
                        toExpand.remove(label);
                    }

                    toExpand.add(newLabel);
                    currentOpenLabels.add(newLabel);
                }
            }

        }
        Map<Integer, Set<Label>> paretoSets = new HashMap<>();
        for (int nodeIndex : graph.getGoals()) {
            paretoSets.put(nodeIndex, closedLabels.get(nodeIndex));
        }

        long endTime = System.nanoTime();
        long  queryDuration = ((endTime - startTime) / 1000000000);
        System.out.println("Query duration -- " + queryDuration);

        return paretoSets;
    }

    /**
     * If the search node is node in the cover, this function extends the paths from the source
     * to the neighbor cover vertices
     * @param graph graph to run the search on
     * @param kpc   KPC cover of the graph
     * @param source    source node id
     * @param openLabels    map of open labels of the overall search
     * @param closedLabels  map of closed labels of the overall search
     * @return  lexicographically ordered queue of labels to extend in kPC search
     */
    private TreeSet<Label> connectSourceToKpc(Graph graph, KPC kpc, int source, Map<Integer, Set<Label>> openLabels, Map<Integer, Set<Label>> closedLabels) {
        TreeSet<Label> toExpand = new TreeSet<>();
        Label sourceLabel = new Label(source, getInitialPathParameters());
        if (kpc.inCover(source)) {
            toExpand.add(sourceLabel);
            openLabels.get(source).add(sourceLabel);
            return toExpand;
        }
        else {
            Map<Integer, Set<Label>> tempClosedLabels = new HashMap<>();
            Map<Integer, Set<Label>> tempOpenLabels = new HashMap<>();
            tempOpenLabels.put(source, new TreeSet<>(Collections.singletonList(sourceLabel)));
            tempClosedLabels.put(source, new TreeSet<>());

            Node sourceNode = graph.getNode(source);
            if (useEllipsePruning) {
                ellipseDistance = getDistance(sourceNode, ellipseGoal) * ellipseCoefficient;
            }

            toExpand.add(sourceLabel);

            while (!toExpand.isEmpty()) {
                Label currentLabel = toExpand.pollFirst();
                int currentNode = currentLabel.getNode();

                tempOpenLabels.get(currentNode).remove(currentLabel);
                tempClosedLabels.get(currentNode).add(currentLabel);

                for (Edge nextEdge : graph.getOutgoingEdges(currentNode)) {
                    int nextNode = nextEdge.getEndPoint();

                    // Ellipse pruning
                    if (useEllipsePruning && !isInEllipse(sourceNode, graph.getNode(nextNode))) {
                        continue;
                    }

                    Label newLabel = new Label(nextNode, linkPaths(currentLabel, nextEdge));
                    if (constrained && infeasiblePath(newLabel)) {
                        continue;
                    }

                    if (!tempOpenLabels.containsKey(nextNode)) {
                        tempOpenLabels.put(nextNode, new TreeSet<>());
                        tempClosedLabels.put(nextNode, new TreeSet<>());
                    }

                    // Domination check by closed labels
                    boolean isDominated = false;
                    Set<Label> currentClosedLabels = tempClosedLabels.get(nextNode);
                    for (Label closed : currentClosedLabels) {
                        if (dominates(closed, newLabel)) {
                            isDominated = true;
                            break;
                        }
                    }
                    if (isDominated) {
                        continue;
                    }

                    // Domination check by and of open labels
                    List<Label> toRemove = new ArrayList<>();
                    Set<Label> currentOpenLabels = tempOpenLabels.get(nextNode);
                    for (Label open : currentOpenLabels) {
                        if (dominates(open, newLabel)) {
                            isDominated = false;
                        }
                        else if (dominates(newLabel, open)) {
                            toRemove.add(open);
                        }
                    }
                    if (isDominated) {
                        continue;
                    }
                    for (Label label : toRemove) {
                        toExpand.remove(label);
                        currentOpenLabels.remove(label);
                    }

                    currentOpenLabels.add(newLabel);
                    if (!kpc.inCover(nextNode)) {
                        toExpand.add(newLabel);
                    }
                }
            }

            for (Integer node : tempOpenLabels.keySet()) {
                if (kpc.inCover(node)) {
                    openLabels.put(node, tempOpenLabels.get(node));
                    closedLabels.put(node, tempClosedLabels.get(node));
                    toExpand.addAll(tempOpenLabels.get(node));
                }
            }

            return toExpand;
        }
    }
}
