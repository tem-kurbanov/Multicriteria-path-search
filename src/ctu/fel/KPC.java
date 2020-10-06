package ctu.fel;

import java.util.*;

public class KPC {
    private final int K;

    private int numCoverNodes;
    private List<Boolean> inCover;
    private Map<Integer, Map<Integer, List<CoverEdge>>> outgoingCoverEdges;

    /**
     * Constructor of the kPC cover (preferably initialized after the Planner class)
     * @param graph graph to build the cover off
     * @param k the cover path size
     */
    KPC(Graph graph, int k) {
        long startTime = System.nanoTime();

        K = k;

        System.out.println("Starting " + this.K + "-PC construction.");

        //Build vertex cover
        this.inCover = new ArrayList<>(Collections.nCopies(graph.getNumNodes(), true));
        this.numCoverNodes = graph.getNumNodes();
        for (int n = 0; n < graph.getNumNodes(); ++n) {
            if (n % 10000 == 0) {
                System.out.println(n);
            }

            // Keep goal vertices in the cover
            if (graph.isGoal(n)) {
                continue;
            }

            boolean stays = false;
            Set<LinkedList<Integer>> builtPaths = new HashSet<>();

            // Forward DFS
            ArrayDeque<LinkedList<Integer>> toExpand = new ArrayDeque<>();
            toExpand.push(new LinkedList<>(Arrays.asList(n)));

            while (!toExpand.isEmpty()) {
                LinkedList<Integer> currentPath = toExpand.pollFirst();
                int currentNode = currentPath.getLast();

                if (currentPath.size() + 1 == this.K) {
                    stays = true;
                    break;
                }

                // Extend current path
                for (Edge nextEdge : graph.getOutgoingEdges(currentNode)) {
                    int nextNode = nextEdge.getEndPoint();

                    // Prohibit loops
                    if (currentPath.contains(nextNode)) {
                        continue;
                    }

                    // New cover path is found
                    if (inCover.get(nextNode)) {
                        builtPaths.add(currentPath);
                        continue;
                    }

                    LinkedList<Integer> newPath = (LinkedList<Integer>) currentPath.clone();
                    newPath.addLast(nextNode);
                    toExpand.push(newPath);
                }
            }

            if (stays) {
                continue;
            }

            // Backward DFS for every path
            for (LinkedList<Integer> initialPath : builtPaths) {
                toExpand = new ArrayDeque<>();
                toExpand.push(initialPath);
                while (!toExpand.isEmpty()) {
                    LinkedList<Integer> currentPath = toExpand.pollFirst();
                    int currentNode = currentPath.getFirst();

                    if (currentPath.size() + 1 == this.K) {
                        stays = true;
                        break;
                    }

                    for (Edge previousEdge : graph.getIncomingEdges(currentNode)) {
                        int previousNode = previousEdge.getStartPoint();

                        // Prohibit loops
                        if (currentPath.contains(previousNode)) {
                            continue;
                        }

                        // New cover path is found
                        if (inCover.get(previousNode)) {
                            continue;
                        }

                        LinkedList newPath = (LinkedList) currentPath.clone();          // SLOW
                        newPath.addFirst(previousNode);
                        toExpand.push(newPath);
                    }
                }
            }

            if (!stays) {
                this.numCoverNodes -= 1;
                inCover.set(n, false);
            }
        }

        System.out.println("Initial number of vertices -- " + graph.getNumNodes());
        System.out.println("Number of cover vertices -- " + this.numCoverNodes);
        System.out.println("Relative vertex percentage -- " + ((float)this.numCoverNodes/(float)graph.getNumNodes() * 100));

        // Build cover edges
        int numInitCoverEdges = 0;
        int numCoverEdges = 0;
        int dominationPruned = 0;

        this.outgoingCoverEdges = new HashMap<>();
        for (int n = 0; n < graph.getNumNodes(); ++n) {
            if (!inCover.get(n)) {
                continue;
            }

            Map<Integer, List<CoverEdge>> tempCoverEdges = new HashMap<>();

            ArrayDeque<CoverEdge> toExpand = new ArrayDeque<>();
            toExpand.addFirst(new CoverEdge(n));

            while (!toExpand.isEmpty()) {
                CoverEdge currentCoverEdge = toExpand.pollFirst();
                int currentNode = currentCoverEdge.getEndPoint();

                for (Edge nextEdge : graph.getOutgoingEdges(currentNode)) {
                    int nextNode = nextEdge.getEndPoint();
                    // Prohibit loops
                    if (currentCoverEdge.getNodeSequence().contains(nextNode)) {
                        continue;
                    }

                    CoverEdge newCoverEdge = linkSuffixEdge(currentCoverEdge, nextEdge);

                    // Path is infeasible
                    if (Planner.constrained && Planner.infeasiblePath(newCoverEdge)) {
                        continue;
                    }

                    // A complete cover edge is found
                    if (inCover.get(nextNode)) {
                        if (!tempCoverEdges.containsKey(nextNode)) {
                            tempCoverEdges.put(nextNode, new ArrayList<>());
                        }
                        tempCoverEdges.get(nextNode).add(newCoverEdge);

                        numCoverEdges += 1;
                        numInitCoverEdges += 1;

                        continue;
                    }

                    toExpand.push(newCoverEdge);
                }
            }

            // Domination pruning
            for (Map.Entry<Integer, List<CoverEdge>> pair : tempCoverEdges.entrySet()) {
                List<CoverEdge> currentCoverEdges = pair.getValue();

                if (currentCoverEdges.size() < 2) {
                    continue;
                }

                TreeSet<Integer> toRemove = new TreeSet<>();
                boolean[] isDominated = new boolean[currentCoverEdges.size()];

                for (int i = 0; i < currentCoverEdges.size(); ++i) {
                    if (isDominated[i]) {
                        continue;
                    }
                    for (int j = 0; j < currentCoverEdges.size(); ++j) {
                        if (isDominated[j] || i == j) {
                            continue;
                        }

                        if (Planner.dominanceCheck(currentCoverEdges.get(i), currentCoverEdges.get(j))) {
                            isDominated[j] = true;
                            dominationPruned += 1;
                            toRemove.add(j);
                        }
                    }
                }

                if (!toRemove.isEmpty()) {
                    numCoverEdges -= toRemove.size();
                    for (int ind : toRemove.descendingSet()) {
                        currentCoverEdges.remove(ind);
                    }
                }
            }

            outgoingCoverEdges.put(n, tempCoverEdges);
        }

        System.out.println();
        System.out.println("Initial number of edges -- " + graph.getNumEdges());
        System.out.println("Initial number of cover edges -- " + numInitCoverEdges);
        System.out.println("Pruned number of cover edges -- " + numCoverEdges);
        System.out.println("Pruned by domination -- " + dominationPruned);
        System.out.println("% pruned edges -- " + (100 * (float)(numInitCoverEdges - numCoverEdges)/(float)numInitCoverEdges));
        System.out.println();
        System.out.println("Finished " + K + "-PC construction.");

        long endTime = System.nanoTime();
        System.out.println("Construction duration -- " + ((endTime - startTime) / 1000000000));
    }

    /**
     * Connects a suffix edge to a built path
     * @param coverEdge path ot be augmented
     * @param suffix    suffix segment
     * @return  augmented path
     */
    private CoverEdge linkSuffixEdge(CoverEdge coverEdge, Edge suffix) {
        Deque<Integer> nodeSequence = coverEdge.getNodeSequence();
        int[] parameters = new int[coverEdge.getParameters().length];

        nodeSequence.addLast(suffix.getEndPoint());

        // Path Parameter Update function
        parameters[0] = coverEdge.getTime() + suffix.getTime();       // time
        parameters[1] = Math.max(coverEdge.getMinSoCBefore(), coverEdge.getConsumption() + Math.max(0, suffix.getConsumption()));    // minSoCBefore
        parameters[2] = Math.max(coverEdge.getConsumption() + suffix.getConsumption(), coverEdge.getMinSoCBefore() -
                Math.min(Planner.getBatteryCapacity(), Planner.getBatteryCapacity() - suffix.getConsumption()));            // consumption
        parameters[3] = Math.min(coverEdge.getMaxSoCAfter() - suffix.getConsumption(), Math.min(Planner.getBatteryCapacity(),
                Planner.getBatteryCapacity() - suffix.getConsumption()));             // maxSoCAfter

        return new CoverEdge(nodeSequence, parameters);

    }

    public int getNumCoverNodes() {
        return numCoverNodes;
    }

    public boolean inCover(int node) {
        return inCover.get(node);
    }

    /**
     * Get the list of cover edges outgoing from a cover node
     * @param node  id of the node in consideration
     * @return  map of the form <end node, associated edges> if the node is in the cover, null otherwise
     */
    Map<Integer, List<CoverEdge>> getOutgoingCoverEdges(int node) {
        if (!inCover.get(node)) {
            return null;
        }
        return outgoingCoverEdges.get(node);
    }

}
