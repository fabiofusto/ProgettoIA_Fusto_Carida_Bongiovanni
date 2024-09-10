package fr.uga.pddl4j.examples;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.*;
import fr.uga.pddl4j.planners.statespace.search.StateSpaceSearch;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;

import java.util.Comparator;
import java.util.Map;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;


@CommandLine.Command(name = "MyAStarPlanner",
        version = "ASP 1.0",
        description = "Solves a specified planning problem using A* search strategy.",
        sortOptions = false,
        mixinStandardHelpOptions = true,
        headerHeading = "Usage:%n",
        synopsisHeading = "%n",
        descriptionHeading = "%nDescription:%n%n",
        parameterListHeading = "%nParameters:%n",
        optionListHeading = "%nOptions:%n")
public class MyAStarPlanner extends AbstractPlanner {
    /**
     * The class logger.
     */
    private static final Logger LOGGER = LogManager.getLogger(MyAStarPlanner.class.getName());

    /**
     * The HEURISTIC property used for planner configuration.
     */
    public static final String HEURISTIC_SETTING = "HEURISTIC";

    /**
     * The default value of the HEURISTIC property used for planner configuration.
     */
    public static final StateHeuristic.Name DEFAULT_HEURISTIC = StateHeuristic.Name.FAST_FORWARD;

    /**
     * The WEIGHT_HEURISTIC property used for planner configuration.
     */
    public static final String WEIGHT_HEURISTIC_SETTING = "WEIGHT_HEURISTIC";

    /**
     * The default value of the WEIGHT_HEURISTIC property used for planner configuration.
     */
    public static final double DEFAULT_WEIGHT_HEURISTIC = 1.0;

    /**
     * The weight of the heuristic.
     */
    private double heuristicWeight;

    /**
     * The name of the heuristic used by the planner.
     */
    private StateHeuristic.Name heuristic;

    /**
     * Creates a new A* search planner with the default configuration.
     */
    public MyAStarPlanner() {
        this(MyAStarPlanner.getDefaultConfiguration());
    }

    private int numOfStatesEvaluated=0;

    /**
     * Creates a new A* search planner with a specified configuration.
     *
     * @param configuration the configuration of the planner.
     */
    public MyAStarPlanner(final PlannerConfiguration configuration) {
        super();
        this.setConfiguration(configuration);
    }

    /**
     * Sets the weight of the heuristic.
     *
     * @param weight the weight of the heuristic. The weight must be greater than 0.
     * @throws IllegalArgumentException if the weight is strictly less than 0.
     */
    @CommandLine.Option(names = {"-w", "--weight"}, defaultValue = "1.0",
            paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        if (weight <= 0) {
            throw new IllegalArgumentException("Weight <= 0");
        }
        this.heuristicWeight = weight;
    }

    /**
     * Set the name of heuristic used by the planner to the solve a planning problem.
     *
     * @param heuristic the name of the heuristic.
     */
    @CommandLine.Option(names = {"-e", "--heuristic"}, defaultValue = "FAST_FORWARD",
            description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
                    + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }

    /**
     * Returns the name of the heuristic used by the planner to solve a planning problem.
     *
     * @return the name of the heuristic used by the planner to solve a planning problem.
     */
    public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }

    /**
     * Returns the weight of the heuristic.
     *
     * @return the weight of the heuristic.
     */
    public final double getHeuristicWeight() {
        return this.heuristicWeight;
    }

    /**
     * Instantiates the planning problem from a parsed problem.
     *
     * @param problem the problem to instantiate.
     * @return the instantiated planning problem or null if the problem cannot be instantiated.
     */
    @Override
    public Problem instantiate(DefaultParsedProblem problem) {
        final Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }

    /**
     * Search a solution plan to a specified domain and problem using A*.
     *
     * @param problem the problem to solve.
     * @return the plan found or null if no plan was found.
     */
    public Plan solve(final Problem problem) throws ProblemNotSupportedException {
        LOGGER.info("* Starting A* search \n");
        // Search a solution
        final long begin = System.currentTimeMillis();
        final Plan plan = this.aStar(problem);
        final long end = System.currentTimeMillis();
        // If a plan is found update the statistics of the planner
        // and log search information
        if (plan != null) {
            LOGGER.info("* A* search succeeded\n");
            LOGGER.info("Number of states evaluated : " + this.numOfStatesEvaluated);
            this.getStatistics().setTimeToSearch(end - begin);
        } else {
            LOGGER.info("* A* search failed\n");
        }
        // Return the plan found or null if the search fails.
        return plan;
    }

    /**
     * Checks the planner configuration and returns if the configuration is valid.
     * A configuration is valid if (1) the domain and the problem files exist and
     * can be read, (2) the timeout is greater than 0, (3) the weight of the
     * heuristic is greater than 0 and (4) the heuristic is a not null.
     *
     * @return <code>true</code> if the configuration is valid <code>false</code> otherwise.
     */
    public boolean hasValidConfiguration() {
        return super.hasValidConfiguration()
                && this.getHeuristicWeight() > 0.0
                && this.getHeuristic() != null;
    }

    /**
     * This method return the default arguments of the planner.
     *
     * @return the default arguments of the planner.
     * @see PlannerConfiguration
     */
    public static PlannerConfiguration getDefaultConfiguration() {
        PlannerConfiguration config =Planner.getDefaultConfiguration();
        config.setProperty(MyAStarPlanner.HEURISTIC_SETTING, MyAStarPlanner.DEFAULT_HEURISTIC.toString());
        config.setProperty(MyAStarPlanner.WEIGHT_HEURISTIC_SETTING,
                Double.toString(MyAStarPlanner.DEFAULT_WEIGHT_HEURISTIC));
        return config;
    }

    /**
     * Returns the configuration of the planner.
     *
     * @return the configuration of the planner.
     */
    @Override
    public PlannerConfiguration getConfiguration() {
        final PlannerConfiguration config = super.getConfiguration();
        config.setProperty(MyAStarPlanner.HEURISTIC_SETTING, this.getHeuristic().toString());
        config.setProperty(MyAStarPlanner.WEIGHT_HEURISTIC_SETTING, Double.toString(this.getHeuristicWeight()));
        return config;
    }

    /**
     * Sets the configuration of the planner. If a planner setting is not defined in
     * the specified configuration, the setting is initialized with its default value.
     *
     * @param configuration the configuration to set.
     */
    @Override
    public void setConfiguration(final PlannerConfiguration configuration) {
        super.setConfiguration(configuration);
        if (configuration.getProperty(MyAStarPlanner.WEIGHT_HEURISTIC_SETTING) == null) {
            this.setHeuristicWeight(MyAStarPlanner.DEFAULT_WEIGHT_HEURISTIC);
        } else {
            this.setHeuristicWeight(Double.parseDouble(configuration.getProperty(
                    MyAStarPlanner.WEIGHT_HEURISTIC_SETTING)));
        }
        if (configuration.getProperty(MyAStarPlanner.HEURISTIC_SETTING) == null) {
            this.setHeuristic(MyAStarPlanner.DEFAULT_HEURISTIC);
        } else {
            this.setHeuristic(StateHeuristic.Name.valueOf(configuration.getProperty(
                    MyAStarPlanner.HEURISTIC_SETTING)));
        }
    }

    /**
     * Search a solution plan for a planning problem using an A* search strategy.
     *
     * @param problem the problem to solve.
     * @return a plan solution for the problem or null if there is no solution
     * @throws ProblemNotSupportedException if the problem to solve is not supported by the planner.

     */

    public Plan aStar(Problem problem) throws ProblemNotSupportedException {
    final StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);
    final State init = new State(problem.getInitialState());
    final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));

    final Map<State, Double> closed = new HashMap<>(); // Usa una HashMap per memorizzare i costi minimi degli stati
    final double weight = this.getHeuristicWeight();
    final PriorityQueue<Node> open = new PriorityQueue<>(100,
            (n1, n2) -> Double.compare(weight * n1.getHeuristic() + n1.getCost(), weight * n2.getHeuristic() + n2.getCost())
    );

    open.add(root);

    final int timeout = this.getTimeout() * 1000;
    long time = System.currentTimeMillis();

    while (!open.isEmpty() && (System.currentTimeMillis() - time) < timeout) {
        final Node current = open.poll();
        this.numOfStatesEvaluated++;

        if (current.satisfy(problem.getGoal())) {
            return extractPlan(current, problem);
        }

        // Se lo stato è già stato visitato con un costo inferiore, salta questo nodo
        if (closed.containsKey(current) && closed.get(current) <= current.getCost()) {
            continue;
        }

        closed.put(current, current.getCost()); // Aggiorna il costo minore per lo stato corrente

        for (Action a : problem.getActions()) {
            if (a.isApplicable(current)) {
                Node next = new Node(current);

                for (ConditionalEffect ce : a.getConditionalEffects()) {
                    if (current.satisfy(ce.getCondition())) {
                        next.apply(ce.getEffect());
                    }
                }

                next.setCost(current.getCost() + 1);
                next.setParent(current);
                next.setAction(problem.getActions().indexOf(a));
                next.setHeuristic(heuristic.estimate(next, problem.getGoal()));

                if (!closed.containsKey(next) || closed.get(next) > next.getCost()) {
                    open.add(next);
                }
            }
        }
    }

    return null; // Nessun piano trovato entro il timeout
}


/*

    public Plan idaStar(Problem problem) throws ProblemNotSupportedException {
        // Check if the problem is supported by the planner
        if (!this.isSupported(problem)) {
            throw new ProblemNotSupportedException("Problem not supported");
        }

        // Initialize the heuristic and root node
        final StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);
        final State init = new State(problem.getInitialState());
        final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));

        // Set the initial threshold to the heuristic estimate of the root
        double threshold = root.getHeuristic();
        final double weight = this.getHeuristicWeight();
        final int timeout = this.getTimeout() * 1000;
        long startTime = System.currentTimeMillis();

        while (true) {
            Node[] solutionNode = new Node[1]; // Array to hold the solution node
            double temp = search(root, 0, threshold, problem, heuristic, weight, solutionNode, startTime, timeout);
            if (temp == -1) {
                return extractPlan(solutionNode[0], problem); // Solution found
            }
            if (temp == Double.MAX_VALUE) {
                return null; // No solution found
            }
            threshold = temp; // Update threshold
        }
    }

    private double search(Node node, double g, double threshold, Problem problem, StateHeuristic heuristic, double weight, Node[] solutionNode, long startTime, int timeout) {
        double f = g + weight * node.getHeuristic();
        if (f > threshold) {
            return f;
        }
        if (System.currentTimeMillis() - startTime >= timeout) {
            return Double.MAX_VALUE; // Timeout
        }
        if (node.satisfy(problem.getGoal())) {
            solutionNode[0] = node; // Solution found
            return -1;
        }

        double min = Double.MAX_VALUE;
        for (int i = 0; i < problem.getActions().size(); i++) {
            Action a = problem.getActions().get(i);
            if (a.isApplicable(node)) {
                Node next = new Node(node);

                for (ConditionalEffect ce : a.getConditionalEffects()) {
                    if (node.satisfy(ce.getCondition())) {
                        next.apply(ce.getEffect());
                    }
                }

                next.setCost(node.getCost() + 1);
                next.setParent(node);
                next.setAction(i);
                next.setHeuristic(heuristic.estimate(next, problem.getGoal()));

                double temp = search(next, g + 1, threshold, problem, heuristic, weight, solutionNode, startTime, timeout);
                if (temp == -1) {
                    return -1; // Solution found
                }
                if (temp < min) {
                    min = temp;
                }
            }
        }
        return min;
    }
*/

    // Finally, we return the search computed or null if no search was found


    /**
     * Extracts a search from a specified node.
     *
     * @param node    the node.
     * @param problem the problem.
     * @return the search extracted from the specified node.
     */
    private Plan extractPlan(final Node node, final Problem problem) {
        Node n = node;
        final Plan plan = new SequentialPlan();
        while (n.getAction() != -1) {
            final Action a = problem.getActions().get(n.getAction());
            plan.add(0, a);
            n = n.getParent();
        }
        return plan;
    }

    /**
     * Returns if a specified problem is supported by the planner. Just ADL problem can be solved by this planner.
     *
     * @param problem the problem to test.
     * @return <code>true</code> if the problem is supported <code>false</code> otherwise.
     */
    @Override
    public boolean isSupported(Problem problem) {
        return !problem.getRequirements().contains(RequireKey.ACTION_COSTS)
                && !problem.getRequirements().contains(RequireKey.CONSTRAINTS)
                && !problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
                && !problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
                && !problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
                && !problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
                && !problem.getRequirements().contains(RequireKey.FLUENTS)
                && !problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
                && !problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
                && !problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
                && !problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
                && !problem.getRequirements().contains(RequireKey.PREFERENCES)
                && !problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
                && !problem.getRequirements().contains(RequireKey.HIERARCHY);
    }


    public static void main(String[] args) {
        try {
            final MyAStarPlanner planner = new MyAStarPlanner();
            CommandLine cmd = new CommandLine(planner);
            cmd.execute(args);
        } catch (IllegalArgumentException e) {
            LOGGER.fatal(e.getMessage());
        }
    }

}