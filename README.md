# ProgettoIA_Fusto_Carida_Bongiovanni

## Task 2

### Compilazione del planner
```sh
$ cd my_planner
$ javac -Xlint:-options -d classes -cp lib/pddl4j-4.0.0.jar src/fr/uga/pddl4j/examples/*.java
```

### Esecuzione del planner
```sh
$ cd my_planner
$ java -cp "classes;lib\pddl4j-4.0.0.jar" fr.uga.pddl4j.examples.MyAStarPlanner <file dominio> <file problema> -e <euristica> -w <peso> -t <timeout>
```

## Task 3

### Terminale Server
```sh
$ cd robotic_planning
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch robotic_planning plansys2_rpl.py
```

### Terminale Client
```sh
$ source absolute/path/to/commandsFile/commands
$ run plan-file absolute/path/to/planFile/qualityplan
```