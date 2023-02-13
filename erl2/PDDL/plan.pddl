Number of literals: 13
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 5.000
b (4.000 | 5.000)b (3.000 | 20.002)b (2.000 | 25.003)b (1.000 | 35.004);;;; Solution Found
; States evaluated: 7
; Cost: 40.005
; Time 0.00
0.000: (start_game cluedo_robot oracle)  [5.000]
5.001: (leave_oracle cluedo_robot oracle point1)  [10.000]
15.002: (collect_hint cluedo_robot point1)  [5.000]
20.003: (consistency_query cluedo_robot point1)  [5.000]
25.004: (go_to_oracle cluedo_robot point1 oracle)  [10.000]
35.005: (solution_query cluedo_robot oracle)  [5.000]
