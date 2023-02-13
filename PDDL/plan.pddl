Number of literals: 14
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 7.000
b (6.000 | 10.000)b (5.000 | 15.001)b (4.000 | 20.002)b (3.000 | 25.003)b (2.000 | 30.004)b (1.000 | 35.005);;;; Solution Found
; States evaluated: 8
; Cost: 40.006
; Time 0.00
0.000: (go_to_waypoint cluedo_robot wp1 wp2)  [10.000]
10.001: (move_arm cluedo_robot wp2)  [5.000]
15.002: (check_complete cluedo_robot wp2)  [5.000]
15.003: (go_home cluedo_robot wp2 home)  [10.000]
25.004: (check_consistency cluedo_robot home)  [5.000]
25.005: (go_oracle cluedo_robot home oracle)  [10.000]
35.006: (oracle cluedo_robot oracle)  [5.000]
