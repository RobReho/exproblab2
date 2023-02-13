(define (domain cluedo)

    (:requirements :strips :typing :disjunctive-preconditions :durative-actions  :duration-inequalities :equality)

    (:types 
        robot
        waypoint
        oracle
    )
    
    (:predicates
        (formation_lap)
        (robot_at_point ?obj - robot ?point - waypoint)
        (robot_at_oracle ?obj -robot ?o - oracle)
        (next_point ?from ?to - waypoint)
        (got_hint ?point - waypoint)
        (high_gripper)
        (comp_cons_hypo)
        (end_game)
    )
    
    (:durative-action start_game
	:parameters (?obj - robot ?o - oracle)
	:duration ( = ?duration 5)
	:condition (and
	        (at start (formation_lap)))
	:effect (and
	        (at end (robot_at_oracle ?obj ?o))
	        (at start (not(formation_lap))))
    )
    
    (:durative-action leave_oracle
	:parameters (?obj - robot ?from - oracle ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
	        (at start (robot_at_oracle ?obj ?from)))
	:effect (and
	        (at end (robot_at_point ?obj ?to))
	        (at end (high_gripper))
	        (at start (not(robot_at_oracle ?obj ?from))))
    )
    
    (:durative-action go_to_oracle
	:parameters (?obj - robot ?from - waypoint ?to - oracle)
	:duration ( = ?duration 10)
	:condition (and
	        (at start (robot_at_point ?obj ?from)))
	:effect (and
	        (at end (robot_at_oracle ?obj ?to))
	        (at end (high_gripper))
	        (at start (not(robot_at_point ?obj ?from))))
    )
    
    (:durative-action go_to_waypoint
	:parameters (?obj - robot ?from - waypoint ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at_point ?obj ?from))
		(at start (next_point ?from ?to)))
	:effect (and
	        (at end (robot_at_point ?obj ?to))
	        (at end (high_gripper))
		(at start (not(robot_at_point ?obj ?from))))
    )
    
    (:durative-action collect_hint
        :parameters (?obj - robot ?point - waypoint)
	:duration ( = ?duration 5)
	:condition (and
		(at start (robot_at_point ?obj ?point))
		(at start (high_gripper)))
	:effect (and
	        (at end (got_hint ?point))
	        (at start (not(high_gripper))))
    )
     
    (:durative-action consistency_query
	:parameters (?obj - robot ?point - waypoint)
	:duration ( = ?duration 5)
	:condition (and
		(at start (robot_at_point ?obj ?point))
		(at start (got_hint ?point)))
	:effect (and
	        (at end (comp_cons_hypo))
		(at start (not(got_hint ?point))))
    )
      
    (:durative-action solution_query
    :parameters (?obj - robot ?o - oracle)
	:duration ( = ?duration 5)
	:condition (and
	        (at start (robot_at_oracle ?obj ?o))
		    (at start (comp_cons_hypo)))
	:effect (and
	        (at end (end_game))
	        (at start (not(comp_cons_hypo))))
    )
    
    
    
   
)
