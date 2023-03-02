(define (domain cluedo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :adl)

(:types
	waypoint
	oracle
	robot
)

(:predicates
	(robot_at ?wp - waypoint)
	(hypothesis_correct)
	(hypothesis_complete)
	(robot_at_oracle ?o - oracle)
	(hint_taken ?wp -waypoint)
)

(:durative-action leave_oracle
	:parameters ( ?from - oracle ?to - waypoint)
	:duration ( = ?duration 5)
	:condition(and
				(at start (robot_at_oracle ?from))
				)
	:effect(and
		(at start (not(robot_at_oracle ?from)))
		(at end(robot_at ?to))
		)
)

(:durative-action collect_hint
	:parameters (?wp - waypoint)
	:duration ( = ?duration 5)
	:condition (and (at start(robot_at ?wp)))
	:effect (and 
				(at end( hint_taken ?wp))
			)
)

(:durative-action go_to_next_point
	:parameters (?from ?to - waypoint)
	:duration ( = ?duration 5)
	:condition (and
		(at start(robot_at ?from))
		)
	:effect (and
		(at end(robot_at ?to))
		(at start (not (robot_at ?from))))
)
		
(:durative-action complete_query
	:parameters(?o - oracle)
	:duration ( = ?duration 5)
	:condition (and 
				(at start (forall (?wp - waypoint) ( hint_taken ?wp)))
				)
	:effect ( and
			(at end (hypothesis_complete))
			)
)	


(:durative-action go_to_oracle
	:parameters ( ?from - waypoint ?to -oracle)
	:duration ( = ?duration 5)
	:condition (and
				(at start(robot_at ?from))
				(at start(hypothesis_complete))
	)
	:effect (and
		(at start(not (robot_at ?from)))
		(at end (robot_at_oracle ?to))
		)
)


(:durative-action solution_query
	:parameters(?o - oracle)
	:duration ( = ?duration 5)
	:condition (and 
				(at start(robot_at_oracle ?o))
				(at start(hypothesis_complete))
				)
				
	:effect (and
			(at end(hypothesis_correct))
			)
)

)
