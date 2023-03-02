(define (problem task)
(:domain cluedo)
(:objects
   wp1 wp2 wp3 wp4 - waypoint
   oracle - oracle
)
(:init
    (robot_at_oracle oracle)
    (not ( hint_taken wp1))
    (not ( hint_taken wp2))
    (not ( hint_taken wp3))
    (not ( hint_taken wp4))
    (not ( hypothesis_complete))
)
(:goal 
    (hypothesis_correct)
)
)
