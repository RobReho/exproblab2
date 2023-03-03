(define (problem task)
(:domain cluedo)
(:objects
    wp1 wp2 wp3 wp4 - waypoint
    oracle - oracle
)
(:init



    (robot_at_oracle oracle)


)
(:goal (and
    (hypothesis_correct)
))
)
