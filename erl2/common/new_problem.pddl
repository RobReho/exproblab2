(define (problem task)
(:domain cluedo)
(:objects
    wp1 wp2 wp3 wp4 - waypoint
    oracle - oracle
)
(:init
    (robot_at wp1)


    (not (hypothesis_complete))



)
(:goal (and
    (hypothesis_correct)
))
)
