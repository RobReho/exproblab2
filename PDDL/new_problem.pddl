(define (problem task)
(:domain cluedo)
(:objects
    cluedo_robot - robot
    point1 point2 point3 point4 - waypoint
    oracle - oracle
)
(:init
    (robot_at_point cluedo_robot point1)

    (next_point point1 point2)
    (next_point point2 point3)
    (next_point point3 point4)
    (next_point point4 point1)

)
(:goal (and
    (end_game)
))
)
