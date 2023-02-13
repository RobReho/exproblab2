(define (problem cluedo_game)
        (:domain domain_cluedo)
        (:objects
            point1 point2 point3 point4 - waypoint
            oracle - oracle
            cluedo_robot - robot
        )
          
        (:init
            (formation_lap)
            (next_point point1 point2)
            (next_point point2 point3)
            (next_point point3 point4)
            (next_point point4 point1)

        )

        (:goal (and
            (end_game))
        )
)
