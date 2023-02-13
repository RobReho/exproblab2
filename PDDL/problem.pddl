(define (problem cluedo_game)
        (:domain cluedo)
        (:objects
            point1 point2 point3 point4 - waypoint
            oracle - oracle
            cluedo_robot - robot
        )
          
        (:init
            (formation_lap)
            (next_wp point1 point2)
            (next_wp point2 point3)
            (next_wp point3 point4)
            (next_wp point4 point1)

        )

        (:goal (and
            (end_game))
        )
)
