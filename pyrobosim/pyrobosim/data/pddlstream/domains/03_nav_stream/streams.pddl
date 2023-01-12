; STREAMS FOR PDDL PLANNING DOMAIN (NAVIGATION STREAMS)
;
; Contains cost functions for actions and streams for navigation.
;
; Accompanying planning domain defined in the `domain.pddl` file.

(define (stream stream_nav_stream)

  ; PATHLENGTH: Length of a path.
  (:function (PathLength ?pth)
             (and (Path ?pth))
  )

  ; PICKPLACECOST: Cost to perform pick and place at a location.
  (:function (PickPlaceCost ?l ?o)
             (and (Location ?l) (Obj ?o))
  )

  ; S-NAVPOSE: Samples a pose from a finite set of navigation poses for that location.
  (:stream s-navpose
    :inputs (?l)
    :domain (Location ?l)
    :outputs (?p)
    :certified (and (Pose ?p) (NavPose ?l ?p))
  )

  ; S-MOTION: Samples a valid path from one pose to another
  (:stream s-motion
    :inputs (?p1 ?p2)
    :domain (and (Pose ?p1) (Pose ?p2))
    :outputs (?pth)
    :certified (and (Path ?pth) (Motion ?p1 ?p2 ?pth)))

)
