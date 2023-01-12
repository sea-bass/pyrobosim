; STREAMS FOR PDDL PLANNING DOMAIN (NAVIGATION + MANIPULATION STREAMS)
;
; Contains cost functions for actions and streams for navigation and manipulation.
;
; Accompanying planning domain defined in the `domain.pddl` file.

(define (stream stream_nav_stream)

  ; PATHLENGTH: Length of a path.
  (:function (PathLength ?pth)
             (and (Path ?pth))
  )

  ; PICKPLACEATPOSECOST: Cost to perform pick and place at a location.
  ;                      The object is at Pose p and robot at Pose pr.
  (:function (PickPlaceAtPoseCost ?l ?o ?p ?pr)
             (and (Location ?l) (Obj ?o) (Pose ?p) (Pose ?pr))
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

  ; S-PLACE: Samples an object placement pose
  (:stream s-place
    :inputs (?l ?o)
    :domain (and (Location ?l) (Obj ?o))
    :outputs (?p)
    :certified (and (Pose ?p) (Placeable ?l ?o ?p))
  )

  ; T-COLLISION-FREE: Check if a placement pose is collision free
  (:stream t-collision-free
    :inputs (?o1 ?p1 ?o2 ?p2)
    :domain (and (Obj ?o1) (Pose ?p1)
                 (Obj ?o2) (Pose ?p2))
    :certified (CollisionFree ?o1 ?p1 ?o2 ?p2)
  )

)
