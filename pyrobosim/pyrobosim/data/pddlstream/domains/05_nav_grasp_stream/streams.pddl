; STREAMS FOR PDDL PLANNING DOMAIN (NAVIGATION + MANIPULATION + GRASP STREAMS)
;
; Contains cost functions for actions and streams for navigation and manipulation.
;
; Accompanying planning domain defined in the `domain.pddl` file.

(define (stream stream_nav_grasp)

  ; PATHLENGTH: Length of a path.
  (:function (PathLength ?pth)
             (and (Path ?pth))
  )

  ; GRASPATPOSECOST: Cost to perform a grasp on an object.
  ;                  The robot is at Pose pr, and the grasp is defined by Grasp g.
  (:function (GraspAtPoseCost ?g ?pr)
             (and (Grasp ?g) (Pose ?pr))
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

  ; S-GRASP: Samples an object grasp pose
  (:stream s-grasp
    :inputs (?o ?po ?pr)
    :domain (and (Obj ?o) (Pose ?po) (Pose ?pr))
    :outputs (?g)
    :certified (and (Grasp ?g) (Graspable ?o ?po ?pr ?g))
  )

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
