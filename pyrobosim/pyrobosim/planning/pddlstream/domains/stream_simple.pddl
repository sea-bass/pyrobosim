; STREAMS FOR PDDL PLANNING DOMAIN (NO STREAMS)
;
; This planning domain contains `move`, `pick`, and `place` actions.
; All actions are symbolic, meaning there are no different types of grasps
; or feasibility checks, under the assumption that a downstream planner exists.
;
; Accompanying planning domain defined in the `domain_nostream.pddl` file.

(define (stream stream_simple)
  
  ; DIST: Distance between two locations.
  (:function (Dist ?l1 ?l2)
             (and (Location ?l1) (Location ?l2))
  )

  ; PICKPLACECOST: Cost to perform pick and place at a location.
  (:function (PickPlaceCost ?l ?o)
             (and (Location ?l) (Obj ?o))
  )

)