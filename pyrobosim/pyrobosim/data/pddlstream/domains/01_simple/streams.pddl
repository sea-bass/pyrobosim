; STREAMS FOR PDDL PLANNING DOMAIN (SIMPLE)
;
; Contains simple cost functions for actions.
;
; Accompanying planning domain defined in the `domain.pddl` file.

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
