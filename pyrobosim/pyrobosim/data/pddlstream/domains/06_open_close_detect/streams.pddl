; STREAMS FOR PDDL PLANNING DOMAIN (OPEN, CLOSE, AND DETECT)
;
; Contains simple cost functions for actions.
;
; Accompanying planning domain defined in the `domain.pddl` file.

(define (stream stream_open_close_detect)

  ; DIST: Distance between two locations.
  (:function (Dist ?l1 ?l2)
             (and (Location ?l1) (Location ?l2))
  )

  ; PICKPLACECOST: Cost to perform pick and place at a location.
  (:function (PickPlaceCost ?l ?o)
             (and (Location ?l) (Obj ?o))
  )

  ; DETECTCOST: Cost to detect objects at a location.
  (:function (DetectCost ?l)
             (and (Location ?l))
  )

  ; OPENCLOSECOST: Cost to open or close a location.
  (:function (OpenCloseCost ?l)
             (and (Location ?l))
  )

)
