; PDDL PLANNING DOMAIN (SIMPLE)
;
; This planning domain contains `navigate`, `pick`, and `place` actions.
;
; All actions are symbolic, meaning there are no different types of grasps
; or feasibility checks, under the assumption that a downstream planner exists.
;
; Accompanying streams are defined in the `streams.pddl` file.


(define (domain domain_simple)
  (:requirements :strips :equality)
  (:predicates
    ; Static predicates
    (Robot ?r)          ; Represents the robot
    (Obj ?o)            ; Object representation
    (Room ?r)           ; Room representation
    (Location ?l)       ; Location representation

    ; Fluent predicates
    (HandEmpty ?r)      ; Whether the robot's gripper is empty
    (CanMove ?r)        ; Whether the robot can move (prevents duplicate moves)
    (Holding ?r ?o)     ; Object the robot is holding
    (At ?o ?l)          ; Robot/Object's location, or location's Room
  )

  ; FUNCTIONS : See their descriptions in the stream PDDL file
  (:functions
    (Dist ?l1 ?l2)
    (PickPlaceCost ?l ?o)
  )

  ; ACTIONS
  ; NAVIGATE: Moves the robot from one location to the other
  (:action navigate
    :parameters (?r ?l1 ?l2)
    :precondition (and (Robot ?r)
                       (CanMove ?r)
                       (Location ?l1)
                       (Location ?l2)
                       (At ?r ?l1))
    :effect (and (not (CanMove ?r))
                 (At ?r ?l2) (not (At ?r ?l1))
                 (increase (total-cost) (Dist ?l1 ?l2)))
  )

  ; PICK: Picks up an object from a specified location
  (:action pick
    :parameters (?r ?o ?l)
    :precondition (and (Robot ?r)
                       (Obj ?o)
                       (Location ?l)
                       (not (Room ?l))
                       (HandEmpty ?r)
                       (At ?r ?l)
                       (At ?o ?l))
    :effect (and (Holding ?r ?o) (CanMove ?r)
                 (not (HandEmpty ?r))
                 (not (At ?o ?l))
                 (increase (total-cost) (PickPlaceCost ?l ?o)))
  )

  ; PLACE: Places an object in a specified location
  (:action place
    :parameters (?r ?o ?l)
    :precondition (and (Robot ?r)
                       (Obj ?o)
                       (Location ?l)
                       (not (Room ?l))
                       (At ?r ?l)
                       (not (HandEmpty ?r))
                       (Holding ?r ?o))
    :effect (and (HandEmpty ?r) (CanMove ?r)
                 (At ?o ?l)
                 (not (Holding ?r ?o))
                 (increase (total-cost) (PickPlaceCost ?l ?o)))
  )

)
