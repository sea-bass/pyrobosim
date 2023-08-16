; PDDL PLANNING DOMAIN (DERIVED PREDICATES)
;
; This planning domain contains `navigate`, `pick`, and `place` actions.
;
; All actions are symbolic, meaning there are no different types of grasps
; or feasibility checks, under the assumption that a downstream planner exists.
;
; This domain introduces the derived predicates `Has`, `HasNone`, and `HasAll`,
; which allow for more complex goal specifications.
;
; Accompanying streams are defined in the `streams.pddl` file.


(define (domain domain_derived)
  (:requirements :strips :equality)
  (:predicates
    ; Static predicates
    (Robot ?r)              ; Represents the robot
    (Obj ?o)                ; Object representation
    (Room ?r)               ; Room representation
    (Location ?l)           ; Location representation
    (Type ?t)               ; Type of location or object
    (Is ?o ?t)              ; Type correspondence of location or object

    ; Fluent predicates
    (HandEmpty ?r)          ; Whether the robot's gripper is empty
    (CanMove ?r)            ; Whether the robot can move (prevents duplicate moves)
    (Holding ?r ?o)         ; Object the robot is holding
    (At ?o ?l)              ; Robot/Object's location
    (AtRoom ?l ?r)          ; Location's corresponding room
    (Has ?loc ?entity)      ; Check existence of entities (object instances or types) in locations
    (HasNone ?loc ?entity)  ; Check nonexistence of entities (object instances or types) in locations
    (HasAll ?loc ?entity)   ; Check exclusivity of entities (object instances or types) in locations
  )

  ; FUNCTIONS : See their descriptions in the stream PDDL file
  (:functions
    (Dist ?l1 ?l2)
    (PickPlaceCost)
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


  ; DERIVED PREDICATES
  ; HAS: Checks locations using entity and location types or instances,
  ;      or even room names as locations
  (:derived (Has ?loc ?entity)
    (or
        ; CASE 1: Location and entity specified as instances
        (and (or (Obj ?entity) (Robot ?entity))
             (Location ?loc)
             (or (At ?entity ?loc)
                 (exists (?s)
                    (and (Room ?loc) (Location ?s)
                         (At ?entity ?s) (AtRoom ?s ?loc))
                 )
             )
        )
        ; CASE 2: Location is a type, entity is instance
        (exists (?l) (and (or (Obj ?entity) (Robot ?entity))
                          (Location ?l)
                          (or (and (Type ?loc) (Is ?l ?loc))
                              (and (Room ?loc) (AtRoom ?l ?loc)))
                          (At ?entity ?l))
        )
        ; CASE 3: Location is instance, entity is a type
        (exists (?o) (and (Type ?entity) (Obj ?o)
                          (Location ?loc)
                          (Is ?o ?entity)
                          (or (At ?o ?loc)
                              (exists (?s)
                                (and (Room ?loc) (Location ?s)
                                     (At ?o ?s) (AtRoom ?s ?loc)))
                          )
                     )
        )
        ; CASE 4: Location and object specified as types
        (exists (?o ?l) (and (Type ?entity) (Obj ?o)
                             (Type ?loc) (Location ?l)
                             (or (and (Type ?loc) (Is ?l ?loc))
                                 (and (Room ?loc) (AtRoom ?l ?loc)))
                             (Is ?o ?entity)
                             (At ?o ?l)
                        )
        )
        ; CASE 5: Robot holding an object instance
        (and (Robot ?loc) (Obj ?entity)
             (Holding ?loc ?entity)
        )
        (exists (?r)
            (and (Robot ?r) (Location ?loc) (Obj ?entity)
                 (Holding ?r ?entity)
                 (or (At ?r ?loc)
                     (and (Room ?loc) (exists (?s)
                        (and (Location ?s) (At ?r ?s) (AtRoom ?s ?loc))))
                 )
            )
        )
        ; CASE 6: Robot holding an object type
        (exists (?o)
            (and (Robot ?loc) (Obj ?o) (Holding ?loc ?o)
                 (Type ?entity) (Is ?o ?entity))
        )
        (exists (?r ?o)
            (and (Robot ?r) (Location ?loc) (Obj ?o) (Holding ?r ?o)
                 (Type ?entity) (Is ?o ?entity)
                 (or (At ?r ?loc)
                     (and (Room ?loc) (exists (?s)
                        (and (Location ?s) (At ?r ?s) (AtRoom ?s ?loc))))
                 )
            )
        )
    )
  )

  ; HASNONE: The opposite of "HAS".
  ; Checks that a location or location type has no object type
  ; or instances of an object type
  (:derived (HasNone ?loc ?entity)
    (not (Has ?loc ?entity))
  )

  ; HASALL: A variant of "HAS" for all rather than any objects.
  ; Checks that an object type or all instances of an object type are in a
  ; specific location or location type
  (:derived (HasAll ?loc ?objtype)
    (or
        ; CASE 1: Location is an instance
        (forall (?o)
            (imply
                (and (Obj ?o) (Type ?objtype) (Is ?o ?objtype))
                (and (Location ?loc)
                     (or (At ?o ?loc)
                         (exists (?s)
                            (and (Room ?loc) (Location ?s)
                                 (At ?o ?s) (At ?s ?loc)))
                     )
                )
            )
        )
        ; CASE 2: Location is a type
        (forall (?o)
            (imply
                (and (Obj ?o) (Type ?objtype) (Is ?o ?objtype))
                (exists (?l) (and
                                (Location ?l)
                                (or (and (Type ?loc) (Is ?l ?loc))
                                    (and (Room ?loc) (At ?l ?loc)))
                                (At ?o ?l))
                )
            )
        )
    )
  )

)
