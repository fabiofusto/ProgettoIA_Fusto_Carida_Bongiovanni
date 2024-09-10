(define (domain modeling_task2)
 (:requirements :strips :typing :adl :existential-preconditions :disjunctive-preconditions)
 
 (:types
    location locatable slot - object
    workstation box agent carrier supply - locatable
  )

 (:predicates
    (at_location ?ltb - locatable ?l - location)
    (empty_box ?b - box)
    (box_on_carrier ?b - box ?c - carrier)
    (content_in_box ?b - box ?c - supply)
    (workstation_has_content ?w - workstation ?c - supply)
    (slot_of_carrier_free ?s - slot ?c - carrier)
    (connected ?loc1 ?loc2 - location)
    (carrier_of_robot ?c - carrier ?r - agent)
    (slot_of_robot_free ?s - slot ?r - agent)
  ) 

  (:action fill_box
    :parameters (?b - box ?c - supply ?r - agent ?l - location)
    :precondition (and
        (empty_box ?b)
        (at_location ?c ?l)
        (at_location ?r ?l)
        (or (exists
                (?car - carrier)
                (and
                    (box_on_carrier ?b ?car)
                    (at_location ?car ?l)
                ))
            (at_location ?b ?l)
        )

    )
    :effect (and
        (not (empty_box ?b))
        (content_in_box ?b ?c)
    )
  )

  (:action empty_box
    :parameters (?b - box ?c - supply ?r - agent ?l - location ?w - workstation)
    :precondition (and
        (content_in_box ?b ?c)
        (at_location ?w ?l)
        (at_location ?r ?l)
        (or
            (exists
                (?car - carrier)
                (and
                    (box_on_carrier ?b ?car)
                    (at_location ?car ?l)
                )

            )
            (at_location ?b ?l)
        ))
    :effect (and
        (workstation_has_content ?w ?c)
        (not (content_in_box ?b ?c))
        (empty_box ?b)
    )
  )

  (:action load_carrier
    :parameters (?c - carrier ?s - slot ?r - agent ?b - box ?l - location)
    :precondition (and
        (at_location ?r ?l)
        (at_location ?b ?l)
        (at_location ?c ?l)
        (carrier_of_robot ?c ?r)
        (slot_of_carrier_free ?s ?c)
        (not (empty_box ?b))
    )
    :effect (and
        (not (at_location ?b ?l))
        (box_on_carrier ?b ?c)
        (not (slot_of_carrier_free ?s ?c))
    )
  )

  (:action move_with_carrier
    :parameters (?r - agent ?c - carrier ?l1 - location ?l2 - location)
    :precondition (and
        (at_location ?c ?l1)
        (at_location ?r ?l1)
        (carrier_of_robot ?c ?r)
        (connected ?l1 ?l2)
    )
    :effect (and
        (at_location ?c ?l2)
        (at_location ?r ?l2)
        (not (at_location ?c ?l1))
        (not (at_location ?r ?l1))
    )
  )

)