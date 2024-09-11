(define (domain modeling_task1)
 (:requirements :strips :typing :adl :existential-preconditions :disjunctive-preconditions)
 
 (:types
    location locatable - object
    workstation box agent supply - locatable
  )

 (:predicates
    (at_location ?ltb - locatable ?l - location)
    (empty_box ?b - box)
    (content_in_box ?b - box ?c - supply)
    (workstation_has_content ?w - workstation ?c - supply)
    (connected ?loc1 ?loc2 - location)
    (free ?r - agent)
    (is_carrying ?r - agent ?b - box)
  ) 

  (:action fill_box
    :parameters (?b - box ?c - supply ?r - agent ?l - location)
    :precondition (and
        (empty_box ?b)
        (at_location ?c ?l)
        (at_location ?r ?l)
        (at_location ?b ?l)
        (free ?r)
    )
    :effect (and
        (not (empty_box ?b))
        (content_in_box ?b ?c)
    )
  )

  (:action pick_up_box
    :parameters (?b - box ?r - agent ?l - location)
    :precondition (and
        (at_location ?r ?l)
        (at_location ?b ?l)
        (free ?r)
    )
    :effect (and
        (not (free ?r))
        (is_carrying ?r ?b)
        (not (at_location ?b ?l))
    )
  )

  (:action empty_box
    :parameters (?b - box ?c - supply ?r - agent ?l - location ?w - workstation)
    :precondition (and
        (content_in_box ?b ?c)
        (at_location ?w ?l)
        (at_location ?r ?l)
        (is_carrying ?r ?b)
        (not (free ?r))
        (not (empty_box ?b))
      )
    :effect (and
        (workstation_has_content ?w ?c)
        (not (content_in_box ?b ?c))
        (empty_box ?b)
        (at_location ?b ?l)
        (not (is_carrying ?r ?b))
        (free ?r)
    )
  )

  (:action move
    :parameters (?r - agent ?l1 - location ?l2 - location)
    :precondition (and
        (at_location ?r ?l1)
        (connected ?l1 ?l2)
    )
    :effect (and
        (at_location ?r ?l2)
        (not (at_location ?r ?l1))
    )
  )

)