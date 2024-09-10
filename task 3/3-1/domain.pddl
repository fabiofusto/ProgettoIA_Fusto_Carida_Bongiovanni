(define (domain durative_task)
 (:requirements :strips :typing :durative-actions :equality :numeric-fluents)
 
 (:types
    location locatable - object
    workstation box agent carrier supply - locatable
  )

 (:predicates
    (at_location ?ltb - locatable ?l - location)
    (empty_box ?b - box)
    (box_on_carrier ?b - box ?c - carrier)
    (content_in_box ?b - box ?c - supply)
    (workstation_has_content ?w - workstation ?c - supply)
    (connected ?loc1 ?loc2 - location)

    (carrier_of_robot ?c - carrier ?r - agent)
  )

  (:functions
    (capacity ?c - carrier)
    (load ?c - carrier)
  )

  (:durative-action fillbox
    :parameters (?b - box ?s - supply ?r - agent ?l - location)
    :duration (= ?duration 2)
    :condition (and
        (at start (empty_box ?b))
        (over all (at_location ?s ?l))
        (over all (at_location ?r ?l))
        (over all (at_location ?b ?l))
    )
    :effect (and
        (at end (not (empty_box ?b)))
        (at end (content_in_box ?b ?s))
    )
  )

  (:durative-action emptybox
    :parameters (?b - box ?c - carrier ?s - supply ?r - agent ?l - location ?w - workstation)
    :duration (= ?duration 3)
    :condition (and
        (at start (content_in_box ?b ?s))
        (over all (at_location ?w ?l))
        (over all (at_location ?r ?l))
        (at start (box_on_carrier ?b ?c))
        (over all (carrier_of_robot ?c ?r))
    )
    :effect (and
        (at start (not (box_on_carrier ?b ?c)))
        (at end (workstation_has_content ?w ?s))
        (at end (not (content_in_box ?b ?s)))
        (at end (empty_box ?b))
        (at end (at_location ?b ?l))
        (at end (decrease (load ?c) 1))
    )
  )

  (:durative-action loadcarrier
    :parameters (?c - carrier ?r - agent ?b - box ?l - location)
    :duration (= ?duration 3)
    :condition (and
        (over all (at_location ?r ?l))
        (at start (at_location ?b ?l))
        (over all (at_location ?c ?l))
        (over all (carrier_of_robot ?c ?r))
        (over all (< (load ?c) (capacity ?c)))
        ;(not (empty_box ?b))
    )
    :effect (and
        (at start (not (at_location ?b ?l)))
        (at end (box_on_carrier ?b ?c))
        (at end (increase (load ?c) 1))
    )
  )

  (:durative-action movewithcarrier
    :parameters (?r - agent ?c - carrier ?l1 - location ?l2 - location)
    :duration (= ?duration 3)
    :condition (and
        (at start (at_location ?c ?l1))
        (at start (at_location ?r ?l1))
        (over all (carrier_of_robot ?c ?r))
        (over all (connected ?l1 ?l2))
    )
    :effect (and
        (at start (not (at_location ?c ?l1)))
        (at start (not (at_location ?r ?l1)))
        (at end (at_location ?c ?l2))
        (at end (at_location ?r ?l2))
    )
  )

)