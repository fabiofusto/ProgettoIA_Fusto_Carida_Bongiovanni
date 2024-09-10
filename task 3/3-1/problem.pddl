(define (problem prob3)
  (:domain durative_task)
  
  (:objects
    robot - agent
    c - carrier
    box1 box2 box3 - box
    bolt valve tool - supply
    workstation1 workstation2 workstation3 - workstation
    warehouse location1 location2 - location
  )
  
  (:init
    (at_location robot warehouse)

    (at_location c warehouse)

    (carrier_of_robot c robot)

    (= (capacity c) 2) 
    (= (load c) 0)    

    (at_location box1 warehouse)
    (at_location box2 warehouse)
    (at_location box3 warehouse)

    (empty_box box1)
    (empty_box box2)
    (empty_box box3)

    (at_location bolt warehouse)
    (at_location valve warehouse)
    (at_location tool warehouse)

    (at_location workstation1 location1)
    (at_location workstation2 location2)
    (at_location workstation3 location2)

    (connected warehouse location1)
    (connected location1 location2)

    (connected location1 warehouse)
    (connected location2 location1)
  )
  
  (:goal
    (and
      (workstation_has_content workstation1 bolt)
      (workstation_has_content workstation2 valve)
      (workstation_has_content workstation3 tool)
      (at_location robot warehouse)
    )
  )
)