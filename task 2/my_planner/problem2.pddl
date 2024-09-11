(define (problem instance2)
  (:domain modeling_task2)
  
  (:objects
    robot1 robot2 - agent
    c c1 - carrier
    s1 s2 s3 s4 - slot
    box1 box2 box3 box4 - box
    bolt valve tool wrench screw - supply
    workstation1 workstation2 workstation3 - workstation
    warehouse location1 location2 location3 - location
  )
  
  (:init
    (at_location robot1 warehouse)
    (at_location robot2 warehouse)

    (at_location c warehouse)
    (at_location c1 warehouse)
    (carrier_of_robot c robot1)
    (carrier_of_robot c1 robot2)
    (slot_of_carrier_free s1 c)
    (slot_of_carrier_free s2 c)
    (slot_of_carrier_free s3 c1)
    (slot_of_carrier_free s4 c1)

    (at_location box1 warehouse)
    (at_location box2 warehouse)
    (at_location box3 warehouse)
    (at_location box4 warehouse)

    (empty_box box1)
    (empty_box box2)
    (empty_box box3)
    (empty_box box4)

    (at_location bolt warehouse)
    (at_location valve warehouse)
    (at_location tool warehouse)
    (at_location wrench warehouse)
    (at_location screw warehouse)

    (at_location workstation1 location1)
    (at_location workstation2 location2)
    (at_location workstation3 location3)

    (connected warehouse location1)
    (connected location1 location2)
    (connected location2 location3)

    (connected location1 warehouse)
    (connected location2 location1)
    (connected location3 location2)
  )
  
  (:goal
    (and
      (workstation_has_content workstation1 bolt)
      (workstation_has_content workstation2 valve)
      (workstation_has_content workstation2 screw)
      (workstation_has_content workstation3 tool)
      (at_location robot1 warehouse)
      (at_location robot2 warehouse)
    )
  )
)