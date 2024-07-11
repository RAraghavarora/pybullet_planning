(define (domain pr2-tamp)
  (:requirements :strips :equality)
  (:constants base left right head)
  (:predicates
    (Arm ?a)
    (Graspable ?o)
    (Stackable ?o ?r)
    (Scannable ?r)
    (Registerable ?o)
    (Sink ?r)
    (Stove ?r)

    (Pose ?o ?p)
    (Grasp ?o ?g)
    (Kin ?a ?o ?p ?g ?q ?t)
    (BaseMotion ?q1 ?t ?q2)
    (Supported ?o ?p ?r)
    (Vis ?o ?p ?bq ?hq ?ht)

    (VisRange ?o ?p ?bq)
    (RegRange ?o ?p ?bq)

    (CanMove) ; TODO: include base
    (AtPose ?o ?p)
    (AtGrasp ?a ?o ?g)
    (HandEmpty ?a)
    (AtBConf ?q)
    (AtConf ?a ?q) ; TODO: a single conf predicate
    (AtAConf ?a ?q) ;;

    (On ?o ?r)
    (Holding ?a ?o)

    (Uncertain ?o)
    (Scanned ?o)
    (Localized ?o)
    (Registered ?o)

    (AConf ?a ?q) ;;
    (DefaultAConf ?a ?q) ;;

    (Controllable ?o) ;;

    (Door ?o) ;;
    (Space ?r) ;; Storage space (ex, minifridge)
    (Joint ?o) ;;
    (Edible ?o) ;;
    (OfType ?o ?t);;
    (Position ?o ?p)  ;; joint position of a body
    (AtPosition ?o ?p)  ;; joint position of a body
    (IsOpenedPosition ?o ?p)  ;;
    (IsClosedPosition ?o ?p)  ;; assume things start out closed
    (OpenedJoint ?o) ;;
    (ClosedJoint ?o) ;;
    (GraspedHandle ?o) ;;
    (HandleGrasp ?o ?g) ;;

    (KinGraspHandle ?a ?o ?p ?g ?q ?aq ?t)  ;; grasp a handle
    (KinUngraspHandle ?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)  ;; ungrasp a handle
    (KinPullDrawerHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)  ;; pull the handle
    (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt ?aq)  ;; pull the handle
    (KinTurnKnob ?a ?o ?p1 ?p2 ?g ?q ?aq1 ?aq2 ?at)

    (AtHandleGrasp ?a ?o ?g)  ;; in contact the handle
    (HandleGrasped ?a ?o)  ;; released the handle
    (GraspHandle ?a ?o ?p1 ?g ?q1 ?aq)
    (UngraspHandle ?a ?o ?p2 ?g ?q2 ?aq)
    (CanUngrasp)
    (CanPull ?a)
    (UngraspBConf ?q)
  )
  (:functions
    (MoveCost)
    (PickCost)
    (PlaceCost)
    (ScanCost)
    (LocalizeCost ?r ?o)
    (RegisterCost)
  )

  (:action move_base
    :parameters (?q1 ?q2 ?t)
    :precondition (and (BaseMotion ?q1 ?t ?q2)
                       (AtBConf ?q1)) ; (CanMove)
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)) (not (CanMove))
                 (increase (total-cost) (MoveCost)))
                 ; (forall (?o) (not (Registered ?o))))
                 ; (forall (?o) (when (Graspable ?o) (not (Registered ?o)))))
  )
  (:action pick
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
                       (Registered ?o) (AtPose ?o ?p) (HandEmpty ?a) (AtBConf ?q))
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a))
                 (increase (total-cost) (PickCost)))
  )
  (:action place
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
                       (AtGrasp ?a ?o ?g) (AtBConf ?q)) ; (Localized ?o)
    :effect (and (AtPose ?o ?p) (HandEmpty ?a) (CanMove)
                 (not (AtGrasp ?a ?o ?g))
                 (increase (total-cost) (PlaceCost)))
  )

  (:action scan
    :parameters (?o ?p ?bq ?hq ?ht)
    :precondition (and (Vis ?o ?p ?bq ?hq ?ht) (VisRange ?o ?p ?bq) (Scannable ?o)
                       (AtPose ?o ?p) (AtBConf ?bq) (Localized ?o))
    :effect (and (Scanned ?o) (CanMove)
                 (increase (total-cost) (ScanCost)))
  )
  (:action localize
    :parameters (?r ?p1 ?o ?p2)
    :precondition (and (Stackable ?o ?r) (Pose ?r ?p1) (Pose ?o ?p2) ; (FiniteScanCost ?r ?o)
                   (AtPose ?o ?p2) (Scanned ?r) (Uncertain ?o))
    :effect (and (Localized ?o) (Supported ?o ?p2 ?r)
                 (not (Uncertain ?o))
                 (increase (total-cost) (LocalizeCost ?r ?o)))
  )
  (:action register
    :parameters (?o ?p ?bq ?hq ?ht)
    :precondition (and (Vis ?o ?p ?bq ?hq ?ht) (RegRange ?o ?p ?bq) (Registerable ?o)
                       (AtPose ?o ?p) (AtBConf ?bq) (Localized ?o))
    :effect (and (Registered ?o) (CanMove)
                 (increase (total-cost) (RegisterCost)))
  )

  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r)
                      (AtPose ?o ?p)))
  )
  (:derived (Holding ?a ?o)
    (exists (?g) (and (Arm ?a) (Grasp ?o ?g)
                      (AtGrasp ?a ?o ?g)))
  )

  ;; Copied from other domain by Raghav
  (:action grasp_handle
      :parameters (?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)
      :precondition (and (Joint ?o) (AConf ?a ?aq1)
                         (KinGraspHandle ?a ?o ?p ?g ?q ?aq2 ?t)
                         (AtPosition ?o ?p) (HandEmpty ?a)
                         (AtBConf ?q) (AtAConf ?a ?aq1)
                         ;(Enabled)
                    )
      :effect (and (AtHandleGrasp ?a ?o ?g) (not (HandEmpty ?a))
                   (not (CanMove)) (CanPull ?a) (not (CanUngrasp))
                   (not (AtAConf ?a ?aq1)) (AtAConf ?a ?aq2)
                   ;(increase (total-cost) (PickCost)) ; TODO: make one third of the cost
                   (increase (total-cost) 0)
              )
    )
    (:action ungrasp_handle
      :parameters (?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)
      :precondition (and (Joint ?o) (AtPosition ?o ?p)
                         (KinUngraspHandle ?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)
                         (AtHandleGrasp ?a ?o ?g) (CanUngrasp)
                         (AtBConf ?q) (UngraspBConf ?q) (AtAConf ?a ?aq1) ;; (DefaultAConf ?a ?aq2)
                         ;(Enabled)
                    )
      :effect (and (GraspedHandle ?o) (HandEmpty ?a) (CanMove)
                   (not (AtHandleGrasp ?a ?o ?g))
                   (not (AtAConf ?a ?aq1)) (AtAConf ?a ?aq2)
                   ;(increase (total-cost) (PlaceCost))
                   (increase (total-cost) 0)
              )
    )

    ;; from fully closed position ?p1 pull to the fully open position ?p2
    (:action pull_handle
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt ?aq)
      :precondition (and (Joint ?o) (not (= ?p1 ?p2)) (CanPull ?a)
                         (AtPosition ?o ?p1) (Position ?o ?p2) (AtHandleGrasp ?a ?o ?g)
                         (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt ?aq)
                         (AtBConf ?q1) (AtAConf ?a ?aq)
                         ;(not (UnsafeApproach ?o ?p2 ?g))
                         ;(not (UnsafeATraj ?at))
                         ;(not (UnsafeBTraj ?bt))
                         ;(Enabled)
                    )
      :effect (and (not (CanPull ?a)) (CanUngrasp)
                  (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
                  (AtBConf ?q2) (not (AtBConf ?q1))
                  (increase (total-cost) 1)
              )
    )

)