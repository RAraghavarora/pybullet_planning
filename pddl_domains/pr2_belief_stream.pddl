(define (stream kuka-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Supported ?o ?p ?r)
                    (Pose ?o ?p) (Observable ?p))
  )
  (:stream sample-pose-inside ;;
    :inputs (?o ?r)
    :domain (Containable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Contained ?o ?p ?r))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )

   (:stream inverse-reachability
     :inputs (?a ?o ?p ?g)
     :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g))
     :outputs (?q)
     :certified (and (BConf ?q) (Reach ?a ?o ?p ?g ?q))
   )

  (:stream inverse-kinematics ; Different from pr2_stream_pigi Why?
    :inputs (?a ?o ?p ?g)
    :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g));  (Observable ?p))
    :outputs (?bq ?t)
    :certified (and (Kin ?a ?o ?p ?g ?bq ?t)
                    (BConf ?bq) (Traj ?a ?t)
                    (RegRange ?o ?p ?bq) (VisRange ?o ?p ?bq))
  )
  (:stream plan-base-motion
    :inputs (?q1 ?q2)
    :domain (and (BConf ?q1) (BConf ?q2))
    :outputs (?t)
    :certified (and (BTraj ?t)
                    (BaseMotion ?q1 ?t ?q2))
  )

  ; Alternatively, could just do inverse visibility
  (:stream test-vis-base
    :inputs (?o ?p ?bq)
    :domain (and (Pose ?o ?p) (BConf ?bq))
    :outputs ()
    :certified (VisRange ?o ?p ?bq)
  )
  (:stream test-reg-base
    :inputs (?o ?p ?bq)
    :domain (and (Pose ?o ?p) (BConf ?bq))
    :outputs ()
    :certified (and (RegRange ?o ?p ?bq) (VisRange ?o ?p ?bq))
  )

  (:stream sample-vis-base
    :inputs (?o ?p)
    :domain (Pose ?o ?p)
    :outputs (?bq)
    :certified (VisRange ?o ?p ?bq)
  )
  (:stream sample-reg-base
    :inputs (?o ?p)
    :domain (Pose ?o ?p)
    :outputs (?bq)
    :certified (and (VisRange ?o ?p ?bq) (RegRange ?o ?p ?bq))
  )
  (:stream inverse-visibility
    :inputs (?o ?p ?bq)
    :domain (VisRange ?o ?p ?bq)
    :outputs (?hq ?ht)
    :certified (and (Vis ?o ?p ?bq ?hq ?ht) ; Only set BConf on last iteration
                    (BConf ?bq) (Conf head ?hq) (Traj head ?ht))
  )
 
   (:stream get-joint-position-open
     :inputs (?o ?p1)
     :domain (and (Joint ?o) (Position ?o ?p1) (IsClosedPosition ?o ?p1))
     :outputs (?p2)
     :certified (and (Position ?o ?p2) (IsOpenedPosition ?o ?p2) (IsSampledPosition ?o ?p1 ?p2))
   )
   (:stream sample-handle-grasp
       :inputs (?o)
       :domain (Joint ?o)
       :outputs (?g)
       :certified (HandleGrasp ?o ?g)
   )
   (:stream inverse-kinematics-grasp-handle
       :inputs (?a ?o ?p ?g)
       :domain (and (Joint ?o) (Controllable ?a) (Position ?o ?p) (HandleGrasp ?o ?g) (IsClosedPosition ?o ?p))
       :outputs (?q ?aq ?t)
       :certified (and (BConf ?q) (AConf ?a ?aq) (ATraj ?t)
                       (GraspHandle ?a ?o ?p ?g ?q ?aq)
                       (KinGraspHandle ?a ?o ?p ?g ?q ?aq ?t))
   )
  (:stream inverse-kinematics-ungrasp-handle
      :inputs (?a ?o ?p ?g ?q ?aq1)
      :domain (and (UngraspHandle ?a ?o ?p ?g ?q ?aq1) (IsOpenedPosition ?o ?p))
      :outputs (?aq2 ?t)
      :certified (and (AConf ?a ?aq2) (ATraj ?t)
                      (KinUngraspHandle ?a ?o ?p ?g ?q ?aq1 ?aq2 ?t))
  )
  (:stream plan-base-pull-handle
      :inputs (?a ?o ?p1 ?p2 ?g ?q1 ?aq)
      :domain (and (GraspHandle ?a ?o ?p1 ?g ?q1 ?aq) (Position ?o ?p2) (IsSampledPosition ?o ?p1 ?p2))
      :outputs (?q2 ?bt)
      :certified (and (BConf ?q2) (UngraspBConf ?q2) (BTraj ?bt)
                      (UngraspHandle ?a ?o ?p2 ?g ?q2 ?aq)
                      (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt ?aq))
  )
)