#!/usr/bin/env roseus
;; In order to run tabletop objects recognition
;; ssh pr1012
;; roslauch roseus_tutorials tabletop-object-detector.launch convert_to_base_link:=true run_rviz:=false

;; Shortcut to load this file
(load "knock.l")
(defun l nil
  (load "pick.l"))

;; Initialization
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_openrave/pr2eus-openrave.l")

;;;
;;; Pr2 model 
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(if (not (boundp '*pr2*)) (pr2))
(setq *ri* (instance pr2-interface :init))
(objects (list *pr2*))

;; Synchronize *pr2*'s and *ri*'s angle-vector
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))

;; Initial position
(defun init  nil 
  (send *pr2* :reset-pose)
  (send *pr2* :arms :shoulder-p :joint-angle -20)
  (send *pr2* :head :neck-p :joint-angle 40)
  (send *pr2* :arms :wrist-r :joint-angle 90)
  (send *pr2* :torso :waist-z :joint-angle 110)
  (send *pr2* :larm :collar-y :joint-angle 90)
  (send *pr2* :rarm :collar-y :joint-angle -90)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 10)
  (send *ri* :wait-interpolation)
  )

;;;
;;; Client node definition
(ros::roseus "tabletop_objectdetection_client")
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 
;;; Use for object dectect
(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfl* (instance ros::transform-listener :init))

(defun tf-object-update (&optional (root-frame-id "/base_footprint"))
  (let (c obj-frame-id ret)
    (dolist (obj-frame-id (list "/tabletop0"
				"/tabletop1"
				"/tabletop2"
				"/tabletop3"
				"/tabletop4"))
      ;; advertise
      (when (send *tfl* :get-latest-common-time root-frame-id obj-frame-id)
	(setq c (send *tfl* :lookup-transform root-frame-id obj-frame-id (ros::time 0)))
	(when c
	  (ros::ros-info "~A ~A" obj-frame-id c)
	  (push c ret))))
    ret))

(defun objectdetection-cb (msg)
  (let ((frame-id (send msg :header :frame_id)) type ret)
    (print msg)
    (unless (eq (char frame-id 0) #\/) (setq frame-id (concatenate string "/" frame-id)))
    (dolist (obj-pose (send msg :objects))
      (setq type (send obj-pose :type))
      (unless (eq (char type 0) #\/) (setq type (concatenate string "/" type)))
      (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
      (send *tfb* :send-transform ret frame-id type)
      (ros::ros-info "~A ~A ~A" ret frame-id type))
    ))



;; Subcribe to Objectdetection
;; Run (objectdetection-cb) whenever msgs comes
(ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Pick 
(defun pick (pos &key (arm :larm))
  (let ((tmp-pos (send pos :copy-worldcoords)))
    (format t  "Start Pick: ~a~%" pos)
    (send *ri* :stop-grasp arm)  ;; Stop grasp
    (send *pr2* :inverse-kinematics 
	  (progn
	    (send tmp-pos :rotate  (deg2rad 90) :y) ;; alignment
	    (send tmp-pos :translate #f(-20 0 0)))  ;; avoid collision
	  :rotation-axis :x  ;; 
	  :move-target (send *pr2* arm :end-coords)
	  :link-list (send *pr2* :link-list
			   (send *pr2* arm :end-coords :parent)
			   (send *pr2* :torso_lift_link))
	  :debug-view t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

    ;; Low down to pick 
    (send *pr2* :torso :waist-z
	  :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) 30))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    (send *ri* :move-gripper arm 0.0 :effort 100 :wait t)
    (print "End Pick!"))
  )

#|
(defun go-home (pos &key (arm :rarm))
  (let ((tmp-pos (send pos :copy-worldcoords)))
    (format t  "Start Go Home: ~a~%" pos)
    (send *pr2* :inverse-kinematics 
	  (progn
	    (send tmp-pos :translate #f(-130 -130 0)))
	  :rotation-axis t ;; 
	  :move-target (send *pr2* arm :end-coords)
	  :link-list (send *pr2* :link-list
			   (send *pr2* arm :end-coords :parent)
			   (send *pr2* :torso_lift_link))
	  :debug-view t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    (send *pr2* :torso :waist-z :joint-angle
	  (- (send *pr2* :torso :waist-z :joint-angle ) 40))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

    (print "End Go Home!"))
  )
|#


(defun place (pos &key (arm :larm) (height -50))
  (let ((tmp-pos (send pos :copy-worldcoords)))
    (format t  "Start Place: ~a~%" pos)
    (send *pr2* :inverse-kinematics 
	  (progn 
	    (send tmp-pos :rotate (deg2rad 90) :y)
	    (send tmp-pos :translate (float-vector height 0.0 0.0)))
	  :rotation-axis :x  ;; 
	  :move-target (send *pr2* arm :end-coords)
	  :link-list (send *pr2* :link-list
			   (send *pr2* arm :end-coords :parent)
			   (send *pr2* :torso_lift_link))
	  :debug-view t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    (send *pr2* :torso :waist-z
     	  :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) 30))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    (send *ri* :stop-grasp)
        (send *pr2* :torso :waist-z
	  :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) -50))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    (print "End Place!"))
  )
;; Pick and place
(defun pick-place (pick-pos place-pos &key (arm :larm))
  (pick pick-pos)
  (unix::sleep 100000)
  (place place-pos))


(defun my-test nil
  (pick (elt *pos* 1))
  (read-char)
  (place (elt *pos* 0) :height -80)
  (read-char)
  (pick (elt *pos* 2))
  (read-char)
  (place (elt *pos* 0) :height -100)
  (read-char)
  (pick (elt *pos* 3))
  (read-char)
  (place (elt *pos* 0) :height -150))
;; Place


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 
;; callback
(setq *pos* nil)
(defun test nil
  (setq *pos* (tf-object-update))
  (ros::spin-once)
  (send *irtviewer* :draw-objects)
  (when *pos*
    (send-all *pos* :draw-on :flush t)
    (print "LALA")
    (sort *pos* #'(lambda (x y) (< (elt (send x :pos ) 2) 
			       (elt (send y :pos ) 2))))
    (dolist (el *pos*)
      (print el)))
  (x::window-main-one)
  (ros::sleep)
  )
;;
;; 
(ros::rate 10)
;;(while (ros::ok)
;;  (test))

;;(test)
