(defun pr2-inverse-kinematics (des)
  (send *pr2* :inverse-kinematics
	(make-coords :pos des)
	:rotation-axis nil
	:move-target (send *pr2* :larm :end-coords)
	:link-list (send *pr2* :link-list
			 (send *pr2* :larm :end-coords :parent)
			 (send *pr2* :torso_lift_link))
	:debug-view t)
  )

(defun pr2-ri-angle-vector (&key (time 20))
  (send *ri* :angle-vector (send *pr2* :angle-vector) time)
  (send *ri* :wait-interpolation))

(defun pr2-hand-move (l &key (arm :larm))
  (send *pr2* :torso :waist-z :joint-angle (elt l 0))
  (send *pr2* arm :collar-y :joint-angle  (elt l 1))
  (send *pr2* arm :shoulder-p :joint-angle (elt l 2))
  (send *pr2* arm :shoulder-r :joint-angle (elt l 3))
  (send *pr2* arm :elbow-p :joint-angle (elt l 4))
  (send *pr2* arm :elbow-r :joint-angle (elt l 5))
  (send *pr2* arm :wrist-p :joint-angle (elt l 6))
  (send *pr2* arm :wrist-r :joint-angle (elt l 7))
  )

(setq ja-home (list 95 35 5 95 -30 0 12 95))
(setq ja-des (list 95 35 5 95 -55 0 12 95))

;; (pr2-hand-move ja-home)
;; (pr2-ri-angle-vector)


(defun grasp nil
  (send *ri* :move-gripper :larm 0.0 :effort 100 :wait t)
  (send *ri* :wait-interpolation))

(defun pr2-daruma-knock (&key (arm :larm))
  (pr2-hand-move ja-des :arm arm)
  (pr2-ri-angle-vector))
(defun pr2-daruma-return (&key (arm :larm))
  (pr2-hand-move ja-home :arm arm)
  (pr2-ri-angle-vector))
(defun pr2-daruma-turn (tr &key (arm :larm))
  (if (tr)
      (send *pr2* arm :wrist-r :joint-angle 95)
      (send *pr2* arm :wrist-r :joint-angle -85)))

(defun knock nil
  (pr2-hand-move ja-home)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (read-char)
  (send *ri* :move-gripper :larm 0.0 :effort 100 :wait t)
  (send *ri* :wait-interpolation)
  (read-char)
  (send *pr2* :larm :elbow-p :joint-angle -55)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 0)
  (send *ri* :wait-interpolation)
  (read-char)
  (send *pr2* :larm :wrist-r :joint-angle 95)
  (send *pr2* :larm :wrist-r :joint-angle -85)
  (send *pr2* :torso :waist-z :joint-angle
	(- (send *pr2* :torso :waist-z :joint-angle) 15))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (read-char)
  (send *pr2* :larm :elbow-p :joint-angle -30)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 0))
	
(defun knock-one nil
  (init)
  (pr2-hand-move ja-home)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :larm 0.0 :effort 100 :wait t)
  (send *ri* :wait-interpolation)
  (unix::sleep 1)
  (send *pr2* :larm :elbow-p :joint-angle -55)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 0)
  (send *ri* :wait-interpolation))

(defun pr2-test nil
  (knock-one)
  (unix::sleep 1)
  (knock-one)
  (unix::sleep 1)
  (setf (elt ja-home 0) 65)
  (knock-one))
