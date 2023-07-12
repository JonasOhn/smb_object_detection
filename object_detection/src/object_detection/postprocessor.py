#!/usr/bin/env python3
import math
import csv
import tf
import rospy
from geometry_msgs.msg import Point, PointStamped
from object_detection_msgs.msg import ObjectDetectionInfoArray

class Postprocessor:
    def __init__(self, filename, threshold, goal_frame, timeout=5.0):
        self.timeout = timeout
        self.artifacts = {}
        self.filename = filename
        self.detection_threshold = threshold
        self.classes = {}
        self.goal_frame = goal_frame
        self.last_msg_time = None
        self.it = 0

        self.detections_sub = rospy.Subscriber("/object_detector/detection_info", ObjectDetectionInfoArray, self.detection_callback)
        self.listener = tf.TransformListener()
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback) # TODO: delete if not used
        rospy.on_shutdown(self.shutdown_procedure)

    def detection_callback(self, msg):
        self.last_msg_time = rospy.get_time()
        for info in msg.info:
            try:
                if info.confidence < 0.5:
                    return
                
                # TODO: check correctness
                point_stamped = PointStamped()
                point_stamped.header.frame_id = msg.header.frame_id
                point_stamped.point = info.position
                point_stamped.header.stamp = msg.header.stamp
                world_point_stamped = self.listener.transformPoint(self.goal_frame, point_stamped)
                world_pos = world_point_stamped.point

                if info.class_id not in self.artifacts:
                    self.artifacts[info.class_id] = []
                
                matched = False

                for det_obj in self.artifacts[info.class_id]:
                    point_dist = math.dist([world_pos.x, world_pos.y, world_pos.z], 
                                           [det_obj['pos'].x, det_obj['pos'].y, det_obj['pos'].z])
                    
                    if point_dist < self.detection_threshold:
                        
                        det_obj['point_cnt'] = det_obj['point_cnt'] + 1
                        det_obj['sum'] = det_obj['sum'] + world_pos
                        
                        updated_point = Point()
                        updated_point.x = det_obj['sum'].x / det_obj['point_cnt']
                        updated_point.y = det_obj['sum'].y / det_obj['point_cnt']
                        updated_point.z = det_obj['sum'].z / det_obj['point_cnt']
                        det_obj['pos'] = updated_point

                        matched = True
                        break
                
                if not matched:
                    self.artifacts[info.class_id].append({'pos':  world_pos,
                                                             'point_cnt': 1,
                                                             'sum': world_pos})
                print(self.artifacts)
                self.save_to_csv()
            except Exception as e:
                print(e)
    
    def timer_callback(self, event):
        if self.last_msg_time is not None:
            if rospy.get_time() - self.last_msg_time > self.timeout:
                self.timer.shutdown()
                self.save_to_csv()

    def save_to_csv(self):
        output_file = f"{self.filename}_{self.it%2}.csv"
        with open(output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Label', 'X', 'Y', 'Z'])
            for label, lists in self.artifacts.items():
                for res in lists:
                    writer.writerow([label, res['pos'].x, res['pos'].y, res['pos'].z])
        print(f"Data written to {output_file}")
        self.it += 1
        self.it %= 2

    def shutdown_procedure(self):
        self.save_to_csv()

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node('postprocessor')
    postprocessor = Postprocessor(filename="tmp", threshold=0.4, goal_frame="base_link")
    
    try:
        postprocessor.run()
    except rospy.ROSInterruptException:
        pass

