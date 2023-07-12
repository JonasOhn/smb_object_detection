#!/usr/bin/env python3
import math
import csv
import rospy
from geometry_msgs.msg import Point
from object_detection_msgs.msg import ObjectDetectionInfoArray

class Postprocessor:
    def __init__(self, filename, threshold):
        self.measurements = {}
        self.artifacts = {}
        self.filename = filename
        self.threshold = threshold
        self.detections_sub = rospy.Subscriber("/object_detector/detection_info", ObjectDetectionInfoArray, self.detection_callback)
        self.classes = {}

    def detection_callback(self, msg):
        # print(type(msg.info))
        for info in msg.info:
            try:
                if info.class_id not in self.measurements:
                    self.measurements[info.class_id] = []
                world_pos = info.position # fill!!!!
                self.measurements[info.class_id].append({'pos': world_pos,
                                                         'conf': info.confidence})
                print(self.measurements)
            except Exception as e:
                print(e)
    
    def process(self):
        # change to world frame ?
        pass

    def save_to_csv(self):
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Label', 'X', 'Y', 'Z'])
            for label, lists in self.artifacts.items():
                for xyz in lists:
                    writer.writerow([label] + xyz)
        print(f"Data written to {self.filename}")

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node('postprocessor')
    postprocessor = Postprocessor("tmp.csv", 0.1)
    
    try:
        postprocessor.run()
    except rospy.ROSInterruptException:
        pass




# def add_artifact(label, position_3d, artifacts, threshold):
#     if not is_list_with_three_numbers(position_3d):
#         return False
#     try:
#         if label in artifacts:
#             other_objects = artifacts[label]
#             for obj_pos in other_objects:
#                 if math.dist(obj_pos, position_3d) < threshold:
#                     return False
#         else:
#             artifacts[label] = []
#         artifacts[label].append(position_3d)
#         return True
#     except Exception as e:
#         print(e)
#         return False

# objs = [['cat', [0.0, 0.0, 0.0]], 
#       ['dog', [0.0, 0.0, 0.0]], 
#       ['cat', [0.5, 0.5, 0.5]], 
#       ['cat', [0.0, 0.05, 0.0]], 
#       ['parrot', [None, 0.0, 0.0, 0.0]]]


# artifacts = {}
# threshold = 0.1

# for el in objs:
#     add_artifact(el[0], el[1], artifacts, threshold)

# print(artifacts)
# write_to_csv("tmp.csv", artifacts)
    





