# gz_ray_label_plugin
Gazebo plugin for labelling point clouds with a RayShape sensor

Example:
```
def callback_ray_labeled_points(data):
  global labelpoint, received_points_ray, worldpoints
  if received_points_ray:
    return
  log("Got labeled data: {}".format(len(data.points)))
  length = len(data.points)
  for i in range(length):
    point = data.points[i]
    entity = point.entityName
    index = point.index
    //DO SOMETHING
    
  received_points_ray = True

def label_points_by_ray():
  global points, received_points_ray
  received_points_ray = False

  pub = rospy.Publisher('/ray/points', LabelPoints, queue_size = 10)
  sub = rospy.Subscriber("/ray/labeled/points", LabelPoints, callback_ray_labeled_points)
  cloud = LabelPoints()
  cloud.scaling = 0.03

  length = len(worldpoints)
  for i in range(length):
    point = LabelPoint()
    point.x = points[i][0]
    point.y = points[i][1]
    point.z = points[i][2]
    point.index = i
    cloud.points.append(point)

  rospy.sleep(1)
  pub.publish(cloud)
  log("waiting for labeled points")
  while not rospy.is_shutdown():
    rospy.sleep(1)
    if received_points_ray:
      log("got")
      break

  log("labeling points done :)")
  sub.unregister()
  pub.unregister()
```
