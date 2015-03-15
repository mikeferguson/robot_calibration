# Name of the "sensor" that generate this data.
string sensor_name

# Features "detected" by the sensor.
geometry_msgs/PointStamped[] features

# Sensor information
ExtendedCameraInfo ext_camera_info

# Debugging data (optional)
sensor_msgs/PointCloud2 cloud
sensor_msgs/Image image
