import rosbag


with rosbag.Bag('output_combined.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('output_arm.bag').read_messages():
        outbag.write(topic, msg, t)

    last  = t
    first = 1

    for topic, msg, t in rosbag.Bag('output_ground.bag').read_messages():
        if first:
            offset = t - last
            first  = 0
        outbag.write(topic, msg, (t-offset))

