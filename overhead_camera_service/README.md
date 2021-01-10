# Overhead_camera_service
This package ...

### message_filters [ROS documentation](http://wiki.ros.org/message_filters)
A set of message filters which take in messages and may output those messages at a later time,
based on the conditions that filter needs met.
An example is the time synchronizer, which takes in messages of *different types from multiple sources*, and outputs them 
only if it has received a message on each of those sources with the same timestamp.

### message_filters::sync_policies::ApproximateTime [ROS documentation](http://wiki.ros.org/message_filters/ApproximateTime)
This is a policy used by message_filters::sync::Synchronizer to match messages coming on a set of topics. 
Contrary to message_filters::sync::ExactTime, it can match messages even if they have different
time stamps. We call size of a set of messages the difference between the latest and earliest
time stamp in the set.