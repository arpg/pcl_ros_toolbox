# pcl_ros_toolbox

ApproxTimeConcatenateServer nodelet: Uses the ApproximateTime message_filter sync_policy to synchronize up to 9 PC inputs on individual topics, requires all inputs all the time (any drops in input connection will drop output)

MixedConcatenateServer nodelet: Stores a queue of PC messages from a common topic and transforms and concatenates them using frame_id field.
2 modes (termination_method):
- message-window termination, after storing termination_value messages, concats those messages and publishes as a whole cloud
- time-window termination, after storing at most MAX_QUEUE_SIZE messages spanning termination_value milliseconds, concats those messages and publishes as a whole cloud. This method may reduce the number of partial clouds used to build the whole cloud to maintain consistency with the termination_value parameter.
