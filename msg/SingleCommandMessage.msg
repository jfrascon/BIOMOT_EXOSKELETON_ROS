Header header
# Number of can frames to be sent.
int32     number_elements
# id's of the can frames to be sent.
int32[]   can_id
# Type of can frame -> type of data transmitted
int32     type
# Values to be sent through the CAN bus with id's specified by can_id vector.
float32[] data
