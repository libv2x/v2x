libv2x_msgs/MsgHeader msg_header
# DL-UNITDATA indication
uint8[6]      source_address
uint8[6]      destination_address
uint8[<=2308] data
uint8         priority
