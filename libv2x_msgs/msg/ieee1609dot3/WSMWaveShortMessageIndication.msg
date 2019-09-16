libv2x_msgs/MsgHeader msg_header
# WSM-WaveShortMessage.indication
uint8         wsmp_version
uint8[<=1]    channel_number
uint8[<=1]    data_rate # 0x2-0x7F, increments of 500 kb/s, IEEE802.11-2016
int8[<=1]     transmit_power_used
uint8[<=8]    channel_load # undefined yet
uint8         user_priority # 0-7
# uint16        length # 1 to WsmMaxLength – h, WsmMaxLength: def=1400, max=2302
uint8[<=2302] data # length is included
uint8[6]      peer_mac_address # EUI48
uint64        provider_service_identifier
