// MESSAGE CLIENT_IP PACKING

#define MAVLINK_MSG_ID_CLIENT_IP 126

typedef struct __mavlink_client_ip_t
{
 uint8_t IP1; /*< */
 uint8_t IP2; /*< */
 uint8_t IP3; /*< */
 uint8_t IP4; /*< */
} mavlink_client_ip_t;

#define MAVLINK_MSG_ID_CLIENT_IP_LEN 4
#define MAVLINK_MSG_ID_126_LEN 4

#define MAVLINK_MSG_ID_CLIENT_IP_CRC 63
#define MAVLINK_MSG_ID_126_CRC 63



#define MAVLINK_MESSAGE_INFO_CLIENT_IP { \
	"CLIENT_IP", \
	4, \
	{  { "IP1", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_client_ip_t, IP1) }, \
         { "IP2", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_client_ip_t, IP2) }, \
         { "IP3", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_client_ip_t, IP3) }, \
         { "IP4", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_client_ip_t, IP4) }, \
         } \
}


/**
 * @brief Pack a client_ip message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param IP1 
 * @param IP2 
 * @param IP3 
 * @param IP4 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_client_ip_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t IP1, uint8_t IP2, uint8_t IP3, uint8_t IP4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CLIENT_IP_LEN];
	_mav_put_uint8_t(buf, 0, IP1);
	_mav_put_uint8_t(buf, 1, IP2);
	_mav_put_uint8_t(buf, 2, IP3);
	_mav_put_uint8_t(buf, 3, IP4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#else
	mavlink_client_ip_t packet;
	packet.IP1 = IP1;
	packet.IP2 = IP2;
	packet.IP3 = IP3;
	packet.IP4 = IP4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CLIENT_IP;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CLIENT_IP_LEN, MAVLINK_MSG_ID_CLIENT_IP_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#endif
}

/**
 * @brief Pack a client_ip message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param IP1 
 * @param IP2 
 * @param IP3 
 * @param IP4 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_client_ip_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t IP1,uint8_t IP2,uint8_t IP3,uint8_t IP4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CLIENT_IP_LEN];
	_mav_put_uint8_t(buf, 0, IP1);
	_mav_put_uint8_t(buf, 1, IP2);
	_mav_put_uint8_t(buf, 2, IP3);
	_mav_put_uint8_t(buf, 3, IP4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#else
	mavlink_client_ip_t packet;
	packet.IP1 = IP1;
	packet.IP2 = IP2;
	packet.IP3 = IP3;
	packet.IP4 = IP4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CLIENT_IP;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CLIENT_IP_LEN, MAVLINK_MSG_ID_CLIENT_IP_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#endif
}

/**
 * @brief Encode a client_ip struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param client_ip C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_client_ip_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_client_ip_t* client_ip)
{
	return mavlink_msg_client_ip_pack(system_id, component_id, msg, client_ip->IP1, client_ip->IP2, client_ip->IP3, client_ip->IP4);
}

/**
 * @brief Encode a client_ip struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param client_ip C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_client_ip_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_client_ip_t* client_ip)
{
	return mavlink_msg_client_ip_pack_chan(system_id, component_id, chan, msg, client_ip->IP1, client_ip->IP2, client_ip->IP3, client_ip->IP4);
}

/**
 * @brief Send a client_ip message
 * @param chan MAVLink channel to send the message
 *
 * @param IP1 
 * @param IP2 
 * @param IP3 
 * @param IP4 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_client_ip_send(mavlink_channel_t chan, uint8_t IP1, uint8_t IP2, uint8_t IP3, uint8_t IP4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CLIENT_IP_LEN];
	_mav_put_uint8_t(buf, 0, IP1);
	_mav_put_uint8_t(buf, 1, IP2);
	_mav_put_uint8_t(buf, 2, IP3);
	_mav_put_uint8_t(buf, 3, IP4);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLIENT_IP, buf, MAVLINK_MSG_ID_CLIENT_IP_LEN, MAVLINK_MSG_ID_CLIENT_IP_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLIENT_IP, buf, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#endif
#else
	mavlink_client_ip_t packet;
	packet.IP1 = IP1;
	packet.IP2 = IP2;
	packet.IP3 = IP3;
	packet.IP4 = IP4;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLIENT_IP, (const char *)&packet, MAVLINK_MSG_ID_CLIENT_IP_LEN, MAVLINK_MSG_ID_CLIENT_IP_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLIENT_IP, (const char *)&packet, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CLIENT_IP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_client_ip_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t IP1, uint8_t IP2, uint8_t IP3, uint8_t IP4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, IP1);
	_mav_put_uint8_t(buf, 1, IP2);
	_mav_put_uint8_t(buf, 2, IP3);
	_mav_put_uint8_t(buf, 3, IP4);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLIENT_IP, buf, MAVLINK_MSG_ID_CLIENT_IP_LEN, MAVLINK_MSG_ID_CLIENT_IP_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLIENT_IP, buf, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#endif
#else
	mavlink_client_ip_t *packet = (mavlink_client_ip_t *)msgbuf;
	packet->IP1 = IP1;
	packet->IP2 = IP2;
	packet->IP3 = IP3;
	packet->IP4 = IP4;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLIENT_IP, (const char *)packet, MAVLINK_MSG_ID_CLIENT_IP_LEN, MAVLINK_MSG_ID_CLIENT_IP_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CLIENT_IP, (const char *)packet, MAVLINK_MSG_ID_CLIENT_IP_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CLIENT_IP UNPACKING


/**
 * @brief Get field IP1 from client_ip message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_client_ip_get_IP1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field IP2 from client_ip message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_client_ip_get_IP2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field IP3 from client_ip message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_client_ip_get_IP3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field IP4 from client_ip message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_client_ip_get_IP4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a client_ip message into a struct
 *
 * @param msg The message to decode
 * @param client_ip C-struct to decode the message contents into
 */
static inline void mavlink_msg_client_ip_decode(const mavlink_message_t* msg, mavlink_client_ip_t* client_ip)
{
#if MAVLINK_NEED_BYTE_SWAP
	client_ip->IP1 = mavlink_msg_client_ip_get_IP1(msg);
	client_ip->IP2 = mavlink_msg_client_ip_get_IP2(msg);
	client_ip->IP3 = mavlink_msg_client_ip_get_IP3(msg);
	client_ip->IP4 = mavlink_msg_client_ip_get_IP4(msg);
#else
	memcpy(client_ip, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CLIENT_IP_LEN);
#endif
}
