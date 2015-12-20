// MESSAGE ENCODER_VALUE PACKING

#define MAVLINK_MSG_ID_ENCODER_VALUE 125

typedef struct __mavlink_encoder_value_t
{
 uint32_t value; /*< */
 uint32_t stamp; /*< */
} mavlink_encoder_value_t;

#define MAVLINK_MSG_ID_ENCODER_VALUE_LEN 8
#define MAVLINK_MSG_ID_125_LEN 8

#define MAVLINK_MSG_ID_ENCODER_VALUE_CRC 158
#define MAVLINK_MSG_ID_125_CRC 158



#define MAVLINK_MESSAGE_INFO_ENCODER_VALUE { \
	"ENCODER_VALUE", \
	2, \
	{  { "value", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_encoder_value_t, value) }, \
         { "stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_encoder_value_t, stamp) }, \
         } \
}


/**
 * @brief Pack a encoder_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param value 
 * @param stamp 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_encoder_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t value, uint32_t stamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ENCODER_VALUE_LEN];
	_mav_put_uint32_t(buf, 0, value);
	_mav_put_uint32_t(buf, 4, stamp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#else
	mavlink_encoder_value_t packet;
	packet.value = value;
	packet.stamp = stamp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ENCODER_VALUE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ENCODER_VALUE_LEN, MAVLINK_MSG_ID_ENCODER_VALUE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#endif
}

/**
 * @brief Pack a encoder_value message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param value 
 * @param stamp 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_encoder_value_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t value,uint32_t stamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ENCODER_VALUE_LEN];
	_mav_put_uint32_t(buf, 0, value);
	_mav_put_uint32_t(buf, 4, stamp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#else
	mavlink_encoder_value_t packet;
	packet.value = value;
	packet.stamp = stamp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ENCODER_VALUE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ENCODER_VALUE_LEN, MAVLINK_MSG_ID_ENCODER_VALUE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#endif
}

/**
 * @brief Encode a encoder_value struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param encoder_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_encoder_value_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_encoder_value_t* encoder_value)
{
	return mavlink_msg_encoder_value_pack(system_id, component_id, msg, encoder_value->value, encoder_value->stamp);
}

/**
 * @brief Encode a encoder_value struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param encoder_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_encoder_value_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_encoder_value_t* encoder_value)
{
	return mavlink_msg_encoder_value_pack_chan(system_id, component_id, chan, msg, encoder_value->value, encoder_value->stamp);
}

/**
 * @brief Send a encoder_value message
 * @param chan MAVLink channel to send the message
 *
 * @param value 
 * @param stamp 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_encoder_value_send(mavlink_channel_t chan, uint32_t value, uint32_t stamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ENCODER_VALUE_LEN];
	_mav_put_uint32_t(buf, 0, value);
	_mav_put_uint32_t(buf, 4, stamp);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_VALUE, buf, MAVLINK_MSG_ID_ENCODER_VALUE_LEN, MAVLINK_MSG_ID_ENCODER_VALUE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_VALUE, buf, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#endif
#else
	mavlink_encoder_value_t packet;
	packet.value = value;
	packet.stamp = stamp;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_VALUE, (const char *)&packet, MAVLINK_MSG_ID_ENCODER_VALUE_LEN, MAVLINK_MSG_ID_ENCODER_VALUE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_VALUE, (const char *)&packet, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ENCODER_VALUE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_encoder_value_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t value, uint32_t stamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, value);
	_mav_put_uint32_t(buf, 4, stamp);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_VALUE, buf, MAVLINK_MSG_ID_ENCODER_VALUE_LEN, MAVLINK_MSG_ID_ENCODER_VALUE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_VALUE, buf, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#endif
#else
	mavlink_encoder_value_t *packet = (mavlink_encoder_value_t *)msgbuf;
	packet->value = value;
	packet->stamp = stamp;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_VALUE, (const char *)packet, MAVLINK_MSG_ID_ENCODER_VALUE_LEN, MAVLINK_MSG_ID_ENCODER_VALUE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_VALUE, (const char *)packet, MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ENCODER_VALUE UNPACKING


/**
 * @brief Get field value from encoder_value message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_encoder_value_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field stamp from encoder_value message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_encoder_value_get_stamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a encoder_value message into a struct
 *
 * @param msg The message to decode
 * @param encoder_value C-struct to decode the message contents into
 */
static inline void mavlink_msg_encoder_value_decode(const mavlink_message_t* msg, mavlink_encoder_value_t* encoder_value)
{
#if MAVLINK_NEED_BYTE_SWAP
	encoder_value->value = mavlink_msg_encoder_value_get_value(msg);
	encoder_value->stamp = mavlink_msg_encoder_value_get_stamp(msg);
#else
	memcpy(encoder_value, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ENCODER_VALUE_LEN);
#endif
}
