// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#ifndef _wescore_lcm_msgs_Range_h
#define _wescore_lcm_msgs_Range_h

#include <stdint.h>
#include <stdlib.h>
#include <lcm/lcm_coretypes.h>
#include <lcm/lcm.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WESCORE_LCM_MSGS_RANGE_ULTRASOUND 0
#define WESCORE_LCM_MSGS_RANGE_INFRARED 1

/// Ultrasonic/Infrared
typedef struct _wescore_lcm_msgs_Range wescore_lcm_msgs_Range;
struct _wescore_lcm_msgs_Range
{
    int64_t    mtime;
    int8_t     radiation_type;
    float      field_of_view;
    float      min_range;
    float      max_range;
    float      range;
};

/**
 * Create a deep copy of a wescore_lcm_msgs_Range.
 * When no longer needed, destroy it with wescore_lcm_msgs_Range_destroy()
 */
wescore_lcm_msgs_Range* wescore_lcm_msgs_Range_copy(const wescore_lcm_msgs_Range* to_copy);

/**
 * Destroy an instance of wescore_lcm_msgs_Range created by wescore_lcm_msgs_Range_copy()
 */
void wescore_lcm_msgs_Range_destroy(wescore_lcm_msgs_Range* to_destroy);

/**
 * Identifies a single subscription.  This is an opaque data type.
 */
typedef struct _wescore_lcm_msgs_Range_subscription_t wescore_lcm_msgs_Range_subscription_t;

/**
 * Prototype for a callback function invoked when a message of type
 * wescore_lcm_msgs_Range is received.
 */
typedef void(*wescore_lcm_msgs_Range_handler_t)(
    const lcm_recv_buf_t *rbuf, const char *channel,
    const wescore_lcm_msgs_Range *msg, void *userdata);

/**
 * Publish a message of type wescore_lcm_msgs_Range using LCM.
 *
 * @param lcm The LCM instance to publish with.
 * @param channel The channel to publish on.
 * @param msg The message to publish.
 * @return 0 on success, <0 on error.  Success means LCM has transferred
 * responsibility of the message data to the OS.
 */
int wescore_lcm_msgs_Range_publish(lcm_t *lcm, const char *channel, const wescore_lcm_msgs_Range *msg);

/**
 * Subscribe to messages of type wescore_lcm_msgs_Range using LCM.
 *
 * @param lcm The LCM instance to subscribe with.
 * @param channel The channel to subscribe to.
 * @param handler The callback function invoked by LCM when a message is
 *     received. This function is invoked by LCM during calls to lcm_handle()
 *     and lcm_handle_timeout().
 * @param userdata An opaque pointer passed to @p handler when it is invoked.
 * @return 0 on success, <0 if an error occured
 */
wescore_lcm_msgs_Range_subscription_t* wescore_lcm_msgs_Range_subscribe(
    lcm_t *lcm, const char *channel, wescore_lcm_msgs_Range_handler_t handler, void *userdata);

/**
 * Removes and destroys a subscription created by wescore_lcm_msgs_Range_subscribe()
 */
int wescore_lcm_msgs_Range_unsubscribe(lcm_t *lcm, wescore_lcm_msgs_Range_subscription_t* hid);

/**
 * Sets the queue capacity for a subscription.
 * Some LCM providers (e.g., the default multicast provider) are implemented
 * using a background receive thread that constantly revceives messages from
 * the network.  As these messages are received, they are buffered on
 * per-subscription queues until dispatched by lcm_handle().  This function
 * how many messages are queued before dropping messages.
 *
 * @param subs the subscription to modify.
 * @param num_messages The maximum number of messages to queue
 *  on the subscription.
 * @return 0 on success, <0 if an error occured
 */
int wescore_lcm_msgs_Range_subscription_set_queue_capacity(
    wescore_lcm_msgs_Range_subscription_t* subs, int num_messages);

/**
 * Encode a message of type wescore_lcm_msgs_Range into binary form.
 *
 * @param buf The output buffer.
 * @param offset Encoding starts at this byte offset into @p buf.
 * @param maxlen Maximum number of bytes to write.  This should generally
 *               be equal to wescore_lcm_msgs_Range_encoded_size().
 * @param msg The message to encode.
 * @return The number of bytes encoded, or <0 if an error occured.
 */
int wescore_lcm_msgs_Range_encode(void *buf, int offset, int maxlen, const wescore_lcm_msgs_Range *p);

/**
 * Decode a message of type wescore_lcm_msgs_Range from binary form.
 * When decoding messages containing strings or variable-length arrays, this
 * function may allocate memory.  When finished with the decoded message,
 * release allocated resources with wescore_lcm_msgs_Range_decode_cleanup().
 *
 * @param buf The buffer containing the encoded message
 * @param offset The byte offset into @p buf where the encoded message starts.
 * @param maxlen The maximum number of bytes to read while decoding.
 * @param msg Output parameter where the decoded message is stored
 * @return The number of bytes decoded, or <0 if an error occured.
 */
int wescore_lcm_msgs_Range_decode(const void *buf, int offset, int maxlen, wescore_lcm_msgs_Range *msg);

/**
 * Release resources allocated by wescore_lcm_msgs_Range_decode()
 * @return 0
 */
int wescore_lcm_msgs_Range_decode_cleanup(wescore_lcm_msgs_Range *p);

/**
 * Check how many bytes are required to encode a message of type wescore_lcm_msgs_Range
 */
int wescore_lcm_msgs_Range_encoded_size(const wescore_lcm_msgs_Range *p);

// LCM support functions. Users should not call these
int64_t __wescore_lcm_msgs_Range_get_hash(void);
uint64_t __wescore_lcm_msgs_Range_hash_recursive(const __lcm_hash_ptr *p);
int __wescore_lcm_msgs_Range_encode_array(
    void *buf, int offset, int maxlen, const wescore_lcm_msgs_Range *p, int elements);
int __wescore_lcm_msgs_Range_decode_array(
    const void *buf, int offset, int maxlen, wescore_lcm_msgs_Range *p, int elements);
int __wescore_lcm_msgs_Range_decode_array_cleanup(wescore_lcm_msgs_Range *p, int elements);
int __wescore_lcm_msgs_Range_encoded_array_size(const wescore_lcm_msgs_Range *p, int elements);
int __wescore_lcm_msgs_Range_clone_array(const wescore_lcm_msgs_Range *p, wescore_lcm_msgs_Range *q, int elements);

#ifdef __cplusplus
}
#endif

#endif
