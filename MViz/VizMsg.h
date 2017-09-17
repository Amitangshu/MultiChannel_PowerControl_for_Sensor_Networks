/**
 * This file is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This file defines the layout of the 'mviz_msg' message type.
 */

#ifndef MVIZ_H_H
#define MVIZ_H_H
#include <message.h>

enum {
  /** The default size of this message type in bytes. */
  MVIZ_H_SIZE = 16,

  /** The Active Message type associated with this message. */
  MVIZ_H_AM_TYPE = 227,

  /* Field version: type uint16_t, offset (bits) 0, size (bits) 16 */
  /** Offset (in bytes) of the field 'version' */
  MVIZ_H_VERSION_OFFSET = 0,
  /** Offset (in bits) of the field 'version' */
  MVIZ_H_VERSION_OFFSETBITS = 0,
  /** Size (in bytes) of the field 'version' */
  MVIZ_H_VERSION_SIZE = 2,
  /** Size (in bits) of the field 'version' */
  MVIZ_H_VERSION_SIZEBITS = 16,

  /* Field interval: type uint16_t, offset (bits) 16, size (bits) 16 */
  /** Offset (in bytes) of the field 'interval' */
  MVIZ_H_INTERVAL_OFFSET = 2,
  /** Offset (in bits) of the field 'interval' */
  MVIZ_H_INTERVAL_OFFSETBITS = 16,
  /** Size (in bytes) of the field 'interval' */
  MVIZ_H_INTERVAL_SIZE = 2,
  /** Size (in bits) of the field 'interval' */
  MVIZ_H_INTERVAL_SIZEBITS = 16,

  /* Field origin: type uint16_t, offset (bits) 32, size (bits) 16 */
  /** Offset (in bytes) of the field 'origin' */
  MVIZ_H_ORIGIN_OFFSET = 4,
  /** Offset (in bits) of the field 'origin' */
  MVIZ_H_ORIGIN_OFFSETBITS = 32,
  /** Size (in bytes) of the field 'origin' */
  MVIZ_H_ORIGIN_SIZE = 2,
  /** Size (in bits) of the field 'origin' */
  MVIZ_H_ORIGIN_SIZEBITS = 16,

  /* Field count: type uint16_t, offset (bits) 48, size (bits) 16 */
  /** Offset (in bytes) of the field 'count' */
  MVIZ_H_COUNT_OFFSET = 6,
  /** Offset (in bits) of the field 'count' */
  MVIZ_H_COUNT_OFFSETBITS = 48,
  /** Size (in bytes) of the field 'count' */
  MVIZ_H_COUNT_SIZE = 2,
  /** Size (in bits) of the field 'count' */
  MVIZ_H_COUNT_SIZEBITS = 16,

  /* Field parent: type uint16_t, offset (bits) 64, size (bits) 16 */
  /** Offset (in bytes) of the field 'parent' */
  MVIZ_H_PARENT_OFFSET = 8,
  /** Offset (in bits) of the field 'parent' */
  MVIZ_H_PARENT_OFFSETBITS = 64,
  /** Size (in bytes) of the field 'parent' */
  MVIZ_H_PARENT_SIZE = 2,
  /** Size (in bits) of the field 'parent' */
  MVIZ_H_PARENT_SIZEBITS = 16,

  /* Field etx: type uint16_t, offset (bits) 80, size (bits) 16 */
  /** Offset (in bytes) of the field 'etx' */
  MVIZ_H_ETX_OFFSET = 10,
  /** Offset (in bits) of the field 'etx' */
  MVIZ_H_ETX_OFFSETBITS = 80,
  /** Size (in bytes) of the field 'etx' */
  MVIZ_H_ETX_SIZE = 2,
  /** Size (in bits) of the field 'etx' */
  MVIZ_H_ETX_SIZEBITS = 16,

  /* Field forward: type uint16_t, offset (bits) 96, size (bits) 16 */
  /** Offset (in bytes) of the field 'forward' */
  MVIZ_H_FORWARD_OFFSET = 12,
  /** Offset (in bits) of the field 'forward' */
  MVIZ_H_FORWARD_OFFSETBITS = 96,
  /** Size (in bytes) of the field 'forward' */
  MVIZ_H_FORWARD_SIZE = 2,
  /** Size (in bits) of the field 'forward' */
  MVIZ_H_FORWARD_SIZEBITS = 16,

  /* Field link_route_addr: type uint16_t, offset (bits) 112, size (bits) 16 */
  /** Offset (in bytes) of the field 'link_route_addr' */
  MVIZ_H_LINK_ROUTE_ADDR_OFFSET = 14,
  /** Offset (in bits) of the field 'link_route_addr' */
  MVIZ_H_LINK_ROUTE_ADDR_OFFSETBITS = 112,
  /** Size (in bytes) of the field 'link_route_addr' */
  MVIZ_H_LINK_ROUTE_ADDR_SIZE = 2,
  /** Size (in bits) of the field 'link_route_addr' */
  MVIZ_H_LINK_ROUTE_ADDR_SIZEBITS = 16,
};

/**
 * Return the value of the field 'version'
 */
uint16_t MVIZ_H_version_get(tmsg_t *msg);

/**
 * Set the value of the field 'version'
 */
void MVIZ_H_version_set(tmsg_t *msg, uint16_t value);

/**
 * Return the value of the field 'interval'
 */
uint16_t MVIZ_H_interval_get(tmsg_t *msg);

/**
 * Set the value of the field 'interval'
 */
void MVIZ_H_interval_set(tmsg_t *msg, uint16_t value);

/**
 * Return the value of the field 'origin'
 */
uint16_t MVIZ_H_origin_get(tmsg_t *msg);

/**
 * Set the value of the field 'origin'
 */
void MVIZ_H_origin_set(tmsg_t *msg, uint16_t value);

/**
 * Return the value of the field 'count'
 */
uint16_t MVIZ_H_count_get(tmsg_t *msg);

/**
 * Set the value of the field 'count'
 */
void MVIZ_H_count_set(tmsg_t *msg, uint16_t value);

/**
 * Return the value of the field 'parent'
 */
uint16_t MVIZ_H_parent_get(tmsg_t *msg);

/**
 * Set the value of the field 'parent'
 */
void MVIZ_H_parent_set(tmsg_t *msg, uint16_t value);

/**
 * Return the value of the field 'etx'
 */
uint16_t MVIZ_H_etx_get(tmsg_t *msg);

/**
 * Set the value of the field 'etx'
 */
void MVIZ_H_etx_set(tmsg_t *msg, uint16_t value);

/**
 * Return the value of the field 'forward'
 */
uint16_t MVIZ_H_forward_get(tmsg_t *msg);

/**
 * Set the value of the field 'forward'
 */
void MVIZ_H_forward_set(tmsg_t *msg, uint16_t value);

/**
 * Return the value of the field 'link_route_addr'
 */
uint16_t MVIZ_H_link_route_addr_get(tmsg_t *msg);

/**
 * Set the value of the field 'link_route_addr'
 */
void MVIZ_H_link_route_addr_set(tmsg_t *msg, uint16_t value);

#endif