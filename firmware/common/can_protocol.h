#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define ELEVATOR_MAX_NODES 4U
#define ELEVATOR_MAX_FLOOR 31U

#define CAN_ID_HEARTBEAT_BASE 0x100U
#define CAN_ID_STATUS_BASE    0x200U
#define CAN_ID_COMMAND_BASE   0x300U
#define CAN_ID_HALL_CALL      0x400U

#define COMMAND_CODE_ASSIGN_TARGET 0x01U
/* 0xEE marks master-detected heartbeat timeout and keeps value distinct from low fault IDs. */
#define FAULT_CODE_HEARTBEAT_TIMEOUT 0xEEU

typedef enum {
    DIRECTION_IDLE = 0U,
    DIRECTION_UP = 1U,
    DIRECTION_DOWN = 2U
} direction_t;

typedef enum {
    ELEVATOR_MODE_IDLE = 0U,
    ELEVATOR_MODE_MOVING = 1U,
    ELEVATOR_MODE_DOOR_OPEN = 2U,
    ELEVATOR_MODE_FAULT = 3U
} elevator_mode_t;

typedef enum {
    DOOR_CLOSED = 0U,
    DOOR_OPENED = 1U
} door_state_t;

typedef struct {
    uint8_t node_id;
    uint8_t floor;
    direction_t motion;
    elevator_mode_t mode;
    door_state_t door;
    uint8_t fault_code;
} elevator_status_t;

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_tx_frame_t;

typedef struct {
    uint8_t floor;
    direction_t direction;
} hall_call_t;

static inline bool is_valid_node(uint8_t node_id) {
    return node_id >= 1U && node_id <= ELEVATOR_MAX_NODES;
}

static inline uint16_t can_id_heartbeat(uint8_t node_id) {
    return (uint16_t)(CAN_ID_HEARTBEAT_BASE + node_id);
}

static inline uint16_t can_id_status(uint8_t node_id) {
    return (uint16_t)(CAN_ID_STATUS_BASE + node_id);
}

static inline uint16_t can_id_command(uint8_t node_id) {
    return (uint16_t)(CAN_ID_COMMAND_BASE + node_id);
}

#endif
