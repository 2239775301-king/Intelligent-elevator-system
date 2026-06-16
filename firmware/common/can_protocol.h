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
#define COMMAND_CODE_CLEAR_FAULT   0x02U
/* 0xEE marks master-detected heartbeat timeout and keeps value distinct from low fault IDs. */
#define FAULT_CODE_HEARTBEAT_TIMEOUT 0xEEU
#define FAULT_CODE_INVALID_COMMAND   0xE1U
#define FAULT_CODE_FLOOR_LIMIT       0xE2U

#define HEARTBEAT_PERIOD_MS_DEFAULT 100U
#define STATUS_PERIOD_MS_DEFAULT    100U
#define MASTER_TICK_MS_DEFAULT       50U

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

typedef enum {
    CAN_FRAME_UNKNOWN = 0U,
    CAN_FRAME_HEARTBEAT = 1U,
    CAN_FRAME_STATUS = 2U,
    CAN_FRAME_COMMAND = 3U,
    CAN_FRAME_HALL_CALL = 4U
} can_frame_type_t;

static inline bool is_valid_node(uint8_t node_id) {
    return node_id >= 1U && node_id <= ELEVATOR_MAX_NODES;
}

static inline bool is_valid_floor(uint8_t floor) {
    return floor <= ELEVATOR_MAX_FLOOR;
}

static inline bool is_valid_direction(direction_t direction) {
    return direction == DIRECTION_IDLE || direction == DIRECTION_UP || direction == DIRECTION_DOWN;
}

static inline bool is_valid_mode(elevator_mode_t mode) {
    return mode == ELEVATOR_MODE_IDLE
        || mode == ELEVATOR_MODE_MOVING
        || mode == ELEVATOR_MODE_DOOR_OPEN
        || mode == ELEVATOR_MODE_FAULT;
}

static inline bool is_valid_door(door_state_t door) {
    return door == DOOR_CLOSED || door == DOOR_OPENED;
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

static inline can_frame_type_t can_frame_type_from_id(uint16_t id) {
    if (id == CAN_ID_HALL_CALL) {
        return CAN_FRAME_HALL_CALL;
    }
    if (id > CAN_ID_HEARTBEAT_BASE && id <= CAN_ID_HEARTBEAT_BASE + ELEVATOR_MAX_NODES) {
        return CAN_FRAME_HEARTBEAT;
    }
    if (id > CAN_ID_STATUS_BASE && id <= CAN_ID_STATUS_BASE + ELEVATOR_MAX_NODES) {
        return CAN_FRAME_STATUS;
    }
    if (id > CAN_ID_COMMAND_BASE && id <= CAN_ID_COMMAND_BASE + ELEVATOR_MAX_NODES) {
        return CAN_FRAME_COMMAND;
    }
    return CAN_FRAME_UNKNOWN;
}

#endif
