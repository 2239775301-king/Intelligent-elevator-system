#include "../inc/master_controller.h"

/* +2 cost makes moving-away cars less preferred without starving distant idle cars. */
#define DIRECTION_MISMATCH_PENALTY 2U

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t previous_ms) {
    return (uint32_t)(now_ms - previous_ms);
}

static uint32_t popcount32(uint32_t value) {
#if defined(__GNUC__) || defined(__clang__)
    return (uint32_t)__builtin_popcount(value);
#else
    uint32_t count = 0U;
    while (value != 0U) {
        value &= (value - 1U);
        count++;
    }
    return count;
#endif
}

static uint8_t clamp_floor(uint8_t floor) {
    return (floor > ELEVATOR_MAX_FLOOR) ? ELEVATOR_MAX_FLOOR : floor;
}

static void set_hall_call(master_controller_t* controller, hall_call_t call) {
    uint32_t mask = (1UL << clamp_floor(call.floor));
    if (call.direction == DIRECTION_DOWN) {
        controller->pending_down_mask |= mask;
        return;
    }
    controller->pending_up_mask |= mask;
}

static bool get_next_call(const master_controller_t* controller, hall_call_t* out_call) {
    for (uint8_t f = 0U; f <= ELEVATOR_MAX_FLOOR; ++f) {
        uint32_t mask = (1UL << f);
        if ((controller->pending_up_mask & mask) != 0U) {
            out_call->floor = f;
            out_call->direction = DIRECTION_UP;
            return true;
        }
        if ((controller->pending_down_mask & mask) != 0U) {
            out_call->floor = f;
            out_call->direction = DIRECTION_DOWN;
            return true;
        }
    }
    return false;
}

static bool get_next_call_after(const master_controller_t* controller, hall_call_t* in_out_call) {
    if (in_out_call->floor >= ELEVATOR_MAX_FLOOR) {
        return false;
    }
    uint8_t start = (uint8_t)(in_out_call->floor + 1U);
    for (uint8_t f = start; f <= ELEVATOR_MAX_FLOOR; ++f) {
        uint32_t mask = (1UL << f);
        if ((controller->pending_up_mask & mask) != 0U) {
            in_out_call->floor = f;
            in_out_call->direction = DIRECTION_UP;
            return true;
        }
        if ((controller->pending_down_mask & mask) != 0U) {
            in_out_call->floor = f;
            in_out_call->direction = DIRECTION_DOWN;
            return true;
        }
    }
    return false;
}

static void clear_call(master_controller_t* controller, hall_call_t call) {
    uint32_t mask = ~(1UL << clamp_floor(call.floor));
    if (call.direction == DIRECTION_DOWN) {
        controller->pending_down_mask &= mask;
        return;
    }
    controller->pending_up_mask &= mask;
}

static void clear_call_for_arrived_car(master_controller_t* controller, const elevator_status_t* status) {
    if (status->mode != ELEVATOR_MODE_DOOR_OPEN || status->door != DOOR_OPENED) {
        return;
    }

    hall_call_t up_call = {status->floor, DIRECTION_UP};
    hall_call_t down_call = {status->floor, DIRECTION_DOWN};
    clear_call(controller, up_call);
    clear_call(controller, down_call);
}

static uint8_t select_best_node(const master_controller_t* controller, hall_call_t call) {
    uint8_t best_node = 0U;
    uint8_t best_cost = UINT8_MAX;

    for (uint8_t node = 1U; node <= ELEVATOR_MAX_NODES; ++node) {
        const master_node_state_t* n = &controller->nodes[node];
        if (!n->online || n->status.mode == ELEVATOR_MODE_FAULT) {
            continue;
        }

        uint8_t distance = (n->status.floor > call.floor)
            ? (uint8_t)(n->status.floor - call.floor)
            : (uint8_t)(call.floor - n->status.floor);
        /* Prefer cars already moving toward the request by adding a small cost to cars moving away. */
        uint8_t penalty = 0U;
        bool moving_away = (n->status.motion == DIRECTION_UP && n->status.floor > call.floor)
            || (n->status.motion == DIRECTION_DOWN && n->status.floor < call.floor);
        if (n->status.mode == ELEVATOR_MODE_MOVING && moving_away) {
            penalty = DIRECTION_MISMATCH_PENALTY;
        }

        uint8_t cost = (uint8_t)(distance + penalty);
        if (cost < best_cost) {
            best_cost = cost;
            best_node = node;
        }
    }

    return best_node;
}

static can_tx_frame_t build_assign_frame(uint8_t node, hall_call_t call) {
    can_tx_frame_t frame = {0};
    frame.id = can_id_command(node);
    frame.dlc = 8U;
    frame.data[0] = clamp_floor(call.floor);
    frame.data[1] = (uint8_t)call.direction;
    frame.data[2] = COMMAND_CODE_ASSIGN_TARGET;
    return frame;
}

void Master_Init(master_controller_t* controller, uint32_t heartbeat_timeout_ms) {
    if (controller == NULL) {
        return;
    }

    for (uint8_t i = 0U; i <= ELEVATOR_MAX_NODES; ++i) {
        controller->nodes[i].online = false;
        controller->nodes[i].last_heartbeat_ms = 0U;
        controller->nodes[i].status.node_id = i;
        controller->nodes[i].status.floor = 0U;
        controller->nodes[i].status.motion = DIRECTION_IDLE;
        controller->nodes[i].status.mode = ELEVATOR_MODE_IDLE;
        controller->nodes[i].status.door = DOOR_CLOSED;
        controller->nodes[i].status.fault_code = 0U;
    }

    controller->pending_up_mask = 0U;
    controller->pending_down_mask = 0U;
    controller->heartbeat_timeout_ms = heartbeat_timeout_ms;
    controller->total_dispatch_count = 0U;
}

void Master_OnHeartbeat(master_controller_t* controller, uint8_t node_id, uint32_t now_ms) {
    if (controller == NULL || !is_valid_node(node_id)) {
        return;
    }

    master_node_state_t* node = &controller->nodes[node_id];
    node->online = true;
    node->last_heartbeat_ms = now_ms;
    if (node->status.mode == ELEVATOR_MODE_FAULT
        && node->status.fault_code == FAULT_CODE_HEARTBEAT_TIMEOUT) {
        node->status.mode = ELEVATOR_MODE_IDLE;
        node->status.motion = DIRECTION_IDLE;
        node->status.door = DOOR_CLOSED;
        node->status.fault_code = 0U;
    }
}

void Master_OnStatus(master_controller_t* controller, const elevator_status_t* status, uint32_t now_ms) {
    if (controller == NULL || status == NULL || !is_valid_node(status->node_id)
        || !is_valid_floor(status->floor)
        || !is_valid_direction(status->motion)
        || !is_valid_mode(status->mode)
        || !is_valid_door(status->door)) {
        return;
    }

    master_node_state_t* node = &controller->nodes[status->node_id];
    node->status = *status;
    node->online = true;
    node->last_heartbeat_ms = now_ms;
    clear_call_for_arrived_car(controller, status);
}

void Master_OnHallCall(master_controller_t* controller, const hall_call_t* call) {
    if (controller == NULL || call == NULL || !is_valid_floor(call->floor)) {
        return;
    }

    hall_call_t normalized = *call;
    if (normalized.direction != DIRECTION_DOWN) {
        normalized.direction = DIRECTION_UP;
    }
    set_hall_call(controller, normalized);
}

static size_t dispatch_pending_calls(
    master_controller_t* controller,
    can_tx_frame_t* out_frames,
    size_t out_capacity) {
    hall_call_t call = {0};
    if (!get_next_call(controller, &call)) {
        return 0U;
    }

    size_t out_count = 0U;
    do {
        if (out_count >= out_capacity) {
            break;
        }
        uint8_t node = select_best_node(controller, call);
        if (node != 0U) {
            out_frames[out_count++] = build_assign_frame(node, call);
            clear_call(controller, call);
            controller->total_dispatch_count++;
        }
    } while (get_next_call_after(controller, &call));

    return out_count;
}

size_t Master_Tick(
    master_controller_t* controller,
    uint32_t now_ms,
    can_tx_frame_t* out_frames,
    size_t out_capacity) {
    if (controller == NULL || out_frames == NULL || out_capacity == 0U) {
        return 0U;
    }

    for (uint8_t node = 1U; node <= ELEVATOR_MAX_NODES; ++node) {
        master_node_state_t* n = &controller->nodes[node];
        if (n->online && elapsed_ms(now_ms, n->last_heartbeat_ms) >= controller->heartbeat_timeout_ms) {
            n->online = false;
            n->status.mode = ELEVATOR_MODE_FAULT;
            n->status.fault_code = FAULT_CODE_HEARTBEAT_TIMEOUT;
        }
    }

    return dispatch_pending_calls(controller, out_frames, out_capacity);
}

void Master_GetObserver(const master_controller_t* controller, master_observer_t* out_observer) {
    if (controller == NULL || out_observer == NULL) {
        return;
    }

    out_observer->pending_call_count = popcount32(controller->pending_up_mask)
        + popcount32(controller->pending_down_mask);
    out_observer->online_node_count = 0U;
    out_observer->timeout_isolation_count = 0U;
    out_observer->total_dispatch_count = controller->total_dispatch_count;
    out_observer->has_schedulable_node = false;

    for (uint8_t node = 1U; node <= ELEVATOR_MAX_NODES; ++node) {
        const master_node_state_t* n = &controller->nodes[node];
        if (n->online) {
            out_observer->online_node_count++;
        }
        if (n->status.fault_code == FAULT_CODE_HEARTBEAT_TIMEOUT) {
            out_observer->timeout_isolation_count++;
        }
        if (n->online && n->status.mode != ELEVATOR_MODE_FAULT) {
            out_observer->has_schedulable_node = true;
        }
    }
}
