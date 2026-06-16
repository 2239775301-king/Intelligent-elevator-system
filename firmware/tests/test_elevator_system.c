#include <assert.h>
#include <stddef.h>

#include "../common/can_protocol.h"
#include "../master_f4/inc/master_controller.h"
#include "../slave_f1/inc/slave_elevator.h"

static void test_master_multi_dispatch(void) {
    master_controller_t master;
    Master_Init(&master, 500U);

    elevator_status_t n1 = {1U, 2U, DIRECTION_IDLE, ELEVATOR_MODE_IDLE, DOOR_CLOSED, 0U};
    elevator_status_t n2 = {2U, 10U, DIRECTION_IDLE, ELEVATOR_MODE_IDLE, DOOR_CLOSED, 0U};
    Master_OnStatus(&master, &n1, 0U);
    Master_OnStatus(&master, &n2, 0U);

    hall_call_t c1 = {3U, DIRECTION_UP};
    hall_call_t c2 = {8U, DIRECTION_DOWN};
    Master_OnHallCall(&master, &c1);
    Master_OnHallCall(&master, &c2);

    can_tx_frame_t frames[4] = {0};
    size_t count = Master_Tick(&master, 10U, frames, 4U);
    assert(count == 2U);
    assert(frames[0].id == can_id_command(1U));
    assert(frames[0].data[0] == 3U);
    assert(frames[1].id == can_id_command(2U));
    assert(frames[1].data[0] == 8U);
}

static void test_master_timeout_isolation(void) {
    master_controller_t master;
    Master_Init(&master, 500U);
    Master_OnHeartbeat(&master, 1U, 0U);

    hall_call_t call = {5U, DIRECTION_UP};
    Master_OnHallCall(&master, &call);

    can_tx_frame_t frame = {0};
    size_t count = Master_Tick(&master, 700U, &frame, 1U);
    assert(count == 0U);
    assert(master.nodes[1U].status.mode == ELEVATOR_MODE_FAULT);
    assert(master.nodes[1U].status.fault_code == FAULT_CODE_HEARTBEAT_TIMEOUT);
}

static void test_slave_state_machine_cycle(void) {
    slave_elevator_t slave;
    Slave_Init(&slave, 1U, 0U);
    Slave_ProcessAssignCommand(&slave, 2U, DIRECTION_UP, 0U);
    assert(slave.mode == ELEVATOR_MODE_MOVING);

    Slave_Tick(&slave, 1000U);
    assert(slave.current_floor == 1U);
    assert(slave.mode == ELEVATOR_MODE_MOVING);

    Slave_Tick(&slave, 2000U);
    assert(slave.current_floor == 2U);
    assert(slave.mode == ELEVATOR_MODE_DOOR_OPEN);
    assert(slave.door == DOOR_OPENED);

    Slave_Tick(&slave, 4000U);
    assert(slave.mode == ELEVATOR_MODE_IDLE);
    assert(slave.door == DOOR_CLOSED);
}

static void test_slave_invalid_and_recovery(void) {
    slave_elevator_t slave;
    Slave_Init(&slave, 1U, 0U);

    Slave_ProcessCommand(&slave, 0xFFU, 0U, DIRECTION_IDLE, 100U);
    assert(slave.mode == ELEVATOR_MODE_FAULT);
    assert(slave.fault_code == FAULT_CODE_INVALID_COMMAND);

    Slave_ProcessCommand(&slave, COMMAND_CODE_CLEAR_FAULT, 0U, DIRECTION_IDLE, 200U);
    assert(slave.mode == ELEVATOR_MODE_IDLE);
    assert(slave.fault_code == 0U);
    assert(slave.current_floor == 0U);
}

int main(void) {
    test_master_multi_dispatch();
    test_master_timeout_isolation();
    test_slave_state_machine_cycle();
    test_slave_invalid_and_recovery();
    return 0;
}
