# Intelligent Elevator System (STM32F4 + STM32F1 + CAN)

一个面向嵌入式面试/课程设计的多电梯协同控制项目骨架，包含：
- `STM32F4` 主控（调度与策略）
- `STM32F1` 从控（单台电梯状态机）
- `CAN 2.0A` 协议（心跳、状态上报、调度指令）

> 当前仓库提供可直接迁移到 HAL 工程的核心逻辑与协议定义，便于后续接入真实硬件外设驱动。

## 目录结构

```text
firmware/
  common/
    can_protocol.h                 # 主从共享的 CAN 协议定义
  master_f4/
    inc/master_controller.h        # 主控调度接口
    src/master_controller.c        # 主控策略、故障降级、心跳超时检测
  slave_f1/
    inc/slave_elevator.h           # 从控状态机接口
    src/slave_elevator.c           # 运行/开关门/故障状态机实现
```

## 模块划分

### 1) 主控 F4（调度层）
- 接收所有轿厢状态上报与心跳
- 接收厅外呼梯请求并分配最优轿厢
- 对超时/故障轿厢进行降级隔离，避免继续派单

### 2) 从控 F1（执行层）
- 维护单台电梯运行状态机（IDLE/MOVING/DOOR_OPEN/FAULT）
- 执行主控下发目标楼层和方向指令
- 周期上报状态并发送心跳

### 3) 协议层（共享）
- 统一消息 ID、方向、状态、错误码
- 统一 8 字节数据区布局

## CAN 报文表（标准帧 11-bit）

> 节点 ID 范围建议：`1~4`（对应 4 台轿厢）

| 报文 | CAN ID | 方向 | 数据字节定义 |
|---|---:|---|---|
| 心跳 | `0x100 + node` | 从控 -> 主控 | `B0=node, B1=mode, B2=door, B3=fault, B4..B7=0` |
| 状态上报 | `0x200 + node` | 从控 -> 主控 | `B0=node, B1=floor, B2=motion, B3=mode, B4=door, B5=fault, B6..B7=0` |
| 派梯指令 | `0x300 + node` | 主控 -> 从控 | `B0=target_floor, B1=direction, B2=cmd, B3..B7=0` |
| 厅外呼梯（可选） | `0x400` | 面板/网关 -> 主控 | `B0=floor, B1=direction, B2..B7=0` |

## 最小可运行里程碑

1. **里程碑 M1：协议联通**
   - 主从均能收发心跳/状态/指令帧
2. **里程碑 M2：单梯闭环**
   - 主控下发目标楼层，从控完成运行 + 到站开门 + 关门
3. **里程碑 M3：多梯调度**
   - 厅外呼梯触发主控选梯并派梯
4. **里程碑 M4：容错降级**
   - 心跳超时自动隔离故障梯，剩余电梯继续服务

## 硬件落地建议

- 波特率：`500 kbps`
- 心跳周期：`100 ms`
- 心跳超时阈值：`500 ms`
- 从控周期状态上报：`100 ms`
- 主控调度周期：`50 ms`

## 下一步接入 HAL

1. 在 CubeMX 分别生成 F1/F4 工程（启用 CAN 中断接收）
2. 将 `firmware/common` 与对应控制器源码拷入工程
3. 在 `HAL_CAN_RxFifo0MsgPendingCallback` 中解析 CAN 帧并调用接口
4. 在定时器中断或主循环中周期调用 `Master_Tick/Slave_Tick`