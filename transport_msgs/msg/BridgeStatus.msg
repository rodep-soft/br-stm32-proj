# robot_interfaces/msg/BridgeStatus.msg
# Bridge and MCU Health Status Message

uint32 sequence         # Incrementing sequence number
uint32 uptime_ms        # MCU uptime in milliseconds
uint8  state            # 0: BOOTING, 1: STANDBY, 2: ACTIVE, 3: FAULT
uint8  can_state        # 0: OK, 1: WARNING, 2: BUS_OFF
uint32 can_rx_count     # Total CAN frames received
uint32 can_tx_count     # Total CAN frames transmitted
uint32 drop_count       # Total dropped frames
uint16 free_heap_bytes  # Available FreeRTOS heap memory in bytes
