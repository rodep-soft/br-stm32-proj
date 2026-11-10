"""Guard the CAN-FD 0x500 wire contract and essential CubeMX2-generated settings."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
dbc = (root / "bno085_can.dbc").read_text()
can_h = (root / "protocols/bno085_can.h").read_text()
can_c = (root / "protocols/bno085_can.c").read_text()

assert "BO_ 1280 BNO085_Data: 20 STM32" in dbc
assert "#define BNO085_CAN_BNO085_DATA_FRAME_ID (0x500u)" in can_h
assert "#define BNO085_CAN_BNO085_DATA_PAYLOAD_LENGTH (18u)" in can_h
assert "#define BNO085_CAN_BNO085_DATA_LENGTH (20u)" in can_h
assert "int bno085_can_bno085_data_pack(" in can_c

generated = root / "c542rc-bno085_cmake/generated/hal"
can = (generated / "mx_fdcan1.c").read_text()
spi = (generated / "mx_spi1.c").read_text()
rcc = (generated / "mx_rcc.c").read_text()
gpio = (generated / "mx_gpio_default.h").read_text()
assert "HAL_RCC_PSI_REF_24MHZ" in rcc and "HAL_RCC_PSI_OUT_144MHZ" in rcc
assert "HAL_RCC_PCLK_PRESCALER1" in rcc
assert "HAL_RCC_FDCAN_CLK_SRC_PCLK1" in can
assert "HAL_FDCAN_FRAME_FORMAT_FD_NO_BRS" in can
assert "HAL_FDCAN_MODE_NORMAL" in can
assert "HAL_FDCAN_AUTO_RETRANSMISSION_DISABLE" in can
ioc = (root / "c542rc-bno085.ioc2").read_text()
assert '"frame_format": "FD_NO_BRS"' in ioc
assert '"auto_retransmission": false' in ioc
assert "HAL_FDCAN_CLOCK_DIV_1" in can
def timing(name):
    return int(re.search(rf"\.{name}\s*=\s*(\d+)", can).group(1))
assert 144_000_000 / timing("nominal_prescaler") / (
    1 + timing("nominal_time_seg1") + timing("nominal_time_seg2")) == 1_000_000
assert "PB8_PIN | PB9_PIN" in can
for token in ("HAL_SPI_CLOCK_POLARITY_HIGH", "HAL_SPI_CLOCK_PHASE_2_EDGE",
              "HAL_SPI_BAUD_RATE_PRESCALER_64", "HAL_SPI_NSS_PIN_MGMT_INTERNAL",
              "PA7_PIN", "PA6_PIN", "PA5_PIN", "HAL_GPIO_MODE_ALTERNATE", "HAL_GPIO_AF_5"):
    assert token in spi, token
assert "PB7_PIN" not in spi
linker = root / "c542rc-bno085_cmake/user_modifiable/Device/STM32C542RCT6/stm32c542xc_flash.ld"
assert "STACK_SIZE = 0x1000;" in linker.read_text(), "SH-2/DMA requires 4 KiB stack"
for pin, state in (("PA1", "SET"), ("PA4", "SET"), ("PB0", "SET"), ("PE2", "RESET")):
    assert re.search(rf"#define {pin}_INIT_STATE\s+HAL_GPIO_PIN_{state}\b", gpio), pin
print("PASS: CAN FD 0x500 (18B/20B) DBC/codec; FDCAN FD_NO_BRS 1Mbps; SPI Mode 3; GPIO startup levels")
