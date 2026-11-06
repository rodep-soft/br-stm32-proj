"""Guard the legacy wire contract and essential CubeMX2-generated settings."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
legacy = root.parents[1] / "sandbox" / "stm32-f303k8"
for relative in ("bno085_can.dbc", "protocols/bno085_can.c", "protocols/bno085_can.h"):
    assert (root / relative).read_bytes() == (legacy / relative).read_bytes(), relative

generated = root / "c542rc-bno085_cmake/generated/hal"
can = (generated / "mx_fdcan1.c").read_text()
spi = (generated / "mx_spi1.c").read_text()
rcc = (generated / "mx_rcc.c").read_text()
gpio = (generated / "mx_gpio_default.h").read_text()
assert "HAL_RCC_PSI_REF_24MHZ" in rcc and "HAL_RCC_PSI_OUT_144MHZ" in rcc
assert "HAL_RCC_PCLK_PRESCALER1" in rcc
assert "HAL_RCC_FDCAN_CLK_SRC_PCLK1" in can
assert "HAL_FDCAN_FRAME_FORMAT_CLASSIC_CAN" in can
assert "HAL_FDCAN_MODE_NORMAL" in can
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
print("PASS: unchanged legacy DBC/codec; Classic CAN 1Mbps; SPI Mode 3; GPIO startup levels")
