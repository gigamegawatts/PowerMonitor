int8_t gm_read8(uint16_t devAddr, uint8_t addr, uint8_t *result);
int8_t gm_read16(uint16_t devAddr, uint8_t addr, uint16_t *result);
int8_t gm_read16rev(uint16_t devAddr, uint8_t addr, uint16_t *result);
int8_t gm_write8(uint16_t devAddr, uint8_t addr, uint8_t data);
int8_t gm_readmultiple(uint16_t devAddr, uint8_t regAddr, uint8_t *result, uint8_t bytesToRead);
int8_t gm_writemultiple(uint16_t devAddr, uint8_t regAddr, uint8_t *val, uint8_t bytesToWrite);