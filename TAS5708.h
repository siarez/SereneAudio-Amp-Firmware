//normal input indexing
static uint8_t TAS5708_conf[] PROGMEM = {0x2, 0x1B, 0x00 , 0x2, 0x06, 0x00 , 0x2, 0x04, 0x05 , 0x2, 0x11, 0x4C , 0x2, 0x12, 0x34 , 0x2, 0x13, 0x1C , 0x2, 0x14, 0x64 , 0x5, 0x20, 0x00, 0x89, 0x77, 0x72 , 0x5, 0x25, 0x01, 0x02, 0x13, 0x45 , 0x5, 0x50, 0x00, 0b00010000, 0x00, 0b10000010, 0x9, 0x3A, 0x00, 0x7F, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xFB, 0x9, 0x3B, 0x00, 0x7F, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xFB , 0x9, 0x3C, 0x00, 0x7F, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xFB , 0x5, 0x40, 0xFD, 0x2D, 0x26, 0x01 , 0x5, 0x41, 0x00, 0x00, 0x00, 0x00 , 0x5, 0x42, 0x00, 0x08, 0x42, 0x10 , 0x5, 0x46, 0x00, 0x00, 0x00, 0x00 , 0x2, 0x08, 0x30 , 0x2, 0x09, 0x30 , 0x2, 0x07, 0b11111111 , 0x9, 0x53, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9, 0x54, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2, 0x1A, 0x15, 0x2, 0x05, 0x00}; //No-DRC
//L-R switch input indexing for pebble
//static uint8_t TAS5708_conf[] PROGMEM = {0x2, 0x1B, 0x00 , 0x2, 0x06, 0x00 , 0x2, 0x04, 0x05 , 0x2, 0x11, 0x4C , 0x2, 0x12, 0x34 , 0x2, 0x13, 0x1C , 0x2, 0x14, 0x64 , 0x5, 0x20, 0x00, 0b10011000, 0x77, 0x72 , 0x5, 0x25, 0x01, 0x02, 0x13, 0x45 , 0x5, 0x50, 0x00, 0b00010000, 0x00, 0b10000010, 0x9, 0x3A, 0x00, 0x7F, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xFB, 0x9, 0x3B, 0x00, 0x7F, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xFB , 0x9, 0x3C, 0x00, 0x7F, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xFB , 0x5, 0x40, 0xFD, 0x2D, 0x26, 0x01 , 0x5, 0x41, 0x00, 0x00, 0x00, 0x00 , 0x5, 0x42, 0x00, 0x08, 0x42, 0x10 , 0x5, 0x46, 0x00, 0x00, 0x00, 0x00 , 0x2, 0x08, 0x30 , 0x2, 0x09, 0x30 , 0x2, 0x07, 0b11111111 , 0x9, 0x53, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9, 0x54, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2, 0x1A, 0x15, 0x2, 0x05, 0x00}; //No-DRC
//static uint8_t TAS5708_conf[] PROGMEM = {0x2, 0x1B, 0x00 , 0x2, 0x06, 0x00 , 0x2, 0x04, 0x05 , 0x2, 0x11, 0x4C , 0x2, 0x12, 0x34 , 0x2, 0x13, 0x1C , 0x2, 0x14, 0x64 , 0x5, 0x20, 0x00, 0x89, 0x77, 0x72 , 0x5, 0x25, 0x01, 0x02, 0x13, 0x45 , 0x5, 0x50, 0x00, 0b00010000, 0x00, 0b10000010, 0x9, 0x3A, 0x00, 0x00, 0x1B, 0x4B, 0x00, 0x7F, 0xE4, 0xB4, 0x9, 0x3B, 0x00, 0x00, 0x1B, 0x4B, 0x00, 0x7F, 0xE4, 0xB4 , 0x9, 0x3C, 0x00, 0x00, 0x0D, 0xA6, 0x00, 0x7F, 0xF2, 0x59 , 0x5, 0x40, 0xFC, 0x83, 0x10, 0xD4 , 0x5, 0x41, 0x0F, 0xC0, 0x00, 0x00 , 0x5, 0x42, 0x00, 0x08, 0x42, 0x10 , 0x5, 0x46, 0x00, 0x00, 0x00, 0x01 , 0x2, 0x08, 0x30 , 0x2, 0x09, 0x30 , 0x2, 0x07, 0b11111111 , 0x9, 0x53, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9, 0x54, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2, 0x1A, 0x15, 0x2, 0x05, 0x00};	//With DRC
//static uint8_t TAS5708_conf[] PROGMEM = {0x2, 0x1B, 0x00 , 0x2, 0x06, 0x00 , 0x2, 0x04, 0x05 , 0x2, 0x11, 0x4C , 0x2, 0x12, 0x34 , 0x2, 0x13, 0x1C , 0x2, 0x14, 0x64 , 0x5, 0x20, 0x00, 0x89, 0x77, 0x72 , 0x5, 0x25, 0x01, 0x02, 0x13, 0x45 , 0x5, 0x50, 0x00, 0b00010000, 0x00, 0b10000010, 0x9, 0x3A, 0x00, 0x00, 0x1B, 0x4B, 0x00, 0x7F, 0xE4, 0xB3, 0x9, 0x3B, 0x00, 0x00, 0x0D, 0xA6, 0x00, 0x7F, 0xF2, 0x57 , 0x9, 0x3C, 0x00, 0x00, 0x06, 0xD3, 0x00, 0x7F, 0xF9, 0x2B , 0x5, 0x40, 0xFC, 0x83, 0x10, 0xD4 , 0x5, 0x41, 0x0F, 0xC0, 0x00, 0x00 , 0x5, 0x42, 0x00, 0x08, 0x42, 0x10 , 0x5, 0x46, 0x00, 0x00, 0x00, 0x01 , 0x2, 0x08, 0x30 , 0x2, 0x09, 0x30 , 0x2, 0x07, 0b11111111 , 0x9, 0x53, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9, 0x54, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2, 0x1A, 0x15, 0x2, 0x05, 0x00};	//With DRC
//static uint8_t TAS5708_conf[] PROGMEM = {0x2, 0x1B, 0x00 , 0x2, 0x06, 0x00 , 0x2, 0x04, 0x05 , 0x2, 0x11, 0x4C , 0x2, 0x12, 0x34 , 0x2, 0x13, 0x1C , 0x2, 0x14, 0x64 , 0x5, 0x20, 0x00, 0x89, 0x77, 0x72 , 0x5, 0x25, 0x01, 0x02, 0x13, 0x45 , 0x5, 0x50, 0x00, 0b00010000, 0x00, 0b10000010, 0x9, 0x3A, 0x00, 0x00, 0x94, 0x42, 0x00, 0x7F, 0x6B, 0xB9, 0x9, 0x3B, 0x00, 0x00, 0x4A, 0x36, 0x00, 0x7F, 0xB5, 0xC5 , 0x9, 0x3C, 0x00, 0x00, 0x4A, 0x36, 0x00, 0x7F, 0xB5, 0xC5, 0x5, 0x40, 0xFB, 0xEE, 0x3E, 0x4D , 0x5, 0x41, 0x0F, 0x99, 0x99, 0x92 , 0x5, 0x42, 0x00, 0x08, 0x42, 0x10 , 0x5, 0x46, 0x00, 0x00, 0x00, 0x01 , 0x2, 0x08, 0x30 , 0x2, 0x09, 0x30 , 0x2, 0x07, 0b11111111 , 0x9, 0x53, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9, 0x54, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2, 0x1A, 0x15, 0x2, 0x05, 0x00};	//With DRC no-clipping