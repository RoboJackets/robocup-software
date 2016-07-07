#pragma once

// Call this when a new radio packet has been received.
// It will handle all OTA operations if the packet is an OTA_START packet.
// Returns nonzero if the packet was used by OTA mode.
int ota_start(void);
