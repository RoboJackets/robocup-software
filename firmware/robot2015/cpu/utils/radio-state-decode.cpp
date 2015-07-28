#include "radio-state-decode.hpp"


std::string decode_marcstate(uint8_t b)
{
    std::string state;

    switch (b & 0x1F) {
    case 0x00:
        state = "SLEEP";
        break;

    case 0x01:
        state = "IDLE";
        break;

    case 0x02:
        state = "XOFF";
        break;

    case 0x03:
        state = "BIAS_SETTLE_MC";
        break;

    case 0x04:
        state = "REG_SETTLE_MC";
        break;

    case 0x05:
        state = "MANCAL";
        break;

    case 0x06:
        state = "BIAS_SETTLE";
        break;

    case 0x07:
        state = "REG_SETTLE";
        break;

    case 0x08:
        state = "STARTCAL";
        break;

    case 0x09:
        state = "BWBOOST";
        break;

    case 0x0A:
        state = "FS_LOCK";
        break;

    case 0x0B:
        state = "IFADCON";
        break;

    case 0x0C:
        state = "ENDCAL";
        break;

    case 0x0D:
        state = "RX";
        break;

    case 0x0E:
        state = "RX_END";
        break;

    case 0x0F:
        state = "RXDCM";
        break;

    case 0x10:
        state = "TXRX_SWITCH";
        break;

    case 0x11:
        state = "RX_FIFO_ERR";
        break;

    case 0x12:
        state = "FSTXON";
        break;

    case 0x13:
        state = "TX";
        break;

    case 0x14:
        state = "TX_END";
        break;

    case 0x15:
        state = "RXTX_SWITCH";
        break;

    case 0x16:
        state = "TX_FIFO_ERR";
        break;

    case 0x17:
        state = "IFADCON_TXRX";
        break;

    default:
        state = "ERROR DECODING STATE";
        break;
    }

    return state;
}
