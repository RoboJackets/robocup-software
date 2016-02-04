#pragma once

#include <cstdint>
#include <vector>
#include <string>

#if __BIG_ENDIAN
#define RTP_HEADER(port, subclass, ack, sfs)                                 \
    (((port & 0x0F) << 4) | ((subclass & 0x03) << 2) | ((ack & 0x01) << 1) | \
     (sfs & 0x01))
#else
#define RTP_HEADER(port, subclass, ack, sfs)                                \
    (((sfs & 0x01) << 7) | ((ack & 0x01) << 6) | ((subclass & 0x03) << 4) | \
     (port & 0x0F))
#endif

#define BASE_STATION_ADDR (1)
#define LOOPBACK_ADDR (127)

namespace rtp {

///
static const unsigned int MAX_DATA_SZ = 120;

/**
 * @brief      { port enumerations for different communication protocols }
 */
enum port {
    SINK = 0x00,
    LINK = 0x01,
    CONTROL = 0x02,
    SETPOINT = 0x03,
    GSTROBE = 0x04,
    DISCOVER = 0x05,
    LOGGER = 0x06,
    TCP = 0x07,
    LEGACY = 0x0E
};

namespace {
// type of data field used in all packet classes
// - must be 8 bits, (typedef used for eaiser compiler
// type error debugging)
typedef uint8_t data_t;
}


class header_data {
public:
    enum type { control, tuning, ota, misc };

    header_data() : t(control), address(0), port_fields(0){};

    size_t size() const { return 2; }

    type t;
    data_t address;

    friend class payload_data;

    // common header byte - union so it's easier to put into vector
    data_t port_fields;
    struct {
#if __BIG_ENDIAN
        data_t sfs : 1, ack : 1, subclass : 2, port : 4;
#else
        data_t port : 4, subclass : 2, ack : 1, sfs : 1;
#endif
    } __attribute__((packed));
};



class payload_data {
public:
    payload_data() {};

    std::vector<data_t> d;

    size_t size() const {
        return d.size();
    }

    template <class T>
    void fill(const std::vector<T>& v) {
        d = v;
    }
    void fill(const std::string& str) {
        for (const char& c : str) d.push_back(c);
        d.push_back('\0');
    }
};

/**
 * @brief      { Real-Time packet definition }
 */
class packet {
public:
    rtp::header_data header;
    rtp::payload_data payload;

    packet(){};
    packet(const std::string& s) { payload.fill(s); }

    template <class T>
    packet(const std::vector<T>& v) {
        payload.fill(v);
    }

    size_t size(bool includeHeader = true) {
        if (includeHeader)
            return payload.size() + header.size();
        else
            return payload.size();
    }

    int port() const { return static_cast<int>(header.port); }
    template <class T>
    void port(T p) {
        header.port = static_cast<unsigned int>(p);
    }

    int subclass() { return header.subclass; }
    template <class T>
    void subclass(T c) {
        header.subclass = static_cast<unsigned int>(c);
    }

    bool ack() { return header.ack; }
    void ack(bool b) { header.ack = b; }

    bool sfs() { return header.sfs; }
    void sfs(bool b) { header.sfs = b; }

    int address() { return header.address; }
    void address(int a) { header.address = static_cast<unsigned int>(a); }

    template <class T>
    void recv(const std::vector<T>& v) {
        payload.fill(v);
        // sort out header
    }

    void pack(std::vector<data_t> *buffer, bool includeHeader = false) const {
        // first byte is total size (excluding the size byte)
        const uint8_t total_size = payload.size() + (includeHeader ? header.size() : 0);
        buffer->reserve(total_size + 1);
        buffer->push_back(total_size);

        // header data
        if (includeHeader) {
            buffer->push_back(header.address);
            buffer->push_back(header.port_fields);
        }

        // payload
        buffer->insert(buffer->end(), payload.d.begin(), payload.d.end());
    }
};
}
