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
typedef uint8_t packet_data_t;
}

/**
 * @brief [brief description]
 * @details [long description]
 * @return [description]
 */
class layer_map {
public:
    typedef packet_data_t data_t;

protected:
    typedef std::vector<data_t> datav_t;
    typedef datav_t::iterator datav_it_t;
    std::vector<data_t> d;

public:
    layer_map(size_t resv) { d.reserve(resv); }

    datav_it_t pack() {
        // make sure all files are in contiguous memory,
        // then return iterator to beginning
        return d.begin();
    }

    data_t* data() { return d.data(); }

    void show_data() {
        for (auto const& i : d) printf("%02X ", i);
    }

    const size_t size() const { return static_cast<const int>(d.size()); }
};

/**
 * @brief [brief description]
 * @details [long description]
 *
 * @param p [description]
 * @return [description]
 */
class header_data : public layer_map {
public:
    enum type { control, tuning, ota, misc };

    header_data() : layer_map(3), t(control), address(0), port_fields(0){};

    datav_it_t pack(size_t payload_size, bool headless = false) {
        if (d.size()) return d.begin();
        // payload size + number of bytes in header is top byte
        // since that's required for the cc1101/cc1201 with
        // variable packet sizes
        d.push_back(payload_size + (headless ? 0 : 2));
        if (headless == false) {
            d.push_back(address);
            d.push_back(port_fields);
        }
        return d.begin();
    }

    type t;
    data_t address;

    friend class payload_data;

    // common header byte - union so it's easier to put into vector
    struct {
        union {
            struct {
                data_t port_fields;
            } __attribute__((packed));
            struct {
#if __BIG_ENDIAN
                data_t sfs : 1, ack : 1, subclass : 2, port : 4;
#else
                data_t port : 4, subclass : 2, ack : 1, sfs : 1;
#endif
            } __attribute__((packed));
        };
    };
};

/**
 * @brief [brief description]
 * @details [long description]
 *
 * @param p [description]
 * @return [description]
 */
class payload_data : public layer_map {
public:
    payload_data() : layer_map(MAX_DATA_SZ){};

    datav_it_t pack() { return d.begin(); }

    datav_it_t add_header(const header_data& h) {
        d.insert(d.begin(), h.d.begin(), h.d.end());
        return d.begin();
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
    bool _packed;

    packet(){};
    packet(const std::string& s) { payload.fill(s); }

    template <class T>
    packet(const std::vector<T>& v) {
        payload.fill(v);
    }

    packet_data_t* packed() {
        if (_packed == false) pack();

        return payload.data();
    }

    size_t size() {
        if (_packed == true)
            return payload.size();
        else
            return payload.size() + header.size();
    }

    const int port() const { return static_cast<const int>(header.port); }
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

private:
    void pack() {
        payload.pack();
        // pack the header, but do a "headless" pack
        header.pack(payload.size(), true);
        payload.add_header(header);
        _packed = true;
    }
};
}
