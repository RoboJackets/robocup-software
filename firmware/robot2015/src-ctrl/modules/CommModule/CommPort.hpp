#pragma once

#include "mbed.h"
#include "rtos.h"
#include "robot-types.hpp"
#include "ThreadHelper.hpp"
#include "MailHelper.hpp"
#include "console.hpp"

#include <algorithm>
#include <vector>
#include <functional>

template <class T>
class CommPort
{
  public:
    // Constructor
    CommPort()
        : is_open(false),
          rx_callback(nullptr),
          tx_callback(nullptr),
          rx_packets(0),
          tx_packets(0)
    {
        Nbr(0);
        resetPacketCount();
    };
    CommPort(uint8_t number, std::function<T> rxC = nullptr, std::function<T> txC = nullptr)
        : is_open(false),
          rx_callback(rxC),
          tx_callback(txC)
    {
        Nbr(number);
        resetPacketCount();
    };


    // Copy constructor
    CommPort(const CommPort<T>& p)
        : is_open(p.is_open),
          rx_callback(p.rx_callback),
          tx_callback(p.tx_callback),
          rx_packets(p.rx_packets),
          tx_packets(p.tx_packets)
    {
        Nbr(p.Nbr());
    };


    // Compare between 2 CommPort objects
    bool operator==(const CommPort& p) const
    {
        return (this->Nbr() == p.Nbr());
    }
    // Compare between a CommPort object and a port number
    bool operator==(unsigned int portNbr) const
    {
        return (this->Nbr() == portNbr ? true : false);
    }


    // Overload the less than operator for sorting/finding ports using iterators
    bool operator< (const CommPort& p) const
    {
        return (this->Nbr() < p.Nbr() ? true : false);
    }


    uint8_t Nbr(void) const
    {
        return this->nbr;
    }

    void Nbr(uint8_t _nbr)
    {
        if (_nbr == 0)
            is_valid = false;
        else
            is_valid = true;

        nbr = _nbr;
    }


    // Open a port or check if a port is capable of providing communication.
    bool Open(void)
    {
        if (isReady()) {
            this->is_open = true;

            return isOpen();
        } else {
            return false;
        }
    }
    void Close(void)
    {
        is_open = false;
    }


    // Check if the port has already been opened.
    bool isOpen(void) const
    {
        return this->is_open;
    }
    bool isClosed(void) const
    {
        return !isOpen();
    }


    // Set functions for each RX/TX callback.
    const CommPort<T>& RXCallback(const std::function<T>& func)
    {
        rx_callback = func;
        return *this;
    }
    const CommPort<T>& TXCallback(const std::function<T>& func)
    {
        tx_callback = func;
        return *this;
    }


    // Methods that return a reference to the TX/RX callback function pointers
    std::function<T>& RXCallback(void)
    {
        return rx_callback;
    }
    std::function<T>& TXCallback(void)
    {
        return tx_callback;
    }


    // Check if an RX/TX callback function has been set for the port.
    bool hasTXCallback(void) const
    {
        return ((tx_callback == nullptr) ? false : true);
    }
    bool hasRXCallback(void) const
    {
        return ((rx_callback == nullptr) ? false : true);
    }


    // Check if the port object is
    bool Exists(void) const
    {
        if (hasRXCallback() || hasTXCallback()) {
            return true;
        } else {
            return this->is_valid;
        }
    }


    // Returns a reference to the TX/RX packet count for modifying
    unsigned int& TXPackets(void)
    {
        return tx_packets;
    }
    unsigned int& RXPackets(void)
    {
        return rx_packets;
    }
    const unsigned int& TXPackets(void) const
    {
        return this->tx_packets;
    }
    const unsigned int& RXPackets(void) const
    {
        return this->rx_packets;
    }


    // Standard display function for a CommPort
    void PrintPort(void)
    {
        printf("%2u\t\t%u\t%u\t%s\t\t%s\t\t%s\r\n",
               Nbr(),
               RXPackets(),
               TXPackets(),
               hasRXCallback() ? "YES" : "NO",
               hasTXCallback() ? "YES" : "NO",
               isOpen() ? "OPEN" : "CLOSED"
              );

        Console::Flush();
    }


  protected:
    // Returns the current packet counts to zero
    void resetPacketCount(void)
    {
        RXPackets() = 0;
        TXPackets() = 0;
    }


    // Returns true if the port can provide an RX callback routine
    bool isReady(void) const
    {
        return (is_open ? true : hasRXCallback());
    }


  private:
    // The number assigned to the port
    uint8_t             nbr;

    // If the port is greater than 0, it is a valid port object
    bool                is_valid;

    // If the port is open, it will also be valid
    bool                is_open;

    // Where each upstream & downstream packet count is stored
    unsigned int        rx_packets;
    unsigned int        tx_packets;

    // the class members that hold the function pointers
    std::function<T>    rx_callback;
    std::function<T>    tx_callback;
};





// Function for defining how to sort a std::vector of CommPort objects
template<class T>
bool PortCompare(const CommPort<T>& a, const CommPort<T>& b)
{
    return a < b;
}




// Class to manage the available/unavailable ports
template <class T>
class CommPorts : CommPort<T>
{
  private:
    typename std::vector<CommPort<T>> ports;
    typename std::vector<CommPort<T>>::iterator pIt;

    const CommPorts<T>& sort(void)
    {
        std::sort(ports.begin(), ports.end(), PortCompare<T>);
        return *this;
    }

    const CommPorts<T>& add(const CommPort<T>& p)
    {
        pIt = find(ports.begin(), ports.end(), p);

        if (pIt == ports.end()) {
            ports.push_back(p);
        }

        return this->sort();
    }


  public:
    CommPorts<T> operator+= (const CommPort<T>& p)
    {
        return this->add(p);
    }

    CommPort<T>& operator[](const int portNbr)
    {
        pIt = find(ports.begin(), ports.end(), (unsigned int)portNbr);

        if (pIt != ports.end()) {
            return *& (*pIt);
        }

        CommPort<T> tmpPort(0);
        return tmpPort;
    }

    const int count(void)
    {
        return ports.size();
    }

    const bool empty(void)
    {
        return ports.empty();
    }


    // Get the total count (across all ports) of each RX/TX packet count
    const unsigned int allRXPackets(void) const
    {
        unsigned int pcks = 0;

        for (auto it = ports.begin(); it != ports.end(); ++it) {
            pcks += it->RXPackets();
        }

        return pcks;
    }
    const unsigned int allTXPackets(void) const
    {
        unsigned int pcks = 0;

        for (auto it = ports.begin(); it != ports.end(); ++it) {
            pcks += it->TXPackets();
        }

        return pcks;
    }

    void PrintPorts(void)
    {
        if (empty() == false) {

            PrintHeader();

            for (auto it = ports.begin(); it != ports.end(); ++it) {
                it->PrintPort();
            }
        }
    }

    void PrintHeader(void)
    {
        printf("PORT\t\tIN\tOUT\tRX CBCK\t\tTX CBCK\t\tSTATE\r\n");
        Console::Flush();
    }

    void PrintFooter(void)
    {
        printf("==========================\r\n"
               "Total:\t\t%u\t%u\r\n\r\n",
               allRXPackets(),
               allTXPackets()
              );
        Console::Flush();
    }
};
