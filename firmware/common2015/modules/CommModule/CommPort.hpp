#pragma once

#include <mbed.h>
#include <rtos.h>

#include <algorithm>
#include <vector>
#include <functional>
#include <stdexcept>

template <class T>
class CommPort
{
  public:
    // Constructor
    CommPort()
        : is_open(false),
          rx_packets(0),
          tx_packets(0),
          rx_callback(nullptr),
          tx_callback(nullptr)
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
          rx_packets(p.rx_packets),
          tx_packets(p.tx_packets),
          rx_callback(p.rx_callback),
          tx_callback(p.tx_callback)
    {
        Nbr(p.Nbr());
    };


    // Compare between 2 CommPort objects
    bool operator==(const CommPort& p) const
    {
        return this->Nbr() == p.Nbr();
    }


    // Overload the less than operator for sorting/finding ports using iterators
    bool operator<(const CommPort& p) const
    {
        return this->Nbr() < p.Nbr();
    }


    uint8_t Nbr() const
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
    bool Open()
    {
        if (isReady()) {
            this->is_open = true;

            return isOpen();
        } else {
            return false;
        }
    }

    void Close()
    {
        is_open = false;
    }


    // Check if the port has already been opened.
    bool isOpen() const
    {
        return this->is_open;
    }
    bool isClosed() const
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
    std::function<T>& RXCallback()
    {
        return rx_callback;
    }
    std::function<T>& TXCallback()
    {
        return tx_callback;
    }


    // Check if an RX/TX callback function has been set for the port.
    bool hasTXCallback() const
    {
        return tx_callback != nullptr;
    }
    bool hasRXCallback() const
    {
        return rx_callback != nullptr;
    }


    // Check if the port object is
    bool Exists() const
    {
        if (hasRXCallback() || hasTXCallback()) {
            return true;
        } else {
            return this->is_valid;
        }
    }


    // Returns a reference to the TX/RX packet count for modifying
    unsigned int TXPackets() const
    {
        return tx_packets;
    }
    unsigned int RXPackets() const
    {
        return rx_packets;
    }
    unsigned int& TXPackets()
    {
        return tx_packets;
    }
    unsigned int& RXPackets()
    {
        return rx_packets;
    }


    // Standard display function for a CommPort
    void PrintPort() const
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
    void resetPacketCount()
    {
        RXPackets() = 0;
        TXPackets() = 0;
    }


    // Returns true if the port can provide an RX callback routine
    bool isReady() const
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

    const CommPorts<T>& sort()
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

        throw std::out_of_range("Unable to find port with given number");
    }

    int count() const
    {
        return ports.size();
    }

    int count_open() const
    {
        int count = 0;

        for (auto it = ports.begin(); it != ports.end(); ++it) {
            if (it->isOpen())
                count++;
        }

        return count;
    }

    bool empty() const
    {
        return ports.empty();
    }


    // Get the total count (across all ports) of each RX/TX packet count
    unsigned int allRXPackets() const
    {
        unsigned int pcks = 0;

        for (auto it = ports.begin(); it != ports.end(); ++it) {
            pcks += static_cast<unsigned int>(it->RXPackets());
        }

        return pcks;
    }
    unsigned int allTXPackets() const
    {
        unsigned int pcks = 0;

        for (auto it = ports.begin(); it != ports.end(); ++it) {
            pcks += static_cast<unsigned int>(it->TXPackets());
        }

        return pcks;
    }

    void PrintPorts()
    {
        if (empty() == false) {

            PrintHeader();

            for (auto it = ports.begin(); it != ports.end(); ++it) {
                it->PrintPort();
            }
        }
    }

    void PrintHeader()
    {
        printf("PORT\t\tIN\tOUT\tRX CBCK\t\tTX CBCK\t\tSTATE\r\n");
        Console::Flush();
    }

    void PrintFooter()
    {
        printf("==========================\r\n"
               "Total:\t\t%u\t%u\r\n\r\n",
               allRXPackets(),
               allTXPackets()
              );
        Console::Flush();
    }
};
