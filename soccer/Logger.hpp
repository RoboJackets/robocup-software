#pragma once

#include <filesystem>
#include <fstream>
#include <optional>

#include "Context.hpp"
#include "Node.hpp"

/**
 * \brief Populates the log frame in Context, and writes it to a file.
 *
 * When viewing logs, reads logs from a file and populates the log frame.
 */
class Logger : public Node {
public:
    Logger(Context* context) : _context(context) {}

    /**
     * \brief Open the given file for reading
     */
    void read(const std::string& filename);

    /**
     * \brief Open the given file for writing.
     */
    void write(const std::string& filename);

    /**
     * \brief Close the current log file.
     */
    void close();

    /**
     * \brief Flush the file to disk.
     */

    void start() override;
    void run() override;
    void stop() override;

private:
    void createLogFrame();

    std::optional<std::fstream> _log_file;

    enum class State {
        kNoFile,
        kWriting,
        kReading
    };

    State _state = State::kNoFile;
    Context* _context;
};