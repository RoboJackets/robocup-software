#pragma once

#include <protobuf/LogFrame.pb.h>

#include <deque>
#include <filesystem>
#include <fstream>
#include <optional>
#include <time.hpp>

#include "Node.hpp"

// Keep the past thirty minutes of logs by default.
constexpr size_t kMaxLogFrames = 60 * 60 * 30;

struct Logs {
    /**
     * \brief A list of all log frames. This will contain at most
     * `kMaxLogFrames` frames.
     *
     * This should not be accessed from the Processor thread, other
     * than to add frames. The container may be accessed by the MainWindow
     * thread while the Context mutex is locked, and frames may be retained
     * and used even while the mutex is not locked (provided a shared_ptr
     * is kept).
     */
    std::deque<std::shared_ptr<Packet::LogFrame>> frames;

    enum class State { kNoFile, kWriting, kReading };

    /**
     * \brief The name of the log file, if it exists.
     */
    std::optional<std::string> filename;

    /**
     * \brief Whether we are recording (or viewing) logs.
     */
    State state = State::kNoFile;

    /**
     * \brief The start time of the entire system.
     */
    RJ::Time start_time;

    /**
     * \brief The log file size in bytes.
     */
    size_t size_bytes = 0;

    /**
     * The count of frames that existed before the given history
     * but were dropped.
     */
    size_t dropped_frames = 0;
};

// Forward-declare context, which needs to use Logs as above
class Context;

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
    std::optional<std::fstream> _log_file;

    Context* _context;
};