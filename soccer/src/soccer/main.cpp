#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QString>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fcntl.h>
#include <rj_common/qt_utils.hpp>
#include <unistd.h>

#include "ui/main_window.hpp"
#include "ui/style_sheet_manager.hpp"

using namespace std;

//  we use this to catch Ctrl+C and kill the program
void signal_handler(int signum) { exit(signum); }

void usage(const char* prog) {
    std::cerr << "usage: " << prog << " [options...]\n";
    std::cerr << "\t-y:           run as the yellow team\n";
    std::cerr << "\t-b:           run as the blue team\n";
    std::cerr << "\t-c <file>:    specify the configuration file\n";
    std::cerr << "\t-s <seed>:    set random seed (hexadecimal)\n";
    std::cerr << "\t-pbk <file>:  playbook file name as contained in "
                 "'soccer/gameplay/playbooks/'\n";
    std::cerr << "\t-vlog <file>: view <file> instead of launching normally\n";
    std::cerr << "\t-ng:          no goalie\n";
    std::cerr << "\t-sim:         use simulator\n";
    std::cerr << "\t-nolog:       don't write log files\n";
    std::cerr << "\t-noref:       don't use external referee commands\n";
    std::cerr << "\t-defend:      specify half of field to defend (plus or minus)\n";
    std::exit(0);
}

int main(int argc, char* argv[]) {
    printf("Starting Soccer...\n");

    //  register our signal handler
    signal(SIGINT, signal_handler);

    // Seed the large random number generator
    long int seed = 0;
    int fd = open("/dev/random", O_RDONLY);
    if (fd >= 0) {
        if (read(fd, &seed, sizeof(seed)) != sizeof(seed)) {
            fprintf(stderr, "Can't read /dev/random, using zero seed: %m\n");
        }
        close(fd);
    } else {
        fprintf(stderr, "Can't open /dev/random, using zero seed: %m\n");
    }

    QApplication app(argc, argv);

    bool blue_team = true;
    QString cfg_file;
    vector<const char*> play_dirs;
    bool sim = false;
    bool log = true;
    QString radio_freq;
    string playbook_file;
    bool noref = false;
    bool defend_plus = false;
    string read_log_file;

    for (int i = 1; i < argc; ++i) {
        const char* var = argv[i];

        if (strcmp(var, "") == 0) {
            continue;
        }

        if (strcmp(var, "--help") == 0) {
            usage(argv[0]);
        } else if (strcmp(var, "-y") == 0) {
            blue_team = false;
        } else if (strcmp(var, "-b") == 0) {
            blue_team = true;
        } else if (strcmp(var, "-sim") == 0) {
            sim = true;
        } else if (strcmp(var, "-nolog") == 0) {
            log = false;
        } else if (strcmp(var, "-freq") == 0) {
            if (i + 1 >= argc) {
                printf("No radio frequency specified after -freq\n");
                usage(argv[0]);
            }

            i++;
            radio_freq = argv[i];
        } else if (strcmp(var, "-c") == 0) {
            if (i + 1 >= argc) {
                printf("no config file specified after -c\n");
                usage(argv[0]);
            }

            i++;
            cfg_file = argv[i];
        } else if (strcmp(var, "-s") == 0) {
            if (i + 1 >= argc) {
                printf("no seed specified after -s\n");
                usage(argv[0]);
            }

            i++;
            seed = strtol(argv[i], nullptr, 16);
        } else if (strcmp(var, "-pbk") == 0) {
            if (i + 1 >= argc) {
                printf("no playbook file specified after -pbk\n");
                usage(argv[0]);
            }

            playbook_file = argv[++i];
        } else if (strcmp(var, "-vlog") == 0) {
            if (i + 1 >= argc) {
                printf("no log file specified after -vlog\n");
                usage(argv[0]);
            }

            read_log_file = argv[++i];
        } else if (strcmp(var, "-noref") == 0) {
            noref = true;
        } else if (strcmp(var, "-defend") == 0) {
            if (i + 1 >= argc) {
                printf("Field half not specified after -defend\n");
                usage(argv[0]);
            }
            i++;
            if (strcmp(argv[i], "plus") == 0) {
                defend_plus = true;
            } else if (strcmp(argv[i], "minus") != 0) {
                printf("Invalid option for defend_x\n");
                usage(argv[0]);
            }
        } else if (strcmp(var, "--ros-args") == 0) {
            // ROS args follow this one, we're done parsing
            break;
        } else {
            printf("Not a valid flag: %s\n", argv[i]);
            usage(argv[0]);
        }
    }

    printf("Running on %s\n", sim ? "simulation" : "real hardware\n");

    printf("seed %016lx\n", seed);
    srand48(seed);

    // Default config file name
    if (cfg_file.isNull()) {
        const auto* filename = sim ? "soccer-sim.cfg" : "soccer-real.cfg";
        const auto share_dir = ament_index_cpp::get_package_share_directory("rj_robocup");
        const std::string config_path = share_dir + "/config/" + filename;

        cfg_file = QString::fromStdString(config_path);
    }

    // ROS2 init
    rclcpp::init(argc, argv);

    auto processor = std::make_unique<Processor>(sim, blue_team, read_log_file);

    Context* context = processor->context();
    context->game_settings.simulation = sim;
    context->game_settings.request_blue_team = blue_team;
    context->game_settings.defend_plus_x = defend_plus;
    context->game_settings.request_goalie_id = 0;

    // If we're reading a log file, we should start off paused.
    context->game_settings.paused = !read_log_file.empty();

    auto win = std::make_unique<MainWindow>(processor.get(), !noref);
    win->initialize();

    win->setUseRefChecked(!noref);

    if (!application_run_directory().exists("./logs")) {
        cerr << "No ./run/logs/ directory - not writing log file" << endl;
    } else if (!log) {
        cerr << "Not writing log file" << endl;
    } else if (read_log_file.empty()) {
        QString log_file = application_run_directory().filePath("./logs/") +
                           QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss.log");
        if (!processor->open_log(log_file)) {
            printf("Failed to open %s: %m\n", (const char*)log_file.toLatin1());
        }
    }

    win->logFileChanged();

    std::thread processor_thread(&Processor::run, processor.get());

    while (!processor->is_initialized()) {  // Wait until processor finishes initializing
    }

    // Sets the initial stylesheet for the application
    // based on the environment variable "SOCCER_THEME"
    if (getenv("SOCCER_THEME") != nullptr) {
        StyleSheetManager::changeStyleSheet(win.get(), QString(getenv("SOCCER_THEME")));
    }

    win->show();

    int ret = QApplication::exec();
    processor->stop();
    processor_thread.join();

    return ret;
}
