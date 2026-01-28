#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <ctype.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <time.h>
#include <dirent.h>
#include <limits.h>
#include <stdarg.h>

/* speedy.c
written with help from ChatGPT

You may use this code freely for noncommercial purposes.  Please give me some credit -- W6EFI
*/

// #define DEBUG 1  // Set to 0 to disable debug output.  update: made it a runtime option
#define KNOB_CLOCKWISE 190
#define KNOB_COUNTERCLOCKWISE 189
#define KNOB_CLICK 185
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE B38400
#define TIMEOUT_SEC 2
#define TARGET_DEVICE_NAME "nullbits SCRAMBLE"  // Adjust this based on your QMK device

static int debug_enabled = 0;
static volatile int keep_running = 1;

// Debug logging
void log_debug(const char *fmt, ...) {
    // if (!debug_enabled) return;  // not the right place for this

    va_list args;
    va_start(args, fmt);

    char timestamp[64];
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);

    fprintf(stderr, "[%s] DEBUG: ", timestamp);
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");

    va_end(args);
}

// Signal handler
void sigint_handler(int sig) {
    (void)sig;
    keep_running = 0;
}

// Trim leading and trailing whitespace
void trim_whitespace(char *str) {
    while (isspace((unsigned char)*str)) str++;
    if (*str == 0) return;
    char *end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    *(end + 1) = '\0';
}

// Find the input device by name
char *find_input_device(char *resolved_path, size_t path_len) {
    DIR *dir = opendir("/dev/input");
    if (!dir) {
        perror("opendir /dev/input");
        return NULL;
    }

    struct dirent *entry;
    char full_path[PATH_MAX];
    char name[256];
    int fd;

    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "event", 5) == 0) {
            snprintf(full_path, sizeof(full_path), "/dev/input/%s", entry->d_name);
            log_debug("Checking input device: %s", full_path);

            fd = open(full_path, O_RDONLY);
            if (fd < 0) continue;

            if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0) {
                name[sizeof(name) - 1] = '\0';
                trim_whitespace(name);
                log_debug("Found device name (trimmed): '%s'", name);

                if (strcmp(name, TARGET_DEVICE_NAME) == 0) {
                    log_debug("Matched device exactly: %s", full_path);
                    strncpy(resolved_path, full_path, path_len);
                    close(fd);
                    closedir(dir);
                    return resolved_path;
                }
            }

            close(fd);
        }
    }

    closedir(dir);
    return NULL;
}

// Open and configure serial port
int open_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error getting terminal attributes");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = TIMEOUT_SEC * 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error setting terminal attributes");
        close(fd);
        return -1;
    }

    return fd;
}

// Extract first integer from string
int extract_integer(const char *str, int *out_int) {
    int i = 0;
    while (str[i]) {
        if (isdigit(str[i]) || (str[i] == '-' && isdigit(str[i + 1]))) {
            *out_int = atoi(&str[i]);
            return 1;
        }
        i++;
    }
    return 0;
}

// Send formatted speed to serial port in "KS%03d;" format
int send_speed(int serial_fd, int speed) {
    char buf[8];
    snprintf(buf, sizeof(buf), "KS%03d;", speed);
    log_debug("Sending speed command: %s", buf);
    int w = write(serial_fd, buf, strlen(buf));
    if (w < 0) {
        perror("Error writing speed to serial");
        return -1;
    }
    return 0;
}

// Request current speed from device
int get_current_speed(int serial_fd, int *speed_out) {
    const char *cmd = "KS;";
    log_debug("Requesting current speed: %s", cmd);

    if (write(serial_fd, cmd, strlen(cmd)) < 0) {
        perror("Error writing KS; command");
        return -1;
    }

    char buffer[256];
    memset(buffer, 0, sizeof(buffer));
    ssize_t n = read(serial_fd, buffer, sizeof(buffer) - 1);
    if (n < 0) {
        perror("Error reading speed from serial");
        return -1;
    }

    buffer[n] = '\0';
    log_debug("Received serial response: %s", buffer);

    if (!extract_integer(buffer, speed_out)) {
        fprintf(stderr, "Failed to extract speed from serial response\n");
        return -1;
    }

    log_debug("Fetched speed: %d", *speed_out);
    return 0;
}

// Daemonize process unless debugging
void daemonize() {
#if !DEBUG
    pid_t pid = fork();
    if (pid < 0) exit(EXIT_FAILURE);
    if (pid > 0) exit(EXIT_SUCCESS);

    if (setsid() < 0) exit(EXIT_FAILURE);
    signal(SIGCHLD, SIG_IGN);
    signal(SIGHUP, SIG_IGN);

    pid = fork();
    if (pid < 0) exit(EXIT_FAILURE);
    if (pid > 0) exit(EXIT_SUCCESS);

    umask(0);
    chdir("/");

    for (int x = sysconf(_SC_OPEN_MAX); x >= 0; x--) {
        close(x);
    }

    open("/dev/null", O_RDWR);
    dup(0);
    dup(0);
#endif
}

int main(int argc, char *argv[]) {

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--debug") == 0) {
            debug_enabled = 1;
        }
    }

    if (!debug_enabled) daemonize();

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    log_debug("Opening serial port: %s", SERIAL_PORT);
    int serial_fd = open_serial(SERIAL_PORT);
    if (serial_fd < 0) exit(EXIT_FAILURE);

    char input_device_path[PATH_MAX];
    if (!find_input_device(input_device_path, sizeof(input_device_path))) {
        fprintf(stderr, "Error: Could not find input device matching '%s'\n", TARGET_DEVICE_NAME);
        close(serial_fd);
        exit(EXIT_FAILURE);
    }

    log_debug("Opening input device: %s", input_device_path);
    int input_fd = open(input_device_path, O_RDONLY);
    if (input_fd < 0) {
        perror("Error opening input event device");
        close(serial_fd);
        exit(EXIT_FAILURE);
    }

    int speed = 0;
    if (get_current_speed(serial_fd, &speed) < 0) {
        close(serial_fd);
        close(input_fd);
        exit(EXIT_FAILURE);
    }

    struct timespec last_event_time = {0, 0};
    int last_event_code = -1;
    struct input_event ev;

    while (keep_running) {
        ssize_t rd = read(input_fd, &ev, sizeof(ev));
        if (rd == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_KEY && ev.value == 1) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);

                long diff_ms = (now.tv_sec - last_event_time.tv_sec) * 1000 +
                               (now.tv_nsec - last_event_time.tv_nsec) / 1000000;

                if (ev.code == last_event_code && diff_ms < 100)
                    continue; // debounce

                last_event_time = now;
                last_event_code = ev.code;

                log_debug("Event received: type=%d code=%d value=%d", ev.type, ev.code, ev.value);

                if (ev.code == KNOB_COUNTERCLOCKWISE) {
                    speed--;
                    send_speed(serial_fd, speed);
                } else if (ev.code == KNOB_CLOCKWISE) {
                    speed++;
                    send_speed(serial_fd, speed);
                } else if (ev.code == KNOB_CLICK) {
                     if (get_current_speed(serial_fd, &speed) == 0) {
                         log_debug("Speed retrieved on click: %d", speed);
                     }
                }

                log_debug("Updated speed: %d", speed);
            }
        } else if (rd < 0 && errno != EINTR) {
            log_debug("Error reading input: %s", strerror(errno));
        }
    }

    close(serial_fd);
    close(input_fd);
    log_debug("Program exiting gracefully.");
    return 0;
}
