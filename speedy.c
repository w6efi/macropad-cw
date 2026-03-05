#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
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
#include <setjmp.h>
#include <time.h>
#include <dirent.h>
#include <limits.h>
#include <stdarg.h>
#include <linux/limits.h>
#include <asm-generic/termbits-common.h>



/* speedy.c
speed daemon.  
proof of concept for how you can create your own control surface for a radio that takes
cat commands.

Listens to rotary encoder and other keys on a NULLBITS Scramble macropad,
which is an example of a QMK-compatible programmable keyboard, and sends cat commands
including speed commands to radio.

macropad device can be changed in the defines.

Assumes an FTDX10 radio cat commands.  (can be changed in the defines)
The macropad runs firmware programmed to send the keycodes I want.   Keycodes were
chosen so as not to conflict with my regular keyboard.   (can be changed in the defines)

to run:
sudo ./speedy [-d,--debug] [-f path-to-radio-device-file]

defaults are no debug, /dev/ttyUSB0.   if debug, will not daemonize.  ^C to exit when in debug

To compile:
gcc -Wall -Wextra -o speedy speedy.c


written with initial help from ChatGPT
You may use this code freely for noncommercial purposes.  Please give me credit -- W6EFI
*/

// #define DEBUG 1  // Set to 0 to disable debug output.  update: made it a runtime option
// all of these defines should have corresponding commandline options
#define KNOB_CLOCKWISE 190
#define KNOB_COUNTERCLOCKWISE 189
#define KNOB_CLICK 185
#define KEYER_ON_KEY 183
#define KEYER_OFF_KEY 186
#define BREAKIN_ON_KEY 184
#define BREAKIN_OFF_KEY 187
#define SET_SPEED_FAVORITE_KEY 188
#define FAVORITE_SPEED 18
#define KEYER_ON "KR1;"
#define KEYER_OFF "KR0;"
#define BREAKIN_ON "BI1;"
#define BREAKIN_OFF "BI0;"
#define QUERY_SPEED "KS;"

#define SERIAL_PORT "/dev/rig" // Radio attached here. Can override with commandline option
#define BAUDRATE B38400
#define TIMEOUT_SEC 2
#define TARGET_DEVICE_NAME "nullbits SCRAMBLE"  // Adjust this based on your QMK device, not needed if full path to device is known
#define TARGET_DEVICE_DEV_LINK_PATH   "/dev/input/by-id/usb-nullbits_SCRAMBLE-event-kbd"   // Define this if you know what it will be


static int debug_enabled = 0;

// Debug logging
void log_debug(const char *fmt, ...) {
   
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



// Trim leading and trailing whitespace
void trim_whitespace(char *str) {
    while (isspace((unsigned char)*str)) str++;
    if (*str == 0) return;
    char *end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    *(end + 1) = '\0';
}

// Find the input device by name (the macropad)


char *find_input_device(char *resolved_path, size_t path_len) {

    // if you know what the device link will be, just compile in this bit (eg because of udev rules)
    #ifdef TARGET_DEVICE_DEV_LINK_PATH
    log_debug("dev is defined: %s", TARGET_DEVICE_DEV_LINK_PATH);
        strncpy(resolved_path, TARGET_DEVICE_DEV_LINK_PATH, path_len);
        return resolved_path;

    #else
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

    #endif

}

// Open and configure a serial port (a tty)
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

// Extract first integer from a string
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

// Send formatted CW speed to serial port in "KS%03d;" format (for FTDX10)
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

// Request current speed from device (for FTDX10)
int get_current_speed(int serial_fd, int *speed_out) {
    //const char *cmd = "KS;";
    const char *cmd = QUERY_SPEED;
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

// Send a cat command
int send_cat(int serial_fd, char * catcommand) {
    char buf[8];
    snprintf(buf, sizeof(buf), catcommand);
    log_debug("Sending CAT command: %s", buf);
    int w = write(serial_fd, buf, strlen(buf));
    if (w < 0) {
        perror("Error writing CAT string to serial");
        return -1;
    }
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

// Signal handler to be used with setjmp (for catching eg ctrl-c)
jmp_buf env;  // global jump buffer, will be used with setjmp

void sigint_handler(int sig) {
    printf("\ncaught sig %d , exiting\n", sig);
    fflush(stdout);
    longjmp(env, 1);  // jump to setjmp call

}

int main(int argc, char *argv[]) {

    // set up signal listeners for interrupt or terminate signals and fire the handler
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);


    char *radio_device_path = SERIAL_PORT;
    char catcommand[8];

    static struct option long_options[] = {
        {"debug", no_argument, 0, 'd'},
        {"f",     required_argument, 0, 'f'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "df:", long_options, NULL)) != -1) {
        switch (opt) {
            case 'd':
                debug_enabled = 1;
                break;
            case 'f':
                radio_device_path = optarg;
                break;
            default:
                fprintf(stderr,
                        "Usage: %s [--debug] [--f <path-to-device-file>]\n",
                        argv[0]);
                return 1;
        }
    }
    if (debug_enabled) {
        printf("Debug mode: %s\n", debug_enabled ? "ON" : "OFF");
        printf("Device file: %s\n",
           radio_device_path ? radio_device_path : "(not specified)");
    }


    if (!debug_enabled) daemonize();

   

    log_debug("Opening radio serial port: %s", radio_device_path);
    int serial_fd = open_serial(radio_device_path);
    if (serial_fd < 0) exit(EXIT_FAILURE);

    char input_device_path[PATH_MAX];
    if (!find_input_device(input_device_path, sizeof(input_device_path))) {
        fprintf(stderr, "Error: Could not find input device matching '%s'\n", TARGET_DEVICE_NAME);
        close(serial_fd);
        exit(EXIT_FAILURE);
    }

    log_debug("Opening input device (macropad): %s", input_device_path);
    int input_fd = open(input_device_path, O_RDONLY);
    if (input_fd < 0) {
        perror("Error opening input event device");
        close(serial_fd);
        exit(EXIT_FAILURE);
    }

    // current cw speed setting
    int speed = 0;
    if (get_current_speed(serial_fd, &speed) < 0) {
        close(serial_fd);
        close(input_fd);
        exit(EXIT_FAILURE);
    }

    // Now listen for macropad keyboard events, with debouncing
    struct timespec last_event_time = {0, 0};
    int last_event_code = -1;
    struct input_event ev;


    while (setjmp(env) == 0) {
     
        ssize_t rd = read(input_fd, &ev, sizeof(ev));
        log_debug("got an undebounced input event");
        
        if (rd == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_KEY && ev.value == 1) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);

                long diff_ms = (now.tv_sec - last_event_time.tv_sec) * 1000 +
                               (now.tv_nsec - last_event_time.tv_nsec) / 1000000;

                if (ev.code == last_event_code && diff_ms < 100)
                    continue; // debouncing -- events too fast, go back up and keep reading til a pause

                last_event_time = now;
                last_event_code = ev.code;

                log_debug("Event received: type=%d code=%d value=%d", ev.type, ev.code, ev.value);

                if (ev.code == KNOB_COUNTERCLOCKWISE) {
                    speed--;
                    send_speed(serial_fd, speed);
                    log_debug("Updated speed: %d", speed);
                } else if (ev.code == KNOB_CLOCKWISE) {
                    speed++;
                    send_speed(serial_fd, speed);
                    log_debug("Updated speed: %d", speed);
                } else if (ev.code == KNOB_CLICK) {
                     if (get_current_speed(serial_fd, &speed) == 0) {
                         log_debug("Speed retrieved on click: %d", speed);
                     }
                } else if (ev.code == SET_SPEED_FAVORITE_KEY){
                    log_debug("set speed to %d", FAVORITE_SPEED);
                    send_speed(serial_fd, FAVORITE_SPEED);
                    speed = FAVORITE_SPEED;
                } else if (ev.code == KEYER_ON_KEY){
                    log_debug("keyer on");
                    strcpy(catcommand, KEYER_ON);
                    send_cat(serial_fd, catcommand);
                } else if (ev.code == KEYER_OFF_KEY){
                    log_debug("keyer off");
                    strcpy(catcommand, KEYER_OFF);
                    send_cat(serial_fd, catcommand);
                } else if (ev.code == BREAKIN_ON_KEY){
                    log_debug("breakin on");
                    strcpy(catcommand, BREAKIN_ON);
                    send_cat(serial_fd, catcommand);
                } else if (ev.code == BREAKIN_OFF_KEY){
                    log_debug("breakin off");
                    strcpy(catcommand, BREAKIN_OFF);
                    send_cat(serial_fd, catcommand);
                } 

                

            }
        } else if (rd < 0 && errno != EINTR) {
            log_debug("Error reading input: %s", strerror(errno));
            // have to clean up or it will loop forever and exhaust the system CPU resource
            close(input_fd);
            close(serial_fd);
            log_debug("Input device gone missing, cleaning up and program exiting");
            return 1;
        }
    }

    close(serial_fd);
    close(input_fd);
    log_debug("Program exiting gracefully.");
    return 0;
}
