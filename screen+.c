/*
 * x11-spi-display.c
 * 
 * Simple X11 screen capture to SPI display driver for Orange Pi
 * Captures X11 framebuffer and sends to 240x320 SPI display with rotation support
 * 
 * Usage: DISPLAY=:0 sudo ./x11-spi-display [color_mode] [rotation]
 *        color_mode: 1-4 (try if colors are wrong)
 *        rotation: 0, 90, 180, 270 (degrees clockwise)
 * 
 * Example: DISPLAY=:0 sudo ./x11-spi-display 1 90
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <linux/spi/spidev.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>

// Display configuration
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320

// GPIO pins (change these if your wiring is different)
#define PIN_DC          72   // Data/Command selection
#define PIN_RESET       78   // Display reset

// SPI configuration
#define SPI_DEVICE      "/dev/spidev1.1"
#define SPI_SPEED       32000000U  // 32 MHz

// Target frame rate
#define TARGET_FPS      25

// X11 and display state
static Display *x11_display = NULL;
static Window root_window;
static int screen_width = 0;
static int screen_height = 0;

// SPI communication
static int spi_fd = -1;

// Threading control
static volatile int keep_running = 1;

// Configuration from command line
static int color_mode = 1;      // 1-4, affects RGB/BGR conversion
static int rotation_degrees = 0; // 0, 90, 180, or 270

// Double buffering for smooth display
static uint16_t *framebuffer[2] = {NULL, NULL};
static int current_write_buffer = 0;
static int ready_buffer = -1;
static pthread_mutex_t buffer_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t buffer_ready = PTHREAD_COND_INITIALIZER;

// Pre-computed RGB565 conversion tables for speed
static uint16_t red_to_rgb565[256];
static uint16_t green_to_rgb565[256];
static uint16_t blue_to_rgb565[256];

// XShm (X Shared Memory) for fast screen capture
static int using_xshm = 0;
static XImage *xshm_image = NULL;
static XShmSegmentInfo shm_info;

/*
 * GPIO Functions
 * Uses Linux sysfs interface - simple but slow enough for our needs
 */

static void gpio_write_value(const char *path, const char *value) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) return;
    write(fd, value, strlen(value));
    close(fd);
}

static void gpio_export(int pin) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", pin);
    
    // Skip if already exported
    if (access(path, F_OK) == 0) return;
    
    // Export the pin
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return;
    
    char pin_str[8];
    snprintf(pin_str, sizeof(pin_str), "%d", pin);
    write(fd, pin_str, strlen(pin_str));
    close(fd);
    
    usleep(100000); // Wait for sysfs to create files
}

static void gpio_set_direction(int pin, const char *direction) {
    char path[128];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    gpio_write_value(path, direction);
}

static void gpio_set(int pin, int value) {
    char path[128];
    char value_str[2];
    
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    value_str[0] = value ? '1' : '0';
    value_str[1] = '\0';
    
    gpio_write_value(path, value_str);
}

/*
 * SPI Functions
 * Uses hardware SPI via Linux kernel driver
 */

static int spi_initialize(void) {
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        return -1;
    }
    
    // Configure SPI mode, bits per word, and speed
    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;
    
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Failed to set SPI mode");
    }
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("Failed to set SPI bits per word");
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("Failed to set SPI speed");
    }
    
    return 0;
}

static int spi_send(uint8_t *data, int length) {
    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)data,
        .rx_buf = 0,
        .len = length,
        .delay_usecs = 0,
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };
    
    int result = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer);
    if (result < 1) {
        perror("SPI transfer failed");
        return -1;
    }
    
    return 0;
}

/*
 * Display Communication Functions
 * Commands and data for ILI9341/similar display controllers
 */

static void display_send_command(uint8_t cmd) {
    gpio_set(PIN_DC, 0);  // Command mode
    spi_send(&cmd, 1);
}

static void display_send_data(uint8_t *data, int length) {
    gpio_set(PIN_DC, 1);  // Data mode
    spi_send(data, length);
}

static void display_send_data_byte(uint8_t data) {
    gpio_set(PIN_DC, 1);
    spi_send(&data, 1);
}

static void display_initialize(void) {
    printf("Initializing display...\n");
    
    // Hardware reset sequence
    gpio_set(PIN_RESET, 0);
    usleep(10000);
    gpio_set(PIN_RESET, 1);
    usleep(150000);
    
    // Display initialization sequence for ILI9341
    display_send_command(0x01);  // Software reset
    usleep(150000);
    
    display_send_command(0x3A);  // Pixel format
    display_send_data_byte(0x55); // 16-bit RGB565
    
    display_send_command(0x36);  // Memory access control
    display_send_data_byte(0x48); // Display orientation
    
    display_send_command(0x11);  // Sleep out
    usleep(120000);
    
    display_send_command(0x29);  // Display on
    usleep(50000);
}

static void display_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Set column address
    display_send_command(0x2A);
    uint8_t col_data[4] = { x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF };
    display_send_data(col_data, 4);
    
    // Set row address
    display_send_command(0x2B);
    uint8_t row_data[4] = { y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF };
    display_send_data(row_data, 4);
    
    // Start memory write
    display_send_command(0x2C);
}

/*
 * Color Conversion Functions
 */

static void prepare_color_tables(void) {
    for (int i = 0; i < 256; i++) {
        red_to_rgb565[i]   = (i & 0xF8) << 8;  // 5 bits for red
        green_to_rgb565[i] = (i & 0xFC) << 3;  // 6 bits for green
        blue_to_rgb565[i]  = (i >> 3);         // 5 bits for blue
    }
}

// Helper to extract color component from pixel based on mask
static int get_mask_shift(uint32_t mask) {
    if (mask == 0) return 0;
    
    int shift = 0;
    while ((mask & 1) == 0) {
        mask >>= 1;
        shift++;
    }
    return shift;
}

static int count_mask_bits(uint32_t mask) {
    int count = 0;
    while (mask) {
        if (mask & 1) count++;
        mask >>= 1;
    }
    return count;
}

static uint8_t extract_color_component(unsigned long pixel, uint32_t mask, int shift) {
    if (mask == 0) return 0;
    
    unsigned long value = (pixel & mask) >> shift;
    int bits = count_mask_bits(mask);
    
    // Already 8-bit?
    if (bits >= 8) return (uint8_t)(value & 0xFF);
    
    // Scale to 8-bit
    return (uint8_t)((value * 255) / ((1u << bits) - 1));
}

static uint16_t convert_to_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t pixel;
    
    switch (color_mode) {
        case 1:  // RGB normal
            pixel = red_to_rgb565[r] | green_to_rgb565[g] | blue_to_rgb565[b];
            break;
            
        case 2:  // RGB with byte swap
            pixel = red_to_rgb565[r] | green_to_rgb565[g] | blue_to_rgb565[b];
            pixel = (pixel >> 8) | (pixel << 8);
            break;
            
        case 3:  // BGR normal
            pixel = ((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3);
            break;
            
        case 4:  // BGR with byte swap
            pixel = ((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3);
            pixel = (pixel >> 8) | (pixel << 8);
            break;
            
        default:
            pixel = red_to_rgb565[r] | green_to_rgb565[g] | blue_to_rgb565[b];
            break;
    }
    
    return pixel;
}

/*
 * Frame Capture and Processing
 */

static void capture_and_convert_frame(XImage *x11_image, uint16_t *output_buffer) {
    // Get bit shifts for color extraction
    int red_shift   = get_mask_shift(x11_image->red_mask);
    int green_shift = get_mask_shift(x11_image->green_mask);
    int blue_shift  = get_mask_shift(x11_image->blue_mask);
    
    // Calculate scaling factors (fixed-point math for speed)
    int x_step, y_step;
    
    if (rotation_degrees == 90 || rotation_degrees == 270) {
        // Rotated 90/270: swap width and height for scaling
        x_step = (screen_height << 16) / DISPLAY_WIDTH;
        y_step = (screen_width << 16) / DISPLAY_HEIGHT;
    } else {
        x_step = (screen_width << 16) / DISPLAY_WIDTH;
        y_step = (screen_height << 16) / DISPLAY_HEIGHT;
    }
    
    // Process each pixel
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            int source_x, source_y;
            
            // Apply rotation transform
            switch (rotation_degrees) {
                case 90:  // 90 degrees clockwise
                    source_x = ((DISPLAY_HEIGHT - 1 - y) * x_step) >> 16;
                    source_y = (x * y_step) >> 16;
                    break;
                    
                case 180:  // 180 degrees
                    source_x = ((DISPLAY_WIDTH - 1 - x) * x_step) >> 16;
                    source_y = ((DISPLAY_HEIGHT - 1 - y) * y_step) >> 16;
                    break;
                    
                case 270:  // 270 degrees clockwise
                    source_x = (y * x_step) >> 16;
                    source_y = ((DISPLAY_WIDTH - 1 - x) * y_step) >> 16;
                    break;
                    
                default:  // 0 degrees (no rotation)
                    source_x = (x * x_step) >> 16;
                    source_y = (y * y_step) >> 16;
                    break;
            }
            
            // Read pixel from X11 image
            uint8_t *row = (uint8_t*)x11_image->data + source_y * x11_image->bytes_per_line;
            unsigned long pixel = 0;
            
            if (x11_image->bits_per_pixel == 32) {
                pixel = *((uint32_t*)(row + source_x * 4));
            } else if (x11_image->bits_per_pixel == 24) {
                uint8_t *p = row + source_x * 3;
                pixel = p[0] | (p[1] << 8) | (p[2] << 16);
            } else if (x11_image->bits_per_pixel == 16) {
                pixel = *((uint16_t*)(row + source_x * 2));
            } else {
                // Fallback: slower but works for any format
                pixel = XGetPixel(x11_image, source_x, source_y);
            }
            
            // Extract RGB components
            uint8_t r = extract_color_component(pixel, x11_image->red_mask, red_shift);
            uint8_t g = extract_color_component(pixel, x11_image->green_mask, green_shift);
            uint8_t b = extract_color_component(pixel, x11_image->blue_mask, blue_shift);
            
            // Convert and store
            output_buffer[y * DISPLAY_WIDTH + x] = convert_to_rgb565(r, g, b);
        }
    }
}

/*
 * Screen Capture Thread
 * Continuously captures X11 screen and prepares framebuffers
 */

static void* screen_capture_thread(void *arg) {
    struct timespec frame_delay = {0, 1000000000 / TARGET_FPS};
    time_t last_fps_report = time(NULL);
    int frame_count = 0;
    
    while (keep_running) {
        // Capture screen
        XImage *captured_image = NULL;
        
        if (using_xshm) {
            // Fast path: shared memory
            XShmGetImage(x11_display, root_window, xshm_image, 0, 0, AllPlanes);
            captured_image = xshm_image;
        } else {
            // Slower path: regular XGetImage
            captured_image = XGetImage(x11_display, root_window, 
                                      0, 0, screen_width, screen_height, 
                                      AllPlanes, ZPixmap);
            if (!captured_image) {
                fprintf(stderr, "XGetImage failed\n");
                nanosleep(&frame_delay, NULL);
                continue;
            }
        }
        
        // Get buffer to write to
        pthread_mutex_lock(&buffer_lock);
        int write_buffer = current_write_buffer;
        pthread_mutex_unlock(&buffer_lock);
        
        // Convert and write to buffer
        capture_and_convert_frame(captured_image, framebuffer[write_buffer]);
        
        // Free regular XImage (XShm doesn't need freeing)
        if (!using_xshm && captured_image) {
            XDestroyImage(captured_image);
        }
        
        // Mark buffer as ready
        pthread_mutex_lock(&buffer_lock);
        ready_buffer = write_buffer;
        current_write_buffer = 1 - current_write_buffer;  // Swap buffers
        pthread_cond_signal(&buffer_ready);
        pthread_mutex_unlock(&buffer_lock);
        
        // FPS reporting
        frame_count++;
        time_t now = time(NULL);
        if (now - last_fps_report >= 5) {
            float fps = frame_count / (float)(now - last_fps_report);
            printf("Capture FPS: %.1f\n", fps);
            frame_count = 0;
            last_fps_report = now;
        }
        
        nanosleep(&frame_delay, NULL);
    }
    
    return NULL;
}

/*
 * Display Update Thread
 * Sends framebuffers to display via SPI
 */

static void* display_update_thread(void *arg) {
    int line_bytes = DISPLAY_WIDTH * 2;  // 2 bytes per pixel (RGB565)
    uint8_t *line_buffer = malloc(line_bytes);
    
    if (!line_buffer) {
        perror("Failed to allocate line buffer");
        return NULL;
    }
    
    while (keep_running) {
        // Wait for a buffer to be ready
        pthread_mutex_lock(&buffer_lock);
        while (ready_buffer < 0 && keep_running) {
            pthread_cond_wait(&buffer_ready, &buffer_lock);
        }
        if (!keep_running) {
            pthread_mutex_unlock(&buffer_lock);
            break;
        }
        
        int send_buffer = ready_buffer;
        ready_buffer = -1;
        pthread_mutex_unlock(&buffer_lock);
        
        // Set display window
        display_set_window(0, 0, DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1);
        gpio_set(PIN_DC, 1);  // Data mode for pixel data
        
        // Send framebuffer line by line
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            uint16_t *source = &framebuffer[send_buffer][y * DISPLAY_WIDTH];
            
            // Pack pixels into bytes (big-endian)
            for (int x = 0; x < DISPLAY_WIDTH; x++) {
                uint16_t pixel = source[x];
                line_buffer[2*x + 0] = (pixel >> 8) & 0xFF;
                line_buffer[2*x + 1] = pixel & 0xFF;
            }
            
            // Send line via SPI
            if (spi_send(line_buffer, line_bytes) < 0) {
                // Fallback: try direct write
                write(spi_fd, line_buffer, line_bytes);
            }
        }
    }
    
    free(line_buffer);
    return NULL;
}

/*
 * Cleanup Functions
 */

static void cleanup_xshm(void) {
    if (using_xshm && xshm_image) {
        XShmDetach(x11_display, &shm_info);
        shmdt(shm_info.shmaddr);
        shmctl(shm_info.shmid, IPC_RMID, 0);
        XDestroyImage(xshm_image);
        xshm_image = NULL;
    }
}

static void signal_handler(int signal) {
    (void)signal;
    keep_running = 0;
    pthread_cond_broadcast(&buffer_ready);
}

/*
 * Main Program
 */

int main(int argc, char **argv) {
    // Parse command line arguments
    if (argc > 1) {
        color_mode = atoi(argv[1]);
        if (color_mode < 1 || color_mode > 4) {
            color_mode = 1;
        }
    }
    
    if (argc > 2) {
        rotation_degrees = atoi(argv[2]);
        if (rotation_degrees != 0 && rotation_degrees != 90 && 
            rotation_degrees != 180 && rotation_degrees != 270) {
            rotation_degrees = 0;
        }
    }
    
    // Check root access (needed for GPIO/SPI)
    if (geteuid() != 0) {
        fprintf(stderr, "Error: This program needs root access for GPIO and SPI.\n");
        fprintf(stderr, "Run with: sudo %s\n", argv[0]);
        return 1;
    }
    
    printf("X11 to SPI Display Driver\n");
    printf("Color mode: %d, Rotation: %d degrees\n", color_mode, rotation_degrees);
    
    // Prepare color conversion tables
    prepare_color_tables();
    
    // Allocate framebuffers
    for (int i = 0; i < 2; i++) {
        framebuffer[i] = malloc(DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t));
        if (!framebuffer[i]) {
            perror("Failed to allocate framebuffer");
            return 1;
        }
        memset(framebuffer[i], 0, DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t));
    }
    
    // Setup GPIO
    printf("Setting up GPIO...\n");
    gpio_export(PIN_DC);
    gpio_export(PIN_RESET);
    usleep(100000);
    gpio_set_direction(PIN_DC, "out");
    gpio_set_direction(PIN_RESET, "out");
    
    // Initialize SPI
    printf("Initializing SPI...\n");
    if (spi_initialize() < 0) {
        return 1;
    }
    
    // Connect to X11
    printf("Connecting to X11...\n");
    const char *display_name = getenv("DISPLAY");
    if (!display_name) {
        fprintf(stderr, "Error: DISPLAY environment variable not set.\n");
        fprintf(stderr, "Try: DISPLAY=:0 sudo %s\n", argv[0]);
        return 1;
    }
    
    x11_display = XOpenDisplay(display_name);
    if (!x11_display) {
        fprintf(stderr, "Error: Cannot connect to X11 display %s\n", display_name);
        return 1;
    }
    
    int screen = DefaultScreen(x11_display);
    root_window = RootWindow(x11_display, screen);
    
    XWindowAttributes window_attrs;
    XGetWindowAttributes(x11_display, root_window, &window_attrs);
    screen_width = window_attrs.width;
    screen_height = window_attrs.height;
    printf("X11 screen: %dx%d\n", screen_width, screen_height);
    
    // Try to use XShm for faster capture
    if (XShmQueryExtension(x11_display)) {
        xshm_image = XShmCreateImage(x11_display, 
                                     DefaultVisual(x11_display, screen),
                                     DefaultDepth(x11_display, screen),
                                     ZPixmap, NULL, &shm_info, 
                                     screen_width, screen_height);
        if (xshm_image) {
            shm_info.shmid = shmget(IPC_PRIVATE, 
                                   xshm_image->bytes_per_line * xshm_image->height, 
                                   IPC_CREAT | 0777);
            if (shm_info.shmid >= 0) {
                shm_info.shmaddr = xshm_image->data = shmat(shm_info.shmid, 0, 0);
                shm_info.readOnly = False;
                XShmAttach(x11_display, &shm_info);
                using_xshm = 1;
                printf("Using XShm for fast capture\n");
            } else {
                XDestroyImage(xshm_image);
                xshm_image = NULL;
            }
        }
    }
    
    // Initialize display
    display_initialize();
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Start threads
    printf("Starting capture and display threads...\n");
    pthread_t capture_thread_id, display_thread_id;
    
    if (pthread_create(&capture_thread_id, NULL, screen_capture_thread, NULL) != 0) {
        perror("Failed to create capture thread");
        return 1;
    }
    
    if (pthread_create(&display_thread_id, NULL, display_update_thread, NULL) != 0) {
        perror("Failed to create display thread");
        keep_running = 0;
        pthread_join(capture_thread_id, NULL);
        return 1;
    }
    
    printf("Running... Press Ctrl+C to exit\n");
    
    // Wait for threads to finish
    pthread_join(capture_thread_id, NULL);
    pthread_join(display_thread_id, NULL);
    
    // Cleanup
    printf("\nCleaning up...\n");
    cleanup_xshm();
    XCloseDisplay(x11_display);
    close(spi_fd);
    
    for (int i = 0; i < 2; i++) {
        if (framebuffer[i]) {
            free(framebuffer[i]);
        }
    }
    
    printf("Done.\n");
    return 0;
}
