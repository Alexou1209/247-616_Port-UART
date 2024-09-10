#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions
#include <stdlib.h>
#include <sys/wait.h> 

void Initialise_PortUART(void);
void codeDuProcessusEnfant1(int pipe1[2]);
void codeDuProcessusEnfant2(int pipe1[2]);

const char *portTTY = "/dev/ttyS1";
int fd; // File Descriptor

int main(void)
{
    Initialise_PortUART();

    int pipe1[2]; // Pipe for communication between Enfant 1 and Enfant 2

    // Create the pipe
    if (pipe(pipe1) == -1) {
        perror("Erreur lors de la création du pipe");
        exit(EXIT_FAILURE);
    }

    pid_t pid, pid2;
    pid = fork();

    if (pid == 0)
    {
        // Enfant 1 (terminal to serial port)
        close(pipe1[0]); // Close reading end of pipe1 (we will write to it)
        printf("Je suis le processus enfant_1, j'écrit sur le port série ce que j'entends sur la console (terminal)\n");
        codeDuProcessusEnfant1(pipe1);
        printf("Fin du Processus Enfant1\n");
        close(pipe1[1]); // Close the writing end of pipe1 when done
    }
    else
    {
        pid2 = fork();
        if (pid2 == 0)
        {
            // Enfant 2 (serial port to terminal)
            close(pipe1[1]); // Close writing end of pipe1 (we will read from it)
            printf("Je suis le processus enfant_2, j'écrit sur la console (terminal) ce que j'entends sur le port série\n");
            codeDuProcessusEnfant2(pipe1);
            printf("Fin du Processus Enfant2\n");
            close(pipe1[0]); // Close the reading end of pipe1 when done
        }

        // Parent process - wait for children to finish
        wait(NULL);
        wait(NULL);
        printf("Fin du Processus Parent\n");
    }
}

void codeDuProcessusEnfant1(int pipe1[2])
{
    char c;
    while (1) {
        // Read from the terminal
        c = getchar();

        // Write to the pipe (to be read by Enfant 2)
        write(pipe1[1], &c, 1);

        // Send the data to the serial port
        if (write(fd, &c, 1) < 0) {
            perror("Erreur d'écriture sur le port série");
            exit(EXIT_FAILURE);
        }

        // If 'q' is entered, exit
        if (c == 'q') {
            break;
        }
    }
    close(fd); // Close the serial port
}

void codeDuProcessusEnfant2(int pipe1[2])
{
    char c;
    int nbytes;

    while (1) {
        // Read from the pipe (which Enfant 1 wrote to)
        nbytes = read(pipe1[0], &c, 1);
        if (nbytes <= 0) {
            perror("Erreur de lecture du pipe");
            exit(EXIT_FAILURE);
        }

        // Print the data to the terminal
        printf("Processus Enfant_2: caractère reçu : %c\n", c);

        // If 'q' is received, exit
        if (c == 'q') {
            break;
        }

        // Optionally send the data back to Enfant 1 (if using the second pipe)
    }
    close(fd); // Close the serial port
}

void Initialise_PortUART(void)
{
    // Open the serial port
    fd = open(portTTY, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Erreur lors de l'ouverture du port série");
        exit(EXIT_FAILURE);
    }

    // Configure the serial port
    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings); // Get the current settings of the serial port

    // Set baud rate
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    // Configure 8N1 (8 data bits, no parity, 1 stop bit)
    SerialPortSettings.c_cflag &= ~PARENB;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;

    // Disable hardware flow control
    SerialPortSettings.c_cflag &= ~CRTSCTS;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver and ignore modem control lines

    // Disable software flow control
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Non-canonical mode, no echo, no signals
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // No output processing
    SerialPortSettings.c_oflag &= ~OPOST;

    // Set VMIN and VTIME for blocking read of 1 character at a time
    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 0;

    // Apply the settings
    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
        perror("Erreur lors de la configuration du port série");
        exit(EXIT_FAILURE);
    }

    tcflush(fd, TCIFLUSH); // Flush the serial port input buffer
}
