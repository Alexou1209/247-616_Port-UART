/**
 * @file    fork-uart.c
 * 
 * @brief Serial Port Programming in C
 * Non Cannonical mode 
 * Sellecting the Serial port Number on Linux   
 * /dev/ttyUSBx - when using USB  to Serial Converter, where x can be 0,1,2...etc  
 * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc 
 * termios structure -  /usr/include/asm-generic/termbits.h  
 * use "man termios" to get more info about  termios structure 
 * @author  Alexandre Dionne
 * @date    2024-10-07
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions
#include <stdlib.h>
#include <sys/wait.h> 


void Initialise_PortUART(void);
void codeDuProcessusEnfant(void);
void codeDuProcessusParent(void);

// device port série à utiliser 
//const char *portTTY = "/dev/ttyGS0"; 
//const char *portTTY = "/dev/ttyS0";
const char *portTTY = "/dev/ttyS1";
//const char *portTTY = "/dev/ttyS2";
//const char *portTTY = "/dev/ttyS3";
//const char *portTTY = "/dev/ttyS4";
//const char *portTTY = "/dev/ttyS5";
//const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter

int fd; // File Descriptor

void main (void)
{
	Initialise_PortUART();
  	pid_t pid;
  	pid = fork();

	if(pid == 0)
    {
		printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console (terminal)\n");
        codeDuProcessusEnfant();
		printf("Fin du Processus fils\n");
    }

    // Appel fonction Parent
    else
    {
		printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série\n");
        codeDuProcessusParent();
		printf("Fin du Processus pere\n");
    }
	wait(NULL);
}

void codeDuProcessusParent()
{
    char read_buffer[32]; // Tampon pour stocker les données reçues
    int nbytes;
    
    while (1) {
        // Lecture des données du port série
        nbytes = read(fd, read_buffer, sizeof(read_buffer));
        if (nbytes < 0) {
            perror("Erreur de lecture");
            exit(EXIT_FAILURE);
        }

        read_buffer[nbytes] = '\0'; // Ajouter le caractère de fin de chaîne
        printf("Processus Père: nombre d'octets reçus : %d --> %s\n", nbytes, read_buffer);

        // Si le caractère '!' est reçu, quitter
        if (read_buffer[0] == '!') {
            break;
        }
    }
    close(fd); // Fermer le port série
}


void codeDuProcessusEnfant()
{
    char c;
    while (1) {
        // Lecture d'un caractère depuis le terminal
        c = getchar();

        // Écriture sur le port série
        if (write(fd, &c, 1) < 0) {
            perror("Erreur d'écriture");
            exit(EXIT_FAILURE);
        }

        // Si le caractère 'q' est entré, quitter
        if (c == 'q') {
            break;
        }
    }
    close(fd); // Fermer le port série
}


void Initialise_PortUART(void)
{
    // Ouverture du port série
    fd = open(portTTY, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Erreur lors de l'ouverture du port série");
        exit(EXIT_FAILURE);
    }

    // Configuration du port série
    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings); // Récupérer les attributs actuels du port

    // Configurer la vitesse en bauds
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    // Configurer 8N1 (8 bits de données, pas de parité, 1 bit d'arrêt)
    SerialPortSettings.c_cflag &= ~PARENB;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;

    // Désactiver le contrôle de flux matériel
    SerialPortSettings.c_cflag &= ~CRTSCTS;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Activer la lecture et ignorer les lignes de contrôle du modem

    // Désactiver le contrôle de flux logiciel
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Mode non-canonique, sans écho ni signaux
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Pas de traitement de sortie
    SerialPortSettings.c_oflag &= ~OPOST;

    // Configurer VMIN et VTIME pour une lecture d'au moins 1 caractère avec un délai infini
    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 0;

    // Appliquer les paramètres
    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
        perror("Erreur lors de la configuration des attributs du port série");
        exit(EXIT_FAILURE);
    }

    tcflush(fd, TCIFLUSH); // Vider le tampon d'entrée du port série
}
