
#include "env.h"
#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "loguru/loguru.hpp"

#include <stdio.h>
#include <stdlib.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhantomxEnv");
    ros::NodeHandle nh;
    ros::Rate r(50);
    PhantomxEnv env(nh, r);

     // Create a socket
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        LOG_F(ERROR, "Error creating socket");
        return 1;
    }


    // Set the timeout in seconds
    
    /*int timeout = 0.1;
    struct timeval tv;
    tv.tv_sec = timeout;
    tv.tv_usec = 0;

    if (setsockopt(serverSocket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
        perror("Error setting socket timeout");
        close(serverSocket);
        exit(EXIT_FAILURE);
    }

     // Get the current timeout
    struct timeval current_timeout;
    socklen_t len = sizeof(current_timeout);
    
    if (getsockopt(serverSocket, SOL_SOCKET, SO_SNDTIMEO, &current_timeout, &len) == 0) {
        printf("Current socket timeout: %ld seconds\n", current_timeout.tv_sec);
    } else {
        perror("Error getting socket timeout");
    }*/

    // Bind the socket to an address and port
    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(12345); // Port number
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1) 
    {
        LOG_F(ERROR, "Error binding socket");
        return 1;
    }

    // Listen for incoming connections
    if (listen(serverSocket, 1) == -1) 
    {
        LOG_F(ERROR, "Error listening for connections");
        return 1;
    }

    LOG_F(INFO, "Server is listening for incoming connections...");

    int clientSocket = 0;
    int count = 0;

    // Accept a connection
    clientSocket = accept(serverSocket, NULL, NULL);
    if (clientSocket == -1) {
        LOG_F(ERROR, "Error accepting connection");
        return 1;
    }
    
    unsigned int data_to_send = 311;
    // Receive the serialized float array
    char buffer[25*sizeof(float)];
    while (ros::ok()){

        ssize_t bytes_received = recv(clientSocket, buffer, sizeof(buffer), 0);
        int num_floats = bytes_received / sizeof(float);
        if (num_floats == 25) 
        {
            // Deserialize the received data into a vector of floats
            std::vector<float> received_values(num_floats);
            std::memcpy(received_values.data(), buffer, bytes_received);
            std::shared_ptr<std::vector<float>> obs = env.step(received_values);
            assert(obs->size() == data_to_send);
            float dataToSend[data_to_send];
            for(int i=0; i<data_to_send; i++) dataToSend[i] = obs->at(i);
            ssize_t sent_bytes =  send(clientSocket, dataToSend, sizeof(dataToSend), 0);
            assert(sent_bytes == sizeof(dataToSend));
        }
        else
        {            
            close(clientSocket);
            clientSocket = accept(serverSocket, NULL, NULL);
            if (clientSocket == -1) {
                LOG_F(ERROR, "Error accepting connection");
                return 1;
            }
            LOG_F(INFO, "closing connection and restart, bytes received: %ld", bytes_received);
        }
    }
}
