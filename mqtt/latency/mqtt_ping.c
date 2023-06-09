//
// Copyright (c) 2017, 2020 ADLINK Technology Inc.
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ADLINK zenoh team, <zenoh@adlink-labs.tech>
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MQTTAsync.h"
#include <assert.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/time.h>


#define CLIENTID    "mqtt_ping"
#define PING_TOPIC  "/test/ping"
#define PONG_TOPIC  "/test/pong"
#define QOS         0

volatile int finished = 0;
volatile int ready = 0;
volatile int subscribed = 0;
volatile int received = 0;
u_int64_t counter = 0;
const char* DEFAULT_BROKER = "tcp://127.0.0.1:1883";
const char* DEFAULT_PONG_TOPIC = "/test/pong";

double interveal = 1; //s

struct ping_data {
    pthread_mutex_t lock;
    pthread_cond_t cond;
    u_int64_t seq_num;
};

struct ping_data ping_info;


struct latency_data {
    struct timeval ts;
    size_t len;
    u_int8_t *payload;
};

// Function to serialize a `struct latency_data` into a byte array
size_t serialize_latency_data(struct latency_data data, u_int8_t **buffer) {
    size_t total_size = sizeof(struct timeval) + sizeof(size_t) + data.len;
    *buffer = (u_int8_t *)malloc(total_size);
    if (*buffer == NULL) {
        return 0;  // Failed to allocate memory
    }
    
    size_t offset = 0;
    
    // Copy the timeval structure
    memcpy(*buffer + offset, &data.ts, sizeof(struct timeval));
    offset += sizeof(struct timeval);
    
    // Copy the len field
    memcpy(*buffer + offset, &data.len, sizeof(size_t));
    offset += sizeof(size_t);
    
    // Copy the payload
    memcpy(*buffer + offset, data.payload, data.len);
    
    return total_size;
}

// Function to deserialize a byte array into a `struct latency_data`
struct latency_data deserialize_latency_data(u_int8_t *buffer, size_t buffer_size) {
    struct latency_data data;
    
    size_t offset = 0;
    
    // Copy the timeval structure
    memcpy(&data.ts, buffer + offset, sizeof(struct timeval));
    offset += sizeof(struct timeval);
    
    // Copy the len field
    memcpy(&data.len, buffer + offset, sizeof(size_t));
    offset += sizeof(size_t);
    
    // Allocate memory for the payload and copy its data
    data.payload = (u_int8_t *)malloc(data.len);
    if (data.payload != NULL) {
        memcpy(data.payload, buffer + offset, data.len);
    }
    
    return data;
}

void onConnectFailure(void* context, MQTTAsync_failureData5* response)
{
    printf("Connect failed, rc %d\n", response ? response->code : 0);
    exit(EXIT_FAILURE);
}


void onConnect(void* context, MQTTAsync_successData5* response)
{
    ready = 1;
}

void onSubscribe(void* context, MQTTAsync_successData5* response)
{
    subscribed = 1;
}

void onSubscribeFailure(void* context, MQTTAsync_failureData5* response)
{
    printf("Subscribe failed, rc %d\n", response->code);
    exit(EXIT_FAILURE);
}


int msgarrvd(void *context, char *topicName, int topicLen, MQTTAsync_message *message)
{   
    
    struct timespec now;
    struct timespec *start;
    if (strncmp(topicName, DEFAULT_PONG_TOPIC, topicLen) == 0 ) {

        clock_gettime(CLOCK_MONOTONIC_RAW, &now);

        struct latency_data data = deserialize_latency_data(message->payload, message->payloadlen);
        //deserialize_data(message->payload, &data);

        // deserialize the message
        start = &data.ts;
        //(struct timespec*) message->payload;
        u_int64_t elapsed = (now.tv_sec - start->tv_sec) * 1000000 + (now.tv_nsec - start->tv_nsec) / 1000;
        u_int64_t ts = (now.tv_sec * 1000000) + (now.tv_nsec / 1000);
        // <protocol>,[latency|througput],[interval|payload],<value>,<unit>, <timestamp>, <size>,<qos>
        printf("mqtt,latency,%.10f,%ld,us,%ld,%ld,%d\n", interveal, elapsed, ts, data.len, QOS );
        fflush(stdout);

        // pthread_mutex_lock(&ping_info.lock);

        // memcpy((void *)&ping_info.seq_num, message->payload, sizeof(u_int64_t));

        // pthread_cond_signal(&ping_info.cond);
        // pthread_mutex_unlock(&ping_info.lock);

    }
    MQTTAsync_freeMessage(&message);
    MQTTAsync_free(topicName);
    return 1;
}


int main(int argc, char* argv[])
{
    MQTTAsync client;
    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer5;
    MQTTAsync_createOptions create_opts = MQTTAsync_createOptions_initializer;
    int rc, c;
    size_t payload = 64;
    char* broker = NULL;
    char* payload_value = NULL;
    u_int8_t* payload_buff = NULL;
    char* name = NULL;
    char* scenario = NULL;
    char* interveal_value = NULL;
    struct timespec start, end;
    u_int64_t seq_num = 0;



     // Parsing arguments
    while((c = getopt(argc, argv, ":b:p:n:s:i:")) != -1 ){
        switch (c) {
            case 'n':
                name = optarg;
                break;
            case 's':
                scenario = optarg;
                break;
            case 'p':
                payload_value = optarg;
                break;
            case 'b':
                broker = optarg;
                break;
            case 'i':
                interveal_value = optarg;
                break;
            default:
                break;
        }
    }

    // Setting defaults

    if (broker == NULL) {
        // We should copy the default
        broker = (char*) calloc(sizeof(char),strlen(DEFAULT_BROKER));
        memcpy(broker, DEFAULT_BROKER, strlen(DEFAULT_BROKER));
    }

    if (payload_value != NULL) {
        payload = (size_t) atoi(payload_value);
    }

    if (interveal_value != NULL) {
        interveal = (double) atof(interveal_value);
    }

    //data = (void*) calloc(sizeof(struct timespec),1);

    create_opts.MQTTVersion = MQTTVERSION_5;
    if ((rc = MQTTAsync_createWithOptions(&client, broker, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL, &create_opts)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to create client object, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }

    conn_opts.keepAliveInterval = 3;
    conn_opts.onSuccess5 = onConnect;
    conn_opts.onFailure5 = onConnectFailure;
    conn_opts.context = client;
    conn_opts.MQTTVersion = MQTTVERSION_5;
    conn_opts.cleanstart = 1;

    if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to start connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }


    while (!ready);


    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;

    opts.onSuccess5 = onSubscribe;
    opts.onFailure5 = onSubscribeFailure;
    opts.context = client;
    if ((rc = MQTTAsync_subscribe(client, PONG_TOPIC, QOS, &opts)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to start subscribe, return code %d\n", rc);
        finished = 1;
    }

    if ((rc = MQTTAsync_setCallbacks(client, client, NULL, msgarrvd, NULL)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to set callbacks, return code %d\n", rc);
        MQTTAsync_destroy(&client);
        exit(EXIT_FAILURE);
    }

    while (!subscribed) ;

    pthread_mutex_init(&ping_info.lock, NULL);
    pthread_cond_init(&ping_info.cond, NULL);
    ping_info.seq_num = 0;


    struct latency_data data;
    data.payload = (u_int8_t*) calloc(sizeof(u_int8_t), payload);
    data.len = payload;

    for(size_t i = 0; i<payload; i++){
        data.payload[i] = (u_int8_t) 0xFF;
    }

    while (1) {
        usleep((useconds_t)(interveal * 1000000));
        MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
        clock_gettime(CLOCK_MONOTONIC_RAW, &start);
        memcpy((void*)&data.ts, (void *) &start, sizeof(struct timespec));
        

        size_t wirelen = serialize_latency_data(data, &payload_buff); //serialize_data(data, payload_buff);
        pubmsg.payload = payload_buff;
        pubmsg.payloadlen = (int) wirelen;
        pubmsg.qos = QOS;
        pubmsg.retained = 0;


        if ((rc = MQTTAsync_sendMessage(client, PING_TOPIC, &pubmsg, NULL)) == MQTTASYNC_SUCCESS)
        {
            // clock_gettime(CLOCK_MONOTONIC_RAW, &start);

            // // The message was sent, we should wait for the reply
            // pthread_mutex_lock(&ping_info.lock);
            // pthread_cond_wait(&ping_info.cond, &ping_info.lock);
            // clock_gettime(CLOCK_MONOTONIC_RAW, &end);
            // pthread_mutex_unlock(&ping_info.lock);

            // received = 0;
            // u_int64_t elapsed = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;

            // if (name && scenario) {
            //     printf("mqtt,%s,latency,%s,%ld,%f,%lu,%ld\n", scenario, name, payload, interveal, ping_info.seq_num, elapsed / 2);
            // } else {
            //     // <protocol>,[latency|througput],[interval|payload],<value>,<unit>
            //     printf("mqtt,latency,%.10f,%ld,us\n", interveal, elapsed / 2);
            // }
            // fflush(stdout);
            // seq_num += 1;
        }

    }

    MQTTAsync_destroy(&client);
    return rc;
}


