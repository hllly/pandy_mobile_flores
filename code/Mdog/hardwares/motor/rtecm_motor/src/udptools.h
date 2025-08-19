#ifndef UDPTOOLS_H
#define UDPTOOLS_H

// Conditionally define 'extern "C"' for C++
#ifdef __cplusplus
extern "C" {
#endif
#include <time.h>
#include <sys/mman.h>
#include <limits.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>

#define EC_PACKED(Bytes) __attribute__((aligned(Bytes), packed))


typedef struct 
{
int fd;
struct sockaddr_in saddr;
struct timespec time_start, time_end;
long long int duration, sum ;
long long int cnt , min , max , avg ;
} EC_PACKED(1) sockfdc;

void updinit(sockfdc *fdc,const char *__cp, uint16_t port)
{
    // fdc->time_start = {0, 0};
    //  fdc->time_end = {0, 0};
     fdc->sum = 0;
     fdc->cnt = 0;
     fdc->min = 500000; 
     fdc->max = 0; 
     fdc->avg = 0;


    fdc->fd = socket(AF_INET, SOCK_DGRAM, 0);
    assert(fdc->fd  != -1);
    memset(&fdc->saddr, 0, sizeof(fdc->saddr));
    fdc->saddr.sin_family = AF_INET;
    fdc->saddr.sin_port = htons(port);
    fdc->saddr.sin_addr.s_addr = inet_addr(__cp);
}

void updSendArray(sockfdc *fdc,float *txData, uint8_t num)
{
    float data[num+1];

    for (size_t i = 0; i < num; i++)
    {
        data[i]=txData[i];
    }
    

    uint8_t tail[4]= {0x00, 0x00, 0x80, 0x7f};
    memcpy(&data[num],tail, 4);
    sendto(fdc->fd, data, sizeof(data), 0, (struct sockaddr*)&fdc->saddr, sizeof(fdc->saddr));


}

void updSenDuration(sockfdc *fdc)
{
    float data[3];
    data[0] = (float)fdc->time_end.tv_sec;  // Modify with the correct variable or remove this line if not needed
    clock_gettime(CLOCK_MONOTONIC, &fdc->time_end);
    fdc->duration = (fdc->time_end.tv_sec - fdc->time_start.tv_sec) * (int)1e9 + (fdc->time_end.tv_nsec - fdc->time_start.tv_nsec);

    fdc->cnt++;
    if (fdc->duration < fdc->min) {
        fdc->min = fdc->duration;
    }
    if (fdc->duration > fdc->max) {
        fdc->max = fdc->duration;
    }
    fdc->sum += fdc->duration;
    fdc->avg = fdc->sum / fdc->cnt;
    fdc->time_start = fdc->time_end;
    data[1] = (float)fdc->duration;
    uint8_t tail[4]= {0x00, 0x00, 0x80, 0x7f};
    memcpy(&data[2],tail, 4);
    sendto(fdc->fd, data, sizeof(data), 0, (struct sockaddr*)&fdc->saddr, sizeof(fdc->saddr));


}

// Conditionally undefine 'extern "C"' for C++
#ifdef __cplusplus
}
#endif

#endif  // UDPTOOLS_H




