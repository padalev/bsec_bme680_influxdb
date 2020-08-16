/* Copyright (C) 2017 alexh.name */
/* I2C code by twartzek 2017 */

/*
 * Read the BME680 sensor with the BSEC library by running an endless loop in
 * the bsec_iot_loop() function under Linux.
 *
 */

/*#define _POSIX_C_SOURCE 200809L*/
#define _XOPEN_SOURCE 700

/* header files */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include "bsec_integration.h"
#include <curl/curl.h>
#include <memory.h>
#include <math.h>


/* definitions */

#define DESTZONE "TZ=Europe/Berlin"
double temp_offset = 5.0;
#define sample_rate_mode (BSEC_SAMPLE_RATE_LP)
char* database = "mydb";
char* measurement = "meas1";
float elevation = 50.0;


int g_i2cFid; // I2C Linux device handle
int i2c_address = BME680_I2C_ADDR_PRIMARY;
char *filename_state = "/etc/bsec_bme680/bsec_iaq.state";
char *filename_config = "/etc/bsec_bme680/bsec_iaq.config";

/* functions */

// open the Linux device
void i2cOpen()
{
  g_i2cFid = open("/dev/i2c-1", O_RDWR);
  if (g_i2cFid < 0) {
    perror("i2cOpen");
    exit(1);
  }
}

// close the Linux device
void i2cClose()
{
  close(g_i2cFid);
}

// set the I2C slave address for all subsequent I2C device transfers
void i2cSetAddress(int address)
{
  if (ioctl(g_i2cFid, I2C_SLAVE, address) < 0) {
    perror("i2cSetAddress");
    exit(1);
  }
}

/*
 * Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr,
                 uint16_t data_len)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

  uint8_t reg[16];
  reg[0]=reg_addr;
  int i;

  for (i=1; i<data_len+1; i++)
    reg[i] = reg_data_ptr[i-1];

  if (write(g_i2cFid, reg, data_len+1) != data_len+1) {
    perror("user_i2c_write");
    rslt = 1;
    exit(1);
  }

  return rslt;
}

/*
 * Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store
 *                                  the read data
 * param[in]        data_len        number of bytes to be read
 *
 * return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr,
                uint16_t data_len)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

  uint8_t reg[1];
  reg[0]=reg_addr;

  if (write(g_i2cFid, reg, 1) != 1) {
    perror("user_i2c_read_reg");
    rslt = 1;
  }

  if (read(g_i2cFid, reg_data_ptr, data_len) != data_len) {
    perror("user_i2c_read_data");
    rslt = 1;
  }

  return rslt;
}

/*
 * System specific implementation of sleep function
 *
 * param[in]       t_ms    time in milliseconds
 *
 * return          none
 */
void _sleep(uint32_t t_ms)
{
  struct timespec ts;
  ts.tv_sec = 0;
  /* mod because nsec must be in the range 0 to 999999999 */
  ts.tv_nsec = (t_ms % 1000) * 1000000L;
  nanosleep(&ts, NULL);
}

/*
 * Capture the system time in microseconds
 *
 * return          system_current_time    system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
  struct timespec spec;
  //clock_gettime(CLOCK_REALTIME, &spec);
  /* MONOTONIC in favor of REALTIME to avoid interference by time sync. */
  clock_gettime(CLOCK_MONOTONIC, &spec);

  int64_t system_current_time_ns = (int64_t)(spec.tv_sec) * (int64_t)1000000000
                                   + (int64_t)(spec.tv_nsec);
  int64_t system_current_time_us = system_current_time_ns / 1000;

  return system_current_time_us;
}

/*
 * Capture the system time in nanoseconds for influxdb entries.
 *
 * return          system_current_time    system timestamp in microseconds
 */
int64_t get_timestamp_ns()
{
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  /* Better use REALTIME in this case so that the entries have the correct absolute timestamp. This can jump however when the system clock is changed. */
  //clock_gettime(CLOCK_MONOTONIC, &spec);

  int64_t system_current_time_ns = (int64_t)(spec.tv_sec) * (int64_t)1000000000
                                   + (int64_t)(spec.tv_nsec);

  return system_current_time_ns;
}

/*
 * Da a curl POST request to the influxdb http socket, that contains the BSEC data.
 *
 * return          none
 */
void influx_write(char *influxline)
{
  CURL *curl;
  CURLcode res;

  /* In windows, this will init the winsock stuff */
  curl_global_init(CURL_GLOBAL_ALL);

  /* get a curl handle */
  curl = curl_easy_init();
  if(curl) {
    /* Build url for curl request */

    char* curlfront = "http://localhost:8086/api/v2/write?bucket=";
    char* curlback = "&precision=ns";

    char* curlstring = (char*)malloc(1000);
    strcpy(curlstring, curlfront);
    strcat(curlstring, database);
    strcat(curlstring, curlback);

    /* Set url and content of curl POST to influxdb */

    curl_easy_setopt(curl, CURLOPT_URL, curlstring);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, influxline);


    /* Perform the request, res will get the return code */
    res = curl_easy_perform(curl);
    /* Check for errors */
    if(res != CURLE_OK)
      fprintf(stderr, "curl_easy_perform() failed: %s\n",
              curl_easy_strerror(res));

    /* always cleanup */
    curl_easy_cleanup(curl);
  }
  curl_global_cleanup();
}

/*
 * Handling of the ready outputs
 *
 * param[in]       timestamp       time in microseconds
 * param[in]       iaq             IAQ signal
 * param[in]       iaq_accuracy    accuracy of IAQ signal
 * param[in]       temperature     temperature signal
 * param[in]       humidity        humidity signal
 * param[in]       pressure        pressure signal
 * param[in]       raw_temperature raw temperature signal
 * param[in]       raw_humidity    raw humidity signal
 * param[in]       gas             raw gas sensor signal
 * param[in]       bsec_status     value returned by the bsec_do_steps() call
 * param[in]       static_iaq      unscaled indoor-air-quality estimate
 * param[in]       co2_equivalent  CO2 equivalent estimate [ppm]
 * param[in]       breath_voc_equivalent  breath VOC concentration estimate [ppm]
 *
 * return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy,
                  float temperature, float humidity, float pressure,
                  float raw_temperature, float raw_humidity, float gas,
                  bsec_library_return_t bsec_status,
                  float static_iaq, float co2_equivalent,
                  float breath_voc_equivalent)
{
  float P = pressure*expf(elevation/((273.15+temperature-0.1)*29.26));
  /*
   * timestamp for localtime only makes sense if get_timestamp_us() uses
   * CLOCK_REALTIME
   */
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);

  printf("%d-%02d-%02d %02d:%02d:%02d,", tm.tm_year + 1900,tm.tm_mon + 1,
         tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec); /* localtime */
  printf("[IAQ (%d)]: %.2f", iaq_accuracy, iaq);
  printf(",[T degC]: %.2f,[H %%rH]: %.2f,[P0 hPa]: %.2f,[P hPa]: %.2f", temperature,
         humidity,pressure / 100,P / 100);
  printf(",[G Ohms]: %.0f", gas);
  printf(",[S]: %d", bsec_status);
  //printf(",[static IAQ]: %.2f", static_iaq);
  printf(",[eCO2 ppm]: %.15f", co2_equivalent);
  printf(",[bVOCe ppm]: %.25f", breath_voc_equivalent);
  printf("\r\n\n");
  fflush(stdout);

  /*
   * Build the string to send to influxdb according to influxdb line protocol.
   */
  char* influxhost = ",host=raspberrypi";

  char influxiaq[80];
  sprintf(influxiaq, " IAQ=%.2f,", iaq);
  char influxiaqacc[80];
  sprintf(influxiaqacc, "IAQacc=%d,", iaq_accuracy);
  char influxT[80];
  sprintf(influxT, "T=%.2f,", temperature);
  char influxH[80];
  sprintf(influxH, "H=%.2f,", humidity);
  char influxP0[80];
  sprintf(influxP0, "P0=%.2f,", pressure);
  char influxP[80];
  sprintf(influxP, "P=%.2f,", P);
  char influxgas[80];
  sprintf(influxgas, "gas=%.0f,", gas);
  char influxstatus[80];
  sprintf(influxstatus, "status=%d,", bsec_status);
  char influxeco2[80];
  sprintf(influxeco2, "eCO2=%.15f,", co2_equivalent);
  char influxbvoce[80];
  sprintf(influxbvoce, "bVOCe=%.25f", breath_voc_equivalent);
  char unixtimestamp[80];
  sprintf(unixtimestamp, " %lli", get_timestamp_ns());

  /* char* influxstring = (char *) malloc(strlen(measurement) + strlen(influxhost) + strlen(influxiaq) + strlen(influxiaqacc) + strlen(influxT) + strlen(influxH) + strlen(influxP0) + strlen(influxgas) + strlen(influxstatus) + strlen(influxeco2) + strlen(influxbvoce) + strlen(unixtimestamp) ); */
  char* influxstring = (char*)malloc(1000);

  strcpy(influxstring, measurement);
  strcat(influxstring, influxhost);
  strcat(influxstring, influxiaq);
  strcat(influxstring, influxiaqacc);
  strcat(influxstring, influxT);
  strcat(influxstring, influxH);
  strcat(influxstring, influxP0);
  strcat(influxstring, influxP);
  strcat(influxstring, influxgas);
  strcat(influxstring, influxstatus);
  strcat(influxstring, influxeco2);
  strcat(influxstring, influxbvoce);
  strcat(influxstring, unixtimestamp);

  /* send the data over http */
  influx_write(influxstring);
}

/*
 * Load binary file from non-volatile memory into buffer
 *
 * param[in,out]   state_buffer    buffer to hold the loaded data
 * param[in]       n_buffer        size of the allocated buffer
 * param[in]       filename        name of the file on the NVM
 * param[in]       offset          offset in bytes from where to start copying
 *                                  to buffer
 * return          number of bytes copied to buffer or zero on failure
 */
uint32_t binary_load(uint8_t *b_buffer, uint32_t n_buffer, char *filename,
                     uint32_t offset)
{
  int32_t copied_bytes = 0;
  int8_t rslt = 0;

  struct stat fileinfo;
  rslt = stat(filename, &fileinfo);
  if (rslt != 0) {
    fprintf(stderr,"stat'ing binary file %s: ",filename);
    perror("");
    return 0;
  }

  uint32_t filesize = fileinfo.st_size - offset;

  if (filesize > n_buffer) {
    fprintf(stderr,"%s: %d > %d\n", "binary data bigger than buffer", filesize,
            n_buffer);
    return 0;
  } else {
    FILE *file_ptr;
    file_ptr = fopen(filename,"rb");
    if (!file_ptr) {
      perror("fopen");
      return 0;
    }
    fseek(file_ptr,offset,SEEK_SET);
    copied_bytes = fread(b_buffer,sizeof(char),filesize,file_ptr);
    if (copied_bytes == 0) {
      fprintf(stderr,"%s empty\n",filename);
    }
    fclose(file_ptr);
    return copied_bytes;
  }
}

/*
 * Load previous library state from non-volatile memory
 *
 * param[in,out]   state_buffer    buffer to hold the loaded state string
 * param[in]       n_buffer        size of the allocated state buffer
 *
 * return          number of bytes copied to state_buffer or zero on failure
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
  int32_t rslt = 0;
  rslt = binary_load(state_buffer, n_buffer, filename_state, 0);
  return rslt;
}

/*
 * Save library state to non-volatile memory
 *
 * param[in]       state_buffer    buffer holding the state to be stored
 * param[in]       length          length of the state string to be stored
 *
 * return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
  FILE *state_w_ptr;
  state_w_ptr = fopen(filename_state,"wb");
  fwrite(state_buffer,length,1,state_w_ptr);
  fclose(state_w_ptr);
}

/*
 * Load library config from non-volatile memory
 *
 * param[in,out]   config_buffer    buffer to hold the loaded state string
 * param[in]       n_buffer         size of the allocated state buffer
 *
 * return          number of bytes copied to config_buffer or zero on failure
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
  int32_t rslt = 0;
  /*
   * Provided config file is 4 bytes larger than buffer.
   * Apparently skipping the first 4 bytes works fine.
   *
   */
  rslt = binary_load(config_buffer, n_buffer, filename_config, 4);
  return rslt;
}

/* main */

/*
 * Main function which configures BSEC library and then reads and processes
 * the data from sensor based on timer ticks
 *
 * return      result of the processing
 */

int main(int argc, char *argv[])
{
  putenv(DESTZONE); // Switch to destination time zone

  int opt;
  char *ptr;
  // put ':' in the starting of the
  // string so that program can
  //distinguish between '?' and ':'
  while((opt = getopt(argc, argv,"d:m:t:e:")) != -1)
  {
    switch(opt)
    {
      case 'd':
        printf("database: %s\n", optarg);
        database = optarg;
        break;
      case 'm':
        printf("measurement: %s\n", optarg);
        measurement = optarg;
        break;
      case 't':
        printf("temperature offset: %s\n", optarg);
        temp_offset = strtod(optarg, &ptr);
        break;
      case 'e':
        printf("elevation: %s\n", optarg);
        elevation = strtof(optarg, &ptr);
        break;
    }
  }

  i2cOpen();
  i2cSetAddress(i2c_address);

  return_values_init ret;

  ret = bsec_iot_init(sample_rate_mode, temp_offset, bus_write, bus_read,
                      _sleep, state_load, config_load);
  if (ret.bme680_status) {
    /* Could not intialize BME680 */
    return (int)ret.bme680_status;
  } else if (ret.bsec_status) {
    /* Could not intialize BSEC library */
    return (int)ret.bsec_status;
  }

  /* Call to endless loop function which reads and processes data based on
   * sensor settings.
   * State is saved every 10.000 samples, which means every 10.000 * 3 secs
   * = 500 minutes (depending on the config).
   *
   */
  bsec_iot_loop(_sleep, get_timestamp_us, output_ready, state_save, 10000);

  i2cClose();
  return 0;
}

