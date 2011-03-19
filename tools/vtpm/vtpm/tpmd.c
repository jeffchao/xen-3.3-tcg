/* Software-Based Trusted Platform Module (TPM) Emulator for Linux
 * Copyright (C) 2005 INTEL Corp
 *
 * This module is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>

#include "tpm_emulator.h"
#include "vtpm_manager.h"

#ifdef VTPM_MULTI_VM
 #define DEV_BE "/dev/vtpm"
#else
 #define PVM_RX_FIFO_D "/var/vtpm/fifos/tpm_cmd_to_%d.fifo"
 #define PVM_TX_FIFO "/var/vtpm/fifos/tpm_rsp_from_all.fifo"

 #define HVM_RX_FIFO_D "/var/vtpm/socks/%d.socket"
#endif

 int dmi_id;
						
#define BUFFER_SIZE 2048

static int devurandom=0;
	  
void get_random_bytes(void *buf, int nbytes) {
  
  if (devurandom == 0) {
    devurandom = open("/dev/urandom", O_RDONLY);
  }

  if (read(devurandom, buf, nbytes) != nbytes) {
      error("Can't get random number.\n");
      exit(-1);
  }
}

uint64_t tpm_get_ticks(void)
{
  //struct timeval tv;
  //int gettimeofday(&tv, struct timezone *tz);
  return 0;
}

int main(int argc, char **argv)
{
  uint8_t type, in[BUFFER_SIZE], *out, *addressed_out;
  char *vtpm_rx_file=NULL;
  uint32_t out_size;
  int in_size, written;
  int i, guest_id=-1;

#ifndef VTPM_MULTI_VM
  int sockfd = -1;
  struct sockaddr_un addr;
  struct sockaddr_un client_addr;
  unsigned int client_length;

#endif
 
  int vtpm_tx_fh=-1, vtpm_rx_fh=-1;
#ifdef VTPM_MULTI_VM
  if (argc < 2) {
    error("Usage: tpmd clear|save|deactivated\n" );
#else
  if (argc < 4) {
    error("Usage: tpmd clear|save|deactivated pvm|hvm vtpmid\n" );
#endif
	  return -1;
  }

#ifndef VTPM_MULTI_VM
  /* setup type of vm */
  if (!strcmp(argv[2], "pvm")) {
    type = VTPM_TYPE_PVM; // Get commands from vTPM Manager through fifo
  } else if (!strcmp(argv[2], "hvm")) {
    type = VTPM_TYPE_HVM; // Get commands from qemu via socket
  } else {
    error("invalid vTPM type '%s'.\n", argv[2]);
  }

  dmi_id = atoi(argv[3]);

  if (type == VTPM_TYPE_PVM) {
    vtpm_rx_file = malloc(10 + strlen(PVM_RX_FIFO_D));
    sprintf(vtpm_rx_file, PVM_RX_FIFO_D, (uint32_t) dmi_id);
  } else {
    vtpm_rx_file = malloc(10 + strlen(HVM_RX_FIFO_D));
    sprintf(vtpm_rx_file, HVM_RX_FIFO_D, (uint32_t) dmi_id);

    if ( (sockfd = socket(PF_UNIX,SOCK_STREAM,0)) < 0) {
          error("Unable to create socket. errno = %d\n", errno);
      exit (-1);
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path,vtpm_rx_file );
    unlink(addr.sun_path);
  }
#endif

#ifdef VTPM_MULTI_VM
  info("Initializing tpm state: %s\n", argv[1]);
#else
  info("Initializing tpm state: %s, type: %s, id: %d\n", argv[1], argv[2], dmi_id);
#endif

  /* initialize TPM emulator */
  if (!strcmp(argv[1], "clear")) {
    tpm_emulator_init(1);
  } else if (!strcmp(argv[1], "save")) {
    tpm_emulator_init(2);
  } else if (!strcmp(argv[1], "deactivated")) {
    tpm_emulator_init(3);
  } else {
    error("invalid startup mode '%s'; must be 'clear', "
      "'save' (default) or 'deactivated", argv[1]);
    return -1;
  }
  
  while (1) {
abort_command:
    if (vtpm_rx_fh < 0) {
#ifdef VTPM_MUTLI_VM
      vtpm_rx_fh = open(DEV_BE, O_RDWR);
#else
      if (type == VTPM_TYPE_PVM) {
        vtpm_rx_fh = open(vtpm_rx_file, O_RDONLY);
      } else {
        if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
          error("Unable to bind(). errno = %d\n", errno);
          exit (-1);
        }

        if (listen(sockfd, 10) <0) {
          error("Unable to listen(). errno = %d\n", errno);
          exit (-1);
        }

        memset(&client_addr, 0, sizeof(client_addr));
        client_length = sizeof(client_addr);

        vtpm_rx_fh = vtpm_tx_fh = accept(sockfd, (struct sockaddr *)&client_addr, &client_length);
      }
#endif
    }
    
    if (vtpm_rx_fh < 0) {
      error("Failed to open devices to listen to guest.\n");
      return -1;
    }
    
    in_size = read(vtpm_rx_fh, in, BUFFER_SIZE);
    if (in_size < 6) { // Magic size of minium TPM command
      info("Recv incomplete command of %d bytes.", in_size);
      if (in_size <= 0) {
          close(vtpm_rx_fh);
          vtpm_rx_fh = -1;
          goto abort_command;
      }
    } else { 
      debug_nostop("Recv[%d]: 0x", in_size);
      for (i=0; i< in_size; i++) 
        debug_more("%x ", in[i]);
      debug_more("\n");
    }

    if (guest_id == -1) {
        guest_id = *((uint32_t *) in);
    } else {
        if (guest_id != *((uint32_t *) in) ) {
            error("WARNING: More than one guest attached\n");
        }
    }

    if (vtpm_tx_fh < 0) {
#ifdef VTPM_MUTLI_VM
      vtpm_tx_fh = open(DEV_BE, O_RDWR);
      vtpm_rx_fh = vtpm_tx_fh;
#else
      if (type == VTPM_TYPE_PVM) {
        vtpm_tx_fh = open(PVM_TX_FIFO, O_WRONLY);
      } // No need to open the other direction for HVM
#endif
    }

    if (vtpm_tx_fh < 0) {
      error("Failed to open devices to respond to guest.\n");
      return -1;
    }

    // Handle the command, but skip the domain id header    
    if (tpm_handle_command(in + sizeof(uint32_t), in_size - sizeof(uint32_t), &out, &out_size) != 0) { 
      error("Handler Failed.\n");
    }

    addressed_out = (uint8_t *) tpm_malloc(sizeof(uint32_t) + out_size);
    *(uint32_t *) addressed_out = *(uint32_t *) in;
    memcpy(addressed_out + sizeof(uint32_t), out, out_size);

    written = write(vtpm_tx_fh, addressed_out, out_size + sizeof(uint32_t));

    if (written != out_size + sizeof(uint32_t)) {
      error("Part of response not written %d/%d.\n", written, out_size);
    } else {
      debug_nostop("Sent[%Zu]: ", out_size + sizeof(uint32_t));
      for (i=0; i< out_size+ sizeof(uint32_t); i++)
        debug_more("%x ", addressed_out[i]);
      debug_more("\n");
    }
    tpm_free(out);
    tpm_free(addressed_out);

  } // loop

  tpm_emulator_shutdown();

  close(vtpm_tx_fh);
#ifndef VTPM_MUTLI_VM
  close(vtpm_rx_fh);
  free (vtpm_rx_file);
#endif

}
