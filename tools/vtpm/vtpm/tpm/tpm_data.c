/* Software-Based Trusted Platform Module (TPM) Emulator for Linux
 * Copyright (C) 2004 Mario Strasser <mast@gmx.net>,
 *                    Swiss Federal Institute of Technology (ETH) Zurich
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
 * $Id: tpm_data.c 98 2006-05-07 14:16:29Z hstamer $
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "tpm_emulator.h"
#include "tpm_structures.h"
#include "tpm_marshalling.h"
#include "vtpm_manager.h"

TPM_DATA tpmData;

BOOL tpm_get_physical_presence(void)
{
  return (tpmData.stclear.flags.physicalPresence || TRUE);
}

static inline void init_pcr_attr(int pcr, BOOL reset, BYTE rl, BYTE el)
{
  int i;
  tpmData.permanent.data.pcrAttrib[pcr].pcrReset = reset;
  for (i = 0; i < TPM_NUM_LOCALITY; i++) {
    tpmData.permanent.data.pcrAttrib[pcr].pcrResetLocal[i] = (rl & (1 << i));
    tpmData.permanent.data.pcrAttrib[pcr].pcrExtendLocal[i] = (el & (1 << i));
  }
}

void tpm_init_data(void)
{
  /* endorsement key */
#ifndef TPM_GENERATE_EK
  uint8_t ek_n[] =  "\xa8\xdb\xa9\x42\xa8\xf3\xb8\x06\x85\x90\x76\x93\xad\xf7"
    "\x74\xec\x3f\xd3\x3d\x9d\xe8\x2e\xff\x15\xed\x0e\xce\x5f\x93"
    "\x92\xeb\xd1\x96\x2b\x72\x18\x81\x79\x12\x9d\x9c\x40\xd7\x1a"
    "\x21\xda\x5f\x56\xe0\xc9\x48\x31\xdd\x96\xdc\xbb\x45\xc6\x8e"
    "\xad\x58\x23\xcb\xbe\xbb\x13\x2d\x6b\x86\xc5\x57\xf5\xdd\x48"
    "\xc1\x3d\xcd\x4d\xda\x81\xc4\x43\x17\xaa\x05\x40\x33\x62\x0a"
    "\x59\xdb\x28\xcd\xb5\x08\x31\xbb\x06\xf5\xf7\x71\xae\x21\xa8"
    "\xf2\x2f\x0e\x17\x80\x5d\x9c\xdf\xaa\xe9\x89\x09\x54\x65\x2b"
    "\x46\xfb\x9d\xb2\x00\x70\x63\x0d\x9a\x6d\x3d\x5e\x11\x78\x65"
    "\x90\xe6\x26\xee\x77\xbe\x08\xff\x07\x60\x5a\xcc\xf1\x0a\xbd"
    "\x44\x92\x6b\xca\xb6\xce\x66\xf9\x93\x40\xae\xf3\x3e\x53\x02"
    "\x3c\xa6\x81\xb3\xbe\xad\x6e\x6c\xa6\xf0\xeb\xdf\xe9\xa2\x83"
    "\x36\x0e\x52\x0d\x64\x17\xd9\xff\xa1\x74\x7c\x2b\xbc\x6a\xcc"
    "\xe5\x4e\xb4\x52\xd9\xec\x43\xbd\x26\x6a\x2b\x19\x19\x6e\x97"
    "\xb8\x1d\x9f\x7b\xe7\x32\x2d\xdd\x7c\x51\xc8\xe4\xf3\x02\xd4"
    "\x7c\x90\x44\xa0\x33\x72\x81\x75\xa9\x16\x27\x5c\x00\x1d\x07"
    "\x81\xd4\xf7\xac\xcb\xfe\xd6\x60\x03\x6f\x7a\xcc\x00\xd1\xc4"
    "\x85\x37";
  uint8_t ek_e[] = "\x01\x00\x01";
  uint8_t ek_p[] = "\xd7\xea\x61\x15\x8b\xa3\x71\xdf\xa8\x74\x77\xca\x88\x95"
    "\xd0\x76\x17\x43\x2c\xf6\x23\x27\x44\xb9\x0e\x18\x35\x7e\xe4"
    "\xc3\xcb\x13\x6e\xfc\x38\x02\x1e\x77\x26\x40\x9d\x17\xb2\x39"
    "\x9c\x7f\x5f\x98\xe6\xf2\x55\x0c\x12\x05\x4c\xb3\x51\xae\x29"
    "\xe7\xcd\xce\x41\x0b\x28\x4d\x97\x13\x4b\x60\xc8\xd8\x70\x81"
    "\xf9\x1c\x12\x44\xdf\x53\x0a\x87\x9d\x33\x92\x4a\x34\x69\xf0"
    "\x70\x5e\x1b\x5d\x65\xc7\x84\x90\xa2\x62\xdf\x83\x14\x10\x69"
    "\xe2\xa7\x18\x43\xd7\x1f\x60\xc9\x03\x8f\xd6\xa4\xce\xb2\x9d"
    "\x40\x37\x70\x17\x4c\xe3\x69\xd4\x59";
  uint8_t ek_q[] = "\xc8\x34\xd2\xd0\x7c\xfa\xdc\x68\xe2\x72\xd7\x92\xe2\x50"
    "\x93\xfc\xbb\x72\x55\x4d\x6b\x7a\x0c\x0b\xcf\x87\x66\x1f\x81"
    "\x71\xf3\x50\xcb\xaa\xe6\x43\x7e\xbe\x11\xc4\xec\x00\x53\xf4"
    "\x78\x13\x2b\x59\x26\x4a\x9f\x91\x61\x8f\xa7\x07\x64\x11\x5a"
    "\xf4\xaf\x9c\x9b\x5a\x5d\x69\x20\x17\x55\x74\xba\xd8\xe4\x59"
    "\x39\x1a\x0a\x7b\x4a\x30\xf0\xc8\x7f\xd9\xaf\x72\xc5\xb6\x71"
    "\xd1\xc0\x8b\x5b\xa2\x2e\xa7\x15\xca\x50\x75\x10\x48\x9c\x2b"
    "\x18\xb9\x67\x8f\x5d\x64\xc3\x28\x9f\x2f\x16\x2f\x08\xda\x47"
    "\xec\x86\x43\x0c\x80\x99\x07\x34\x0f";
#endif

  int i;
  /* reset all data to NULL, FALSE or 0 */
  memset(&tpmData, 0, sizeof(tpmData));
  tpmData.permanent.data.tag = TPM_TAG_PERMANENT_DATA;
  /* set permanent flags */
  tpmData.permanent.flags.tag = TPM_TAG_PERMANENT_FLAGS;
  tpmData.permanent.flags.disable = FALSE;
  tpmData.permanent.flags.deactivated = FALSE;
  tpmData.permanent.flags.ownership = TRUE;
  tpmData.permanent.flags.readPubek = TRUE;
  tpmData.permanent.flags.allowMaintenance = TRUE;
  tpmData.permanent.flags.enableRevokeEK = TRUE;
  /* set TPM vision */
  tpmData.permanent.data.version.major = 1;
  tpmData.permanent.data.version.minor = 2;
  tpmData.permanent.data.version.revMajor = VERSION_MAJOR;
  tpmData.permanent.data.version.revMinor = VERSION_MINOR;
  /* setup PCR attributes */
  for (i = 0; i < min(16, TPM_NUM_PCR); i++) {
    init_pcr_attr(i, FALSE, 0x00, 0x1f);
  }
  if (TPM_NUM_PCR >= 24) {
    init_pcr_attr(16, TRUE, 0x1f, 0x1f);
    init_pcr_attr(17, TRUE, 0x10, 0x1c);
    init_pcr_attr(18, TRUE, 0x10, 0x1c);
    init_pcr_attr(19, TRUE, 0x10, 0x0c);
    init_pcr_attr(20, TRUE, 0x14, 0x0e);
    init_pcr_attr(21, TRUE, 0x04, 0x04);
    init_pcr_attr(22, TRUE, 0x04, 0x04);
    init_pcr_attr(23, TRUE, 0x1f, 0x1f);
  }
  for (i = 24; i < TPM_NUM_PCR; i++) {
    init_pcr_attr(i, TRUE, 0x00, 0x00);
  }
  /* set tick type */
/* removed since v1.2 rev 94
  tpmData.permanent.data.tickType = TICK_INC;
*/
#ifdef TPM_GENERATE_EK
  /* generate a new endorsement key */
  rsa_generate_key(&tpmData.permanent.data.endorsementKey, 2048);
#else
  /* setup endorsement key */
  rsa_import_key(&tpmData.permanent.data.endorsementKey, 
    RSA_MSB_FIRST, ek_n, 256, ek_e, 3, ek_p, ek_q);
#endif
#ifdef TPM_GENERATE_SEED_DAA
  /* generate the DAA seed (cf. [TPM_Part2], v1.2 rev 94, Section 7.4) */
  tpm_get_random_bytes(tpmData.permanent.data.tpmDAASeed.digest, 
    sizeof(tpmData.permanent.data.tpmDAASeed.digest));
#else
  /* setup DAA seed */
  memcpy(tpmData.permanent.data.tpmDAASeed.digest, 
    "\x77\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    "\x00\x00\x00\x77", 20);
#endif

  memcpy(tpmData.permanent.data.ekReset.nonce, "\xde\xad\xbe\xef", 4);
}

void tpm_release_data(void)
{
  int i;
  /* release the EK, SRK as well as all other rsa keys */
  if (tpmData.permanent.data.endorsementKey.size > 0)
    rsa_release_private_key(&tpmData.permanent.data.endorsementKey);
  if (tpmData.permanent.data.srk.valid)
    rsa_release_private_key(&tpmData.permanent.data.srk.key);
  for (i = 0; i < TPM_MAX_KEYS; i++)
    if (tpmData.permanent.data.keys[i].valid)
      rsa_release_private_key(&tpmData.permanent.data.keys[i].key);
}

#ifdef TPM_STORE_TO_FILE

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

 static int vtpm_tx_fh=-1, vtpm_rx_fh=-1;

#ifdef VTPM_MUTLI_VM
 #define DEV_FE "/dev/tpm"
#else
 #define VTPM_RX_FIFO_D  "/var/vtpm/fifos/vtpm_rsp_to_%d.fifo"
 #define VTPM_TX_FIFO  "/var/vtpm/fifos/vtpm_cmd_from_all.fifo"

 extern int dmi_id;
 static char *vtpm_rx_name=NULL; 
#endif

static int write_to_file(uint8_t *data, size_t data_length)
{
  int res, out_data_size, in_header_size;
  BYTE *ptr, *out_data, *in_header;
  UINT32 result, len, in_rsp_size;
  UINT16 tag = VTPM_TAG_REQ;
	
  printf("Saving NVM\n");
  if (vtpm_tx_fh < 0) {
#ifdef VTPM_MUTLI_VM
    vtpm_tx_fh = open(DEV_FE, O_RDWR);
#else
	vtpm_tx_fh = open(VTPM_TX_FIFO, O_WRONLY);
#endif
  }

  if (vtpm_tx_fh < 0) {
		return -1;
  }
 
  // Send request to VTPM Manager to encrypt data
#ifdef VTPM_MUTLI_VM
  out_data_size = len = VTPM_COMMAND_HEADER_SIZE_CLT + data_length;
#else
  out_data_size = len = VTPM_COMMAND_HEADER_SIZE_SRV + data_length;
#endif
  
  out_data = ptr = (BYTE *) malloc(len);

  if (ptr == NULL
#ifndef VTPM_MUTLI_VM
      || tpm_marshal_UINT32(&ptr, &len, dmi_id)
#endif
	  || tpm_marshal_UINT16(&ptr, &len, tag)
#ifdef VTPM_MUTLI_VM
	  || tpm_marshal_UINT32(&ptr, &len, out_data_size)
#else
	  || tpm_marshal_UINT32(&ptr, &len, out_data_size - sizeof(uint32_t))
#endif  
	  || tpm_marshal_UINT32(&ptr, &len, VTPM_ORD_SAVENVM)
	  || tpm_marshal_BYTE_ARRAY(&ptr, &len, data, data_length)) {
	free(out_data);
	return -1;
  }
  
  printf("\tSending SaveNVM Command.\n");
  res = write(vtpm_tx_fh, out_data, out_data_size);
  free(out_data);
  if (res != out_data_size) return -1;

  if (vtpm_rx_fh < 0) {
#ifdef VTPM_MUTLI_VM
    vtpm_rx_fh = vtpm_tx_fh
#else
    if (vtpm_rx_name == NULL) {
      vtpm_rx_name = malloc(10 + strlen(VTPM_RX_FIFO_D));
      sprintf(vtpm_rx_name, VTPM_RX_FIFO_D, (uint32_t) dmi_id);
    }
	vtpm_rx_fh = open(vtpm_rx_name, O_RDONLY);
#endif
  }

  if (vtpm_rx_fh < 0) {
		return -1;
  }
  
  // Read Header of response so we can get the size & status
#ifdef VTPM_MUTLI_VM
  in_header_size = len = VTPM_COMMAND_HEADER_SIZE_CLT;
#else
  in_header_size = len = VTPM_COMMAND_HEADER_SIZE_SRV;
#endif
  in_header = ptr = malloc(in_header_size);
  
  printf("\tReading SaveNVM header.\n");
  res = read(vtpm_rx_fh, in_header, in_header_size);

  if ( (res != in_header_size)
#ifndef VTPM_MUTLI_VM
       || tpm_unmarshal_UINT32(&ptr, &len, (UINT32*)&dmi_id)
#endif
	   || tpm_unmarshal_UINT16(&ptr, &len, &tag)
	   || tpm_unmarshal_UINT32(&ptr, &len, &in_rsp_size)
	   || tpm_unmarshal_UINT32(&ptr, &len, &result) ) {
	  free(in_header);
	  return -1;
  }
  free(in_header);
  
  if (result != VTPM_SUCCESS) {
      return -1;  
  }

#ifdef VTPM_MUTLI_VM
  close(vtpm_tx_fh); close(vtpm_rx_fh);
#endif
	  
  printf("\tFinishing up SaveNVM\n");
  return (0);
}

static int read_from_file(uint8_t **data, size_t *data_length)
{
  int res, out_data_size, in_header_size;
  uint8_t *ptr, *out_data, *in_header;
  UINT16 tag = VTPM_TAG_REQ;
  UINT32 len, in_rsp_size, result;
#ifdef VTPM_MUTLI_VM
	int vtpm_rx_fh, vtpm_tx_fh;
#endif
	
  printf("Loading NVM.\n");
  if (vtpm_tx_fh < 0) {
#ifdef VTPM_MUTLI_VM
    vtpm_tx_fh = open(DEV_FE, O_RDWR);
#else
	vtpm_tx_fh = open(VTPM_TX_FIFO, O_WRONLY);
#endif
  }

  if (vtpm_tx_fh < 0) {
		return -1;
  }
 
  // Send request to VTPM Manager to encrypt data
#ifdef VTPM_MUTLI_VM
  out_data_size = len = VTPM_COMMAND_HEADER_SIZE_CLT;
#else
  out_data_size = len = VTPM_COMMAND_HEADER_SIZE_SRV;
#endif
  out_data = ptr = (BYTE *) malloc(len);

  if (ptr == NULL
#ifndef VTPM_MUTLI_VM
      || tpm_marshal_UINT32(&ptr, &len, dmi_id)
#endif  
      || tpm_marshal_UINT16(&ptr, &len, tag)
#ifdef VTPM_MUTLI_VM
      || tpm_marshal_UINT32(&ptr, &len, out_data_size)
#else
      || tpm_marshal_UINT32(&ptr, &len, out_data_size - sizeof(uint32_t))
#endif
      || tpm_marshal_UINT32(&ptr, &len, VTPM_ORD_LOADNVM)) {
    free(out_data);
    return -1;
  }

  printf("\tSending LoadNVM command\n");
  res = write(vtpm_tx_fh, out_data, out_data_size);
  free(out_data);
  if (res != out_data_size) return -1;

    if (vtpm_rx_fh < 0) {
#ifdef VTPM_MUTLI_VM
    vtpm_rx_fh = vtpm_tx_fh;
#else
    if (vtpm_rx_name == NULL) {
      vtpm_rx_name = malloc(10 + strlen(VTPM_RX_FIFO_D));
      sprintf(vtpm_rx_name, VTPM_RX_FIFO_D, (uint32_t) dmi_id);
    }
	vtpm_rx_fh = open(vtpm_rx_name, O_RDONLY);
#endif
  }

  if (vtpm_rx_fh < 0) {
		return -1;
  }
  
  // Read Header of response so we can get the size & status
#ifdef VTPM_MUTLI_VM
  in_header_size = len = VTPM_COMMAND_HEADER_SIZE_CLT;
#else
  in_header_size = len = VTPM_COMMAND_HEADER_SIZE_SRV;
#endif
  in_header = ptr = malloc(in_header_size);
  
  printf("\tReading LoadNVM header\n");
  res = read(vtpm_rx_fh, in_header, in_header_size);

  if ( (res != in_header_size)
#ifndef VTPM_MUTLI_VM
       || tpm_unmarshal_UINT32(&ptr, &len, (UINT32*)&dmi_id)
#endif
       || tpm_unmarshal_UINT16(&ptr, &len, &tag)
       || tpm_unmarshal_UINT32(&ptr, &len, &in_rsp_size)
       || tpm_unmarshal_UINT32(&ptr, &len, &result) ) {
      free(in_header);
      return -1;
  }
  free(in_header);
  
  if (result != VTPM_SUCCESS) {
      return -1;  
  }

  // Read Encrypted data from VTPM Manager
  *data_length = in_rsp_size - VTPM_COMMAND_HEADER_SIZE_CLT;
  *data = (uint8_t *) malloc(*data_length);

  printf("\tReading clear data from LoadNVM.\n");
  res = read(vtpm_rx_fh, *data, *data_length);
#ifdef VTPM_MUTLI_VM
  close(vtpm_rx_fh);close(vtpm_tx_fh);
#endif 
	
  printf("\tReturing from loading NVM\n");
  if (res != *data_length) {
      free(*data);
      return -1;
  } else {
      return 0;
  }

}

#else

static int write_to_file(uint8_t *data, size_t data_length)
{
  info("TPM_STORE_TO_FILE disabled, no data written");
  return 0;
}

static int read_from_file(uint8_t **data, size_t *data_length)
{
  info("TPM_STORE_TO_FILE disabled, no data read");
  return 0;
}

#endif /* TPM_STORE_TO_FILE */

int tpm_store_permanent_data(void)
{
  uint8_t *buf, *ptr;
  UINT32 buf_length, len;

  /* marshal data */
  buf_length = len = 4 + sizeof_TPM_STCLEAR_FLAGS(tpmData.stclear.flags)
    + sizeof_TPM_PERMANENT_FLAGS(tpmData.permanent.flags) 
    + sizeof_TPM_STANY_FLAGS(tpmData.stany.flags) + 2
    + sizeof_TPM_STCLEAR_DATA(tpmData.stclear.data) 
    + sizeof_TPM_PERMANENT_DATA(tpmData.permanent.data)
    + sizeof_TPM_STANY_DATA(tpmData.stany.data);
  buf = ptr = tpm_malloc(buf_length);
  if (buf == NULL
      || tpm_marshal_TPM_VERSION(&ptr, &len, &tpmData.permanent.data.version)
      || tpm_marshal_TPM_STCLEAR_FLAGS(&ptr, &len, &tpmData.stclear.flags)
      || tpm_marshal_TPM_PERMANENT_FLAGS(&ptr, &len, &tpmData.permanent.flags)
      || tpm_marshal_TPM_STANY_FLAGS(&ptr, &len, &tpmData.stany.flags)
      || tpm_marshal_BOOL(&ptr, &len, tpmData.permanent.flags.selfTestSucceeded)
      || tpm_marshal_BOOL(&ptr, &len, tpmData.permanent.flags.owned)
      || tpm_marshal_TPM_STCLEAR_DATA(&ptr, &len, &tpmData.stclear.data)
      || tpm_marshal_TPM_PERMANENT_DATA(&ptr, &len, &tpmData.permanent.data)
      || tpm_marshal_TPM_STANY_DATA(&ptr, &len, &tpmData.stany.data)) {
    tpm_free(buf);
    return -1;
  }

  if (write_to_file(buf, buf_length - len)) {
    tpm_free(buf);
    return -1; 
  }
  tpm_free(buf);
  return 0;
}

int tpm_restore_permanent_data(void)
{
  uint8_t *buf, *ptr;
  size_t buf_length;
  UINT32 len;
  TPM_VERSION ver;

  /* read data */
  if (read_from_file(&buf, &buf_length)) return -1;
  ptr = buf;
  len = (uint32_t) buf_length;
  /* unmarshal data */
  if (tpm_unmarshal_TPM_VERSION(&ptr, &len, &ver)
      || memcmp(&ver, &tpmData.permanent.data.version, sizeof(TPM_VERSION))
      || tpm_unmarshal_TPM_STCLEAR_FLAGS(&ptr, &len, &tpmData.stclear.flags)
      || tpm_unmarshal_TPM_PERMANENT_FLAGS(&ptr, &len, &tpmData.permanent.flags)
      || tpm_unmarshal_TPM_STANY_FLAGS(&ptr, &len, &tpmData.stany.flags)
      || tpm_unmarshal_BOOL(&ptr, &len, &tpmData.permanent.flags.selfTestSucceeded)
      || tpm_unmarshal_BOOL(&ptr, &len, &tpmData.permanent.flags.owned)
      || tpm_unmarshal_TPM_STCLEAR_DATA(&ptr, &len, &tpmData.stclear.data)
      || tpm_unmarshal_TPM_PERMANENT_DATA(&ptr, &len, &tpmData.permanent.data)
      || tpm_unmarshal_TPM_STANY_DATA(&ptr, &len, &tpmData.stany.data)) {
    tpm_free(buf);
    return -1;
  }

  tpm_free(buf);
  return 0;
}

int tpm_erase_permanent_data(void)
{
  int res = write_to_file((uint8_t *) "", 0);
  return res;
}
