
#if (XVC_USED == 1)

#  include <assert.h>
#  include <stdio.h>
#  include <stdint.h>
#  include <string.h>

#  include "FreeRTOS.h"
#  include "portmacro.h"
#  include "task.h"

#endif // (XVC_USED == 1)

#  include "pio.h"

#if (XVC_USED == 1)

#  include "lwip/api.h"
#  include "lwip/mem.h"

#endif // (XVC_USED == 1)

#  include "evm300.h"

#  include "xvc_server.h"

#if (XVC_USED == 1)

#  define XVC_COMMAND_STR_GETINFO "getinfo:"
#  define XVC_COMMAND_STR_GETINFO_LEN 8
#  define XVC_COMMAND_STR_SETTCK "settck:"
#  define XVC_COMMAND_STR_SETTCK_LEN 7
#  define XVC_COMMAND_STR_SHIFT "shift:"
#  define XVC_COMMAND_STR_SHIFT_LEN 6

/* The longest command string is “getinfo:”. */
#  define XVC_COMMAND_STR_MAX_LEN 8
/* The shorted command string is “shift:”. */
#  define XVC_COMMAND_STR_MIN_LEN 6

#  define XVC_INFO_STR "xvcServer_v1.0:1024\n"
#  define XVC_INFO_STR_LEN 20

#  define XVC_SHIFT_VECTOR_LENGTH_NOT_SET 0xFFFFFFFF
/* We allocate a buffer of 1024 bytes for the shift vector. However, the vector
is split in halves (one for TMS, one for TDI), so the actual maximum length of
a vector in bytes is 512. However, the length in the reply to the getinfo:
command always is the combined size for both vectors. */
#  define XVC_SHIFT_VECTOR_MAX_LEN_IN_BYTES 512
/* This is the maximum number of bits that can be shifted in a single
operation. */
#  define XVC_SHIFT_VECTOR_MAX_LEN_IN_BITS (XVC_SHIFT_VECTOR_MAX_LEN_IN_BYTES * 8)

/* The shift vectors are the largest thing that we read, so if we size the
buffer for these, everything else will fit in easily. */
#  define XVC_BUFFER_MAX_LEN (XVC_SHIFT_VECTOR_MAX_LEN_IN_BYTES * 2)

/**
 * Enum representing the commands supported by the XVC protocol version 1.0.
 *
 * See https://github.com/Xilinx/XilinxVirtualCable for the full protocol
 * specification.
 */
typedef enum {
  /**
   * Represents the “getinfo:” command.
   */
  XVC_COMMAND_GETINFO,

  /**
   * Indicates that the next command has to be received.
   */
  XVC_COMMAND_NOT_SET,

  /**
   * Represents the “settck:” command.
   */
  XVC_COMMAND_SETTCK,

  /**
   * Represents the “shift:” command.
   */
  XVC_COMMAND_SHIFT
} xvc_command_t;

/**
 * Structure storing the state for a connection to the XVC server.
 */
struct xvc_server_connection_state_t {
  /**
   * Command that is currently being processed.
   *
   * This is XVC_COMMAND_NOT_SET when the server is expecting the next command.
   */
  xvc_command_t current_command;

  /**
   * Buffer that can be used for temporarily storing data received from the
   * network or that is going to be sent to the network.
   *
   * The buffer can store up to XVC_BUFFER_MAX_LEN bytes.
   */
  void *buffer;

  /**
   * Number of bytes currently stored in the buffer.
   */
  size_t buffer_len;

  /**
   * Size of the vectors for the currently processed “shift:” command in bits.
   *
   * This is equal to the number of TCK cycles that are needed for the shift
   * operation. A value of XVC_SHIFT_VECTOR_LENGTH_NOT_SET indicates that the
   * vector size still needs to be received from the client.
   */
  uint32_t shift_vector_length_in_bits;
};

/**
 * Enum representing the state of the XVC server.
 */
typedef enum {
  /**
   * Configuration change for the GPIO pins is in progress.
   */
  XVC_SERVER_CONFIG_IN_PROGRESS,

  /**
   * Disabling the server has been requested but an I/O operation is still in
   * progress.
   */
  XVC_SERVER_STATE_DISABLE_REQUESTED,

  /**
   * Server is disabled.
   */
  XVC_SERVER_STATE_DISABLED,

  /**
   * Transition to the disabled state is in progress.
   */
  XVC_SERVER_STATE_DISABLING,

  /**
   * Server is enabled.
   */
  XVC_SERVER_STATE_ENABLED,

  /**
   * Transition to the enabled state is in progress.
   */
  XVC_SERVER_STATE_ENABLING,

  /**
   * JTAG I/O operation is in progress.
   */
  XVC_SERVER_STATE_IO_IN_PROGRESS
} xvc_server_state_t;

/**
 * PIO clear output data register.
 *
 * This register is shared by the MMCTDO, XILTCK, XILTDI, and XILTMS pins.
 */
static WoReg *xvc_server_pio_codr;

/**
 * PIO data status register.
 *
 * This register is shared by the MMCTDO, XILTCK, XILTDI, and XILTMS pins.
 */
static RoReg *xvc_server_pio_pdsr;

/**
 * PIO set output data register.
 *
 * This register is shared by the MMCTDO, XILTCK, XILTDI, and XILTMS pins.
 */
static WoReg *xvc_server_pio_sodr;

/**
 * State of the server.
 *
 * The state defines which operations are allowed and how the GPIO pins are
 * configured.
 */
static volatile xvc_server_state_t xvc_server_state = (
  XVC_SERVER_STATE_DISABLED
);

/**
 * Calculates the number of bytes needed to store the specified number of bits.
 */
inline static uint32_t xvc_server_bits_to_bytes(uint32_t bits);

/**
 * Free memory associated with an xvc_server_connection_state_t instance.
 */
static void xvc_server_connection_state_free(
  struct xvc_server_connection_state_t *state
);

/**
 * Initialize an xvc_server_connection_state_t instance.
 *
 * Returns XVC_STATUS_OK on success.
 */
static xvc_status_t xvc_server_connection_state_init(
  struct xvc_server_connection_state_t *state
);

/**
 * Resets the connection state.
 *
 * This is used in order to reuse an previously initialized instance for a new
 * connection.
 */
static void xvc_server_connection_state_reset(
  struct xvc_server_connection_state_t *state
);

/**
 * Shift TDI/TMS bits into the JTAG TAPs and return the associated TDO bits.
 *
 * The bits are shifted out least significant byte and least significant bit
 * first. tdi_tdo_bytes and tms_bytes must be sufficiently sizes for
 * length_in_bits. The resulting TDO bits are returned in tdi_tdo_bytes.
 */
static xvc_status_t xvc_server_do_shift(
  uint8_t *tdi_tdo_bytes,
  uint8_t *tms_bytes,
  uint32_t length_in_bits
);

/**
 * Get current status of TDO.
 *
 * Returns 0 when TDO is low and 1 when TDO is high.
 */
inline static xvc_bool_t xvc_server_get_tdo(void);

/**
 * Handle received data for a TCP connection.
 *
 * This function consumes all data in recv_buffer, unless there is an error.
 *
 * Returns XVC_STATUS_OK on success and XVC_STATUS_ERROR on error.
 */
static xvc_status_t xvc_server_handle_receive(
  struct netconn *connection,
  struct xvc_server_connection_state_t *state,
  struct pbuf *recv_buffer
);

/**
 * Tell whether a JTAG cable is connected to the header on the PCB.
 */
inline static xvc_bool_t xvc_server_jtag_cable_connected(void);

/**
 * Read an XVC command from a buffer.
 *
 * On success, the received command is written to *command and XVC_STATUS_OK is
 * returned.
 *
 * On error, XVC_STATUS_ERROR is returned.
 *
 * When recv_buffer does not contain a full command and more data is needed,
 * XVC_STATUS_MORE_DATA_NEEDED is returned and the number of bytes consumed is
 * written to *bytes_received. This number must be passed as *bytes_received
 * the next time the function is called with more input. The same applies to
 * *command_buffer, which contains the data read so far.
 *
 * The buffer pointed to by command_buffer must be sized so that it can store
 * the longest allowed command string.
 */
static xvc_status_t xvc_server_read_command(
  struct pbuf *recv_buffer,
  uint8_t *command_buffer,
  size_t *bytes_received,
  xvc_command_t *command
);

/**
 * Read TDI/TMS data for the XVC shift command from a buffer.
 *
 * On success, the received data is available in data_buffer (first the bytes
 * containing TMS and then the bytes containing TDI).
 *
 * On error, XVC_STATUS_ERROR is returned.
 *
 * When recv_buffer does not contain the full byte sequence needed for both TMS
 * and TDI given the specified length_in_bits, XVC_STATUS_MORE_DATA_NEEDED is
 * returned and the number of bytes consumed is written to *bytes_received.
 * This number must be passed as *bytes_received the next time the function is
 * called with more input. The same applies to data_buffer, which contains the
 * data read so far.
 *
 * The buffer pointed to by data_buffer must be sized so that it can store all
 * the bits for both TMS and TDI (so the double of length_in_bits).
 */
static xvc_status_t xvc_server_read_data_shift_vector(
  struct pbuf *recv_buffer,
  uint32_t length_in_bits,
  uint8_t *data_buffer,
  size_t *bytes_received
);

/**
 * Read a uint32 (in little-endian format) from a buffer.
 *
 * On success, the received number is written to *data_buffer and XVC_STATUS_OK
 * is returned.
 *
 * On error, XVC_STATUS_ERROR is returned.
 *
 * When recv_buffer does not contain at least four bytes and more data is
 * needed, XVC_STATUS_MORE_DATA_NEEDED is returned and the number of bytes
 * consumed is written to *bytes_received. This number must be passed as
 * *bytes_received the next time the function is called with more input. The
 * same applies to *data_buffer, which contains the data read so far.
 *
 * The buffer pointed to by data_buffer must be able to store at least four
 * bytes.
 */
static xvc_status_t xvc_server_read_uint32(
  struct pbuf *recv_buffer, uint32_t *data_buffer, size_t *bytes_received
);

/**
 * Set the GPIO pin for TCK to high.
 *
 * The pin must previously have been configured as an output.
 */
inline static void xvc_server_set_tck_high(void);

/**
 * Set the GPIO pin for TCK to low.
 *
 * The pin must previously have been configured as an output.
 */
inline static void xvc_server_set_tck_low(void);

/**
 * Set the GPIO pins for TDI and TMS.
 *
 * A value of 0 means that the respective pin should be set low and a value of
 * 1 means that the respective pin should be set high. Specifying any other
 * value results in undefined behavior (clearing or setting other pins).
 *
 * The pins must previously have been configured as outputs.
 */
inline static void xvc_server_set_tdi_tms(xvc_bool_t tdi, xvc_bool_t tms);

/**
 * Update the configuration for the TCK, TDI, and TMS GPIO pins.
 *
 * This also changes the state of the JTAGSEL pin.
 *
 * Returns XVC_STATUS_ERROR when the server state is not expected_state.
 * Returns XVC_STATUS_OK otherwise.
 */
static xvc_status_t xvc_server_update_gpio_config(
  xvc_server_state_t expected_state
);

/**
 * Tell whether a USB cable is connected to the front-panel.
 *
 * A cable is only considered “connected” if the other end is connected to a
 * powered USB host device.
 */
inline static xvc_bool_t xvc_server_usb_cable_connected(void);

inline static uint32_t xvc_server_bits_to_bytes(uint32_t bits) {
  return (bits + 7) / 8;
}

static void xvc_server_connection_state_free(
  struct xvc_server_connection_state_t *state
) {
  mem_free(state->buffer);
  state->buffer = NULL;
}

static xvc_status_t xvc_server_connection_state_init(
  struct xvc_server_connection_state_t *state
) {
  state->buffer = mem_malloc(XVC_BUFFER_MAX_LEN);
  if (!state->buffer) {
    return XVC_STATUS_ERROR;
  }
  xvc_server_connection_state_reset(state);
  return XVC_STATUS_OK;
}

static void xvc_server_connection_state_reset(
  struct xvc_server_connection_state_t *state
) {
  state->current_command = XVC_COMMAND_NOT_SET;
  state->buffer_len = 0;
  state->shift_vector_length_in_bits = XVC_SHIFT_VECTOR_LENGTH_NOT_SET;
}

static xvc_status_t xvc_server_do_shift(
  uint8_t *tdi_tdo_bytes,
  uint8_t *tms_bytes,
  uint32_t length_in_bits
) {
  xvc_bool_t disable_requested = XVC_FALSE;
  uint32_t length_in_bytes = xvc_server_bits_to_bytes(length_in_bits);
  uint32_t last_byte_index = length_in_bytes - 1;
  xvc_bool_t next_tdi, next_tms;
  uint32_t next_bit_index_in_byte, next_byte_index;
  uint8_t tdi_next_byte, tms_next_byte;
  assert(length_in_bytes <= XVC_SHIFT_VECTOR_MAX_LEN_IN_BYTES);
  if (!length_in_bits) {
    return XVC_STATUS_OK;
  }
  /* The GPIO configuration must not be changed while we are shifting out bits.
  Therefore, we only proceed if the server is in the enabled state and put it
  into the I/O-in-progress state, so that no other task changes the GPIO
  configuration. */
  taskENTER_CRITICAL();
  if (xvc_server_state != XVC_SERVER_STATE_ENABLED) {
    taskEXIT_CRITICAL();
    return XVC_STATUS_ERROR;
  }
  xvc_server_state = XVC_SERVER_STATE_IO_IN_PROGRESS;
  taskEXIT_CRITICAL();
  /* Ensure that we start with TCK set to low. */
  xvc_server_set_tck_low();
  /* Normally, we do the calculations between setting TCK to high and sampling
  TDO, but for the first cycle we have to do this before starting the loop. */
  next_bit_index_in_byte = 0;
  next_byte_index = 0;
  tdi_next_byte = tdi_tdo_bytes[0];
  tms_next_byte = tms_bytes[0];
  next_tdi = tdi_next_byte & 1;
  next_tms = tms_next_byte & 1;
  tdi_tdo_bytes[0] = 0;
  for (uint32_t bit_index = 0; bit_index < length_in_bits; ++bit_index) {
    uint32_t bit_index_in_byte = next_bit_index_in_byte;
    uint32_t byte_index = next_byte_index;
    xvc_server_set_tdi_tms(next_tdi, next_tms);
    xvc_server_set_tck_high();
    /* After setting TCK high, we have to wait for a while before sampling TDO
    anyway, so we use this time to calculate TDI and TMS for the next
    iteration. When writing the last bit for a byte, we have to retrieve the
    next byte. For TMS, this is just a small optimization, but for TDI it is
    critical, because we overwrite TDI with TDO. */
    if (bit_index_in_byte == 7 && byte_index != last_byte_index) {
      next_byte_index += 1;
      next_bit_index_in_byte = 0;
      tdi_next_byte = tdi_tdo_bytes[next_byte_index];
      tms_next_byte = tms_bytes[next_byte_index];
      /* We have to set the byte in the TDI/TDO vector to zero, so that
      subsequent binary or operations use the correct initial value. */
      tdi_tdo_bytes[next_byte_index] = 0;
    } else {
      next_bit_index_in_byte += 1;
    }
    next_tdi = (tdi_next_byte >> next_bit_index_in_byte) & 1;
    next_tms = (tms_next_byte >> next_bit_index_in_byte) & 1;
    tdi_tdo_bytes[byte_index] |= (xvc_server_get_tdo() << bit_index_in_byte);
    xvc_server_set_tck_low();
}
  /* When we are done with using GPIO, we have to reset the server state, so
  that the GPIO configuration can be modified again. */
  taskENTER_CRITICAL();
  if (xvc_server_state == XVC_SERVER_STATE_DISABLE_REQUESTED) {
    disable_requested = XVC_TRUE;
  }
  xvc_server_state = XVC_SERVER_STATE_ENABLED;
  taskEXIT_CRITICAL();
  if (disable_requested) {
    xvc_server_disable();
    /* Returing XVC_STATUS_ERROR ensures that the connection is closed right
    away. */
    return XVC_STATUS_ERROR;
  }
  return XVC_STATUS_OK;
}

inline static xvc_bool_t xvc_server_get_tdo(void) {
  return (*xvc_server_pio_pdsr >> (MMCTDO & 0x1F)) & 1;
}

static xvc_status_t xvc_server_handle_receive(
  struct netconn *connection,
  struct xvc_server_connection_state_t *state,
  struct pbuf *recv_buffer
) {
  xvc_status_t status = XVC_STATUS_OK;
  while (recv_buffer->len || recv_buffer->next) {
    if (!recv_buffer->len) {
      recv_buffer = recv_buffer->next;
      continue;
    }
    if (state->current_command == XVC_COMMAND_NOT_SET) {
      xvc_command_t command;
      status = xvc_server_read_command(
        recv_buffer, state->buffer, &state->buffer_len, &command
      );
      switch (status) {
      case XVC_STATUS_OK:
        state->current_command = command;
        /* Reset buffer_len, so that it can be used for the next receive
        operation. */
        state->buffer_len = 0;
        break;
      case XVC_STATUS_MORE_DATA_NEEDED:
        continue;
      default:
#  ifdef DEBUG_XVC
        printf("XVC: Error while reading command.\r\n");
#  endif // DEBUG_XVC
        return XVC_STATUS_ERROR;
      }
    }
    switch (state->current_command) {
    case XVC_COMMAND_GETINFO:
      if (
        netconn_write(
          connection,
          XVC_INFO_STR,
          XVC_INFO_STR_LEN,
          NETCONN_COPY
        ) != ERR_OK
      ) {
#  ifdef DEBUG_XVC
        printf("XVC: Error while writing response to getinfo command.\r\n");
#  endif // DEBUG_XVC
        return XVC_STATUS_ERROR;
      }
      /* We are now ready to receive the next command. */
      state->current_command = XVC_COMMAND_NOT_SET;
      continue;
    case XVC_COMMAND_SETTCK:
      {
        status = xvc_server_read_uint32(
          recv_buffer, state->buffer, &state->buffer_len
        );
        switch (status) {
        case XVC_STATUS_OK:
          /* Reset buffer_len, so that it can be used for the next receive
          operation. */
          state->buffer_len = 0;
          break;
        case XVC_STATUS_MORE_DATA_NEEDED:
          continue;
        default:
#  ifdef DEBUG_XVC
        printf("XVC: Error while reading parameter for settck command.\r\n");
#  endif // DEBUG_XVC
          return XVC_STATUS_ERROR;
        }
        /* We do not allow setting the clock period, so we simply reply with
        the fixed period (that is is an estimate of the actual period). Due to
        having to drive everything from the CPU, we cannot support a clock rate
        of more than 1.5 MHz, and this already is pretty low, so there is no
        need to support setting an even lower clock rate. */
        uint32_t clock_period = 690;
        /* The XVC protocol expects the value to be in little endian format, so
        this only works when running on a little endian architecture. */
        if (
          netconn_write(
            connection, &clock_period, sizeof(clock_period), NETCONN_COPY
          ) != ERR_OK
        ) {
#  ifdef DEBUG_XVC
          printf("XVC: Error while writing response to settck command.\r\n");
#  endif // DEBUG_XVC
          return XVC_STATUS_ERROR;
        }
        /* We are now ready to receive the next command. */
        state->current_command = XVC_COMMAND_NOT_SET;
        continue;
      }
    case XVC_COMMAND_SHIFT:
      if (
        state->shift_vector_length_in_bits == XVC_SHIFT_VECTOR_LENGTH_NOT_SET
      ) {
        status = xvc_server_read_uint32(
          recv_buffer, state->buffer, &state->buffer_len
        );
        switch (status) {
        case XVC_STATUS_OK:
          state->shift_vector_length_in_bits = *(
            (uint32_t *) state->buffer
          );
          /* Reset buffer_len, so that it can be used for the next receive
          operation. */
          state->buffer_len = 0;
          break;
        case XVC_STATUS_MORE_DATA_NEEDED:
          continue;
        default:
#  ifdef DEBUG_XVC
        printf(
          "XVC: Error while reading vector length for shift command.\r\n"
        );
#  endif // DEBUG_XVC
          return XVC_STATUS_ERROR;
        }
      }
      if (
        state->shift_vector_length_in_bits > XVC_SHIFT_VECTOR_MAX_LEN_IN_BITS
      ) {
        return XVC_STATUS_ERROR;
      }
      status = xvc_server_read_data_shift_vector(
        recv_buffer,
        state->shift_vector_length_in_bits,
        state->buffer,
        &state->buffer_len
      );
      switch (status) {
      case XVC_STATUS_OK:
        {
          uint8_t *tdi_tdo_bytes, *tms_bytes;
          uint32_t shift_vector_length_in_bytes = xvc_server_bits_to_bytes(
            state->shift_vector_length_in_bits
          );
          tms_bytes = state->buffer;
          tdi_tdo_bytes = tms_bytes + shift_vector_length_in_bytes;
          if (
            xvc_server_do_shift(
              tdi_tdo_bytes, tms_bytes, state->shift_vector_length_in_bits
            ) != XVC_STATUS_OK
          ) {
#  ifdef DEBUG_XVC
          printf(
            "XVC: Error while trying to shift bits command.\r\n"
          );
#  endif // DEBUG_XVC
            return XVC_STATUS_ERROR;
          }
          if (
            netconn_write(
              connection,
              tdi_tdo_bytes,
              shift_vector_length_in_bytes,
              NETCONN_COPY
            ) != ERR_OK
          ) {
#  ifdef DEBUG_XVC
            printf("XVC: Error while writing response to shift command.\r\n");
#  endif // DEBUG_XVC
            return XVC_STATUS_ERROR;
          }
          /* Reset buffer_len, so that it can be used for the next receive
          operation. */
          state->buffer_len = 0;
          break;
        }
      case XVC_STATUS_MORE_DATA_NEEDED:
        continue;
      default:
#  ifdef DEBUG_XVC
        printf(
          "XVC: Error while reading data for shift command.\r\n"
        );
#  endif // DEBUG_XVC
        return XVC_STATUS_ERROR;
      }
      /* We are now ready to receive the next command. */
      state->current_command = XVC_COMMAND_NOT_SET;
      state->shift_vector_length_in_bits = XVC_SHIFT_VECTOR_LENGTH_NOT_SET;
      break;
    default:
      /* We should never end up in this branch. If we do, there is a bug in the
      code. */
#  ifdef DEBUG_XVC
      printf(
        "XVC: Unhandled case in switch statement.\r\n"
      );
#  endif // DEBUG_XVC
      return XVC_STATUS_ERROR;
    }
  }
  if (status == XVC_STATUS_MORE_DATA_NEEDED) {
    return XVC_STATUS_OK;
  }
  return status;
}

inline static xvc_bool_t xvc_server_jtag_cable_connected(void) {
  /* When a JTAG cable is connected, NXILJTAG is pulled low. */
  return pio_get_pin_value(NXILJTAG) ? XVC_FALSE : XVC_TRUE;
}

static xvc_status_t xvc_server_read_command(
  struct pbuf *recv_buffer,
  uint8_t *command_buffer,
  size_t *bytes_received,
  xvc_command_t *command
) {
  uint16_t copy_len;
  assert(recv_buffer->len);
  if (*bytes_received < XVC_COMMAND_STR_MIN_LEN) {
    /* The string that we have read so far is shorter than the shortest command
    string, so we can read more bytes. */
    copy_len = pbuf_copy_partial(
      recv_buffer,
      command_buffer + *bytes_received,
      XVC_COMMAND_STR_MIN_LEN - *bytes_received,
      0
    );
    *bytes_received += copy_len;
    pbuf_header(recv_buffer, -copy_len);
  }
  assert(*bytes_received);
  if (command_buffer[0] == 'g') {
    /* The only command string starting with “g” is the “getinfo:” command. So,
    we can read more bytes. If they do not match, we cannot do anything
    sensible with the remaining bytes anyway. */
    copy_len = pbuf_copy_partial(
      recv_buffer,
      command_buffer + *bytes_received,
      XVC_COMMAND_STR_GETINFO_LEN - *bytes_received,
      0
    );
    *bytes_received += copy_len;
    pbuf_header(recv_buffer, -copy_len);
    /* If the bytes that we have read so far do not match, this cannot be a
    valid command, and we can return an error right away. */
    if (memcmp(command_buffer, XVC_COMMAND_STR_GETINFO, *bytes_received)) {
#  ifdef DEBUG_XVC
      printf("XVC: Mismatch when expecting getinfo command.\r\n");
#  endif // DEBUG_XVC
      return XVC_STATUS_ERROR;
    }
    /* If the bytes match and we have received enough bytes for comparing the
    whole string, we are done. */
    if (*bytes_received == XVC_COMMAND_STR_GETINFO_LEN) {
      if (command) {
        *command = XVC_COMMAND_GETINFO;
      }
      return XVC_STATUS_OK;
    }
    return XVC_STATUS_MORE_DATA_NEEDED;
  } else if (command_buffer[0] == 's') {
    if (*bytes_received < 2) {
      return XVC_STATUS_MORE_DATA_NEEDED;
    }
    if (command_buffer[1] == 'e') {
      /* The only command string starting with “se” is the “settck:” command.
      So, we can read more bytes. If they do not match, we cannot do anything
      sensible with the remaining bytes anyway. */
      copy_len = pbuf_copy_partial(
        recv_buffer,
        command_buffer + *bytes_received,
        XVC_COMMAND_STR_SETTCK_LEN - *bytes_received,
        0
      );
      *bytes_received += copy_len;
      pbuf_header(recv_buffer, -copy_len);
      /* If the bytes that we have read so far do not match, this cannot be a
      valid command, and we can return an error right away. */
      if (
        memcmp(command_buffer, XVC_COMMAND_STR_SETTCK, *bytes_received)
      ) {
#  ifdef DEBUG_XVC
        printf("XVC: Mismatch when expecting settck command.\r\n");
#  endif // DEBUG_XVC
        return XVC_STATUS_ERROR;
      }
      /* If the bytes match and we have received enough bytes for comparing the
      whole string, we are done. */
      if (*bytes_received == XVC_COMMAND_STR_SETTCK_LEN) {
        if (command) {
          *command = XVC_COMMAND_SETTCK;
        }
        return XVC_STATUS_OK;
      }
      return XVC_STATUS_MORE_DATA_NEEDED;
    } else if (command_buffer[1] == 'h') {
      /* The only command string starting with “sh” is the “settck:” command.
      So, we can read more bytes. If they do not match, we cannot do anything
      sensible with the remaining bytes anyway. */
      copy_len = pbuf_copy_partial(
        recv_buffer,
        command_buffer + *bytes_received,
        XVC_COMMAND_STR_SHIFT_LEN - *bytes_received,
        0
      );
      *bytes_received += copy_len;
      pbuf_header(recv_buffer, -copy_len);
      /* If the bytes that we have read so far do not match, this cannot be a
      valid command, and we can return an error right away. */
      if (memcmp(command_buffer, XVC_COMMAND_STR_SHIFT, *bytes_received)) {
#  ifdef DEBUG_XVC
        printf("XVC: Mismatch when expecting shift command.\r\n");
#  endif // DEBUG_XVC
        return XVC_STATUS_ERROR;
      }
      /* If the bytes match and we have received enough bytes for comparing the
      whole string, we are done. */
      if (*bytes_received == XVC_COMMAND_STR_SHIFT_LEN) {
        if (command) {
          *command = XVC_COMMAND_SHIFT;
        }
        return XVC_STATUS_OK;
      }
      return XVC_STATUS_MORE_DATA_NEEDED;
    }
  }
  /* There are no commands that start with a different character sequence, so
  the buffer’s contents cannot represent the prefix of a valid command
  string. */
#  ifdef DEBUG_XVC
  printf("XVC: Invalid input when expecting command.\r\n");
#  endif // DEBUG_XVC
  return XVC_STATUS_ERROR;
}

static xvc_status_t xvc_server_read_data_shift_vector(
  struct pbuf *recv_buffer,
  uint32_t length_in_bits,
  uint8_t *data_buffer,
  size_t *bytes_received
) {
  /* The bits for TDI and TMS are provided as separate sequences, so we have to
  divide by eight and then multiply by two instead of dividing by four (the
  rounding works in a different way). */
  uint16_t bytes_remaining = (
    xvc_server_bits_to_bytes(length_in_bits) * 2 - *bytes_received
  );
  assert(length_in_bits <= XVC_SHIFT_VECTOR_MAX_LEN_IN_BITS);
  if (recv_buffer->len >= bytes_remaining) {
    pbuf_copy_partial(
      recv_buffer, data_buffer + *bytes_received, bytes_remaining, 0
    );
    pbuf_header(recv_buffer, -bytes_remaining);
    return XVC_STATUS_OK;
  }
  pbuf_copy_partial(
    recv_buffer, data_buffer + *bytes_received, recv_buffer->len, 0
  );
  *bytes_received += recv_buffer->len;
  pbuf_header(recv_buffer, -recv_buffer->len);
  return XVC_STATUS_MORE_DATA_NEEDED;
}

static xvc_status_t xvc_server_read_uint32(
  struct pbuf *recv_buffer, uint32_t *data_buffer, size_t *bytes_received
) {
  /* When this function is called and no data has been read yet, we have to
  reset the value to which we are going to append. */
  if (!*bytes_received) {
    *data_buffer = 0;
  }
  while (recv_buffer->len && *bytes_received < sizeof(uint32_t)) {
    uint32_t value = pbuf_get_at(recv_buffer, 0);
    pbuf_header(recv_buffer, -1);
    /* The XVC protocol uses the little endian format. */
    *data_buffer |= value << (*bytes_received * 8);
    ++(*bytes_received);
  }
  if (*bytes_received == sizeof(uint32_t)) {
    return XVC_STATUS_OK;
  }
  return XVC_STATUS_MORE_DATA_NEEDED;
}

inline static void xvc_server_set_tck_high(void) {
	*xvc_server_pio_sodr = 1 << (XILTCK & 0x1F);
}

inline static void xvc_server_set_tck_low(void) {
	*xvc_server_pio_codr = 1 << (XILTCK & 0x1F);
}

inline static void xvc_server_set_tdi_tms(xvc_bool_t tdi, xvc_bool_t tms) {
  uint32_t tdi_clear = (~tdi & 1) << (XILTDI & 0x1F);
  uint32_t tms_clear = (~tms & 1) << (XILTMS & 0x1F);
  uint32_t tdi_set = tdi << (XILTDI & 0x1F);
  uint32_t tms_set = tms << (XILTMS & 0x1F);
  *xvc_server_pio_codr = tdi_clear | tms_clear;
  *xvc_server_pio_sodr = tdi_set | tms_set;
}

static xvc_status_t xvc_server_update_gpio_config(
  xvc_server_state_t expected_state
) {
  xvc_bool_t jtag_cable_connected;
  xvc_bool_t usb_cable_connected;
  /* We must not interfere with a running I/O operation or a concurrently
  running configuration update, so we have to check the state in a critical
  section. */
  taskENTER_CRITICAL();
  if (xvc_server_state != expected_state) {
    taskEXIT_CRITICAL();
    return XVC_STATUS_ERROR;
  }
  xvc_server_state = XVC_SERVER_CONFIG_IN_PROGRESS;
  taskEXIT_CRITICAL();
  /* When a JTAG cable is connected to the JTAG header on the PCB, we set
  JTAGSEL high, so that the on-board USB JTAG adapter is not used. We also
  disable the XVC server in this case. If not JTAG cable is connected, we check
  whether a device is connected to the USB port of the USB JTAG adapter. If it
  is, we set JTAGSEL low, so that the adapter can be used and disable the
  XVC server. */
  jtag_cable_connected = xvc_server_jtag_cable_connected();
  usb_cable_connected = xvc_server_usb_cable_connected();
  /* When the XVC server is enabled or disabled, we have to change the GPIO pin
  configuration. */
  if (
    (expected_state == XVC_SERVER_STATE_DISABLED)
    || (expected_state == XVC_SERVER_STATE_DISABLING)
    || jtag_cable_connected
    || usb_cable_connected
  ) {
    pio_configure_pin(XILTCK, PIO_TYPE_PIO_INPUT);
    pio_configure_pin(XILTDI, PIO_TYPE_PIO_INPUT);
    pio_configure_pin(XILTMS, PIO_TYPE_PIO_INPUT);
  }
  if (
    (expected_state == XVC_SERVER_STATE_ENABLED)
    || (expected_state == XVC_SERVER_STATE_ENABLING)
  ) {
    if (jtag_cable_connected || usb_cable_connected) {
      return XVC_STATUS_ERROR;
    }
    pio_set_pin_high(JTAGSEL);
    pio_configure_pin(XILTCK, PIO_TYPE_PIO_OUTPUT_0);
    pio_configure_pin(XILTDI, PIO_TYPE_PIO_OUTPUT_0);
    pio_configure_pin(XILTMS, PIO_TYPE_PIO_OUTPUT_0);
  }
  if (jtag_cable_connected) {
    pio_set_pin_high(JTAGSEL);
  } else if (usb_cable_connected) {
    pio_set_pin_low(JTAGSEL);
  }
  /* When we are done, we restore the original state. */
  xvc_server_state = expected_state;
  return XVC_STATUS_OK;
}

inline static xvc_bool_t xvc_server_usb_cable_connected(void) {
  /* When the USB port is connected (to a powered host device), USBPOWER is
  pulled high. */
  return pio_get_pin_value(USBPOWER) ? XVC_TRUE : XVC_FALSE;
}

xvc_status_t xvc_server_disable(void) {
  /* We can only disable the XVC server if no other operation is in
  progress. */
  taskENTER_CRITICAL();
  if (xvc_server_state == XVC_SERVER_STATE_DISABLED) {
    taskEXIT_CRITICAL();
    return XVC_STATUS_OK;
  }
  if (xvc_server_state == XVC_SERVER_STATE_IO_IN_PROGRESS) {
    xvc_server_state = XVC_SERVER_STATE_DISABLE_REQUESTED;
    taskEXIT_CRITICAL();
    return XVC_STATUS_ERROR;
  }
  if (xvc_server_state != XVC_SERVER_STATE_ENABLED) {
    taskEXIT_CRITICAL();
    return XVC_STATUS_ERROR;
  }
  xvc_server_state = XVC_SERVER_STATE_DISABLING;
  taskEXIT_CRITICAL();
  xvc_server_update_gpio_config(XVC_SERVER_STATE_DISABLING);
  taskENTER_CRITICAL();
  xvc_server_state = XVC_SERVER_STATE_DISABLED;
  taskEXIT_CRITICAL();
  return XVC_STATUS_OK;
}

xvc_status_t xvc_server_enable(void) {
  xvc_status_t status;
  /* We can only enable the XVC server if no other operation is in
  progress. */
  taskENTER_CRITICAL();
  if (xvc_server_state == XVC_SERVER_STATE_ENABLED) {
    taskEXIT_CRITICAL();
    return XVC_STATUS_OK;
  }
  if (xvc_server_state != XVC_SERVER_STATE_DISABLED) {
    taskEXIT_CRITICAL();
    return XVC_STATUS_ERROR;
  }
  xvc_server_state = XVC_SERVER_STATE_ENABLING;
  taskEXIT_CRITICAL();
  status = xvc_server_update_gpio_config(XVC_SERVER_STATE_ENABLING);
  taskENTER_CRITICAL();
  if (status == XVC_STATUS_OK) {
    xvc_server_state = XVC_SERVER_STATE_ENABLED;
  } else {
    xvc_server_state = XVC_SERVER_STATE_DISABLED;
  }
  taskEXIT_CRITICAL();
  return status;
}

void xvc_server_init(void) {
  /* Initialize variables that are later needed for GPIO operations. We use the
  fact that all relevant pins are in the same PIO group, so they all share the
  same set of registers. */
  xvc_server_pio_codr = &pio_get_pin_group(XILTMS)->PIO_CODR;
  xvc_server_pio_pdsr = &pio_get_pin_group(XILTMS)->PIO_PDSR;
  xvc_server_pio_sodr = &pio_get_pin_group(XILTMS)->PIO_SODR;
  /* Initialize GPIO pin configuration that never changes. */
  pio_configure_pin(MMCTDO, PIO_TYPE_PIO_INPUT);
  pio_configure_pin(NXILJTAG, PIO_TYPE_PIO_INPUT);
  pio_configure_pin(USBPOWER, PIO_TYPE_PIO_INPUT);
  pio_configure_pin(JTAGSEL, PIO_TYPE_PIO_OUTPUT_0);
}

#else // (XVC_USED == 1)

void xvc_server_init(void) {
  /* Initialize GPIO pin configuration that never changes. */
  pio_configure_pin(NXILJTAG, PIO_TYPE_PIO_INPUT);
  pio_configure_pin(USBPOWER, PIO_TYPE_PIO_INPUT);
  pio_configure_pin(JTAGSEL, PIO_TYPE_PIO_OUTPUT_0);
}

#endif // (XVC_USED == 1)

#if (XVC_USED == 1)

xvc_bool_t xvc_server_is_enabled(void) {
  switch (xvc_server_state) {
  case XVC_SERVER_STATE_ENABLED:
  case XVC_SERVER_STATE_IO_IN_PROGRESS:
    return XVC_TRUE;
  default:
    return XVC_FALSE;
  }
}

void xvc_server_update_jtagsel(void) {
  /* When a JTAG cable is connected to the JTAG header on the PCB, we set
  JTAGSEL high, so that the on-board USB JTAG adapter is not used. We also
  disable the XVC server in this case. If not JTAG cable is connected, we check
  whether a device is connected to the USB port of the USB JTAG adapter. If it
  is, we set JTAGSEL low, so that the adapter can be used and disable the
  XVC server. */
  if (xvc_server_jtag_cable_connected()) {
    if (xvc_server_state != XVC_SERVER_STATE_DISABLED) {
      xvc_server_disable();
      return;
    }
    if (!pio_get_pin_value(JTAGSEL)) {
      xvc_server_update_gpio_config(XVC_SERVER_STATE_DISABLED);
      return;
    }
  } else if (pio_get_pin_value(USBPOWER)) {
    if (xvc_server_state != XVC_SERVER_STATE_DISABLED) {
      xvc_server_disable();
      return;
    }
    if (pio_get_pin_value(JTAGSEL)) {
      xvc_server_update_gpio_config(XVC_SERVER_STATE_DISABLED);
      return;
    }
  }
}

#else // (XVC_USED == 1)

void xvc_server_update_jtagsel(void) {
  /* When a JTAG cable is connected to the JTAG header on the PCB, we set
  JTAGSEL high, so that the on-board USB JTAG adapter is not used. We also
  disable the XVC server in this case. If not JTAG cable is connected, we check
  whether a device is connected to the USB port of the USB JTAG adapter. If it
  is, we set JTAGSEL low, so that the adapter can be used and disable the
  XVC server. */
  if (!pio_get_pin_value(NXILJTAG)) {
    pio_set_pin_high(JTAGSEL);
  } else if (pio_get_pin_value(USBPOWER)) {
    pio_set_pin_low(JTAGSEL);
  }
}

#endif // (XVC_USED == 1)
#if (XVC_USED == 1)

portTASK_FUNCTION(vXvcServer, parameters) {
  struct netconn *listener;
  err_t listener_status;
  struct xvc_server_connection_state_t state;
#  ifdef DEBUG_XVC
  printf("XVC: Starting XVC server task.\r\n");
#  endif // DEBUG_XVC
  if (xvc_server_connection_state_init(&state) != XVC_STATUS_OK) {
#  ifdef DEBUG_XVC
    printf("XVC: Server state could not be initialized.\r\n");
#  endif // DEBUG_XVC
    return;
  }
  listener = netconn_new(NETCONN_TCP);
  if (!listener) {
#  ifdef DEBUG_XVC
    printf("XVC: Could not create listener for XVC server.\r\n");
#  endif // DEBUG_XVC
    return;
  }
  if (netconn_bind(listener, NULL, XVC_PORT) != ERR_OK) {
#  ifdef DEBUG_XVC
    printf("XVC: Could not bind listener for XVC server.\r\n");
#  endif // DEBUG_XVC
    goto cleanup;
  }
  if (netconn_listen(listener) != ERR_OK) {
#  ifdef DEBUG_XVC
    printf("XVC: Could not start listener for XVC server.\r\n");
#  endif // DEBUG_XVC
    goto cleanup;
  }
  /* When a connection is closed, we want to accept a new one, so we loop
  forever. */
  do {
    struct netconn *connection = NULL;
    err_t connection_status;
#  ifdef DEBUG_XVC
    printf("XVC: Listener waiting for a new connection.\r\n");
#  endif // DEBUG_XVC
    listener_status = netconn_accept(listener, &connection);
    switch (listener_status) {
    case ERR_OK:
      break;
    case ERR_MEM:
    case ERR_TIMEOUT:
      /* Depending on the flags that are used when compiling lwIP, the call to
      netconn_accept might block or it might return with ERR_TIMEOUT if no new
      connection is available. In the latter case, we want to wait for a moment
      before trying again. The same applies if there currently is not enough
      memory to handle the new connection. */
      vTaskDelay(10);
      listener_status = ERR_OK;
      continue;
    default:
      /* Any other kind of error is considered fatal, and there is no sense in
      listening for further connections. The loop will break because
      listener_status is not ERR_OK. */
#  ifdef DEBUG_XVC
      printf("XVC: netconn_accept failed.\r\n");
#  endif // DEBUG_XVC
      continue;
    }
#  ifdef DEBUG_XVC
    printf("XVC: Accepted a new connection.\r\n");
#  endif // DEBUG_XVC
    /* We only accept a single connection anyway, so we can handle
    communication for this connection right from this task. */
    do {
      struct pbuf *recv_buffer = NULL;
      int status;
      /* If the XVC server is not enabled, close the connection right away. */
      if (xvc_server_state != XVC_SERVER_STATE_ENABLED) {
#  ifdef DEBUG_XVC
        printf("XVC: XVC server is not enabled, closing connection.\r\n");
#  endif // DEBUG_XVC
        connection_status = ERR_ABRT;
        break;
      }
      connection_status = netconn_recv_tcp_pbuf(connection, &recv_buffer);
      switch (connection_status) {
      case ERR_OK:
        break;
      case ERR_TIMEOUT:
        /* If lwIP is operating in non-blocking mode (this depends on compiler
        flags, there might be a timeout and it makes sense to check for
        received data later). */
        connection_status = ERR_OK;
        continue;
      default:
        /* All other error codes indicate non-recoverable errors, so we close
        the connection. The loop is going to break because connection_status is
        not ERR_OK. */
#  ifdef DEBUG_XVC
        printf("XVC: netconn_recv_tcp_buf failed.\r\n");
#  endif // DEBUG_XVC
        continue;
      }
      status = xvc_server_handle_receive(connection, &state, recv_buffer);
      pbuf_free(recv_buffer);
      if (status != XVC_STATUS_OK) {
        /* When there is an error, we close the connection. */
        connection_status = ERR_ABRT;
        break;
      }
    } while (connection_status == ERR_OK);
#  ifdef DEBUG_XVC
    printf("XVC: Closed connection.\r\n");
#  endif // DEBUG_XVC
    netconn_close(connection);
    netconn_delete(connection);
    connection = NULL;
    xvc_server_connection_state_reset(&state);
  } while (listener_status == ERR_OK);
cleanup:
#  ifdef DEBUG_XVC
    printf("XVC: Server task is quitting.\r\n");
#  endif // DEBUG_XVC
  if (listener != NULL) {
    netconn_close(listener);
    netconn_delete(listener);
    listener = NULL;
  }
  xvc_server_connection_state_free(&state);
}

#endif // (XVC_USED == 1)
