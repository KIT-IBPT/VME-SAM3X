#ifndef XVC_SERVER_H
#define XVC_SERVER_H

#  if (XVC_USED == 1)

#    include "portmacro.h"

/**
 * TCP port used by the XVC server.
 */
#  define XVC_PORT 2542

/**
 * Enum representing a boolean.
 */
typedef enum {
    /**
     * False.
     */
    XVC_FALSE = 0,

    /**
     * True.
     */
    XVC_TRUE = 1
} xvc_bool_t;

/**
 * Enum representing the status of an operation.
 */
typedef enum {
    /**
     * Indicates success.
     */
    XVC_STATUS_OK = 0,

    /**
     * Indicates failure.
     */
    XVC_STATUS_ERROR = 1,

    /**
     * Indicates that the function has to be called again with additional data.
     */
    XVC_STATUS_MORE_DATA_NEEDED = 2
} xvc_status_t;

/**
 * Disable the XVC server.
 */
xvc_status_t xvc_server_disable(void);

/**
 * Enable the XVC server.
 */
xvc_status_t xvc_server_enable(void);

#  endif // (XVC_USED == 1)

/**
 * Initialize the system for the XVC server (e.g. configure GPIO).
 *
 * This must be called before using any of the other functions.
 */
void xvc_server_init(void);

#  if (XVC_USED == 1)

/**
 * Check whether the XVC server is enabled.
 */
xvc_bool_t xvc_server_is_enabled(void);

#  endif // (XVC_USED == 1)

/**
 * Check whether a JTAG adapter is connected and update JTAGSEL accordingly.
 *
 * This also disables the XVC server is an external JTAG adapter is connected
 * or a USB host device is connected to the on-board USB JTAG adapter.
 */
void xvc_server_update_jtagsel(void);

#  if (XVC_USED == 1)

/**
 * Task function for the XVC server.
 */
portTASK_FUNCTION_PROTO(vXvcServer, parameters);

#  endif // (XVC_USED == 1)
#endif // XVC_SERVER_H
