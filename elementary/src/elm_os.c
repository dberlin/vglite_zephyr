#include "elm_os.h"
#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>

vg_lite_error_t elm_os_set_tls(void* tls) {
  if (tls == NULL) return VG_LITE_INVALID_ARGUMENT;
  k_thread_custom_data_set(tls);
  return VG_LITE_SUCCESS;
}

void* elm_os_get_tls(void) {
  return k_thread_custom_data_get();
}

void elm_os_reset_tls(void) {
  k_thread_custom_data_set(NULL);
}

#else
#include "FreeRTOS.h"
#include "task.h"

vg_lite_error_t elm_os_set_tls(void* tls)
{
    if(tls == NULL)
        return VG_LITE_INVALID_ARGUMENT;
    vTaskSetThreadLocalStoragePointer(NULL, 1, tls);
        return VG_LITE_SUCCESS;
}

void * elm_os_get_tls(void)
{
    return pvTaskGetThreadLocalStoragePointer(NULL, 1);
}

void elm_os_reset_tls(void)
{
    vTaskSetThreadLocalStoragePointer(NULL, 1, NULL);
}

#endif