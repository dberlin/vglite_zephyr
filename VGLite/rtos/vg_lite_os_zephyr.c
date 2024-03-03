#include <zephyr/kernel.h>

#include "vg_lite_hal.h"
#include "vg_lite_hw.h"
#include "vg_lite_os.h"

/* If bit31 is activated this indicates a bus error */
#define IS_AXI_BUS_ERR(x) ((x) & (1U << 31))

#if !defined(VG_DRIVER_SINGLE_THREAD)
#define ISR_WAIT_TIME 0x1FFFF
#define MAX_MUTEX_TIME 100
#define TASK_WAIT_TIME 20

/* command queue task parameter */
#define QUEUE_TASK_NAME "queue_task"
#ifndef QUEUE_TASK_PRIO
#define QUEUE_TASK_PRIO 0
#endif /* QUEUE_TASK_PRIO */
#define QUEUE_TASK_SIZE 1024
K_THREAD_STACK_DEFINE(queue_thread_stack, QUEUE_TASK_SIZE);
#define QUEUE_LENGTH 8
#define MAX_QUEUE_WAIT_NUM 10

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

typedef struct vg_lite_queue {
  uint32_t cmd_physical;
  uint32_t cmd_offset;
  uint32_t cmd_size;
  vg_lite_os_async_event_t *event;
} vg_lite_queue_t;

typedef struct vg_lite_os {
  struct k_thread queue_thread_data;
  k_tid_t thread_handle;
  struct k_queue queue_handle;
} vg_lite_os_t;

K_SEM_DEFINE(mutex, 0, 1);
static vg_lite_os_t os_obj = {0};

struct k_sem semaphore[TASK_LENGTH] = {};
struct k_sem command_semaphore;
uint32_t curContext;
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */
K_SEM_DEFINE(int_queue, 0, 1);
volatile uint32_t int_flags;

void __attribute__((weak)) vg_lite_bus_error_handler() {
  /*
   * Default implementation of the bus error handler does nothing. Application
   * should override this handler if it requires to be notified when a bus
   * error event occurs.
   */
  return;
}

#if !defined(VG_DRIVER_SINGLE_THREAD)
/* command queue function */
void command_queue(void *, void *, void *) {
  uint32_t even_got;
  k_queue_init(&os_obj.queue_handle);
  // os_obj.queue_handle = xQueueCreate(QUEUE_LENGTH, sizeof(vg_lite_queue_t *
  // ));
  k_sem_init(&command_semaphore, 0, 30);
  while (true) {
    even_got = 0;
    if (k_sem_take(&command_semaphore, K_FOREVER) == 0) {
      if (!k_queue_is_empty(&os_obj.queue_handle)) {
        vg_lite_queue_t *peek_queue =
            k_queue_get(&os_obj.queue_handle, K_MSEC(TASK_WAIT_TIME));
        if (peek_queue != NULL) {
#if defined(PRINT_COMMAND_BUFFER)
          int i = 0;
          for (i = 0; i < (peek_queue->cmd_size + 3) / 4; i++) {
            if (i % 4 == 0) printf("\r\n");
            printf("0x%08x ", ((uint32_t *)(peek_queue->cmd_physical +
                                            peek_queue->cmd_offset))[i]);
          }
#endif
          vg_lite_hal_poke(VG_LITE_HW_CMDBUF_ADDRESS,
                           peek_queue->cmd_physical + peek_queue->cmd_offset);
          vg_lite_hal_poke(VG_LITE_HW_CMDBUF_SIZE,
                           (peek_queue->cmd_size + 7) / 8);

          if (vg_lite_hal_wait_interrupt(ISR_WAIT_TIME, (uint32_t)~0,
                                         &even_got))
            peek_queue->event->signal = VG_LITE_HW_FINISHED;
          else
#if defined(PRINT_DEBUG_REGISTER)
          {
            unsigned int debug;
            unsigned int iter;
            for (iter = 0; iter < 16; iter++) {
              vg_lite_hal_poke(0x470, iter);
              debug = vg_lite_hal_peek(0x450);
              printf("0x450[%d] = 0x%x\n", iter, debug);
            }
            for (iter = 0; iter < 16; iter++) {
              vg_lite_hal_poke(0x470, iter << 16);
              debug = vg_lite_hal_peek(0x454);
              printf("0x454[%d] = 0x%x\n", iter, debug);
            }
            for (iter = 0; iter < 16; iter++) {
              vg_lite_hal_poke(0x478, iter);
              debug = vg_lite_hal_peek(0x468);
              printf("0x468[%d] = 0x%x\n", iter, debug);
            }
            for (iter = 0; iter < 16; iter++) {
              vg_lite_hal_poke(0x478, iter);
              debug = vg_lite_hal_peek(0x46C);
              printf("0x46C[%d] = 0x%x\n", iter, debug);
            }
#endif
            /* wait timeout */
            peek_queue->event->signal = VG_LITE_IDLE;
#if defined(PRINT_DEBUG_REGISTER)
          }
#endif
          k_sem_give(&semaphore[peek_queue->event->semaphore_id]);
          vg_lite_os_free((void *)peek_queue);
        }
      }
    }
  }
}

int32_t vg_lite_os_set_tls(void *tls) {
  if (tls == NULL) return VG_LITE_INVALID_ARGUMENT;

  k_thread_custom_data_set(tls);
  return VG_LITE_SUCCESS;
}

void *vg_lite_os_get_tls() { return k_thread_custom_data_get(); }
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */

void *vg_lite_os_malloc(uint32_t size) { return k_malloc(size); }

void vg_lite_os_free(void *memory) { k_free(memory); }

#if !defined(VG_DRIVER_SINGLE_THREAD)
void vg_lite_os_reset_tls() { k_thread_custom_data_set(NULL); }
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */

void vg_lite_os_sleep(uint32_t msec) { k_sleep(K_MSEC(msec)); }

int32_t vg_lite_os_initialize(void) {
#if !defined(VG_DRIVER_SINGLE_THREAD)
  static int task_number = 0;
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */

  int_flags = 0;

#if !defined(VG_DRIVER_SINGLE_THREAD)

  if (task_number == 0) {
    if (k_sem_take(&mutex, K_MSEC(TASK_WAIT_TIME)) == 0) {
      if (os_obj.thread_handle == NULL) {
        os_obj.thread_handle =
            k_thread_create(&os_obj.queue_thread_data, queue_thread_stack,
                            K_THREAD_STACK_SIZEOF(queue_thread_stack),
                            command_queue, NULL, NULL, NULL, 0, 0, K_NO_WAIT);
        if (os_obj.thread_handle == 0) {
          /* command queue task create fail */
          k_sem_give(&mutex);
          return VG_LITE_MULTI_THREAD_FAIL;
        }
      }
      task_number++;
      k_sem_give(&mutex);
      return VG_LITE_SUCCESS;
    }
  }
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */
  return VG_LITE_SUCCESS;
}

void vg_lite_os_deinitialize(void) {
  /* TODO: Remove clock. */
  /* TODO: Remove power. */
}

#if !defined(VG_DRIVER_SINGLE_THREAD)
int32_t vg_lite_os_lock() {
  if (k_sem_take(&mutex, K_MSEC(MAX_MUTEX_TIME)) != 0)
    return VG_LITE_MULTI_THREAD_FAIL;

  return VG_LITE_SUCCESS;
}

int32_t vg_lite_os_unlock() {
  k_sem_give(&mutex);
  return VG_LITE_SUCCESS;
}

int32_t vg_lite_os_submit(uint32_t context, uint32_t physical, uint32_t offset,
                          uint32_t size, vg_lite_os_async_event_t *event) {
  vg_lite_queue_t *queue_node =
      (vg_lite_queue_t *)vg_lite_os_malloc(sizeof(vg_lite_queue_t));
  if (queue_node == NULL) return VG_LITE_MULTI_THREAD_FAIL;

  queue_node->cmd_physical = physical;
  queue_node->cmd_offset = offset;
  queue_node->cmd_size = size;
  queue_node->event = event;

  /* Current command buffer has been sent to the command queue. */
  event->signal = VG_LITE_IN_QUEUE;

  if (k_queue_alloc_append(&os_obj.queue_handle, queue_node) != 0)
    return VG_LITE_MULTI_THREAD_FAIL;
  curContext = context;

  if (vg_lite_os_wait_event(event) == VG_LITE_SUCCESS) {
    k_sem_give(&command_semaphore);
    return VG_LITE_SUCCESS;
  }

  return VG_LITE_MULTI_THREAD_FAIL;
}

int32_t vg_lite_os_wait(uint32_t timeout, vg_lite_os_async_event_t *event) {
  if (k_sem_take(&semaphore[event->semaphore_id], K_FOREVER) == 0) {
    if (event->signal == VG_LITE_HW_FINISHED) {
      k_sem_give(&semaphore[event->semaphore_id]);
      return VG_LITE_SUCCESS;
    }
    k_sem_give(&semaphore[event->semaphore_id]);
    return VG_LITE_TIMEOUT;
  }
  return VG_LITE_TIMEOUT;
}
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */

void vg_lite_os_IRQHandler(void) {
  uint32_t flags = vg_lite_hal_peek(VG_LITE_INTR_STATUS);

  if (flags) {
    /* Combine with current interrupt flags. */
    int_flags |= flags;

    /* Wake up any waiters. */
    k_sem_give(&int_queue);
  }
}

int32_t vg_lite_os_wait_interrupt(uint32_t timeout, uint32_t mask,
                                  uint32_t *value) {
#if _BAREMETAL
  uint32_t int_status = 0;
  int_status = vg_lite_hal_peek(VG_LITE_INTR_STATUS);
  (void)value;

  while (int_status == 0) {
    int_status = vg_lite_hal_peek(VG_LITE_INTR_STATUS);
    usleep(1);
  }

  if (IS_AXI_BUS_ERR(*value)) {
    vg_lite_bus_error_handler();
  }
  return 1;
#else /*for rt500*/
  if (k_sem_take(&int_queue, K_MSEC(timeout)) == 0) {
    if (value != NULL) {
      *value = int_flags & mask;
      if (IS_AXI_BUS_ERR(*value)) {
        vg_lite_bus_error_handler();
      }
    }
    int_flags = 0;

    return 1;
  }
  return 0;
#endif
}

#if !defined(VG_DRIVER_SINGLE_THREAD)
int32_t vg_lite_os_init_event(vg_lite_os_async_event_t *event,
                              uint32_t semaphore_id, int32_t state) {
  if (event->semaphore_id >= TASK_LENGTH) return VG_LITE_INVALID_ARGUMENT;

  if (k_sem_init(&semaphore[semaphore_id], 1, 1) != 0)
    return VG_LITE_OUT_OF_MEMORY;

  event->semaphore_id = semaphore_id;
  event->signal = state;

  return VG_LITE_SUCCESS;
}

int32_t vg_lite_os_delete_event(vg_lite_os_async_event_t *event) {
  if (event->semaphore_id >= TASK_LENGTH) return VG_LITE_INVALID_ARGUMENT;

  k_sem_reset(&semaphore[event->semaphore_id]);

  return VG_LITE_SUCCESS;
}

int32_t vg_lite_os_wait_event(vg_lite_os_async_event_t *event) {
  if (event->semaphore_id >= TASK_LENGTH) return VG_LITE_INVALID_ARGUMENT;

  if (k_sem_take(&semaphore[event->semaphore_id], K_FOREVER) != 0)
    return VG_LITE_MULTI_THREAD_FAIL;

  return VG_LITE_SUCCESS;
}

int32_t vg_lite_os_signal_event(vg_lite_os_async_event_t *event) {
  if (event->semaphore_id >= TASK_LENGTH) return VG_LITE_INVALID_ARGUMENT;
  k_sem_give(&semaphore[event->semaphore_id]);
  return VG_LITE_SUCCESS;
}

int8_t vg_lite_os_query_context_switch(uint32_t context) {
  if (!curContext || curContext == context) return FALSE;
  return TRUE;
}
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */
