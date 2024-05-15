#include "hci.h"

#include "../protocolBuffer/hci.pb.h" // Generated by nanopb from hci.proto
#include <pb_encode.h>

#define STACKSIZE 512
#define PRIORITY 2

#define PREAMBLE 0xAA
#define REQUEST 0x01
#define RESPONSE 0x02

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

K_FIFO_DEFINE(hci_send_fifo);
K_FIFO_DEFINE(hci_read_fifo);

LOG_MODULE_DECLARE(AHU);

#define MAX_MESSAGE_LENGTH 15  // Expected maximum message size

typedef struct {
    void *fifo_reserved;  // Reserved for kernel FIFO usage
    uint8_t message[MAX_MESSAGE_LENGTH];  // Fixed-size array to hold the message
    size_t message_length;  // Actual length of the message
} hci_queue_item;

/* function add_to_hci_queue()
 * −−−−−−−−−−−−−−− */
void add_to_hci_queue(char movement_id) {
    printk("Adding to HCI queue: movement_id = %c\n", movement_id);

    // Allocate memory for a new queue item
    hci_queue_item *hciQueueItem = k_calloc(1, sizeof(hci_queue_item));
    if (!hciQueueItem) {
        LOG_ERR("Failed to allocate memory for hci_queue_item");
        return;
    }

    // Directly store the character into the message buffer
    hciQueueItem->message[0] = movement_id;
    hciQueueItem->message_length = 1;  // Set the message length to 1 since we're sending a single character

    // Place the item into the HCI send queue
    k_fifo_put(&hci_send_fifo, hciQueueItem);
}

/* function init_hci()
 * −−−−−−−−−−−−−−−
 * Initialize HCI module
 *
 * Returns: None
 *
 * Errors: Prints an error message if setting UART callback fails */
void init_hci() {
  /* Enables UART receive interrupt 
   * interrupt when data is received, triggering the callback function */
  uart_irq_rx_enable(uart_dev);
}

void uart_send(uint8_t *buffer, size_t len) {
  // uart_poll_out(uart_dev, 'c');
  for (size_t i = 0; i < len; i++) {
    printk("buffer[i]: %c\n", buffer[i]);
    uart_poll_out(uart_dev, buffer[i]);

  }
  uart_poll_out(uart_dev, '\n');
}

/* function thread_send_hci()
 * −−−−−−−−−−−−−−−
 * Thread to send messages from the HCI send queue
 *
 * Returns: None
 *
 * Errors: None */
void thread_send_hci(void *p1, void *p2, void *p3) {
  for (;;) {
    hci_queue_item *queueItem = k_fifo_get(&hci_send_fifo, K_FOREVER);

    if (!queueItem) {
      k_msleep(100);
      continue;
    }

    printk("queueItem: %s & %d\n", queueItem->message, queueItem->message_length);

    uart_send(queueItem->message, queueItem->message_length);

    k_free(queueItem);

    k_msleep(100);
  }
}

/* function decode_hci_message()
 * −−−−−−−−−−−−−−−
 * Decode and process received HCI message
 *
 * Returns: None
 *
 * Errors: None */
void decode_hci_message(char *message) {
  int message_len = message[2] & 0x0F;
  char buf[15];
  memcpy(buf, &(message[3]), message_len);
  if (strcmp(buf, "0")) {
    LOG_INF("Pushbutton Status: Pressed (%s)", buf);
  } else {
    LOG_INF("Pushbutton Status: Not Pressed (%s)", buf);
  }
}

/* function thread_read_hci()
 * −−−−−−−−−−
 * Thread to read messages from HCI read queue
 *
 * Returns: None
 *
 * Errors: None */
void thread_read_hci(void *p1, void *p2, void *p3) {
  for (;;) {
    hci_queue_item *queueItem = k_fifo_get(&hci_read_fifo, K_MSEC(100));
    if (!queueItem) {
        k_msleep(100);
        continue;
    }

    decode_hci_message(queueItem->message);

    k_free(queueItem);
    k_msleep(100);
  }
}

K_THREAD_DEFINE(threadSendHCI, STACKSIZE, thread_send_hci, NULL, NULL, NULL,
                PRIORITY, 0, 0);
K_THREAD_DEFINE(threadReadHCI, STACKSIZE, thread_read_hci, NULL, NULL, NULL,
                PRIORITY, 0, 0);