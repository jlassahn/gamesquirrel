
void usb_init(void);
void usb_start(void);
void usb_tick(void); // FIXME becomes an interrupt handler
int usb_send(const char *data, int max_bytes);
int usb_receive(char *data, int max_bytes);

