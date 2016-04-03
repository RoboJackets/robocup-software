#include <hidapi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <wchar.h>

#define BASE_VID 0x524A
#define BASE_PID 0x4253

// typedef struct hid_device_info

int main(int argc, char *argv[]) {
   hid_device *handle;
   struct hid_device_info *devices, *currentDevice;
   devices = hid_enumerate(0x00, 0x00);
   currentDevice = devices;
   while (currentDevice) {
      // printf("lp start\r\n");
      printf("Device Found\n  type: %04hx %04hx\n",
         currentDevice->vendor_id, 
         currentDevice->product_id);
      // printf("path: %s\n  serial_number: %ls",
      //    currentDevice->path, 
      //    currentDevice->serial_number);
      // printf("\n");
      printf("  Manufacturer: %ls\n", currentDevice->manufacturer_string);
      printf("  Product:      %ls\n", currentDevice->product_string);
      printf("  Release:      %hx\n", currentDevice->release_number);
      printf("  Interface:    %d\n",  currentDevice->interface_number);
      // printf("\n");
      // struct hid_device_info *tmp = currentDevice->next;
      // printf("mid\r\n");
      currentDevice = currentDevice->next;
      // printf("lp end\r\n");
   }
   hid_free_enumeration(devices);

   // unsigned char buf[9];
   // memset(buf, 0x00, sizeof(buf));
   // buf[0] = 0x01;
   // buf[1] = 0x81;

   handle = hid_open(BASE_VID, BASE_PID, 0x0);
   if (handle == NULL) {
      printf("unable to open device\r\n");
      return -1;
   }
      
   printf("opened device\r\n");
   const unsigned char LEN = 64;
   unsigned char buf[LEN];
   for (char count = 'A';;) {
      // sprintf(buf, "%d", count++);
      buf[0] = count++;
      buf[1] = '\0';
      hid_write(handle, buf, 2/*strlen(buf)*/);
      if (count == 'z') {
        count = 'A';
      }
      sleep(1);
   }

   return 0;
}
