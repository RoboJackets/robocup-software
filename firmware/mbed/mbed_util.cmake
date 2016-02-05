

function(MBED_ADD_INCS_ETH dir_list lib_src_root dev_vendor)
    set(dir_list
        ${dir_list}
        ${lib_src_root}/build/net/eth
        ${lib_src_root}/build/net/eth/EthernetInterface
        ${lib_src_root}/build/net/eth/Socket
        ${lib_src_root}/build/net/eth/lwip
        ${lib_src_root}/build/net/eth/lwip/include
        ${lib_src_root}/build/net/eth/lwip/include/ipv4
        ${lib_src_root}/build/net/eth/lwip/include/lwip
        ${lib_src_root}/build/net/eth/lwip/include/netif
        ${lib_src_root}/build/net/eth/lwip-sys
        ${lib_src_root}/build/net/eth/lwip-sys/arch
        ${lib_src_root}/build/net/eth/lwip-eth/arch/TARGET_${dev_vendor}
    )
endfunction()


function(MBED_ADD_INCS_RTOS dir_list lib_src_root)
    set(dir_list
        ${dir_list}
        ${lib_src_root}/build/rtos
        ${lib_src_root}/build/rtos/TARGET_CORTEX_M
    )
endfunction()


function(MBED_ADD_INCS_USB dir_list lib_src_root)
    set(dir_list
        ${dir_list}
        ${lib_src_root}/build/usb
        ${lib_src_root}/build/usb/USBAudio
        ${lib_src_root}/build/usb/USBDevice
        ${lib_src_root}/build/usb/USBHID
        ${lib_src_root}/build/usb/USBMIDI
        ${lib_src_root}/build/usb/USBMSD
        ${lib_src_root}/build/usb/USBSerial
        ${lib_src_root}/build/usb_host/USBHost
        ${lib_src_root}/build/usb_host/USBHost3GModule
        ${lib_src_root}/build/usb_host/USBHostHID
        ${lib_src_root}/build/usb_host/USBHostHub
        ${lib_src_root}/build/usb_host/USBHostMIDI
        ${lib_src_root}/build/usb_host/USBHostMSD
        ${lib_src_root}/build/usb_host/USBHostSerial
    )
endfunction()


function(MBED_ADD_INCS_DSP dir_list lib_src_root)
    set(dir_list
        ${dir_list}
        ${lib_src_root}/build/dsp
    )
endfunction()


function(MBED_ADD_INCS_RPC dir_list lib_src_root)
    set(dir_list
        ${dir_list}
        ${lib_src_root}/build/rpc
    )
endfunction()

function(MBED_SET_PLATFORM platform)

endfunction()
