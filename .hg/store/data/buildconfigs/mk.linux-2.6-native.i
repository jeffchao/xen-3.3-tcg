         x   w      )���������x�^|ۄ _Ħ�j�<G            uEXTRAVERSION = native
IMAGE_TARGET = bzImage
INSTALL_BOOT_PATH = $(DESTDIR)/boot
include buildconfigs/mk.linux-2.6-xen
     x     #   x      ;A    ����hԐW��n��?��߾���                      EXTRAVERSION = -native
     �     6   �      ;P   �����U������֣�'r�.�C               R   R   *
XEN_LINUX_ALLOW_INTERFACE_MISMATCH := y

     �        y      ?�   �����=]h:��$X��#_$FD               S   |         �     }   |     ?�   ����7�
3��f�*�y�OѸ���            uEXTRAVERSION = -native
IMAGE_TARGET = bzImage
INSTALL_BOOT_PATH = $(DESTDIR)/boot

include buildconfigs/mk.linux-2.6-common
