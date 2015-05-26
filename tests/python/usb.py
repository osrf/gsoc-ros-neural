import usb.core

for dev in usb.core.find(find_all=True):
  print "Device: ", dev.filename
  print " id vendor: %d (0x%04x)" % (dev.idVendor, dev.idVendor)
  print " idProduct: %d (0x%04x)" % (dev.idProduct, dev.idProduct)

