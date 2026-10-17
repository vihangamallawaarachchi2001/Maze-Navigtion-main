import ctypes as C
from PIL import Image
x=C.CDLL('libX11.so.6'); co=C.CDLL('libXcomposite.so.1')
x.XOpenDisplay.argtypes=[C.c_char_p]; x.XOpenDisplay.restype=C.c_void_p
d=x.XOpenDisplay(None)
co.XCompositeNameWindowPixmap.argtypes=[C.c_void_p,C.c_ulong];co.XCompositeNameWindowPixmap.restype=C.c_ulong
x.XGetGeometry.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),*[C.POINTER(C.c_int)]*2,*[C.POINTER(C.c_uint)]*4]
class XI(C.Structure):
 _fields_=[('width',C.c_int),('height',C.c_int),('xoffset',C.c_int),('format',C.c_int),('data',C.c_void_p),('byte_order',C.c_int),('bitmap_unit',C.c_int),('bitmap_bit_order',C.c_int),('bitmap_pad',C.c_int),('depth',C.c_int),('bytes_per_line',C.c_int),('bits_per_pixel',C.c_int)]
x.XGetImage.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_ulong,C.c_int];x.XGetImage.restype=C.POINTER(XI)
import sys
for wid,path in zip(sys.argv[1::2],sys.argv[2::2]):
 p=co.XCompositeNameWindowPixmap(d,int(wid,16));root=C.c_ulong();xx=C.c_int();yy=C.c_int();w=C.c_uint();h=C.c_uint();border=C.c_uint();depth=C.c_uint()
 x.XGetGeometry(d,p,C.byref(root),C.byref(xx),C.byref(yy),C.byref(w),C.byref(h),C.byref(border),C.byref(depth))
 im=x.XGetImage(d,p,0,0,w.value,h.value,0xffffffff,2).contents
 raw=C.string_at(im.data,im.bytes_per_line*im.height)
 out=Image.frombytes('RGB',(im.width,im.height),raw,'raw','BGRX',im.bytes_per_line,1)
 out.save(path);print(path,out.getextrema())
