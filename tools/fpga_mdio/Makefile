CC = gcc
AR = ar
CFLAGS = -std=c99
#LFLAGS = -lm -pthread -DVME_DEBUG
LFLAGS =
DEL_FILE = rm -f
MV_FILE = mv -f
OBJ_FILE_NODE = ./*.o
#VPATH = 
INCPATH     =     -I../include/
TARGET   = fpga_mdio

OBJECTS += fpga_mdio.o


.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o $@ $<
$(TARGET):$(OBJECTS)
	$(CC) $(LFLAGS) -o $(TARGET) $(OBJECTS)
	
clean:
	-$(DEL_FILE) $(TARGET)
	-$(DEL_FILE) $(OBJ_FILE_NODE)


