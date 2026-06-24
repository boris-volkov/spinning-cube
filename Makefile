CC      = gcc
CFLAGS  = -O2 -Wall -Wextra $(shell sdl2-config --cflags)
LDFLAGS = $(shell sdl2-config --libs) -lm

cube: cube.c
	$(CC) $(CFLAGS) -o cube cube.c $(LDFLAGS)

loomis: loomis.c
	$(CC) $(CFLAGS) -o loomis loomis.c $(LDFLAGS)

.PHONY: clean
clean:
	rm -f cube loomis
